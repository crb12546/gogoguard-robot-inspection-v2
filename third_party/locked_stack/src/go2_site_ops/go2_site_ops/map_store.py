"""Immutable, versioned map artifacts and guarded map activation.

The store keeps large mapping outputs immutable after sealing.  Mutable state
such as the currently active map lives outside version directories and always
pins the exact manifest hash it approved.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import shutil
import stat
import struct
import tempfile
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .coordinate_contract import ContractError
from .localization_quality import resolve_localization_quality_profile
from .release_calibration import ReleaseCalibrationBundle


MAP_SCHEMA = "go2.map_version.v1"
ACTIVATION_SCHEMA = "go2.map_activation.v1"
IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")


class MapStoreError(ValueError):
    """Raised when a map artifact or lifecycle transition is unsafe."""


@dataclass(frozen=True)
class ArtifactSpec:
    kind: str
    relative_path: str


ARTIFACT_SPECS = {
    "glim_dump": ArtifactSpec("directory", "artifacts/glim"),
    "map_cloud": ArtifactSpec("file", "artifacts/map/map.ply"),
    "map_quality": ArtifactSpec("directory", "artifacts/map/quality"),
    "potree": ArtifactSpec("directory", "artifacts/potree"),
    "site_overview": ArtifactSpec("directory", "artifacts/overview"),
    "stability_evidence": ArtifactSpec("file", "artifacts/review/stability_evidence.json"),
    "review_tasks": ArtifactSpec("file", "artifacts/review/tasks.json"),
    "reference_route": ArtifactSpec("file", "artifacts/review/reference_route.json"),
    "capture_manifests": ArtifactSpec("directory", "artifacts/review/capture_manifests"),
    "review_audit": ArtifactSpec("file", "artifacts/review/decisions.json"),
    "localization_map": ArtifactSpec("file", "artifacts/localization/map.pcd"),
    "route": ArtifactSpec("file", "artifacts/navigation/route.json"),
    "runtime_profile": ArtifactSpec("file", "artifacts/navigation/runtime_profile.json"),
    "calibration_bundle": ArtifactSpec("file", "artifacts/calibration/release_bundle.json"),
    "validation_evidence": ArtifactSpec("directory", "artifacts/validation/evidence"),
    "validation_report": ArtifactSpec("file", "artifacts/validation/report.json"),
}

PURPOSE_REQUIREMENTS = {
    "review": {"glim_dump", "map_cloud", "potree", "site_overview", "stability_evidence", "review_tasks"},
    # A release is the small, motion-critical package mirrored to the robot.
    # Heavy GLIM/Potree/review geometry remains immutable in its parent review
    # version and is bound through parentVersionId + the review audit's exact
    # source manifest hash.
    "release": {
        "review_audit",
        "localization_map",
        "route",
        "runtime_profile",
        "calibration_bundle",
        "validation_evidence",
        "validation_report",
    },
}

ACTIVATION_GATES = (
    "reviewComplete",
    "localizationValidated",
    "routeValidated",
    "safetyGatePassed",
)

REVIEW_DECISIONS = {
    "object_persistence": {"keep_fixed", "remove_temporary", "defer"},
    "traversability": {"allow", "forbid", "defer"},
}


def _canonical_json(value: Mapping[str, Any]) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            chunk = handle.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _validate_identifier(value: str, label: str) -> str:
    value = str(value).strip()
    if not IDENTIFIER.fullmatch(value):
        raise MapStoreError("invalid %s: %r" % (label, value))
    return value


def _validate_site_id(value: str) -> str:
    value = str(value).strip()
    if (
        not value
        or len(value) > 128
        or value in {".", ".."}
        or "/" in value
        or "\\" in value
        or any(ord(character) < 32 for character in value)
    ):
        raise MapStoreError("invalid site_id: %r" % value)
    return value


def _validate_hash(value: str, label: str) -> str:
    value = str(value).strip().lower()
    if not SHA256.fullmatch(value):
        raise MapStoreError("%s must be a SHA-256 hex digest" % label)
    return value


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=".%s." % path.name,
        dir=str(path.parent),
    )
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, sort_keys=True, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(str(temporary_path), str(path))
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def _reject_unsafe_source(path: Path) -> None:
    if path.is_symlink():
        raise MapStoreError("artifact source cannot be a symlink: %s" % path)
    if path.is_file():
        return
    if not path.is_dir():
        raise MapStoreError("artifact source must be a regular file or directory: %s" % path)
    for child in path.rglob("*"):
        if child.is_symlink():
            raise MapStoreError("artifact tree cannot contain symlinks: %s" % child)
        if not child.is_file() and not child.is_dir():
            raise MapStoreError("artifact tree contains a special file: %s" % child)


def _copy_artifact(source: Path, destination: Path, kind: str) -> None:
    source = Path(source)
    _reject_unsafe_source(source)
    source = source.resolve()
    if kind == "file":
        if not source.is_file():
            raise MapStoreError("expected file artifact: %s" % source)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(source), str(destination))
        return
    if not source.is_dir():
        raise MapStoreError("expected directory artifact: %s" % source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(str(source), str(destination))


def _regular_files(root: Path) -> Iterable[Path]:
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise MapStoreError("sealed map contains a symlink: %s" % path)
        if path.is_file():
            yield path
        elif not path.is_dir():
            raise MapStoreError("sealed map contains a special file: %s" % path)


class MapVersionStore:
    """Create, verify, resolve and activate immutable mapping results."""

    def __init__(self, root: Path):
        self.root = Path(root).resolve()
        self.versions_root = self.root / "versions"
        self.sites_root = self.root / "sites"
        self.staging_root = self.root / ".staging"
        self.versions_root.mkdir(parents=True, exist_ok=True)
        self.sites_root.mkdir(parents=True, exist_ok=True)
        self.staging_root.mkdir(parents=True, exist_ok=True)
        self._verified: Dict[str, Dict[str, Any]] = {}

    def create_version(
        self,
        *,
        site_id: str,
        created_by: str,
        source_session_ids: Sequence[str],
        artifacts: Mapping[str, Path],
        coordinate_contract_hash: str,
        calibration_hash: str,
        purpose: str = "review",
        parent_version_id: Optional[str] = None,
        notes: str = "",
        version_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        site_id = _validate_site_id(site_id)
        if not str(created_by).strip():
            raise MapStoreError("created_by is required")
        if purpose not in PURPOSE_REQUIREMENTS:
            raise MapStoreError("unsupported map purpose: %s" % purpose)
        session_ids = [
            _validate_identifier(value, "source_session_id")
            for value in source_session_ids
        ]
        if not session_ids:
            raise MapStoreError("at least one source session is required")
        if len(session_ids) != len(set(session_ids)):
            raise MapStoreError("source session ids must be unique")
        coordinate_contract_hash = _validate_hash(
            coordinate_contract_hash,
            "coordinate_contract_hash",
        )
        calibration_hash = _validate_hash(calibration_hash, "calibration_hash")

        supplied = set(artifacts)
        unknown = supplied - set(ARTIFACT_SPECS)
        if unknown:
            raise MapStoreError("unknown map artifacts: %s" % sorted(unknown))
        missing = PURPOSE_REQUIREMENTS[purpose] - supplied
        if missing:
            raise MapStoreError("missing %s artifacts: %s" % (purpose, sorted(missing)))

        parent: Optional[Dict[str, Any]] = None
        if parent_version_id is not None:
            parent_version_id = _validate_identifier(parent_version_id, "parent_version_id")
            parent = self.verify(parent_version_id)
            if parent["siteId"] != site_id:
                raise MapStoreError("parent map belongs to another site")
        if purpose == "release":
            if parent is None:
                raise MapStoreError("release map requires a parent review version")
            if parent.get("purpose") != "review":
                raise MapStoreError("release map parent must be a review version")

        if version_id is None:
            version_id = "%s-%s" % (
                time.strftime("%Y%m%dT%H%M%S", time.gmtime()),
                uuid.uuid4().hex[:8],
            )
        version_id = _validate_identifier(version_id, "version_id")
        destination = self.versions_root / version_id
        if destination.exists():
            raise MapStoreError("map version already exists: %s" % version_id)

        staging = Path(tempfile.mkdtemp(prefix=".%s." % version_id, dir=str(self.staging_root)))
        try:
            artifact_index: Dict[str, Dict[str, str]] = {}
            for name in sorted(supplied):
                spec = ARTIFACT_SPECS[name]
                target = staging / spec.relative_path
                _copy_artifact(Path(artifacts[name]), target, spec.kind)
                artifact_index[name] = {
                    "kind": spec.kind,
                    "path": spec.relative_path,
                }

            self._validate_artifact_format(
                staging,
                artifact_index,
                purpose,
                coordinate_contract_hash=coordinate_contract_hash,
                calibration_hash=calibration_hash,
                parent_version=parent,
                parent_root=(
                    self.versions_root / parent_version_id
                    if parent_version_id is not None
                    else None
                ),
            )
            files = self._file_index(staging)
            manifest: Dict[str, Any] = {
                "schema": MAP_SCHEMA,
                "versionId": version_id,
                "siteId": site_id,
                "purpose": purpose,
                "createdAt": time.time(),
                "createdBy": str(created_by).strip(),
                "parentVersionId": parent_version_id,
                "sourceSessionIds": sorted(session_ids),
                "coordinateContractHash": coordinate_contract_hash,
                "calibrationHash": calibration_hash,
                "notes": str(notes),
                "artifacts": artifact_index,
                "files": files,
                "fileSetHash": _sha256_bytes(_canonical_json({"files": files})),
            }
            _write_json_atomic(staging / "manifest.json", manifest)
            os.rename(str(staging), str(destination))
            try:
                self._make_read_only(destination)
            except Exception:
                self._make_writable(destination)
                shutil.rmtree(str(destination))
                raise
            return self.verify(version_id)
        except Exception:
            if staging.exists():
                shutil.rmtree(str(staging))
            raise

    def verify(self, version_id: str) -> Dict[str, Any]:
        version_id = _validate_identifier(version_id, "version_id")
        directory = self.versions_root / version_id
        manifest_path = directory / "manifest.json"
        if not manifest_path.is_file() or manifest_path.is_symlink():
            raise MapStoreError("map manifest does not exist: %s" % version_id)
        cached = self._verified.get(version_id)
        if cached is not None and self._cache_matches(directory, manifest_path, cached):
            return dict(cached["manifest"])
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise MapStoreError("invalid map manifest: %s" % exc)
        if not isinstance(manifest, dict) or manifest.get("schema") != MAP_SCHEMA:
            raise MapStoreError("unsupported map manifest schema")
        if manifest.get("versionId") != version_id:
            raise MapStoreError("map manifest version id mismatch")
        _validate_site_id(str(manifest.get("siteId", "")))
        purpose = manifest.get("purpose")
        if purpose not in PURPOSE_REQUIREMENTS:
            raise MapStoreError("unsupported map purpose in manifest")
        if not SHA256.fullmatch(str(manifest.get("coordinateContractHash", ""))):
            raise MapStoreError("invalid coordinate contract hash in manifest")
        if not SHA256.fullmatch(str(manifest.get("calibrationHash", ""))):
            raise MapStoreError("invalid calibration hash in manifest")
        artifacts = manifest.get("artifacts")
        if not isinstance(artifacts, dict):
            raise MapStoreError("invalid artifact index in manifest")
        unknown_artifacts = set(artifacts) - set(ARTIFACT_SPECS)
        missing_artifacts = PURPOSE_REQUIREMENTS[purpose] - set(artifacts)
        if unknown_artifacts or missing_artifacts:
            raise MapStoreError("artifact index does not match map purpose")
        for name, record in artifacts.items():
            spec = ARTIFACT_SPECS[name]
            if record != {"kind": spec.kind, "path": spec.relative_path}:
                raise MapStoreError("unsafe artifact location in manifest: %s" % name)

        actual_files = self._file_index(directory)
        expected_files = manifest.get("files")
        if actual_files != expected_files:
            raise MapStoreError("map version file integrity check failed: %s" % version_id)
        expected_set_hash = _sha256_bytes(_canonical_json({"files": actual_files}))
        if manifest.get("fileSetHash") != expected_set_hash:
            raise MapStoreError("map version file-set hash mismatch: %s" % version_id)

        result = dict(manifest)
        result["manifestHash"] = _sha256_file(manifest_path)
        self._verified[version_id] = {
            "manifest": dict(result),
            "manifestStat": self._stat_fingerprint(manifest_path),
            "fileStats": {
                record["path"]: self._stat_fingerprint(directory / record["path"])
                for record in actual_files
            },
        }
        return result

    def install_release_version(
        self,
        source_directory: Path,
        *,
        expected_site_id: str,
        expected_version_id: str,
        expected_manifest_hash: str,
        expected_robot_id: str,
        expected_sensor_id: str,
    ) -> Dict[str, Any]:
        """Verify and atomically import one already-sealed runtime release.

        The robot intentionally receives only the slim release directory, not
        its large parent review map.  The release manifest and review audit
        retain the exact parent identity.  Nothing becomes addressable under
        ``versions/`` until every received byte has passed verification.
        """

        expected_site_id = _validate_site_id(expected_site_id)
        expected_version_id = _validate_identifier(
            expected_version_id, "expected_version_id"
        )
        expected_manifest_hash = _validate_hash(
            expected_manifest_hash, "expected_manifest_hash"
        )
        expected_robot_id = str(expected_robot_id).strip()
        expected_sensor_id = str(expected_sensor_id).strip()
        if not expected_robot_id or not expected_sensor_id:
            raise MapStoreError("robot and sensor identity are required for release import")

        def assert_hardware(version_root: Path, manifest: Mapping[str, Any]) -> None:
            try:
                calibration_record = manifest["artifacts"]["calibration_bundle"]
                calibration_payload = json.loads(
                    (version_root / calibration_record["path"]).read_text(
                        encoding="utf-8"
                    )
                )
                calibration = ReleaseCalibrationBundle.from_dict(calibration_payload)
            except (KeyError, OSError, ValueError, ContractError) as exc:
                raise MapStoreError(
                    "received release calibration cannot be read: %s" % exc
                )
            if (
                calibration.robot_id != expected_robot_id
                or calibration.sensor_id != expected_sensor_id
            ):
                raise MapStoreError(
                    "received release belongs to different robot/sensor hardware"
                )
        source_directory = Path(source_directory)
        _reject_unsafe_source(source_directory)
        source_directory = source_directory.resolve()
        manifest_path = source_directory / "manifest.json"
        if not manifest_path.is_file() or manifest_path.is_symlink():
            raise MapStoreError("received release has no regular manifest.json")
        if manifest_path.stat().st_size > 10 * 1024 * 1024:
            raise MapStoreError("received release manifest is too large")
        try:
            announced = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise MapStoreError("received release manifest is invalid: %s" % exc)
        if not isinstance(announced, dict):
            raise MapStoreError("received release manifest must be an object")
        if announced.get("versionId") != expected_version_id:
            raise MapStoreError("received release version id does not match request")

        destination = self.versions_root / expected_version_id
        if destination.exists():
            existing = self.verify(expected_version_id)
            if (
                existing.get("siteId") != expected_site_id
                or existing.get("purpose") != "release"
                or existing.get("manifestHash") != expected_manifest_hash
            ):
                raise MapStoreError(
                    "an immutable version id already belongs to different bytes"
                )
            assert_hardware(destination, existing)
            return existing

        container = Path(
            tempfile.mkdtemp(
                prefix=".import-%s." % expected_version_id,
                dir=str(self.staging_root),
            )
        )
        installed_destination = False
        try:
            isolated_store = MapVersionStore(container)
            staged = isolated_store.versions_root / expected_version_id
            shutil.copytree(str(source_directory), str(staged))
            received = isolated_store.verify(expected_version_id)
            if received.get("purpose") != "release":
                raise MapStoreError("robot can import only a release map")
            if received.get("siteId") != expected_site_id:
                raise MapStoreError("received release belongs to another site")
            if received.get("manifestHash") != expected_manifest_hash:
                raise MapStoreError("received release manifest hash does not match request")
            assert_hardware(staged, received)
            # A received source is normally already mode 0555/0444.  Make the
            # isolated tree movable, then restore the sealed modes immediately
            # after the atomic rename.  It is not referenced by an active
            # pointer at any point in this operation.
            self._make_writable(staged)
            if destination.exists():
                raise MapStoreError("release destination appeared during import")
            os.rename(str(staged), str(destination))
            installed_destination = True
            self._make_read_only(destination)
            self._verified.pop(expected_version_id, None)
            return self.verify(expected_version_id)
        except Exception:
            if installed_destination and destination.exists():
                try:
                    self._make_writable(destination)
                    shutil.rmtree(str(destination))
                except OSError:
                    pass
            raise
        finally:
            if container.exists():
                self._make_writable(container)
                shutil.rmtree(str(container))

    def activate(
        self,
        *,
        site_id: str,
        version_id: str,
        approved_by: str,
        gates: Mapping[str, bool],
    ) -> Dict[str, Any]:
        site_id = _validate_site_id(site_id)
        version = self.verify(version_id)
        if version["siteId"] != site_id:
            raise MapStoreError("map version belongs to another site")
        if version["purpose"] != "release":
            raise MapStoreError("only a release map can be activated")
        if not str(approved_by).strip():
            raise MapStoreError("approved_by is required")
        missing = [name for name in ACTIVATION_GATES if gates.get(name) is not True]
        if missing:
            raise MapStoreError("map activation gates are not satisfied: %s" % missing)
        unknown = set(gates) - set(ACTIVATION_GATES)
        if unknown:
            raise MapStoreError("unknown map activation gates: %s" % sorted(unknown))

        activation = {
            "schema": ACTIVATION_SCHEMA,
            "siteId": site_id,
            "versionId": version_id,
            "manifestHash": version["manifestHash"],
            "activatedAt": time.time(),
            "approvedBy": str(approved_by).strip(),
            "gates": {name: True for name in ACTIVATION_GATES},
        }
        site_root = self.sites_root / site_id
        site_root.mkdir(parents=True, exist_ok=True)
        _write_json_atomic(site_root / "active.json", activation)
        audit_path = site_root / "activation_audit.jsonl"
        with audit_path.open("a", encoding="utf-8") as handle:
            handle.write(_canonical_json(activation).decode("utf-8"))
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        return dict(activation)

    def rollback_activation(
        self,
        *,
        site_id: str,
        failed_activation: Mapping[str, Any],
        previous_activation: Optional[Mapping[str, Any]],
        rolled_back_by: str,
        reason: str,
    ) -> Dict[str, Any]:
        """Restore the exact previous active pointer after runtime rejection.

        The current pointer must still equal the activation being rolled back;
        this prevents a slow installer from overwriting a newer concurrent
        operator decision.  Version directories are immutable and remain
        installed, so rollback changes only the small mutable site pointer.
        """
        site_id = _validate_site_id(site_id)
        actor = str(rolled_back_by).strip()
        reason = str(reason).strip()
        if not actor or not reason:
            raise MapStoreError("rollback actor and reason are required")
        expected_failed = dict(failed_activation)
        current = self.active(site_id)
        if current != expected_failed:
            raise MapStoreError(
                "active map changed concurrently; refusing stale rollback"
            )

        restored = None
        if previous_activation is not None:
            restored = dict(previous_activation)
            if (
                restored.get("schema") != ACTIVATION_SCHEMA
                or restored.get("siteId") != site_id
                or not str(restored.get("approvedBy", "")).strip()
                or any(restored.get("gates", {}).get(name) is not True for name in ACTIVATION_GATES)
            ):
                raise MapStoreError("previous active map pointer is invalid")
            previous_version = self.verify(str(restored.get("versionId", "")))
            if (
                previous_version.get("siteId") != site_id
                or previous_version.get("purpose") != "release"
                or restored.get("manifestHash") != previous_version.get("manifestHash")
            ):
                raise MapStoreError("previous active map no longer verifies")

        site_root = self.sites_root / site_id
        pointer = site_root / "active.json"
        if restored is None:
            pointer.unlink()
        else:
            _write_json_atomic(pointer, restored)
        record = {
            "schema": "go2.map_activation_rollback.v1",
            "siteId": site_id,
            "rolledBackAt": time.time(),
            "rolledBackBy": actor,
            "reason": reason,
            "failedActivation": expected_failed,
            "restoredActivation": restored,
        }
        audit_path = site_root / "activation_rollback_audit.jsonl"
        with audit_path.open("a", encoding="utf-8") as handle:
            handle.write(_canonical_json(record).decode("utf-8"))
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        return record

    def active(self, site_id: str) -> Optional[Dict[str, Any]]:
        site_id = _validate_site_id(site_id)
        path = self.sites_root / site_id / "active.json"
        if not path.is_file():
            return None
        if path.is_symlink():
            raise MapStoreError("active map pointer cannot be a symlink")
        try:
            activation = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise MapStoreError("invalid active map pointer: %s" % exc)
        if activation.get("schema") != ACTIVATION_SCHEMA:
            raise MapStoreError("unsupported active map pointer schema")
        if activation.get("siteId") != site_id:
            raise MapStoreError("active map pointer site mismatch")
        version = self.verify(str(activation.get("versionId", "")))
        if version["siteId"] != site_id:
            raise MapStoreError("active map belongs to another site")
        if activation.get("manifestHash") != version["manifestHash"]:
            raise MapStoreError("active map manifest hash no longer matches")
        return activation

    def list_versions(self, site_id: str) -> List[Dict[str, Any]]:
        site_id = _validate_site_id(site_id)
        versions = []
        for directory in sorted(self.versions_root.iterdir()):
            if not directory.is_dir() or directory.name.startswith("."):
                continue
            manifest = self.verify(directory.name)
            if manifest["siteId"] == site_id:
                versions.append(manifest)
        return sorted(versions, key=lambda item: item["createdAt"], reverse=True)

    def resolve_artifact(
        self,
        version_id: str,
        artifact_name: str,
        relative_path: str = "",
    ) -> Path:
        version_id = _validate_identifier(version_id, "version_id")
        manifest = self._manifest_for_serving(version_id)
        artifacts = manifest.get("artifacts", {})
        if artifact_name not in artifacts:
            raise MapStoreError("map artifact does not exist: %s" % artifact_name)
        record = artifacts[artifact_name]
        base = (self.versions_root / version_id / record["path"]).resolve()
        if record["kind"] == "file":
            if relative_path not in {"", base.name}:
                raise MapStoreError("file artifact has no child paths")
            candidate = base
        else:
            relative = Path(relative_path or ".")
            if relative.is_absolute() or ".." in relative.parts:
                raise MapStoreError("unsafe artifact path")
            candidate = (base / relative).resolve()
            try:
                candidate.relative_to(base)
            except ValueError:
                raise MapStoreError("unsafe artifact path")
        if candidate.is_symlink() or not candidate.is_file():
            raise MapStoreError("map artifact file does not exist")
        cache = self._verified.get(version_id)
        version_root = self.versions_root / version_id
        relative = candidate.relative_to(version_root).as_posix()
        if cache is None or cache["fileStats"].get(relative) != self._stat_fingerprint(candidate):
            # A changed stat invalidates the trusted immutable version. A full
            # hash verification makes the failure explicit before any bytes
            # are served to the browser or localization process.
            self._verified.pop(version_id, None)
            self.verify(version_id)
        return candidate

    def artifact_path(self, version_id: str, artifact_name: str) -> Path:
        """Return one fully verified sealed artifact root for trusted services."""
        version_id = _validate_identifier(version_id, "version_id")
        manifest = self.verify(version_id)
        artifacts = manifest.get("artifacts", {})
        if artifact_name not in artifacts:
            raise MapStoreError("map artifact does not exist: %s" % artifact_name)
        record = artifacts[artifact_name]
        candidate = self.versions_root / version_id / record["path"]
        if candidate.is_symlink():
            raise MapStoreError("sealed artifact root cannot be a symlink")
        if record["kind"] == "file" and not candidate.is_file():
            raise MapStoreError("sealed file artifact is unavailable")
        if record["kind"] == "directory" and not candidate.is_dir():
            raise MapStoreError("sealed directory artifact is unavailable")
        return candidate

    @staticmethod
    def _validate_artifact_format(
        root: Path,
        artifacts: Mapping[str, Mapping[str, str]],
        purpose: str,
        coordinate_contract_hash: str,
        calibration_hash: str,
        parent_version: Optional[Mapping[str, Any]] = None,
        parent_root: Optional[Path] = None,
    ) -> None:
        if purpose == "release":
            if parent_version is None or parent_root is None:
                raise MapStoreError("release artifacts require a verified parent review map")
        if "glim_dump" in artifacts:
            glim = root / artifacts["glim_dump"]["path"]
            required = ("traj_imu.txt", "traj_lidar.txt")
            missing = [name for name in required if not (glim / name).is_file()]
            if missing:
                raise MapStoreError("GLIM dump is incomplete: %s" % missing)
        if "map_cloud" in artifacts:
            cloud = root / artifacts["map_cloud"]["path"]
            if cloud.suffix.lower() != ".ply" or cloud.stat().st_size == 0:
                raise MapStoreError("map_cloud must be a non-empty PLY file")
        if "map_quality" in artifacts:
            quality = root / artifacts["map_quality"]["path"]
            result_path = quality / "result.json"
            metrics_path = quality / "metrics.json"
            report_path = quality / "report.html"
            manifest_path = quality / "manifest.json"
            try:
                result = json.loads(result_path.read_text(encoding="utf-8"))
                metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid map quality bundle: %s" % exc)
            if (
                not report_path.is_file()
                or report_path.is_symlink()
                or not isinstance(result, dict)
                or result.get("schema") != "go2.map_geometry_quality.v1"
                or result.get("geometryEligible") is not True
                or result.get("state") not in {"passed", "warning"}
                or result.get("metricsSha256") != _sha256_file(metrics_path)
                or not isinstance(metrics, dict)
                or metrics.get("schema") != "go2.map_geometry_metrics.v1"
                or not isinstance(manifest, dict)
                or manifest.get("schema") != "go2.map_quality_bundle.v1"
            ):
                raise MapStoreError("map quality bundle did not pass its geometry gate")
            if "map_cloud" not in artifacts:
                raise MapStoreError("map quality bundle has no source map cloud")
            cloud = root / artifacts["map_cloud"]["path"]
            if metrics.get("sourcePlySha256") != _sha256_file(cloud):
                raise MapStoreError("map quality bundle belongs to a different map cloud")
        if "potree" in artifacts:
            potree = root / artifacts["potree"]["path"]
            required = ("metadata.json", "hierarchy.bin", "octree.bin")
            missing = [name for name in required if not (potree / name).is_file()]
            if missing:
                raise MapStoreError("Potree 2.x artifact is incomplete: %s" % missing)
            try:
                metadata = json.loads((potree / "metadata.json").read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid Potree metadata.json: %s" % exc)
            if not isinstance(metadata, dict) or "boundingBox" not in metadata:
                raise MapStoreError("Potree metadata.json has no boundingBox")
        if "site_overview" in artifacts:
            overview = root / artifacts["site_overview"]["path"]
            metadata_path = overview / "metadata.json"
            try:
                overview_metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid site overview metadata: %s" % exc)
            if (
                not isinstance(overview_metadata, dict)
                or overview_metadata.get("schema") != "go2.site_overview.v1"
                or overview_metadata.get("frame") != "map"
            ):
                raise MapStoreError("site overview must be go2.site_overview.v1 in map")
            try:
                resolution = float(overview_metadata["resolutionM"])
                width_value = float(overview_metadata["widthPx"])
                height_value = float(overview_metadata["heightPx"])
                width = int(width_value)
                height = int(height_value)
                origin = [float(value) for value in overview_metadata["origin"]]
                affine = [float(value) for value in overview_metadata["imageToMapAffine"]]
                bounds = overview_metadata["bounds"]
                bounds_min = [float(value) for value in bounds["min"]]
                bounds_max = [float(value) for value in bounds["max"]]
            except (KeyError, TypeError, ValueError):
                raise MapStoreError("site overview geometry metadata is invalid")
            if (
                not 0.02 <= resolution <= 0.50
                or width < 1
                or height < 1
                or width_value != width
                or height_value != height
                or width * height > 100_000_000
                or len(origin) != 2
                or len(affine) != 6
                or len(bounds_min) != 2
                or len(bounds_max) != 2
                or not all(
                    math.isfinite(value)
                    for value in origin + affine + bounds_min + bounds_max
                )
                or abs(affine[0] - origin[0]) > 1.0e-9
                or abs(affine[1] - resolution) > 1.0e-9
                or abs(affine[2]) > 1.0e-9
                or abs(affine[3] - (origin[1] + height * resolution)) > 1.0e-9
                or abs(affine[4]) > 1.0e-9
                or abs(affine[5] + resolution) > 1.0e-9
                or abs(bounds_min[0] - origin[0]) > 1.0e-9
                or abs(bounds_min[1] - origin[1]) > 1.0e-9
                or abs(bounds_max[0] - (origin[0] + width * resolution)) > 1.0e-9
                or abs(bounds_max[1] - (origin[1] + height * resolution)) > 1.0e-9
            ):
                raise MapStoreError("site overview affine does not match its raster geometry")
            overview_files = overview_metadata.get("files")
            expected_paths = {
                "pdalPipeline": "pipeline.json",
                "densityGeoTiff": "density.tif",
                "densityPng": "density.png",
            }
            if not isinstance(overview_files, dict):
                raise MapStoreError("site overview has no file index")
            for key, expected_path in expected_paths.items():
                record = overview_files.get(key)
                candidate = overview / expected_path
                if (
                    not candidate.is_file()
                    or candidate.is_symlink()
                    or not isinstance(record, dict)
                    or record.get("path") != expected_path
                    or record.get("sha256") != _sha256_file(candidate)
                ):
                    raise MapStoreError("site overview file hash mismatch: %s" % key)
            png = overview / "density.png"
            with png.open("rb") as handle:
                header = handle.read(24)
            if (
                len(header) != 24
                or header[:8] != b"\x89PNG\r\n\x1a\n"
                or header[12:16] != b"IHDR"
                or struct.unpack(">II", header[16:24]) != (width, height)
            ):
                raise MapStoreError("site overview PNG dimensions do not match metadata")
            if "map_cloud" not in artifacts:
                raise MapStoreError("site overview has no source map cloud")
            cloud = root / artifacts["map_cloud"]["path"]
            if overview_metadata.get("sourceMapCloudSha256") != _sha256_file(cloud):
                raise MapStoreError("site overview belongs to a different map cloud")
        if "stability_evidence" in artifacts:
            evidence_path = root / artifacts["stability_evidence"]["path"]
            try:
                evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid stability evidence: %s" % exc)
            if (
                not isinstance(evidence, dict)
                or evidence.get("schema") not in {
                    "go2.stability_evidence.v1",
                    "go2.dynamic_map_evidence.v2",
                }
                or not isinstance(evidence.get("sessionIds"), list)
                or len(evidence["sessionIds"]) < 2
            ):
                raise MapStoreError("stability evidence must cover at least two acquisitions")
        if "reference_route" in artifacts:
            reference_path = root / artifacts["reference_route"]["path"]
            try:
                reference = json.loads(reference_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid map reference route: %s" % exc)
            if (
                not isinstance(reference, dict)
                or reference.get("schema") != "go2.map_reference_route.v1"
                or reference.get("frame") != "map"
                or reference.get("source") != "field_trace"
                or not isinstance(reference.get("waypoints"), list)
                or len(reference["waypoints"]) < 2
                or not isinstance(reference.get("registration"), dict)
                or not isinstance(reference.get("provenance"), dict)
            ):
                raise MapStoreError("map reference route contract is invalid")
            previous_timestamp = None
            for index, waypoint in enumerate(reference["waypoints"]):
                try:
                    values = [float(waypoint[name]) for name in ("x", "y", "yaw")]
                    timestamp = float(waypoint["sourceTimestamp"])
                except (KeyError, TypeError, ValueError):
                    raise MapStoreError(
                        "map reference waypoint %d is invalid" % index
                    )
                if not all(math.isfinite(value) for value in values + [timestamp]):
                    raise MapStoreError(
                        "map reference waypoint %d is non-finite" % index
                    )
                if previous_timestamp is not None and timestamp <= previous_timestamp:
                    raise MapStoreError("map reference timestamps are not increasing")
                previous_timestamp = timestamp
            for name in (
                "referenceTraceSha256",
                "captureManifestSha256",
                "glimTrajectorySha256",
                "mountCalibrationSha256",
            ):
                _validate_hash(
                    str(reference["provenance"].get(name, "")),
                    "map reference %s" % name,
                )
            supplied_reference_hash = _validate_hash(
                str(reference.get("artifactSha256", "")),
                "map reference artifactSha256",
            )
            unhashed_reference = dict(reference)
            unhashed_reference.pop("artifactSha256", None)
            if supplied_reference_hash != _sha256_bytes(
                _canonical_json(unhashed_reference)
            ):
                raise MapStoreError("map reference route hash does not match contents")
        if "capture_manifests" in artifacts:
            capture_root = root / artifacts["capture_manifests"]["path"]
            capture_files = sorted(capture_root.glob("*.json"))
            if not capture_files:
                raise MapStoreError("capture manifest artifact is empty")
            for capture_path in capture_files:
                if capture_path.is_symlink() or not capture_path.is_file():
                    raise MapStoreError("capture manifest must be a regular JSON file")
                try:
                    capture = json.loads(capture_path.read_text(encoding="utf-8"))
                except (OSError, ValueError) as exc:
                    raise MapStoreError("invalid capture manifest: %s" % exc)
                if (
                    not isinstance(capture, dict)
                    or capture.get("schema") != "go2.mapping_capture_manifest.v2"
                    or capture.get("captureClass") != "production_field_mapping"
                    or capture.get("productionMapEligible") is not True
                    or not isinstance(capture.get("calibrationBinding"), dict)
                    or capture.get("captureAuthority", {}).get("kind")
                    != "robot_local_persistent"
                    or capture.get("captureAuthority", {}).get("productionEligible")
                    is not True
                ):
                    raise MapStoreError("map contains a non-production capture manifest")
                supplied_hash = _validate_hash(
                    str(capture.get("manifestSha256", "")),
                    "capture manifest hash",
                )
                unhashed_capture = dict(capture)
                unhashed_capture.pop("manifestSha256", None)
                if supplied_hash != _sha256_bytes(_canonical_json(unhashed_capture)):
                    raise MapStoreError("capture manifest hash does not match contents")
                evidence = capture.get("sessionEvidence")
                bag = capture.get("bag")
                route_contract = capture.get("referenceRoute")
                binding = capture.get("calibrationBinding")
                if (
                    not isinstance(evidence, dict)
                    or capture.get("sessionEvidenceSha256")
                    != _sha256_bytes(_canonical_json(evidence))
                    or not isinstance(bag, dict)
                    or int(
                        bag.get("messageCounts", {}).get(
                            "/mapping/acquisition_pass", 0
                        )
                    )
                    < 1
                    or not isinstance(route_contract, dict)
                    or route_contract.get("present")
                    is not isinstance(evidence.get("referenceRoute"), dict)
                    or route_contract.get("required")
                    is not route_contract.get("present")
                    or not isinstance(binding, dict)
                ):
                    raise MapStoreError("capture manifest evidence contract is invalid")
                for name in (
                    "coordinateContractSha256",
                    "mountCalibrationSha256",
                    "sensorInternalCalibrationSha256",
                ):
                    _validate_hash(
                        str(binding.get(name, "")),
                        "capture calibration %s" % name,
                    )
                for name in (
                    "robotSealSha256",
                    "robotDeclaredSealSha256",
                    "robotFileSetHash",
                    "diagnosticsSha256",
                ):
                    _validate_hash(
                        str(capture["captureAuthority"].get(name, "")),
                        "capture authority %s" % name,
                    )
        if "review_tasks" in artifacts:
            tasks_path = root / artifacts["review_tasks"]["path"]
            try:
                tasks = json.loads(tasks_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid review task artifact: %s" % exc)
            if (
                not isinstance(tasks, dict)
                or tasks.get("schema") != "go2.review_queue.v1"
                or not isinstance(tasks.get("summary"), dict)
                or not isinstance(tasks.get("tasks"), list)
            ):
                raise MapStoreError("review task artifact does not match go2.review_queue.v1")
            potree_root = (
                root / artifacts["potree"]["path"]
                if "potree" in artifacts
                else None
            )
            seen_task_ids = set()
            for task in tasks["tasks"]:
                if not isinstance(task, dict):
                    raise MapStoreError("review task list contains a non-object")
                task_id = str(task.get("taskId", ""))
                if not IDENTIFIER.fullmatch(task_id) or task_id in seen_task_ids:
                    raise MapStoreError("review task id is invalid or duplicated")
                seen_task_ids.add(task_id)
                if task.get("kind") != "object_persistence":
                    continue
                evidence = task.get("evidence")
                rows = evidence.get("acquisitionComparison") if isinstance(evidence, dict) else None
                visual = evidence.get("visualComparison") if isinstance(evidence, dict) else None
                if not isinstance(rows, list) or not isinstance(visual, dict):
                    raise MapStoreError("object review task has no visual comparison evidence")
                state_by_acquisition = {
                    str(row.get("acquisitionId", "")): str(row.get("state", ""))
                    for row in rows
                    if isinstance(row, dict)
                }
                if (
                    visual.get("schema") != "go2.review_visual_comparison.v1"
                    or visual.get("frame") != "map"
                    or visual.get("cameraContract") != "same_task_bounds_v1"
                    or potree_root is None
                ):
                    raise MapStoreError("object review visual comparison contract is invalid")
                for expected_state in ("present", "clear"):
                    source = visual.get(expected_state)
                    if not isinstance(source, dict):
                        raise MapStoreError("object review visual comparison is incomplete")
                    acquisition_id = str(source.get("acquisitionId", ""))
                    expected_relative = "comparisons/%s/metadata.json" % acquisition_id
                    if (
                        not IDENTIFIER.fullmatch(acquisition_id)
                        or state_by_acquisition.get(acquisition_id) != expected_state
                        or source.get("artifact") != "potree"
                        or source.get("metadataPath") != expected_relative
                    ):
                        raise MapStoreError(
                            "object review visual comparison does not match acquisition evidence"
                        )
                    metadata_path = potree_root / expected_relative
                    if metadata_path.is_symlink() or not metadata_path.is_file():
                        raise MapStoreError("object review comparison metadata is unavailable")
                    if source.get("metadataSha256") != _sha256_file(metadata_path):
                        raise MapStoreError("object review comparison metadata hash mismatch")
                    provenance_path = metadata_path.parent / "provenance.json"
                    if provenance_path.is_symlink() or not provenance_path.is_file():
                        raise MapStoreError("object review comparison provenance is unavailable")
                    if (
                        source.get("provenancePath")
                        != "comparisons/%s/provenance.json" % acquisition_id
                        or source.get("provenanceSha256")
                        != _sha256_file(provenance_path)
                    ):
                        raise MapStoreError("object review comparison provenance hash mismatch")
                    try:
                        provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
                    except (OSError, ValueError) as exc:
                        raise MapStoreError(
                            "invalid object review comparison provenance: %s" % exc
                        )
                    if (
                        not isinstance(provenance, dict)
                        or provenance.get("schema")
                        != "go2.review_comparison_provenance.v1"
                        or provenance.get("acquisitionId") != acquisition_id
                        or provenance.get("frame") != "map"
                        or provenance.get("sourcePlySha256")
                        != source.get("sourcePlySha256")
                        or provenance.get("metadataSha256")
                        != source.get("metadataSha256")
                        or provenance.get("sourcePointCount")
                        != source.get("pointCount")
                    ):
                        raise MapStoreError(
                            "object review comparison provenance does not match the task"
                        )
                    for name in ("hierarchy.bin", "octree.bin"):
                        candidate = metadata_path.parent / name
                        if candidate.is_symlink() or not candidate.is_file():
                            raise MapStoreError(
                                "object review Potree comparison is incomplete: %s" % name
                            )
                readiness = task.get("decisionReadiness")
                if not isinstance(readiness, dict) or readiness.get("ready") is not True:
                    raise MapStoreError("published object review task is not decision-ready")
        audit_payload = None
        if "review_audit" in artifacts:
            audit_path = root / artifacts["review_audit"]["path"]
            try:
                audit_payload = json.loads(audit_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid review audit artifact: %s" % exc)
            if (
                not isinstance(audit_payload, dict)
                or audit_payload.get("schema") != "go2.review_audit.v1"
                or not isinstance(audit_payload.get("summary"), dict)
                or not isinstance(audit_payload.get("decisions"), list)
            ):
                raise MapStoreError("review audit artifact does not match go2.review_audit.v1")
            summary = audit_payload["summary"]
            if summary.get("pending") != 0:
                raise MapStoreError("review audit still contains unanswered tasks")
            try:
                total = int(summary.get("total", -1))
                decided_count = int(summary.get("decided", -1))
                deferred_count = int(summary.get("deferred", -1))
            except (TypeError, ValueError):
                raise MapStoreError("review audit summary contains invalid counts")
            if min(total, decided_count, deferred_count) < 0:
                raise MapStoreError("review audit summary contains negative counts")
            if decided_count + deferred_count != total:
                raise MapStoreError("review audit summary counts do not add up")
            if purpose == "release" and deferred_count:
                raise MapStoreError(
                    "review audit contains deferred facts; collect more evidence before release"
                )
            if len(audit_payload["decisions"]) != int(summary.get("total", -1)):
                raise MapStoreError("review audit decision count does not match summary")
            state_counts = {"decided": 0, "deferred": 0}
            decision_ids = set()
            for record in audit_payload["decisions"]:
                if (
                    not isinstance(record, dict)
                    or record.get("state") not in {"decided", "deferred"}
                    or not str(record.get("taskId", "")).strip()
                    or not str(record.get("decision", "")).strip()
                    or not str(record.get("decidedBy", "")).strip()
                ):
                    raise MapStoreError("review audit contains an incomplete decision")
                task_id = str(record["taskId"])
                kind = str(record.get("kind", ""))
                decision = str(record["decision"])
                if task_id in decision_ids:
                    raise MapStoreError("review audit contains duplicate task ids")
                decision_ids.add(task_id)
                if kind not in REVIEW_DECISIONS or decision not in REVIEW_DECISIONS[kind]:
                    raise MapStoreError("review audit contains an invalid field-fact decision")
                state_counts[str(record["state"])] += 1
            if state_counts != {
                "decided": decided_count,
                "deferred": deferred_count,
            }:
                raise MapStoreError("review audit summary does not match decision states")
            supplied_audit_hash = _validate_hash(
                str(audit_payload.get("auditHash", "")), "review auditHash"
            )
            unhashed = dict(audit_payload)
            unhashed.pop("auditHash", None)
            actual_audit_hash = _sha256_bytes(_canonical_json(unhashed))
            if supplied_audit_hash != actual_audit_hash:
                raise MapStoreError("review auditHash does not match its contents")
            if parent_version is not None:
                if (
                    audit_payload.get("siteId") != parent_version.get("siteId")
                    or audit_payload.get("sourceReviewVersionId")
                    != parent_version.get("versionId")
                    or audit_payload.get("sourceReviewManifestHash")
                    != parent_version.get("manifestHash")
                ):
                    raise MapStoreError("review audit is not bound to the parent review map")
                try:
                    parent_tasks = json.loads(
                        (
                            Path(parent_root)
                            / parent_version["artifacts"]["review_tasks"]["path"]
                        ).read_text(encoding="utf-8")
                    )
                except (KeyError, OSError, ValueError) as exc:
                    raise MapStoreError("parent review tasks cannot be verified: %s" % exc)
                expected_tasks = {
                    str(task.get("taskId", "")): str(task.get("kind", ""))
                    for task in parent_tasks.get("tasks", [])
                    if isinstance(task, dict)
                }
                actual_tasks = {
                    str(record["taskId"]): str(record["kind"])
                    for record in audit_payload["decisions"]
                }
                if (
                    not all(expected_tasks)
                    or expected_tasks != actual_tasks
                ):
                    raise MapStoreError(
                        "review audit decisions do not exactly match parent review tasks"
                    )
        if purpose == "release":
            calibration_path = root / artifacts["calibration_bundle"]["path"]
            try:
                calibration_payload = json.loads(
                    calibration_path.read_text(encoding="utf-8")
                )
                calibration_bundle = ReleaseCalibrationBundle.from_dict(
                    calibration_payload
                )
            except (OSError, ValueError, ContractError) as exc:
                raise MapStoreError("invalid release calibration bundle: %s" % exc)
            if calibration_bundle.digest != calibration_hash:
                raise MapStoreError(
                    "release calibration bundle does not match manifest hash"
                )
            if calibration_bundle.coordinate_contract.digest != coordinate_contract_hash:
                raise MapStoreError(
                    "release calibration bundle uses a different coordinate contract"
                )
            parent_capture_record = parent_version.get("artifacts", {}).get(
                "capture_manifests"
            )
            if isinstance(parent_capture_record, dict):
                parent_capture_root = Path(parent_root) / parent_capture_record["path"]
                for capture_path in sorted(parent_capture_root.glob("*.json")):
                    try:
                        capture = json.loads(capture_path.read_text(encoding="utf-8"))
                        binding = capture["calibrationBinding"]
                    except (KeyError, OSError, TypeError, ValueError) as exc:
                        raise MapStoreError(
                            "parent capture calibration cannot be verified: %s" % exc
                        )
                    if (
                        binding.get("robotId") != calibration_bundle.robot_id
                        or binding.get("sensorId") != calibration_bundle.sensor_id
                        or binding.get("coordinateContractSha256")
                        != calibration_bundle.coordinate_contract.digest
                        or binding.get("mountCalibrationSha256")
                        != calibration_bundle.mount_calibration.digest
                        or binding.get("sensorInternalCalibrationSha256")
                        != calibration_bundle.sensor_internal_calibration.digest
                    ):
                        raise MapStoreError(
                            "release calibration differs from parent capture"
                        )
            if "reference_route" in parent_version.get("artifacts", {}):
                try:
                    parent_reference = json.loads(
                        (
                            Path(parent_root)
                            / parent_version["artifacts"]["reference_route"]["path"]
                        ).read_text(encoding="utf-8")
                    )
                    registered_mount_hash = parent_reference["provenance"][
                        "mountCalibrationSha256"
                    ]
                except (KeyError, OSError, TypeError, ValueError) as exc:
                    raise MapStoreError(
                        "parent map reference route cannot be verified: %s" % exc
                    )
                if registered_mount_hash != calibration_bundle.mount_calibration.digest:
                    raise MapStoreError(
                        "release mount calibration differs from parent reference route"
                    )
            route = root / artifacts["route"]["path"]
            try:
                route_payload = json.loads(route.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid route artifact: %s" % exc)
            if (
                not isinstance(route_payload, dict)
                or route_payload.get("schema") != "go2.route.v1"
                or route_payload.get("frame") != "map"
                or not IDENTIFIER.fullmatch(str(route_payload.get("routeId", "")))
                or not isinstance(route_payload.get("waypoints"), list)
                or len(route_payload["waypoints"]) < 2
            ):
                raise MapStoreError(
                    "release route must be go2.route.v1 in map with routeId and waypoints"
                )
            for index, waypoint in enumerate(route_payload["waypoints"]):
                if not isinstance(waypoint, dict):
                    raise MapStoreError("route waypoint %d is not an object" % index)
                try:
                    values = [float(waypoint[name]) for name in ("x", "y", "yaw")]
                except (KeyError, TypeError, ValueError):
                    raise MapStoreError("route waypoint %d must contain numeric x,y,yaw" % index)
                if not all(math.isfinite(value) for value in values):
                    raise MapStoreError("route waypoint %d contains non-finite values" % index)
            localization = root / artifacts["localization_map"]["path"]
            if localization.suffix.lower() != ".pcd" or localization.stat().st_size == 0:
                raise MapStoreError("release localization map must be a non-empty PCD")
            with localization.open("rb") as handle:
                pcd_header = handle.read(4096).decode("ascii", errors="ignore")
            if "FIELDS x y z" not in pcd_header or "DATA " not in pcd_header:
                raise MapStoreError("release localization PCD has no x/y/z data header")

            profile = root / artifacts["runtime_profile"]["path"]
            try:
                profile_payload = json.loads(profile.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid runtime profile: %s" % exc)
            MapVersionStore._validate_runtime_profile(profile_payload)

            report = root / artifacts["validation_report"]["path"]
            try:
                report_payload = json.loads(report.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid map validation report: %s" % exc)
            MapVersionStore._validate_validation_report(report_payload)
            evidence_root = root / artifacts["validation_evidence"]["path"]
            dataset_identity_path = evidence_root / "replay_dataset.json"
            try:
                dataset_identity = json.loads(
                    dataset_identity_path.read_text(encoding="utf-8")
                )
            except (OSError, ValueError) as exc:
                raise MapStoreError("invalid replay dataset evidence: %s" % exc)
            if (
                not isinstance(dataset_identity, dict)
                or dataset_identity.get("schema") != "go2.replay_dataset_identity.v1"
                or not isinstance(dataset_identity.get("files"), list)
                or dataset_identity.get("datasetHash")
                != report_payload["replayDatasetHash"]
            ):
                raise MapStoreError(
                    "sealed replay dataset identity does not match validation report"
                )
            _validate_hash(
                str(dataset_identity.get("datasetHash", "")),
                "replay dataset identity hash",
            )
            dataset_files = dataset_identity["files"]
            normalized_dataset_files = []
            seen_dataset_paths = set()
            for record in dataset_files:
                if not isinstance(record, dict):
                    raise MapStoreError("replay dataset identity contains a non-object file")
                path_text = str(record.get("path", ""))
                relative = Path(path_text)
                if (
                    not path_text
                    or relative.is_absolute()
                    or ".." in relative.parts
                    or path_text in seen_dataset_paths
                ):
                    raise MapStoreError("replay dataset identity contains an unsafe path")
                seen_dataset_paths.add(path_text)
                try:
                    size = int(record["size"])
                except (KeyError, TypeError, ValueError):
                    raise MapStoreError("replay dataset identity contains an invalid size")
                if size < 0:
                    raise MapStoreError("replay dataset identity contains a negative size")
                digest = _validate_hash(
                    str(record.get("sha256", "")),
                    "replay dataset file hash",
                )
                normalized_dataset_files.append(
                    {"path": path_text, "size": size, "sha256": digest}
                )
            actual_dataset_hash = _sha256_bytes(
                _canonical_json({"files": normalized_dataset_files})
            )
            if actual_dataset_hash != dataset_identity["datasetHash"]:
                raise MapStoreError("replay dataset identity hash does not match its file list")
            dataset_file_by_path = {
                record["path"]: record for record in normalized_dataset_files
            }
            if "validation_plan.json" not in dataset_file_by_path:
                raise MapStoreError("replay dataset identity has no validation plan")
            trace_manifest_record = dataset_file_by_path.get("trace_manifest.json")
            if (
                not isinstance(trace_manifest_record, dict)
                or trace_manifest_record.get("sha256")
                != report_payload["validationSuite"]["traceManifestHash"]
            ):
                raise MapStoreError(
                    "replay dataset trace manifest differs from validation report"
                )
            sealed_gate_evidence = {}
            for gate_name in ACTIVATION_GATES:
                evidence_path = evidence_root / (gate_name + ".json")
                try:
                    gate_evidence = json.loads(
                        evidence_path.read_text(encoding="utf-8")
                    )
                except (OSError, ValueError) as exc:
                    raise MapStoreError(
                        "invalid validation evidence for %s: %s" % (gate_name, exc)
                    )
                expected_gate = report_payload["checks"][gate_name]
                if (
                    not isinstance(gate_evidence, dict)
                    or gate_evidence.get("schema") != "go2.validation_evidence.v1"
                    or gate_evidence.get("gate") != gate_name
                    or gate_evidence.get("passed") is not True
                    or gate_evidence.get("toolVersion")
                    != expected_gate["toolVersion"]
                    or gate_evidence.get("replayDatasetHash")
                    != report_payload["replayDatasetHash"]
                ):
                    raise MapStoreError(
                        "sealed validation evidence is incomplete: %s" % gate_name
                    )
                actual_evidence_hash = _sha256_file(evidence_path)
                if actual_evidence_hash != expected_gate["evidenceHash"]:
                    raise MapStoreError(
                        "validation evidence hash does not match report: %s" % gate_name
                    )
                sealed_gate_evidence[gate_name] = gate_evidence
            if audit_payload is None:
                raise MapStoreError("release map has no review audit")
            if (
                sealed_gate_evidence["reviewComplete"].get("reviewAuditHash")
                != audit_payload["auditHash"]
            ):
                raise MapStoreError(
                    "reviewComplete evidence does not match the sealed review audit"
                )
            if (
                sealed_gate_evidence["localizationValidated"].get("localizationMapHash")
                != _sha256_file(localization)
            ):
                raise MapStoreError(
                    "localization validation evidence used a different localization map"
                )
            if (
                sealed_gate_evidence["routeValidated"].get("routeHash")
                != _sha256_bytes(_canonical_json(route_payload))
            ):
                raise MapStoreError("route validation evidence used a different route")
            if (
                sealed_gate_evidence["safetyGatePassed"].get("runtimeProfileHash")
                != _sha256_file(profile)
            ):
                raise MapStoreError(
                    "safety validation evidence used a different runtime profile"
                )

    @staticmethod
    def _validate_runtime_profile(profile: Any) -> None:
        if not isinstance(profile, dict) or profile.get("schema") != "go2.runtime_profile.v1":
            raise MapStoreError("unsupported runtime profile schema")
        expected_frames = {
            "map": "map",
            "odom": "odom",
            "base": "base_link",
        }
        if profile.get("frames") != expected_frames:
            raise MapStoreError("runtime profile violates the map/odom/base frame contract")
        localization = profile.get("localization")
        if not isinstance(localization, dict):
            raise MapStoreError("runtime profile has no localization section")
        if localization.get("mapArtifact") != "localization_map":
            raise MapStoreError("runtime profile must select the sealed localization_map")
        quality_profile = str(localization.get("qualityProfileId", ""))
        _validate_identifier(quality_profile, "quality_profile_id")
        try:
            resolve_localization_quality_profile(quality_profile)
        except ValueError as exc:
            raise MapStoreError(str(exc)) from exc
        zone = localization.get("initializationZone")
        if not isinstance(zone, dict) or zone.get("kind") != "circle":
            raise MapStoreError("runtime profile requires a circular initialization zone")
        center = zone.get("center")
        if not isinstance(center, dict):
            raise MapStoreError("initialization zone has no center")
        try:
            center_values = [float(center[name]) for name in ("x", "y", "z")]
            radius = float(zone["radiusM"])
            expected_yaw = float(zone["expectedYawRad"])
            yaw_tolerance = float(zone["yawToleranceDeg"])
        except (KeyError, TypeError, ValueError):
            raise MapStoreError(
                "initialization zone center/radius/yaw must be numeric"
            )
        if not all(
            math.isfinite(value)
            for value in center_values + [radius, expected_yaw, yaw_tolerance]
        ):
            raise MapStoreError("initialization zone contains non-finite values")
        if radius < 0.5 or radius > 6.0:
            raise MapStoreError("initialization zone radius must be between 0.5m and 6m")
        if yaw_tolerance < 5.0 or yaw_tolerance > 90.0:
            raise MapStoreError(
                "initialization yaw tolerance must be between 5 and 90 degrees"
            )
        if profile.get("routeArtifact") != "route":
            raise MapStoreError("runtime profile must select the sealed route")
        navigation = profile.get("navigation")
        expected_navigation = {
            "controllerProfileId": "go2-nav2-mppi-omni-v1",
            "collisionProfileId": "go2-mid360-collision-v1",
        }
        if navigation != expected_navigation:
            raise MapStoreError(
                "runtime profile must pin the approved Nav2 controller/collision profiles"
            )
        patrol = profile.get("patrol")
        if not isinstance(patrol, dict):
            raise MapStoreError("runtime profile has no patrol section")
        if set(patrol) != {
            "loopMode",
            "speedLimitMps",
            "startMaxDistanceM",
            "startMaxYawDeg",
            "pathSampleSpacingM",
        }:
            raise MapStoreError("runtime patrol settings are incomplete or contain unknown keys")
        if patrol.get("loopMode") != "once":
            raise MapStoreError("only the validated once loop mode is currently supported")
        try:
            speed = float(patrol["speedLimitMps"])
            start_distance = float(patrol["startMaxDistanceM"])
            start_yaw = float(patrol["startMaxYawDeg"])
            spacing = float(patrol["pathSampleSpacingM"])
        except (KeyError, TypeError, ValueError):
            raise MapStoreError("runtime patrol settings must be numeric")
        if not all(math.isfinite(value) for value in (speed, start_distance, start_yaw, spacing)):
            raise MapStoreError("runtime patrol settings contain non-finite values")
        if speed < 0.10 or speed > 0.50:
            raise MapStoreError("patrol speed limit must be between 0.10m/s and 0.50m/s")
        if start_distance < 0.50 or start_distance > 5.0:
            raise MapStoreError("patrol start distance gate must be between 0.5m and 5m")
        if start_yaw < 0.0 or start_yaw > 90.0:
            raise MapStoreError("patrol start yaw gate must be between 0 and 90 degrees")
        if spacing < 0.05 or spacing > 0.50:
            raise MapStoreError("patrol path sample spacing must be between 0.05m and 0.50m")

    @staticmethod
    def _validate_validation_report(report: Any) -> None:
        if not isinstance(report, dict) or report.get("schema") != "go2.map_validation.v1":
            raise MapStoreError("unsupported map validation report schema")
        checks = report.get("checks")
        if not isinstance(checks, dict) or set(checks) != set(ACTIVATION_GATES):
            raise MapStoreError("map validation report must contain every activation gate")
        for name in ACTIVATION_GATES:
            result = checks[name]
            if not isinstance(result, dict) or result.get("passed") is not True:
                raise MapStoreError("map validation did not pass: %s" % name)
            _validate_hash(str(result.get("evidenceHash", "")), "%s evidenceHash" % name)
            if not str(result.get("toolVersion", "")).strip():
                raise MapStoreError("map validation check has no toolVersion: %s" % name)
        _validate_hash(str(report.get("replayDatasetHash", "")), "replayDatasetHash")
        suite = report.get("validationSuite")
        if not isinstance(suite, dict) or set(suite) != {
            "planHash",
            "sealedInputHash",
            "traceManifestHash",
        }:
            raise MapStoreError("map validation report has no sealed suite binding")
        for name in ("planHash", "sealedInputHash", "traceManifestHash"):
            _validate_hash(str(suite.get(name, "")), "validationSuite.%s" % name)
        try:
            completed_at = float(report["completedAt"])
        except (KeyError, TypeError, ValueError):
            raise MapStoreError("map validation report has no numeric completedAt")
        if not math.isfinite(completed_at) or completed_at <= 0.0:
            raise MapStoreError("map validation report completedAt is invalid")

    @staticmethod
    def _file_index(root: Path) -> List[Dict[str, Any]]:
        files = []
        for path in _regular_files(root):
            relative = path.relative_to(root).as_posix()
            if relative == "manifest.json":
                continue
            files.append(
                {
                    "path": relative,
                    "size": path.stat().st_size,
                    "sha256": _sha256_file(path),
                }
            )
        return files

    @staticmethod
    def _make_read_only(root: Path) -> None:
        for path in _regular_files(root):
            path.chmod(0o444)
        for path in sorted((item for item in root.rglob("*") if item.is_dir()), reverse=True):
            path.chmod(0o555)
        root.chmod(0o555)

    @staticmethod
    def _make_writable(root: Path) -> None:
        if not root.exists():
            return
        for path in root.rglob("*"):
            path.chmod(0o755 if path.is_dir() else 0o644)
        root.chmod(0o755)

    @staticmethod
    def _stat_fingerprint(path: Path) -> Tuple[int, ...]:
        record = path.stat()
        return (
            int(record.st_dev),
            int(getattr(record, "st_ino", 0)),
            int(record.st_size),
            int(record.st_mtime_ns),
            int(record.st_ctime_ns),
            int(stat.S_IMODE(record.st_mode)),
        )

    def _cache_matches(
        self,
        directory: Path,
        manifest_path: Path,
        cached: Mapping[str, Any],
    ) -> bool:
        try:
            if cached.get("manifestStat") != self._stat_fingerprint(manifest_path):
                return False
            file_stats = cached.get("fileStats", {})
            for relative, expected in file_stats.items():
                if self._stat_fingerprint(directory / relative) != expected:
                    return False
        except (OSError, ValueError):
            return False
        return True

    def _manifest_for_serving(self, version_id: str) -> Dict[str, Any]:
        cached = self._verified.get(version_id)
        manifest_path = self.versions_root / version_id / "manifest.json"
        if cached is not None:
            try:
                if cached.get("manifestStat") == self._stat_fingerprint(manifest_path):
                    return dict(cached["manifest"])
            except OSError:
                pass
        return self.verify(version_id)
