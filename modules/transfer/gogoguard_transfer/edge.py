from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
from pathlib import Path
from typing import BinaryIO


SAFE_SESSION = re.compile(r"^[0-9]{8}T[0-9]{6}Z-[A-Za-z0-9]{8}$")
SAFE_MAP_JOB = re.compile(r"^map-[A-Za-z0-9]{12}$")
SAFE_ARTIFACT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
NAVIGATION_WORKSPACE = "navigation-workspace.json"
CHECKPOINT_ASSET = "checkpoints.json"
MUTABLE_NAVIGATION_ARTIFACTS = {NAVIGATION_WORKSPACE, CHECKPOINT_ASSET}
REQUIRED_MAP_ARTIFACTS = {"map.json", "map.ply", "overview.svg", "glim-build.json"}


class TransferContractError(RuntimeError):
    pass


def _json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TransferContractError(f"{path.name} must contain a JSON object")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


class EdgeArtifactExchange:
    """Immutable recording export and map import boundary on the robot."""

    def __init__(self, data_root: Path) -> None:
        self.data_root = Path(data_root)
        self.recordings_root = self.data_root / "recordings"
        self.map_jobs_root = self.data_root / "map-jobs"
        self.import_root = self.data_root / "workstation-imports"
        self.import_root.mkdir(parents=True, exist_ok=True)

    def recording_descriptor(self, session_id: str) -> dict:
        root = self._recording_root(session_id)
        session = _json(root / "session.json")
        manifest = _json(root / "recording_bundle.json")
        if session.get("state") != "sealed":
            raise TransferContractError("only sealed recordings may be exported")
        if manifest.get("schema") != "gogoguard.recording_bundle.v1":
            raise TransferContractError("recording bundle schema is unsupported")
        if manifest.get("session_id") != session_id:
            raise TransferContractError("recording bundle identity mismatch")
        files = manifest.get("files")
        if not isinstance(files, list) or not files:
            raise TransferContractError("recording bundle contains no files")
        total = 0
        for item in files:
            relative = self._safe_relative(str(item.get("path", "")))
            path = (root / relative).resolve()
            if root.resolve() not in path.parents or not path.is_file():
                raise TransferContractError(f"recording file is unavailable: {relative}")
            if path.stat().st_size != int(item.get("bytes", -1)):
                raise TransferContractError(f"recording file size changed: {relative}")
            total += path.stat().st_size
        return {
            "schema": "gogoguard.edge_recording_export.v1",
            "session": session,
            "manifest": manifest,
            "bytes_total": total,
        }

    def recording_file(self, session_id: str, relative_name: str) -> Path:
        root = self._recording_root(session_id)
        relative = self._safe_relative(relative_name)
        allowed = {"session.json", "recording_bundle.json"}
        descriptor = self.recording_descriptor(session_id)
        allowed.update(str(item["path"]) for item in descriptor["manifest"]["files"])
        if relative.as_posix() not in allowed:
            raise TransferContractError("file is not part of the sealed recording bundle")
        target = (root / relative).resolve()
        if root.resolve() not in target.parents or not target.is_file():
            raise TransferContractError("recording file is unavailable")
        return target

    def receive_map_artifact(
        self,
        job_id: str,
        artifact_name: str,
        reader: BinaryIO,
        content_length: int,
        expected_sha256: str,
    ) -> dict:
        self._validate_job(job_id)
        self._validate_artifact_name(artifact_name)
        if content_length < 0 or content_length > 32 * 1024**3:
            raise TransferContractError("artifact content length is invalid")
        if not re.fullmatch(r"[0-9a-f]{64}", expected_sha256):
            raise TransferContractError("artifact SHA-256 is required")
        staging = self.import_root / job_id / "artifacts"
        staging.mkdir(parents=True, exist_ok=True)
        target = staging / artifact_name
        if target.is_file():
            if target.stat().st_size == content_length and _sha256(target) == expected_sha256:
                # An HTTP client may already be streaming the request body when
                # this idempotency check succeeds. Consume it before replying;
                # returning early closes the socket under the sender and
                # surfaces as BrokenPipeError on large repeated deployments.
                remaining = content_length
                while remaining:
                    chunk = reader.read(min(1024 * 1024, remaining))
                    if not chunk:
                        raise TransferContractError("artifact upload ended before Content-Length")
                    remaining -= len(chunk)
                return {"name": artifact_name, "bytes": content_length, "sha256": expected_sha256}
            if artifact_name not in MUTABLE_NAVIGATION_ARTIFACTS:
                raise TransferContractError("immutable staged artifact already exists with different content")
        temporary = target.with_name(f".{target.name}.part")
        digest = hashlib.sha256()
        remaining = content_length
        try:
            with temporary.open("wb") as output:
                while remaining:
                    chunk = reader.read(min(1024 * 1024, remaining))
                    if not chunk:
                        raise TransferContractError("artifact upload ended before Content-Length")
                    output.write(chunk)
                    digest.update(chunk)
                    remaining -= len(chunk)
                output.flush()
                os.fsync(output.fileno())
            actual = digest.hexdigest()
            if actual != expected_sha256:
                raise TransferContractError("artifact SHA-256 mismatch")
            os.replace(temporary, target)
        finally:
            if temporary.exists():
                temporary.unlink()
        return {"name": artifact_name, "bytes": content_length, "sha256": expected_sha256}

    def map_import_descriptor(self, job_id: str) -> dict:
        """Describe immutable artifacts already held for a map deployment."""
        self._validate_job(job_id)
        final_artifacts = self.map_jobs_root / job_id / "artifacts"
        staged_artifacts = self.import_root / job_id / "artifacts"
        if final_artifacts.is_dir():
            state = "committed"
            root = final_artifacts
        elif staged_artifacts.is_dir():
            state = "staged"
            root = staged_artifacts
        else:
            state = "empty"
            root = None
        files = [] if root is None else [
            {"name": path.name, "bytes": path.stat().st_size, "sha256": _sha256(path)}
            for path in sorted(root.iterdir())
            if (
                path.is_file()
                and not path.name.startswith(".")
                and path.name not in MUTABLE_NAVIGATION_ARTIFACTS
            )
        ]
        if state == "committed":
            for name in sorted(MUTABLE_NAVIGATION_ARTIFACTS):
                current = self.map_jobs_root / job_id / name
                legacy = final_artifacts / name
                path = current if current.is_file() else legacy
                if path.is_file():
                    files.append(
                        {
                            "name": name,
                            "bytes": path.stat().st_size,
                            "sha256": _sha256(path),
                        }
                    )
        return {
            "schema": "gogoguard.edge_map_import.v1",
            "job_id": job_id,
            "state": state,
            "files": files,
        }

    def commit_map_import(self, job_id: str, payload: dict) -> dict:
        self._validate_job(job_id)
        if payload.get("schema") != "gogoguard.workstation_map_deployment.v1":
            raise TransferContractError("map deployment schema is unsupported")
        if payload.get("job_id") != job_id:
            raise TransferContractError("map deployment identity mismatch")
        files = payload.get("files")
        if not isinstance(files, list):
            raise TransferContractError("map deployment file manifest is required")
        names = {str(item.get("name", "")) for item in files}
        if not REQUIRED_MAP_ARTIFACTS.issubset(names):
            missing = sorted(REQUIRED_MAP_ARTIFACTS - names)
            raise TransferContractError(f"map deployment is missing artifacts: {missing}")
        staging_root = self.import_root / job_id
        staging_artifacts = staging_root / "artifacts"
        final_root = self.map_jobs_root / job_id
        final_artifacts = final_root / "artifacts"
        final_exists = final_artifacts.is_dir()
        for item in files:
            name = str(item.get("name", ""))
            self._validate_artifact_name(name)
            staged_path = staging_artifacts / name
            final_path = final_artifacts / name
            if name in MUTABLE_NAVIGATION_ARTIFACTS:
                committed_navigation_asset = final_root / name
                legacy_navigation_asset = final_artifacts / name
                path = (
                    staged_path
                    if staged_path.is_file()
                    else committed_navigation_asset
                    if committed_navigation_asset.is_file()
                    else legacy_navigation_asset
                )
            else:
                path = final_path if final_exists else staged_path
            if not path.is_file():
                raise TransferContractError(f"map artifact is missing: {name}")
            if path.stat().st_size != int(item.get("bytes", -1)):
                raise TransferContractError(f"map artifact size mismatch: {name}")
            if _sha256(path) != str(item.get("sha256", "")):
                raise TransferContractError(f"map artifact hash mismatch: {name}")
        verification_root = final_artifacts if final_exists else staging_artifacts
        artifact = _json(verification_root / "map.json")
        if artifact.get("source") != "cloud-glim":
            raise TransferContractError("navigation accepts only cloud GLIM maps")

        staged_navigation_assets = {
            name: staging_artifacts / name
            for name in MUTABLE_NAVIGATION_ARTIFACTS
            if (staging_artifacts / name).is_file()
        }
        if final_exists:
            # A repeated or resumed deployment can leave a verified staging
            # copy behind. Immutable GLIM files stay authoritative, while the
            # separately stored operator workspace and route-bound checkpoint
            # asset may advance together as one workspace revision.
            for name, source in staged_navigation_assets.items():
                target = final_root / name
                temporary = final_root / f".{name}.tmp"
                shutil.copyfile(source, temporary)
                os.replace(temporary, target)
            shutil.rmtree(staging_root, ignore_errors=True)
        else:
            final_root.mkdir(parents=True, exist_ok=True)
            staged_root_assets: dict[str, Path] = {}
            for name, source in staged_navigation_assets.items():
                root_asset = staging_root / name
                os.replace(source, root_asset)
                staged_root_assets[name] = root_asset
            os.replace(staging_artifacts, final_artifacts)
            for name, source in staged_root_assets.items():
                if source.is_file():
                    os.replace(source, final_root / name)
            shutil.rmtree(staging_root, ignore_errors=True)

        job = {
            "schema": "gogoguard.map_job.v1",
            "job_id": job_id,
            "session_id": str(payload.get("session_id", "")),
            "state": "complete",
            "progress": 100,
            "stage": "deployed_to_robot",
            "message": "工作站地图已部署到机器狗",
            "created_at": payload.get("created_at"),
            "updated_at": payload.get("updated_at"),
            "artifact_root": str(final_artifacts),
            "overview_url": f"/artifacts/{job_id}/overview.svg",
            "point_cloud_url": f"/artifacts/{job_id}/map.json",
            "metrics": dict(payload.get("metrics") or {}) | {"deployment": "workstation"},
            "bytes_transferred": sum(int(item["bytes"]) for item in files),
            "bytes_total": sum(int(item["bytes"]) for item in files),
            "transfer_rate_bps": 0.0,
            "error": None,
        }
        (final_root / "job.json").write_text(
            json.dumps(job, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        return job

    def _recording_root(self, session_id: str) -> Path:
        if not SAFE_SESSION.fullmatch(session_id):
            raise TransferContractError("invalid recording session id")
        root = self.recordings_root / session_id
        if not root.is_dir() or root.is_symlink():
            raise TransferContractError("recording session is unavailable")
        return root

    @staticmethod
    def _safe_relative(value: str) -> Path:
        path = Path(value)
        if not value or path.is_absolute() or ".." in path.parts:
            raise TransferContractError("unsafe recording file path")
        return path

    @staticmethod
    def _validate_job(job_id: str) -> None:
        if not SAFE_MAP_JOB.fullmatch(job_id):
            raise TransferContractError("invalid map job id")

    @staticmethod
    def _validate_artifact_name(name: str) -> None:
        if not SAFE_ARTIFACT.fullmatch(name):
            raise TransferContractError("invalid map artifact name")
