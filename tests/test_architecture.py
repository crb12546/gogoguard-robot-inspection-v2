from __future__ import annotations

import ast
import unittest
from pathlib import Path


class ArchitectureBoundaryTest(unittest.TestCase):
    def test_product_modules_do_not_import_peer_implementations(self) -> None:
        root = Path(__file__).resolve().parents[1]
        module_roots = {
            "gogoguard_device_io", "gogoguard_data_capture", "gogoguard_evidence",
            "gogoguard_map_factory", "gogoguard_interaction",
            "gogoguard_inspection", "gogoguard_mission",
            "gogoguard_platform_edge",
        }
        failures = []
        for path in [*root.glob("modules/*/**/*.py"), *root.glob("services/*/**/*.py")]:
            own = next((name for name in module_roots if name in path.parts), None)
            if own is None:
                continue
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                names = []
                if isinstance(node, ast.Import):
                    names = [item.name.split(".")[0] for item in node.names]
                elif isinstance(node, ast.ImportFrom) and node.module:
                    names = [node.module.split(".")[0]]
                for name in names:
                    if name in module_roots and name != own:
                        # evidence is the shared append-only infrastructure; all
                        # other peer implementation imports are forbidden.
                        if name != "gogoguard_evidence":
                            failures.append(f"{path}: {own} imports peer {name}")
        self.assertEqual(failures, [], "\n".join(failures))
