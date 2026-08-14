from __future__ import annotations

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class LivoxTimestampMapperTest(unittest.TestCase):
    def test_mapper_reanchors_after_host_clock_steps(self) -> None:
        compiler = shutil.which("c++") or shutil.which("g++")
        self.assertIsNotNone(compiler, "a C++ compiler is required")
        source = ROOT / "tests/cpp/no_sync_timestamp_mapper_test.cpp"
        include = ROOT / "third_party/locked_stack/src/livox_ros_driver2/src"
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "timestamp-mapper-test"
            subprocess.run(
                [
                    str(compiler),
                    "-std=c++17",
                    "-pthread",
                    "-I",
                    str(include),
                    str(source),
                    "-o",
                    str(executable),
                ],
                check=True,
            )
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    unittest.main()
