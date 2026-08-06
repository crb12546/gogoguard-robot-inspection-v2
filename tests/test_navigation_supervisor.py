import tempfile
import threading
import time
import unittest
from pathlib import Path

from gogoguard_navigation.supervisor import NavigationSupervisorServer, SupervisorService
from gogoguard_navigation.supervisor_client import NavigationSupervisorClient


class FakeManager:
    def __init__(self):
        self.starts = 0

    def status(self):
        return {"runtime_process": {"running": self.starts > 0}, "candidate": None}

    def start_runtime(self, candidate_id):
        self.starts += 1
        time.sleep(0.03)
        return {"candidate_id": candidate_id}

    def stop_runtime(self): return {"stopped": True}
    def start_patrol(self): return {"started": True}
    def stop_patrol(self): return {"stopped": True}
    def reset_localization(self): return {"reset": True}
    def prepare(self, job_id): return {"job_id": job_id}
    def profile(self): return {"schema": "profile"}
    def update_profile(self, profile): return {"profile": profile}
    def rollback_profile(self): return {"rolled_back": True}
    def diagnostics(self): return {"summary": "ok"}


class NavigationSupervisorTest(unittest.TestCase):
    def test_long_operation_returns_receipt_and_is_deduplicated(self):
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "supervisor.sock"
            manager = FakeManager()
            server = NavigationSupervisorServer(socket_path, SupervisorService(manager))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            client = NavigationSupervisorClient(socket_path)
            first = client.start_runtime("map-123456789abc")
            second = client.start_runtime("map-123456789abc")
            self.assertEqual(first["operationId"], second["operationId"])
            self.assertIn(first["state"], {"accepted", "running"})
            deadline = time.time() + 1.0
            operation = None
            while time.time() < deadline:
                operation = client.status()["operations"][0]
                if operation["state"] == "complete":
                    break
                time.sleep(0.01)
            self.assertEqual(operation["state"], "complete")
            self.assertEqual(manager.starts, 1)
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
