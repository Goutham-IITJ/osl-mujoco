"""Connection tests need localhost sockets; never import hardware drivers."""
import json
import math
import tempfile
import unittest
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from hardware.knee_telemetry import KneeTelemetryServer
from oslbench.telemetry.knee import KneeReceiver
from oslbench.apps.knee_telemetry import KneePresentation, connect_receiver, main


class ConnectionTests(unittest.TestCase):
    def setUp(self):
        self.server = KneeTelemetryServer(port=0, capacity=3, token="test-token", source="synthetic")
        self.addCleanup(self.server.close)

    def publish(self, tick, **overrides):
        row = dict(time_s=tick*.005, dt_s=.005, reference_deg=-30., actual_deg=-28.,
                   requested_current_A=5., commanded_current_A=4., encoder_valid=True)
        row.update(overrides)
        return self.server.publish(**row)

    def receiver(self):
        return KneeReceiver("127.0.0.1", self.server.port, "test-token")

    def test_authentication_and_read_only(self):
        with self.assertRaises(HTTPError) as error:
            KneeReceiver("127.0.0.1", self.server.port, "wrong")
        self.assertEqual(error.exception.code, 401)
        request = Request(f"http://127.0.0.1:{self.server.port}/v1/samples", data=b'{}',
                          headers={"Authorization": "Bearer test-token"})
        with self.assertRaises(HTTPError) as error:
            urlopen(request)
        self.assertEqual(error.exception.code, 501)

    def test_batch_order_overflow_and_no_duplicates(self):
        for tick in range(5):
            self.assertTrue(self.publish(tick))
        receiver = self.receiver()
        batch = receiver.poll()
        self.assertEqual([r["sequence"] for r in batch["samples"]], [2, 3, 4])
        self.assertEqual(receiver.missing, 2)
        self.assertEqual(receiver.poll()["samples"], [])

    def test_publish_does_not_wait_for_buffer(self):
        with self.server._lock:
            self.assertFalse(self.publish(0))
        self.assertTrue(self.publish(1))
        self.assertEqual(self.receiver().poll()["missing_this_batch"], 1)
        self.assertEqual(self.server.dropped, 1)

    def test_invalid_values_and_reused_time(self):
        self.assertFalse(self.publish(0, actual_deg=math.nan))
        self.assertTrue(self.publish(1))
        self.assertFalse(self.publish(1))
        self.assertFalse(self.publish(2, encoder_valid=1))
        self.assertEqual(self.server.rejected, 3)

    def test_session_restart_is_not_silently_merged(self):
        receiver = self.receiver()
        self.server.metadata["session_id"] = "new-session"
        with self.assertRaisesRegex(ValueError, "session changed"):
            receiver.poll()

    def test_display_sign_invalid_encoder_and_staleness(self):
        self.publish(0)
        self.publish(1, encoder_valid=False)
        receiver = self.receiver()
        batch = receiver.poll()
        view = KneePresentation(receiver.metadata)
        batch["producer_age_s"] = 1
        view.add(batch)
        result = view.snapshot()
        self.assertEqual(result["points"], [[0., 30., 28.], [.005, 30., None]])
        self.assertEqual(result["metrics"]["hardware_target_RMS_deg"], 2)
        self.assertEqual(result["metrics"]["clipped_samples"], 2)
        self.assertEqual(result["status"], "STALE")

    def test_lan_requires_token(self):
        with self.assertRaises(ValueError):
            KneeTelemetryServer("0.0.0.0", 0)

    def test_receiver_can_start_before_server(self):
        from unittest.mock import patch
        from urllib.error import URLError
        expected = object()
        with patch("oslbench.apps.knee_telemetry.KneeReceiver",
                   side_effect=[URLError("not started"), expected]) as factory:
            self.assertIs(connect_receiver("127.0.0.1", 8794, None), expected)
            self.assertEqual(factory.call_count, 2)

    def test_edited_controller_only_adds_telemetry(self):
        import ast
        import re
        root = Path(__file__).resolve().parents[1]
        edited = (root/"hardware/knee_step_test_PID_control_me.py").read_text(encoding="utf-8")
        ast.parse(edited)  # Never import/execute the physical controller.
        stripped, count = re.subn(
            r"(?m)^[ \t]*# BEGIN TELEMETRY ADDITION: (\w+)\n.*?^[ \t]*# END TELEMETRY ADDITION: \1\n",
            "", edited, flags=re.DOTALL)
        self.assertEqual(count, 4)
        original = root/"build/knee_hardware_controller_comparison/supplied_knee_controller.txt"
        if original.exists():
            self.assertEqual(stripped, original.read_text(encoding="utf-8"))
        tree = ast.parse(edited)
        publish = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                       and isinstance(n.func, ast.Attribute) and n.func.attr == "publish")
        expression = ast.Expression(publish)
        from unittest.mock import Mock
        telemetry = Mock()
        values = dict(telemetry=telemetry, now_t=10.005, t0_pid=10., dt=.005,
                      reference_deg=-10., actual_deg=-9., current_cmd=70.,
                      current_clamped=60., jump_bad=False, abort_reason=None)
        from types import SimpleNamespace
        values['STATE'] = SimpleNamespace(i2c_fail_streak=0)
        eval(compile(expression, '<isolated telemetry hook>', 'eval'), values)
        packet = telemetry.publish.call_args.kwargs
        self.assertEqual(packet['requested_current_A'], 70.)
        self.assertEqual(packet['commanded_current_A'], 60.)
        self.assertTrue(packet['encoder_valid'])
        values['STATE'].i2c_fail_streak = 1
        eval(compile(expression, '<isolated telemetry hook>', 'eval'), values)
        self.assertFalse(telemetry.publish.call_args.kwargs['encoder_valid'])

    def test_polling_pause_catches_up_without_duplicate_samples(self):
        receiver = self.receiver()
        self.publish(0)
        self.assertEqual(len(receiver.poll()["samples"]), 1)
        # No polls while the producer continues (e.g. a short Wi-Fi outage).
        self.publish(1)
        self.publish(2)
        self.assertEqual([r["sequence"] for r in receiver.poll()["samples"]], [1, 2])
        self.assertEqual(receiver.missing, 0)

    def test_bad_batch_is_rejected_without_advancing_cursor(self):
        from unittest.mock import patch
        receiver = self.receiver()
        self.publish(0)
        batch = self.server.batch()
        batch["samples"][0] = dict(batch["samples"][0], dt_s=-1)
        with patch.object(receiver, "get", return_value=batch):
            with self.assertRaises(ValueError):
                receiver.poll()
        self.assertEqual(receiver.sequence, -1)
        self.assertEqual(len(receiver.poll()["samples"]), 1)

    def test_receiver_records_and_preserves_existing_output(self):
        import os
        from unittest.mock import patch
        self.publish(0)
        self.publish(1)
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, OSL_TELEMETRY_TOKEN="test-token"):
            output = Path(directory)/"recording"
            args = ["--port", str(self.server.port), "--outdir", str(output),
                    "--seconds", ".15", "--dashboard-port", "0", "--no-browser"]
            self.assertEqual(main(args), 0)
            summary = json.loads((output/"summary.json").read_text())
            self.assertEqual(summary["metrics"]["samples"], 2)
            batches = [json.loads(line) for line in (output/"batches.jsonl").read_text().splitlines()]
            self.assertEqual(sum(len(b["samples"]) for b in batches), 2)
            with self.assertRaises(FileExistsError):
                main(args)


if __name__ == "__main__":
    unittest.main()
