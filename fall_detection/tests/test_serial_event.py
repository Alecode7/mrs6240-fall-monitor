import unittest
import time
from datetime import datetime
from queue import Queue

from fall_detection.serial_event import (
    SerialLogReader,
    command_bytes,
    parse_board_status,
    parse_fall_event,
)


class SerialEventTests(unittest.TestCase):
    def test_parse_fall_event_extracts_state_and_fields(self):
        event = parse_fall_event(
            "FALL_EVENT state=confirmed frame=530 t=1234 drop_cm=51 z=30 reason=low_still"
        )

        self.assertIsNotNone(event)
        assert event is not None
        self.assertEqual(event.state, "confirmed")
        self.assertEqual(event.fields["frame"], "530")
        self.assertEqual(event.fields["reason"], "low_still")
        self.assertIsInstance(event.received_at, datetime)

    def test_parse_fall_event_supports_suspected_confirmed_and_recovered(self):
        for state in ("suspected", "confirmed", "recovered"):
            event = parse_fall_event(f"FALL_EVENT state={state} frame=10")
            self.assertIsNotNone(event)
            assert event is not None
            self.assertEqual(event.state, state)

    def test_parse_fall_event_ignores_unrelated_and_malformed_lines(self):
        self.assertIsNone(parse_fall_event("FALL frame=530 state=0"))
        self.assertIsNone(parse_fall_event("FALL_EVENT frame=530"))
        self.assertIsNone(parse_fall_event("FALL_EVENT state=unknown frame=530"))

    def test_parse_fall_lines_accepts_shell_prompt_prefix(self):
        event = parse_fall_event("psic:/$ FALL_EVENT state=confirmed frame=530")
        status = parse_board_status("psic:/$ FALL frame=530 state=2 target=1 z=20 h=30")

        self.assertIsNotNone(event)
        self.assertIsNotNone(status)
        assert event is not None and status is not None
        self.assertEqual(event.state, "confirmed")
        self.assertEqual(status.frame_index, 530)
        self.assertEqual(status.state, 2)

    def test_event_identity_is_stable_for_duplicate_serial_delivery(self):
        first = parse_fall_event("FALL_EVENT state=suspected frame=10 t=100")
        second = parse_fall_event("FALL_EVENT state=suspected frame=10 t=100")

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        assert first is not None and second is not None
        self.assertEqual(first.identity, second.identity)

    def test_event_identity_falls_back_to_raw_line_when_no_frame_or_time_exists(self):
        first = parse_fall_event("FALL_EVENT state=confirmed reason=low_active")
        second = parse_fall_event("FALL_EVENT state=confirmed reason=low_active")

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        assert first is not None and second is not None
        self.assertEqual(first.identity, second.identity)

    def test_command_line_always_uses_crlf(self):
        self.assertEqual(command_bytes("mmwc start"), b"mmwc start\r\n")
        self.assertEqual(command_bytes("mmwc stop\r\n"), b"mmwc stop\r\n")
        self.assertEqual(command_bytes("  "), b"")

    def test_parse_board_status_extracts_target_telemetry(self):
        status = parse_board_status(
            "FALL frame=2610 state=0 track=1 target=1 x=24 y=34 z=155 h=35 "
            "pts=15 mot=15 v=21 raw=17 clusters=2 fresh=1 low=0 overflow=8494"
        )

        self.assertIsNotNone(status)
        assert status is not None
        self.assertEqual(status.frame_index, 2610)
        self.assertEqual(status.state, 0)
        self.assertEqual(status.x_cm, 24)
        self.assertEqual(status.z_cm, 155)
        self.assertEqual(status.height_cm, 35)
        self.assertEqual(status.point_count, 15)
        self.assertEqual(status.motion_count, 15)
        self.assertEqual(status.overflow, 8494)

    def test_parse_board_status_accepts_no_target_status_without_target_fields(self):
        status = parse_board_status(
            "FALL frame=2620 state=0 track=3 target=0 raw=43 clusters=3 fresh=0 low=0 overflow=8494"
        )

        self.assertIsNotNone(status)
        assert status is not None
        self.assertFalse(status.has_target)
        self.assertIsNone(status.z_cm)
        self.assertEqual(status.raw_count, 43)


class SerialReaderBoundaryTests(unittest.TestCase):
    def test_reader_starts_disconnected(self):
        reader = SerialLogReader()
        self.assertFalse(reader.is_connected)
        reader.stop()

    def test_reader_emits_received_lines_and_transmitted_commands(self):
        class FakeSerial:
            def __init__(self, **_kwargs):
                self.is_open = True
                self.written = []
                self.lines = [b"FALL_EVENT state=confirmed frame=3\r\n"]

            def readline(self):
                if self.lines:
                    return self.lines.pop(0)
                return b""

            def write(self, payload):
                self.written.append(payload)

            def close(self):
                self.is_open = False

        records = Queue()
        fake_serial = FakeSerial()
        reader = SerialLogReader(records, serial_factory=lambda **kwargs: fake_serial)
        reader.connect("COM12", 115200)
        reader.send_line("mmwc stop")

        received = records.get(timeout=1.0)
        transmitted = records.get(timeout=1.0)
        reader.stop()

        self.assertEqual(received.direction, "rx")
        self.assertIsNotNone(received.event)
        self.assertEqual(received.event.state, "confirmed")
        self.assertEqual(transmitted.direction, "tx")
        self.assertEqual(transmitted.line, "mmwc stop")
        self.assertEqual(fake_serial.written, [b"mmwc stop\r\n"])

    def test_reader_attaches_board_status_to_received_fall_line(self):
        class FakeSerial:
            def __init__(self, **_kwargs):
                self.is_open = True
                self.lines = [
                    b"FALL frame=2610 state=0 track=1 target=1 x=24 y=34 z=155 h=35 pts=15 mot=15 v=21 raw=17 clusters=2 fresh=1 low=0 overflow=8494\r\n"
                ]

            def readline(self):
                if self.lines:
                    return self.lines.pop(0)
                return b""

            def close(self):
                self.is_open = False

        records = Queue()
        reader = SerialLogReader(records, serial_factory=lambda **kwargs: FakeSerial())
        reader.connect("COM12", 115200)
        record = records.get(timeout=1.0)
        reader.stop()

        self.assertIsNotNone(record.status)
        assert record.status is not None
        self.assertEqual(record.status.frame_index, 2610)
        self.assertEqual(record.status.z_cm, 155)

    def test_reader_releases_port_after_read_error(self):
        class FakeSerial:
            def __init__(self, **_kwargs):
                self.is_open = True

            def readline(self):
                raise OSError("device disconnected")

            def close(self):
                self.is_open = False

        records = Queue()
        reader = SerialLogReader(records, serial_factory=lambda **kwargs: FakeSerial())
        reader.connect("COM3", 115200)
        record = records.get(timeout=1.0)
        for _ in range(20):
            if not reader.is_connected:
                break
            time.sleep(0.01)
        reader.stop()

        self.assertEqual(record.direction, "error")
        self.assertIn("device disconnected", record.line)
        self.assertFalse(reader.is_connected)


if __name__ == "__main__":
    unittest.main()
