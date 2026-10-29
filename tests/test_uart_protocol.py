import struct
import unittest

import helpers  # noqa: F401  (sets up sys.path)
from src.protocol.uart import (
    CMD_FRAME_LEN,
    TEL_FMT,
    TEL_FRAME_LEN,
    FrameParser,
    UartCommand,
    make_frame,
    pack_command,
)


class UartProtocolTest(unittest.TestCase):
    def test_command_frame_matches_firmware_layout(self):
        frame = pack_command(0.5, -0.25, 1.0, 3, 8, 2, True)
        self.assertEqual(len(frame), CMD_FRAME_LEN)
        self.assertEqual(CMD_FRAME_LEN, 16)
        self.assertEqual(frame.hex(), "a5003800b4003c004200480201" "00" "ff" "e3")
        self.assertEqual(UartCommand(0.5, -0.25, 1.0, 3, 8, 2, True).to_bytes(), frame)
        self.assertEqual(UartCommand(0.5, -0.25, 1.0, 3, 8, 2, True, beep=True, robot_id=7).to_bytes().hex(),
                         "a5003800b4003c004200480201" "01" "07" "1a")

    def test_telemetry_parser(self):
        self.assertEqual(TEL_FRAME_LEN, 7)
        tel1 = make_frame(struct.pack(TEL_FMT, 5, 12.5, True, False))
        tel2 = make_frame(struct.pack(TEL_FMT, 7, 20.0, False, True))
        bad = bytearray(tel1)
        bad[-1] ^= 0xFF

        parser = FrameParser(TEL_FMT)
        frames = parser.feed(b"\x00\xa5\x13" + tel1 + bytes(bad) + b"junk" + tel2[:3])
        self.assertEqual(frames, [(5, 12.5, True, False)])
        self.assertEqual(parser.feed(tel2[3:]), [(7, 20.0, False, True)])


if __name__ == "__main__":
    unittest.main()
