"""Firmware updater: release tag selection, image checks, flash only when needed."""
import struct
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import helpers  # noqa: F401  (sets up sys.path)

from utils import flash_firmware as ff

IMAGE = struct.pack("<II", 0x2003_0000, 0x0800_01C5) + bytes(range(256)) * 4


class TagsTest(unittest.TestCase):
    def test_prefix(self):
        self.assertEqual(ff.tag_prefix("feature/uart_for_rpi"), "feature-uart_for_rpi-v0.0.")
        self.assertEqual(ff.tag_prefix("main"), "v0.0.")

    def test_newest_first_numeric_order(self):
        prefix = ff.tag_prefix("feature/uart_for_rpi")
        out = "\n".join(
            f"{i:040x}\trefs/tags/{tag}"
            for i, tag in enumerate([
                f"{prefix}2", f"{prefix}10", f"{prefix}9",
                f"{prefix}3-rc", "feature-uart_for_rpi-other-v0.0.50", "v0.0.99",
            ])
        )
        self.assertEqual(ff.parse_tags(out, prefix), [f"{prefix}10", f"{prefix}9", f"{prefix}2"])

    def test_falls_back_when_newest_release_has_no_bin_yet(self):
        with mock.patch.object(ff, "list_release_tags", return_value=["t5", "t4", "t3"]), \
             mock.patch.object(ff, "download", side_effect=lambda tag: None if tag == "t5" else Path(tag)), \
             mock.patch("builtins.print"):
            self.assertEqual(ff.find_latest("b"), ("t4", Path("t4")))

    def test_no_releases(self):
        with mock.patch.object(ff, "list_release_tags", return_value=[]):
            self.assertRaises(LookupError, ff.find_latest, "b")


class ImageCheckTest(unittest.TestCase):
    def test_valid_image(self):
        ff.check_image(IMAGE)

    def test_rejects_html_and_garbage(self):
        for data in (b"<!DOCTYPE html><html>Not Found</html>", b"", struct.pack("<II", 0x2002_0000, 0x0800_0100)):
            with self.subTest(data=data[:16]):
                self.assertRaises(ValueError, ff.check_image, data)


class MainTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.image = Path(self.tmp.name) / "firmware-t1.bin"
        self.image.write_bytes(IMAGE)
        self.mcu = bytearray(IMAGE)
        self.calls = []
        patches = [
            mock.patch.object(ff, "find_latest", return_value=("t1", self.image)),
            mock.patch.object(ff, "st_flash", side_effect=self.fake_st_flash),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def fake_st_flash(self, *args):
        self.calls.append(args[0] if args[0] != "--reset" else args[1])
        if args[0] == "read":
            Path(args[1]).write_bytes(self.mcu[: int(args[3])])
        else:
            self.mcu[:] = Path(args[2]).read_bytes()

    def run_main(self, *argv):
        with mock.patch("builtins.print"):
            return ff.main(list(argv))

    def test_same_firmware_is_not_rewritten(self):
        self.assertEqual(self.run_main(), 0)
        self.assertEqual(self.calls, ["read"])

    def test_different_firmware_is_flashed(self):
        self.mcu[100] ^= 0xFF
        self.assertEqual(self.run_main(), 0)
        self.assertEqual(self.calls, ["read", "write"])
        self.assertEqual(bytes(self.mcu), IMAGE)

    def test_force_and_dry_run(self):
        self.assertEqual(self.run_main("--force"), 0)
        self.assertEqual(self.calls, ["write"])
        self.calls.clear()
        self.assertEqual(self.run_main("--dry-run"), 0)
        self.assertEqual(self.calls, [])

    def test_offline_keeps_firmware(self):
        with mock.patch.object(ff, "find_latest", side_effect=ff.Offline("no network")):
            self.assertEqual(self.run_main(), 0)
        self.assertEqual(self.calls, [])

    def test_no_st_link_is_an_error(self):
        with mock.patch.object(ff, "st_flash", side_effect=ff.subprocess.CalledProcessError(255, "st-flash")):
            self.assertEqual(self.run_main(), 1)


if __name__ == "__main__":
    unittest.main()
