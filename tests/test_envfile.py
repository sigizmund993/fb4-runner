"""The .env reader: file values, environment overrides, in-place updates."""
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import helpers  # noqa: F401  (sets up sys.path)

from utils import envfile


class EnvFileTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.path = Path(tmp.name) / ".env"
        self.path.write_text(
            "# comment\n"
            "FB4_A=1\n"
            "export FB4_B = 'two words'\n"
            "FB4_C=x # trailing comment\n"
            "FB4_EMPTY=\n"
            "not a setting\n"
        )
        env = mock.patch.dict(os.environ, {k: v for k, v in os.environ.items() if not k.startswith("FB4_")}, clear=True)
        env.start()
        self.addCleanup(env.stop)

    def test_read(self):
        self.assertEqual(envfile.read(self.path), {"FB4_A": "1", "FB4_B": "two words", "FB4_C": "x", "FB4_EMPTY": ""})

    def test_get(self):
        self.assertEqual(envfile.get("FB4_A", path=self.path), "1")
        self.assertEqual(envfile.get("FB4_EMPTY", "dflt", path=self.path), "dflt")
        self.assertEqual(envfile.get("FB4_MISSING", "dflt", path=self.path), "dflt")
        self.assertIsNone(envfile.get("FB4_A", path=self.path.with_name("nope")))

    def test_environment_overrides_file(self):
        os.environ["FB4_A"] = "9"
        self.assertEqual(envfile.get("FB4_A", path=self.path), "9")

    def test_put_replaces_in_place(self):
        envfile.put("FB4_B", "7", self.path)
        lines = self.path.read_text().splitlines()
        self.assertEqual(lines[0], "# comment")
        self.assertEqual(lines[2], "FB4_B=7")
        self.assertEqual(len(lines), 6)

    def test_put_appends_and_creates(self):
        envfile.put("FB4_NEW", "3", self.path)
        self.assertEqual(envfile.read(self.path)["FB4_NEW"], "3")
        fresh = self.path.with_name("fresh.env")
        envfile.put("FB4_ROBOT_ID", "4", fresh, template=self.path.with_name("no_template"))
        self.assertEqual(fresh.read_text(), "FB4_ROBOT_ID=4\n")

    def test_put_starts_from_template(self):
        template = self.path.with_name(".env.example")
        template.write_text("# robot id\nFB4_ROBOT_ID=\nFB4_ROBOT_TEAM=yellow\n")
        fresh = self.path.with_name("fresh.env")
        envfile.put("FB4_ROBOT_ID", "4", fresh, template=template)
        self.assertEqual(fresh.read_text(), "# robot id\nFB4_ROBOT_ID=4\nFB4_ROBOT_TEAM=yellow\n")


if __name__ == "__main__":
    unittest.main()
