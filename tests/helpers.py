import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for path in (ROOT, ROOT / "src"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def require_protobuf():
    """Skips the calling test module when ssl_packet_package is not available."""
    try:
        from ssl_packet_package.protopy.spbunited.robot import control_pb2, telemetry_pb2  # noqa: F401
    except ImportError as e:
        raise unittest.SkipTest(f"ssl_packet_package unavailable ({e}); run: git submodule update --init")
