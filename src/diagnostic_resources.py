"""Resource samples for an active FB4 diagnostics connection."""

from ctypes import CDLL, POINTER, Structure, byref, c_float, c_uint16, c_void_p
import os
from pathlib import Path
from stat import S_ISREG
from threading import Event, Thread
from time import monotonic, time

PROC_STAT = Path("/proc/stat")
MEMINFO = Path("/proc/meminfo")
PI_TEMP = Path("/sys/class/thermal/thermal_zone0/temp")
HAILO_MONITOR_DIR = Path("/tmp/nnc_utilization")


def pi_temperature():
    try:
        return int(PI_TEMP.read_text()) / 1000
    except (OSError, ValueError):
        return None


def cpu_ticks():
    try:
        with PROC_STAT.open() as handle:
            row = handle.readline().split()
        if row[0] == "cpu" and len(row) >= 9:
            ticks = [int(value) for value in row[1:9]]
            return sum(ticks), ticks[3] + ticks[4]
    except (OSError, ValueError, IndexError):
        pass
    return None


def memory_info():
    try:
        info = {}
        with MEMINFO.open() as handle:
            for row in handle:
                key, _, value = row.partition(":")
                if key in ("MemTotal", "MemAvailable"):
                    info[key] = int(value.split()[0])
        total = info["MemTotal"]
        available = info["MemAvailable"]
        if total > 0 and 0 <= available <= total:
            used = total - available
            return round(used / total * 100, 1), round(used / 1024, 1), round(total / 1024, 1)
    except (OSError, ValueError, KeyError, IndexError):
        pass
    return None, None, None


def hailo_utilization():
    # HailoRT 4.x appends a temporary-file suffix to the utilization snapshot.
    latest = None
    for path in HAILO_MONITOR_DIR.glob("nnc_utilization*"):
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            with os.fdopen(fd, "rb") as handle:
                metadata = os.fstat(handle.fileno())
                if not S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or not 0 <= time() - metadata.st_mtime < 3:
                    continue
                percent = float(handle.readline(64))
                if 0 <= percent < 101 and (latest is None or metadata.st_mtime > latest[0]):
                    latest = metadata.st_mtime, min(percent, 100.0)
        except (OSError, ValueError):
            continue
    return latest[1] if latest is not None else None


class ChipTemperature(Structure):
    # hailo_chip_temperature_info_t in the HailoRT 4.x C API.
    _fields_ = [("ts0", c_float), ("ts1", c_float), ("sample_count", c_uint16)]


class ResourceSampler:
    def __init__(self):
        self._previous_cpu = None
        self._next_sample = 0.0
        self._hat_temperature = None
        self._stop = Event()

    def __enter__(self):
        if os.access("/dev/hailo0", os.R_OK | os.W_OK):
            Thread(target=self._read_hat_temperature, daemon=True, name="hailo-temperature").start()
        return self

    def __exit__(self, *_exception):
        self._stop.set()

    def sample(self):
        now = monotonic()
        if now < self._next_sample:
            return None
        self._next_sample = now + 1

        ticks = cpu_ticks()
        cpu = None
        if ticks is not None and self._previous_cpu is not None:
            total_delta = ticks[0] - self._previous_cpu[0]
            idle_delta = ticks[1] - self._previous_cpu[1]
            if total_delta > 0 and 0 <= idle_delta <= total_delta:
                cpu = round((total_delta - idle_delta) / total_delta * 100, 1)
        self._previous_cpu = ticks

        percent, used, total = memory_info()
        return {
            "cpu_percent": cpu,
            "hailo_percent": hailo_utilization(),
            "memory_percent": percent,
            "memory_used_mib": used,
            "memory_total_mib": total,
            "rpi_temp": pi_temperature(),
            "hat_temp": self._hat_temperature,
        }

    def _read_hat_temperature(self):
        paths = sorted(Path("/usr/lib").glob("libhailort.so.4.*"), reverse=True)
        if not paths:
            return
        try:
            os.environ.setdefault("HAILORT_LOGGER_PATH", "NONE")
            lib = CDLL(str(paths[0]))
            lib.hailo_create_pcie_device.argtypes = [c_void_p, POINTER(c_void_p)]
            lib.hailo_get_chip_temperature.argtypes = [c_void_p, POINTER(ChipTemperature)]
            lib.hailo_release_device.argtypes = [c_void_p]
        except (OSError, AttributeError):
            return

        device = c_void_p()
        if lib.hailo_create_pcie_device(None, byref(device)) != 0:
            return
        try:
            while not self._stop.is_set():
                sample = ChipTemperature()
                status = lib.hailo_get_chip_temperature(device, byref(sample))
                readings = [temp for temp in (sample.ts0, sample.ts1) if -40 <= temp <= 150]
                self._hat_temperature = max(readings) if status == 0 and sample.sample_count and readings else None
                self._stop.wait(1)
        finally:
            lib.hailo_release_device(device)
