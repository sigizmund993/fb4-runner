#!/usr/bin/env python3
"""Flashes the STM32 with the latest CI release of a firmware branch.

CI (FootBot4_Firmware/.github/workflows/release.yml) tags every push to branch B as
"<B with / replaced by ->-v0.0.N" ("v0.0.N" for main) and attaches firmware-<tag>.bin
to the GitHub release of that tag.

    python3 src/utils/flash_firmware.py              # FB4_FW_RELEASE of FB4_FW_BRANCH (.env)
    python3 src/utils/flash_firmware.py --dry-run    # find and download, don't flash
    python3 src/utils/flash_firmware.py --release 5  # = v0.0.5 = feature-uart_for_rpi-v0.0.5
    python3 src/utils/flash_firmware.py --branch main --release latest --force

Reads the MCU flash back first and only writes when it differs, so it is safe to run on
every boot. Without network it keeps the current firmware and exits 0. Standard library
only: runs with the system python3, without the runner's venv. Needs git and st-flash
(apt install stlink-tools).
"""
import argparse
import os
import re
import struct
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # src, when run as a script
from utils import envfile  # noqa: E402

REPO = envfile.get("FB4_FW_REPO", "SPbUnited/FootBot4_Firmware")
BRANCH = envfile.get("FB4_FW_BRANCH", "feature/uart_for_rpi")
# "latest", a release number of BRANCH ("5" or "v0.0.5") or a full tag
RELEASE = envfile.get("FB4_FW_RELEASE", "latest")
CACHE_DIR = Path(envfile.get("FB4_FW_CACHE", str(Path.home() / ".cache" / "fb4-firmware")))
FLASH_BASE = 0x0800_0000
FLASH_SIZE = 2 * 1024 * 1024  # STM32F429ZI
RAM_RANGE = (0x2000_0000, 0x2003_0000)  # SRAM1..3, where the initial stack pointer must be
NETWORK_TIMEOUT_S = 20
# CI pushes the tag before the release assets are uploaded: if the newest tag has no .bin
# yet, fall back to the ones before it
MAX_TAGS_TRIED = 3


class Offline(Exception):
    pass


def tag_prefix(branch: str) -> str:
    return "v0.0." if branch == "main" else branch.replace("/", "-") + "-v0.0."


def parse_tags(ls_remote_output: str, prefix: str) -> list[str]:
    """Release tags of the branch from `git ls-remote --tags`, newest first."""
    numbered = []
    for line in ls_remote_output.splitlines():
        ref = line.split("\t")[-1].removeprefix("refs/tags/")
        number = ref.removeprefix(prefix)
        if ref.startswith(prefix) and number.isdigit():
            numbered.append((int(number), ref))
    return [ref for _, ref in sorted(numbered, reverse=True)]


def release_tag(release: str, branch: str) -> str | None:
    """Full tag of a release given as "5", "v0.0.5" or the full tag; None for "latest"."""
    if release == "latest":
        return None
    number = re.fullmatch(r"(?:v0\.0\.)?(\d+)", release)
    return tag_prefix(branch) + number.group(1) if number else release


def list_release_tags(branch: str) -> list[str]:
    # git ls-remote has no API rate limit, unlike api.github.com (60 requests/h unauthenticated)
    prefix = tag_prefix(branch)
    try:
        out = subprocess.run(
            ["git", "ls-remote", "--tags", "--refs", f"https://github.com/{REPO}.git", f"{prefix}*"],
            capture_output=True, text=True, timeout=NETWORK_TIMEOUT_S, check=True,
            env={**os.environ, "GIT_TERMINAL_PROMPT": "0"},
        ).stdout
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        raise Offline(f"can't list tags of {REPO}: {getattr(e, 'stderr', '') or e}".strip()) from e
    return parse_tags(out, prefix)


def asset_url(tag: str) -> str:
    return f"https://github.com/{REPO}/releases/download/{tag}/firmware-{tag}.bin"


def check_image(data: bytes) -> None:
    """Rejects anything that is not an STM32F4 image linked at FLASH_BASE (HTML error pages etc.)."""
    if not 8 <= len(data) <= FLASH_SIZE:
        raise ValueError(f"image size {len(data)} B is not plausible")
    sp, reset = struct.unpack_from("<II", data)
    if not RAM_RANGE[0] < sp <= RAM_RANGE[1]:
        raise ValueError(f"initial stack pointer 0x{sp:08x} is not in RAM")
    if not (FLASH_BASE <= reset < FLASH_BASE + len(data) and reset & 1):
        raise ValueError(f"reset vector 0x{reset:08x} is not a thumb address inside the image")


def download(tag: str) -> Path | None:
    """Returns the cached .bin of tag, downloading it if needed; None if the release has no .bin."""
    path = CACHE_DIR / f"firmware-{tag}.bin"
    if path.exists():
        return path
    try:
        with urllib.request.urlopen(asset_url(tag), timeout=NETWORK_TIMEOUT_S) as resp:
            data = resp.read()
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None
        raise Offline(f"downloading {tag}: {e}") from e
    except (urllib.error.URLError, TimeoutError) as e:
        raise Offline(f"downloading {tag}: {e}") from e
    check_image(data)
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".part")
    tmp.write_bytes(data)
    tmp.rename(path)  # never leave a partial file under the final name
    return path


def find_latest(branch: str) -> tuple[str, Path]:
    tags = list_release_tags(branch)
    if not tags:
        raise LookupError(f"no releases for branch {branch} (tags {tag_prefix(branch)}N) in {REPO}")
    for tag in tags[:MAX_TAGS_TRIED]:
        path = download(tag)
        if path is not None:
            return tag, path
        print(f"{tag}: no firmware .bin in the release (CI still running?), trying the previous one")
    raise LookupError(f"none of {tags[:MAX_TAGS_TRIED]} has a firmware .bin")


def st_flash(*args: str) -> None:
    subprocess.run(["st-flash", *args], check=True, timeout=120)


def mcu_has_image(image: Path) -> bool:
    with tempfile.TemporaryDirectory() as tmp:
        dump = Path(tmp) / "mcu.bin"
        size = image.stat().st_size
        st_flash("read", str(dump), hex(FLASH_BASE), str(size))
        return dump.read_bytes() == image.read_bytes()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--branch", default=BRANCH, help=f"firmware branch (default {BRANCH})")
    parser.add_argument(
        "--release", "--tag", default=RELEASE,
        help=f'"latest", a release number of the branch (5 or v0.0.5) or a full tag (default {RELEASE})',
    )
    parser.add_argument("--force", action="store_true", help="flash even if the MCU already has this image")
    parser.add_argument("--dry-run", action="store_true", help="find and download only")
    args = parser.parse_args(argv)

    try:
        tag = release_tag(args.release, args.branch)
        if tag is None:
            tag, image = find_latest(args.branch)
        else:
            image = download(tag)
            if image is None:
                raise LookupError(f"release {tag} has no firmware .bin")
    except Offline as e:
        print(f"offline, keeping the current firmware: {e}")
        return 0
    except (LookupError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"{tag}: {image} ({image.stat().st_size} B)")
    if args.dry_run:
        return 0

    try:
        if not args.force and mcu_has_image(image):
            print("MCU already runs this firmware")
            return 0
        print(f"flashing {tag}")
        # st-flash verifies the written flash itself and fails if it differs
        st_flash("--reset", "write", str(image), hex(FLASH_BASE))
    except FileNotFoundError:
        print("error: st-flash not found (apt install stlink-tools)", file=sys.stderr)
        return 1
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        print(f"error: {e} (is the ST-Link connected?)", file=sys.stderr)
        return 1
    print(f"flashed {tag}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
