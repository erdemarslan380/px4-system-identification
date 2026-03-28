#!/usr/bin/env python3

"""Upload CubeOrange firmware with a safer bootloader write size.

The upstream PX4 uploader works for many boards as-is, but on this CubeOrange
setup the default PROG_MULTI chunk size can fail late in the programming phase
with "Bootloader reports OPERATION FAILED". Reducing the chunk size to 128
bytes makes the upload reliable on the attached hardware.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path


DEFAULT_WORKSPACE = Path.home() / "PX4-Autopilot-Identification"
DEFAULT_FIRMWARE = (
    DEFAULT_WORKSPACE
    / "build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4"
)
DEFAULT_UPLOADER = DEFAULT_WORKSPACE / "Tools/px4_uploader.py"


def load_uploader_module(uploader_path: Path):
    spec = importlib.util.spec_from_file_location("px4_uploader_custom", uploader_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load uploader module from {uploader_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        default=str(DEFAULT_WORKSPACE),
        help="PX4 workspace root. Default: %(default)s",
    )
    parser.add_argument(
        "--firmware",
        default=str(DEFAULT_FIRMWARE),
        help="Firmware .px4 file to upload. Default: %(default)s",
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="CubeOrange USB bootloader port. Default: %(default)s",
    )
    parser.add_argument(
        "--baud-flightstack",
        default="57600",
        help="Flight stack baud used for reboot commands. Default: %(default)s",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=128,
        help="Bootloader PROG_MULTI chunk size. Default: %(default)s",
    )
    parser.add_argument(
        "--force-erase",
        action="store_true",
        default=True,
        help="Request a full chip erase before programming.",
    )
    parser.add_argument(
        "--no-force-erase",
        action="store_false",
        dest="force_erase",
        help="Do not request a full chip erase.",
    )
    parser.add_argument(
        "--noninteractive",
        action="store_true",
        help="Use uploader noninteractive progress output.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workspace = Path(args.workspace).expanduser()
    firmware = Path(args.firmware).expanduser()
    uploader_path = workspace / "Tools/px4_uploader.py"

    if not uploader_path.exists():
        raise SystemExit(f"Uploader not found: {uploader_path}")

    if not firmware.exists():
        raise SystemExit(f"Firmware not found: {firmware}")

    uploader = load_uploader_module(uploader_path)
    uploader.ProtocolConfig.PROG_MULTI_MAX = args.chunk_size

    argv = [
        str(uploader_path),
        "--port",
        args.port,
        "--baud-flightstack",
        args.baud_flightstack,
    ]

    if args.force_erase:
        argv.append("--force-erase")

    if args.noninteractive:
        argv.append("--noninteractive")

    argv.append(str(firmware))
    sys.argv = argv
    return uploader.main()


if __name__ == "__main__":
    raise SystemExit(main())
