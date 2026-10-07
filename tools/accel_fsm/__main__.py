# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Record raw accelerometer data on a watch.

python -m tools.accel_fsm record start walk --url /dev/cu.usbserial-XXXX
python -m tools.accel_fsm record pull -o recordings/ --url ...
python -m tools.accel_fsm sim flick.fsm recordings/*.bin --odr 26
"""

import argparse
import datetime
import os
import pathlib
import sys

from . import recording


def _device(args):
    from .device import Device

    url = args.url or os.environ.get("ACCEL_FSM_URL")
    if not url:
        sys.exit("pass --url (tty or socket://host:port) or set ACCEL_FSM_URL")
    return Device(url)


def cmd_record(args):
    with _device(args) as dev:
        if args.action == "start":
            line = f"accelrec start {args.label} {args.rate} {args.max_kib}"
        elif args.action == "remove":
            line = f"accelrec remove {args.name}"
        elif args.action == "pull":
            names = args.names or dev.list()
            out = pathlib.Path(args.output)
            out.mkdir(parents=True, exist_ok=True)
            for name in names:
                data = dev.pull(name)
                rec = recording.parse(data)
                stamp = datetime.datetime.fromtimestamp(
                    rec.start_time, datetime.timezone.utc
                )
                path = out / f"{stamp:%Y%m%d-%H%M%S}-{rec.label or name}.bin"
                path.write_bytes(data)
                print(
                    f"{name}: {len(rec.samples)} samples, {rec.duration_s:.1f} s -> {path}"
                )
            return
        else:
            line = f"accelrec {args.action}"
        for out_line in dev.command(line):
            print(out_line)


def cmd_info(args):
    for path in args.recordings:
        rec = recording.load(path)
        samples = rec.samples
        print(f"{path}:")
        print(f"  label {rec.label!r}, board {rec.board!r}, {rec.rate_hz:.2f} Hz")
        print(
            f"  {len(samples)} samples, {rec.duration_s:.1f} s, complete {rec.complete}"
        )
        stamp = datetime.datetime.fromtimestamp(rec.start_time, datetime.timezone.utc)
        print(f"  started {stamp:%Y-%m-%d %H:%M:%S} UTC")
        for index, missing in rec.gaps():
            print(f"  gap at sample {index}: {missing:+d} samples")
        for interval in rec.rate_changes():
            print(f"  rate changed to {1e6 / interval:.2f} Hz in some chunks")
        if samples:
            for axis, name in enumerate("xyz"):
                values = [s[axis] for s in samples]
                print(f"  {name}: min {min(values)} max {max(values)} mg")


def cmd_export(args):
    rec = recording.load(args.recording)
    with open(args.output, "w") as f:
        f.write("t_s,x_mg,y_mg,z_mg\n")
        f.writelines(
            f"{i * rec.interval_us / 1e6:.4f},{x},{y},{z}\n"
            for i, (x, y, z) in enumerate(rec.samples)
        )


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="accel_fsm", description=__doc__.split("\n")[0]
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    url = argparse.ArgumentParser(add_help=False)
    url.add_argument("--url", help="PULSE tty or socket://host:port (or ACCEL_FSM_URL)")
    p = sub.add_parser("record", help="control the on-watch recorder")
    rsub = p.add_subparsers(dest="action", required=True)
    s = rsub.add_parser("start", parents=[url])
    s.add_argument("label", help="activity label, e.g. walk or flick")
    s.add_argument("--rate", type=int, default=52, help="sampling rate in Hz")
    s.add_argument("--max-kib", type=int, default=1024, help="file size limit")
    rsub.add_parser("stop", parents=[url])
    rsub.add_parser("status", parents=[url])
    rsub.add_parser("list", parents=[url])
    s = rsub.add_parser("pull", parents=[url])
    s.add_argument("names", nargs="*", help="recordings to pull (default: all)")
    s.add_argument("-o", "--output", default=".", help="output directory")
    s = rsub.add_parser("remove", parents=[url])
    s.add_argument("name", help="recording name, or all")
    p.set_defaults(func=cmd_record)

    p = sub.add_parser("info", help="describe recordings")
    p.add_argument("recordings", nargs="+")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("export", help="convert a recording to CSV")
    p.add_argument("recording")
    p.add_argument("-o", "--output", required=True)
    p.set_defaults(func=cmd_export)

    args = parser.parse_args(argv)
    try:
        args.func(args)
    except recording.RecordingError as e:
        sys.exit(f"error: {e}")


if __name__ == "__main__":
    main()
