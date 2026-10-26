# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os
import time

from pbl import emulator
from pbl.command import CommandContextError, PblCommand


class _QemuCommand(PblCommand):
    group = "emulator"

    def emulated_build(self):
        """The build directory, refusing boards QEMU cannot run."""
        build = self.build_dir()
        if not build.config.CONFIG_QEMU:
            raise CommandContextError(
                f"board {build.board} is not an emulated board"
            )
        return build


class Screenshot(_QemuCommand):
    def __init__(self):
        super().__init__("screenshot", "Capture the emulator's screen to a PNG")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument(
            "-o",
            "--output",
            type=os.path.abspath,
            help="Where to write the PNG (default: screenshot.png in the build)",
        )
        return parser

    def do_run(self, args, unknown):
        build = self.emulated_build()
        output = args.output or build.join("screenshot.png")
        if not output.lower().endswith(".png"):
            self.die("the screenshot's output path must end in .png")
        if os.path.exists(output):
            os.unlink(output)

        with emulator.Monitor(build.join(emulator.MONITOR_SOCKET)) as monitor:
            response = monitor.command(f"screendump {output} -f png")

        if not os.path.exists(output) or os.path.getsize(output) == 0:
            self.die(
                f"QEMU wrote no screenshot to {output}\nmonitor response:\n{response}"
            )
        self.inf("wrote", output)


class _InputCommand(_QemuCommand):
    """Injects touch events into the running emulator."""

    def qmp(self, build):
        return emulator.Qmp(build.join(emulator.QMP_SOCKET))


class Touch(_InputCommand):
    def __init__(self):
        super().__init__("touch", "Tap the emulator's screen at a pixel")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument("x", type=int, help="x coordinate in screen pixels")
        parser.add_argument("y", type=int, help="y coordinate in screen pixels")
        return parser

    def do_run(self, args, unknown):
        build = self.emulated_build()
        with self.qmp(build) as qmp:
            width, height = qmp.touch_display()
            qmp.send(qmp.move(width, height, args.x, args.y) + qmp.button(True))
            time.sleep(0.05)
            qmp.send(qmp.button(False))
        self.inf(f"tapped ({args.x}, {args.y}) on {width}x{height}")


class Swipe(_InputCommand):
    def __init__(self):
        super().__init__("swipe", "Swipe across the emulator's screen")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument("x1", type=int, help="start x in screen pixels")
        parser.add_argument("y1", type=int, help="start y in screen pixels")
        parser.add_argument("x2", type=int, help="end x in screen pixels")
        parser.add_argument("y2", type=int, help="end y in screen pixels")
        parser.add_argument(
            "--steps", type=int, default=12, help="number of move steps"
        )
        parser.add_argument(
            "--duration", type=float, default=0.25, help="total duration in seconds"
        )
        return parser

    def do_run(self, args, unknown):
        build = self.emulated_build()
        x1, y1, x2, y2 = args.x1, args.y1, args.x2, args.y2
        steps = max(1, args.steps)

        with self.qmp(build) as qmp:
            width, height = qmp.touch_display()
            qmp.send(qmp.move(width, height, x1, y1) + qmp.button(True))
            for step in range(1, steps + 1):
                qmp.send(
                    qmp.move(
                        width,
                        height,
                        x1 + (x2 - x1) * step // steps,
                        y1 + (y2 - y1) * step // steps,
                    )
                )
                time.sleep(args.duration / steps)
            qmp.send(qmp.button(False))
        self.inf(f"swiped ({x1}, {y1}) -> ({x2}, {y2}) on {width}x{height}")
