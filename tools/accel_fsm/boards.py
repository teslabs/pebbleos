# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Accelerometer axis mapping per board, as in fw/board/boards/board_*.c."""

import dataclasses


@dataclasses.dataclass(frozen=True)
class AxisConfig:
    # Sensor axis feeding each watch axis (x, y, z), and its sign
    axis_map: tuple
    axis_dir: tuple

    def to_sensor(self, sample):
        """Convert a watch-frame sample to the sensor frame."""
        out = [0, 0, 0]
        for watch_axis, sensor_axis in enumerate(self.axis_map):
            out[sensor_axis] = self.axis_dir[watch_axis] * sample[watch_axis]
        return tuple(out)

    def to_watch(self, sample):
        return tuple(
            self.axis_dir[watch_axis] * sample[sensor_axis]
            for watch_axis, sensor_axis in enumerate(self.axis_map)
        )


BOARDS = {
    "obelix": AxisConfig(axis_map=(0, 1, 2), axis_dir=(-1, 1, 1)),
    "asterix": AxisConfig(axis_map=(1, 0, 2), axis_dir=(1, 1, 1)),
    "getafix": AxisConfig(axis_map=(0, 1, 2), axis_dir=(-1, 1, 1)),
}

IDENTITY = AxisConfig(axis_map=(0, 1, 2), axis_dir=(1, 1, 1))


class BoardError(Exception):
    pass


def get(name):
    try:
        return BOARDS[name]
    except KeyError:
        raise BoardError(
            f"unknown board {name!r}, known: {', '.join(sorted(BOARDS))}"
        ) from None
