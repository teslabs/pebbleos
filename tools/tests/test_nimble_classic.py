# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""NimBLE's actual Classic controller extension, with OS and transport substitutes."""

import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NIMBLE = ROOT / "third_party/nimble/mynewt-nimble/nimble"


class NimbleClassicTest(unittest.TestCase):
    def test_controller_accounting_and_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "classic"
            subprocess.run(
                [
                    "cc",
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-Wno-unused-parameter",
                    "-DCONFIG_NIMBLE_BLE_CLASSIC=1",
                    "-fsanitize=address,undefined",
                    "-g",
                    "-I" + str(ROOT / "third_party/nimble/port/include"),
                    "-I" + str(NIMBLE / "host/include"),
                    "-I" + str(NIMBLE / "transport/include"),
                    str(Path(__file__).with_name("nimble_classic_harness.c")),
                    "-o",
                    str(executable),
                ],
                check=True,
            )
            cases = (
                "shared pool",
                "separate pools",
                "mixed completions",
                "excess completion",
                "malformed input",
                "reset and reuse",
                "handle dispatch",
                "LE-only controller",
                "cross-transport handle collision",
                "failed connection has undefined link type",
            )
            for index, case in enumerate(cases):
                with self.subTest(case=case):
                    subprocess.run([str(executable), str(index)], check=True)
