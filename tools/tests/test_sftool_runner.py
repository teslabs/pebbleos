# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import shlex
import unittest
from unittest.mock import patch

from pbl.runners.core import RunnerConfig, RunnerError
from pbl.runners.sftool import SfToolRunner


class SfToolRunnerTest(unittest.TestCase):
    def runner(self):
        return SfToolRunner(
            RunnerConfig(
                board_dir="boards/obelix",
                soc="SF32LB52",
                hex_file="/tmp/firmware image.hex",
                resources_file="/tmp/system resources.pbpack",
                sftool="/tmp/tool directory/sftool",
            ),
            tty="/dev/test port",
        )

    def test_flash_failure_is_not_reported_as_success(self):
        for command in ("flash", "erase"):
            with (
                self.subTest(command=command),
                patch("pbl.runners.core.subprocess.call", return_value=101),
                self.assertRaisesRegex(RunnerError, "exit status 101"),
            ):
                self.runner().run(command)

    def test_flash_paths_remain_separate_arguments(self):
        with patch("pbl.runners.core.subprocess.call", return_value=0) as call:
            self.runner().run("flash")
        self.assertEqual(
            shlex.split(call.call_args.args[0]),
            [
                "/tmp/tool directory/sftool",
                "-c",
                "SF32LB52",
                "-p",
                "/dev/test port",
                "write_flash",
                "/tmp/firmware image.hex",
                "/tmp/system resources.pbpack@0x12620000",
            ],
        )


if __name__ == "__main__":
    unittest.main()
