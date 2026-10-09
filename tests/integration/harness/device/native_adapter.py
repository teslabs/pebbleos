# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import os
import signal
import socket
import subprocess

from harness.device import DeviceAdapter
from harness.errors import HarnessError

FLASH_IMAGE = "native_flash.bin"


class NativeAdapter(DeviceAdapter):
    """The firmware as a native host process, headless, on a fresh flash.
    It serves its console and the QEMU serial protocol on TCP sockets the
    harness binds and hands down, and connects its Bluetooth HCI UART to a
    controller."""

    type = "native"

    def __init__(self, config):
        super().__init__(config)
        self.workdir = config.results_dir
        self.console_port = None
        self.pebble_port = None
        self._listeners = []
        self._process = None
        self._log = None
        self._bt_hci = None

    def _command_line(self):
        return [
            self.build.executable,
            "-f", os.path.join(self.workdir, FLASH_IMAGE),
            "-c", f"fd:{self._listeners[0].fileno()}",
            "-p", f"fd:{self._listeners[1].fileno()}",
            *(["-b", str(self._bt_hci)] if self._bt_hci else []),
        ]  # fmt: skip

    def _start(self):
        command = self._command_line()
        self._log.write(" ".join(command) + "\n")
        self._log.flush()
        self._process = subprocess.Popen(
            command,
            cwd=self.workdir,
            env={**os.environ, "SDL_VIDEODRIVER": "dummy"},
            stdin=subprocess.DEVNULL,
            stdout=self._log,
            stderr=subprocess.STDOUT,
            pass_fds=[listener.fileno() for listener in self._listeners],
        )

    def connect(self):
        # The sockets queue connections whether the firmware runs or not.
        if self._process is not None and self._process.poll() is not None:
            raise HarnessError(
                f"the firmware exited with {self._process.returncode}; see "
                f"{os.path.join(self.workdir, 'native.log')}"
            )
        return super().connect()

    def _listen(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        self._listeners.append(listener)
        return listener.getsockname()[1]

    def _stop(self):
        if self._process is None:
            return
        if self._process.poll() is None:
            self._process.send_signal(signal.SIGTERM)
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
        self._process = None

    def _device_launch(self):
        if not os.path.isfile(self.build.executable):
            raise HarnessError(f"no {self.build.executable} -- run 'pbl build' first")
        os.makedirs(self.workdir, exist_ok=True)
        flash = os.path.join(self.workdir, FLASH_IMAGE)
        if os.path.exists(flash):
            os.unlink(flash)
        self._bt_hci = self._start_bluetooth()
        self.console_port = self._listen()
        self.pebble_port = self._listen()
        self._log = open(os.path.join(self.workdir, "native.log"), "a")  # noqa: SIM115
        self._start()

    def _close_device(self):
        self._stop()
        while self._listeners:
            self._listeners.pop().close()
        if self._log is not None:
            self._log.close()
            self._log = None
        self._stop_bluetooth()

    def _hard_reset(self):
        self._stop()
        self._power_cycle_bluetooth()
        self._start()
        return True

    def default_connections(self):
        console = f"serial:socket://127.0.0.1:{self.console_port}"
        # With a Bluetooth controller, the phone holds the protocol session.
        if self.build.config.get("CONFIG_BT_HCI_UART"):
            return [console]
        return [console, f"qemu:127.0.0.1:{self.pebble_port}"]
