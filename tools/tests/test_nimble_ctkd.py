# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""CTKD negotiation and Bluetooth Core Vol 3, Part H, D.9/D.10 vectors."""

import ctypes
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NIMBLE = ROOT / "third_party/nimble/mynewt-nimble/nimble/host"
MBEDTLS = ROOT / "third_party/mbedtls/mbedtls"


class CtkdTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        build = Path(cls.directory.name)
        (build / "crypto_config.h").write_text(
            "#define MBEDTLS_AES_C\n#define MBEDTLS_CIPHER_C\n#define MBEDTLS_CMAC_C\n"
        )
        library = build / "ctkd.so"
        subprocess.run(
            [
                "cc",
                "-std=c11",
                "-shared",
                "-fPIC",
                "-Wall",
                "-Wextra",
                "-Werror",
                '-DMBEDTLS_CONFIG_FILE="crypto_config.h"',
                "-I" + str(build),
                "-I" + str(ROOT / "third_party/nimble/port/include/sf32lb52"),
                "-I" + str(NIMBLE / "include"),
                "-I" + str(MBEDTLS / "include"),
                "-I" + str(MBEDTLS / "library"),
                str(NIMBLE / "src/ble_sm_ctkd.c"),
                *(
                    str(MBEDTLS / "library" / f"{name}.c")
                    for name in (
                        "aes",
                        "cipher",
                        "cipher_wrap",
                        "cmac",
                        "platform_util",
                        "constant_time",
                    )
                ),
                "-o",
                str(library),
            ],
            check=True,
        )
        cls.lib = ctypes.CDLL(str(library))

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_specification_vectors(self):
        ltk = bytes.fromhex("368df9bce3264b58bd066c33334fbf64")[::-1]
        for ct2, expected in (
            (True, "287ad379dca402530a39f1f43047b835"),
            (False, "bc1ca4ef633fc1bd0d8230afee388fb0"),
        ):
            with self.subTest(ct2=ct2):
                output = ctypes.create_string_buffer(16)
                self.assertEqual(self.lib.ble_sm_ctkd_derive(ltk, ct2, output), 0)
                self.assertEqual(output.raw[::-1].hex(), expected)

    def test_negotiation(self):
        pair = bytes([1, 0, 0x2D, 16, 0x0B, 0x0B])
        self.assertEqual(self.lib.ble_sm_ctkd_pairing(pair, pair), 3)
        for index, bit in ((2, 1), (2, 8), (4, 8), (5, 8)):
            modified = bytearray(pair)
            modified[index] &= ~bit
            for a, b in ((pair, bytes(modified)), (bytes(modified), pair)):
                self.assertEqual(self.lib.ble_sm_ctkd_pairing(a, b), 0)
        for key_size in (0, 7, 15, 17):
            modified = pair[:3] + bytes([key_size]) + pair[4:]
            self.assertEqual(self.lib.ble_sm_ctkd_pairing(pair, modified), 0)
            self.assertEqual(self.lib.ble_sm_ctkd_pairing(modified, pair), 0)
        legacy_ctkd = pair[:2] + bytes([0x0D]) + pair[3:]
        self.assertEqual(self.lib.ble_sm_ctkd_pairing(pair, legacy_ctkd), 1)
        self.assertEqual(self.lib.ble_sm_ctkd_pairing(legacy_ctkd, pair), 1)
