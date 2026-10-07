# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import pytest
from harness.helpers.pairing import WatchPairing


@pytest.fixture
def watch(dut, prompt):
    """The watch with no phone paired and no pairing prompt up, so that it
    takes a new pairing; left that way."""
    pairing = WatchPairing(dut)
    pairing.reset()
    yield pairing
    pairing.reset()
