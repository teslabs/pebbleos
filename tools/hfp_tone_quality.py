# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""Measure a known synthetic tone in 8 kHz, signed 16-bit H4 SCO captures."""

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np


def analyze(data, frequency=440.0):
    if not math.isfinite(frequency) or not 0 < frequency < 4000:
        raise ValueError("Tone frequency must be between 0 and 4000 Hz")
    samples, valid = [], []
    counts = [0] * 4
    handle = None
    offset = 0
    while offset < len(data):
        if len(data) - offset < 4 or data[offset] != 3:
            raise ValueError(f"Invalid H4 SCO header at byte {offset}")
        word, length = struct.unpack_from("<HB", data, offset + 1)
        if word & 0xC000 or not length or length % 2 or offset + 4 + length > len(data):
            raise ValueError(f"Invalid PCM packet at byte {offset}")
        if handle is not None and handle != word & 0xFFF:
            raise ValueError("Capture contains multiple SCO handles")
        handle = word & 0xFFF
        status = word >> 12
        counts[status] += 1
        samples.extend(struct.unpack_from(f"<{length // 2}h", data, offset + 4))
        valid.extend([status == 0] * (length // 2))
        offset += 4 + length
    if sum(valid) < 32:
        raise ValueError("Capture needs at least 32 valid PCM samples")
    samples = np.asarray(samples, dtype=float)
    valid = np.asarray(valid, dtype=bool)
    phase = np.arange(len(samples)) * (2 * np.pi * frequency / 8000)
    basis = np.column_stack((np.sin(phase), np.cos(phase), np.ones(len(samples))))
    coefficients, _, rank, _ = np.linalg.lstsq(basis[valid], samples[valid], rcond=None)
    if rank != 3:
        raise ValueError("Tone fit is underdetermined")
    tone = basis[:, :2] @ coefficients[:2]
    residual = samples - (tone + coefficients[2])

    def ratio(mask):
        signal = np.mean(tone[mask] ** 2)
        noise = np.mean(residual[mask] ** 2)
        if signal < 1:
            return None
        return round(float(10 * np.log10(signal / max(noise, 1e-12))), 2)

    return {
        "frequency_hz": frequency,
        "duration_ms": len(samples) / 8,
        "packets_by_status": counts,
        "bad_sample_percent": round(100 * (1 - float(np.mean(valid))), 3),
        "tone_peak": round(float(np.hypot(*coefficients[:2])), 2),
        "dc_offset": round(float(coefficients[2]), 2),
        "valid_tone_to_error_db": ratio(valid),
        "all_tone_to_error_db": ratio(np.ones(len(samples), dtype=bool)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--frequency", type=float, default=440.0)
    args = parser.parse_args()
    try:
        result = analyze(args.capture.read_bytes(), args.frequency)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
