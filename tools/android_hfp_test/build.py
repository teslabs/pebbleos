#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""Build a dependency-free local Telecom test APK with Android SDK command-line tools."""

import argparse
import os
import shutil
import subprocess
import zipfile
from pathlib import Path

SOURCE = Path(__file__).resolve().parent
ROOT = SOURCE.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sdk",
        type=Path,
        default=Path(
            os.environ.get("ANDROID_HOME", Path.home() / "Library/Android/sdk")
        ),
    )
    parser.add_argument(
        "--java-home",
        type=Path,
        default=Path(
            os.environ.get(
                "JAVA_HOME",
                "/Applications/Android Studio.app/Contents/jbr/Contents/Home",
            )
        ),
    )
    parser.add_argument("--output", type=Path, default=ROOT / "build-android-hfp-test")
    args = parser.parse_args()
    build = args.output.resolve()
    build.mkdir(parents=True, exist_ok=True)
    classes = build / "classes"
    if classes.exists():
        shutil.rmtree(classes)
    classes.mkdir()
    platform = args.sdk / "platforms/android-36/android.jar"
    tools = args.sdk / "build-tools/36.0.0"
    env = dict(os.environ, JAVA_HOME=str(args.java_home))

    def run(*command):
        subprocess.run([str(part) for part in command], env=env, check=True)

    run(
        args.java_home / "bin/javac",
        "--release",
        "8",
        "-classpath",
        platform,
        "-d",
        classes,
        *sorted((SOURCE / "src").rglob("*.java")),
    )
    run(
        tools / "d8",
        "--min-api",
        "31",
        "--lib",
        platform,
        "--output",
        build,
        *sorted(classes.rglob("*.class")),
    )
    unsigned = build / "unsigned.apk"
    run(
        tools / "aapt2",
        "link",
        "-I",
        platform,
        "--manifest",
        SOURCE / "AndroidManifest.xml",
        "-o",
        unsigned,
    )
    with zipfile.ZipFile(unsigned, "a", compression=zipfile.ZIP_DEFLATED) as apk:
        apk.write(build / "classes.dex", "classes.dex")
    aligned = build / "aligned.apk"
    run(tools / "zipalign", "-f", "4", unsigned, aligned)
    key = build / "debug.keystore"
    if not key.exists():
        run(
            args.java_home / "bin/keytool",
            "-genkeypair",
            "-keystore",
            key,
            "-storepass",
            "android",
            "-keypass",
            "android",
            "-alias",
            "debug",
            "-keyalg",
            "RSA",
            "-keysize",
            "2048",
            "-validity",
            "3650",
            "-dname",
            "CN=HFP Local Test",
        )
    output = build / "hfp-test.apk"
    run(
        tools / "apksigner",
        "sign",
        "--ks",
        key,
        "--ks-pass",
        "pass:android",
        "--out",
        output,
        aligned,
    )
    run(tools / "apksigner", "verify", output)
    print(output)


if __name__ == "__main__":
    main()
