#!/bin/sh
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Compiler wrapper that names the source file being compiled.
#
# Asserts and log messages report the file they came from. __FILE_NAME__
# would name the header a macro was expanded in, so every object has to be
# built with the name of its own source instead -- a per-source compile
# definition, which Meson has no way to express. The whole compiler
# command line is passed through with the definition appended; GCC accepts
# -D anywhere on it.

name=
for arg in "$@"; do
  case "$arg" in
    *.c|*.S|*.s) name=${arg##*/} ;;
  esac
done

if [ -n "$name" ]; then
  exec "$@" "-D__FILE_NAME_LEGACY__=\"$name\""
fi
exec "$@"
