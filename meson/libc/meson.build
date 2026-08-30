# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Kconfig-driven C library selection (see lib/c/Kconfig). Sets
# pbl_libc_link_args and pbl_libc_libs for the firmware link, and adds the
# flags that make the toolchain libc match what the firmware expects:
# 32-bit time_t, the integer-only printf aliases, and the POSIX/BSD names
# that -std=c11 would otherwise hide.

# newlib defaults time_t to 64-bit; keep it 32-bit (long) to match the
# firmware's RTC/storage and its %ld format strings.
newlib_time_define = '-D_USE_LONG_TIME_T'
# The firmware calls the integer-only printf names; map them onto the real
# ones (the historical in-tree libc aliased them in its headers).
sniprintf_defines = ['-Dsniprintf=snprintf', '-Dvsniprintf=vsnprintf']
default_source_define = '-D_DEFAULT_SOURCE'

if cfg.get('CONFIG_LIBC_NEWLIB', false) or cfg.get('CONFIG_LIBC_NEWLIB_NANO', false)
  if cfg.get('CONFIG_LIBC_NEWLIB_NANO', false)
    pbl_libc_link_args = ['-nostartfiles', '-specs=nano.specs', '-specs=nosys.specs']
    pbl_libc_name = 'newlib-nano'
  else
    pbl_libc_link_args = ['-nostartfiles', '-specs=nosys.specs']
    pbl_libc_name = 'newlib'
  endif
  libc_args = [newlib_time_define, default_source_define] + sniprintf_defines
elif cfg.get('CONFIG_LIBC_PICOLIBC', false)
  picolibc_specs = ''
  if not cfg.get('CONFIG_LIBC_PICOLIBC_FROM_SOURCE', false)
    # gcc echoes the name back unchanged when it cannot find the file.
    found = run_command(
      cc.cmd_array() + ['-print-file-name=picolibc.specs'],
      check: false,
    ).stdout().strip()
    if found != '' and found != 'picolibc.specs' and fs.exists(found)
      picolibc_specs = found
    else
      warning(
        'No pre-built picolibc in the toolchain (not using the PebbleOS SDK '
        + 'toolchain?); building picolibc from source (third_party/picolibc).',
      )
    endif
  endif

  if picolibc_specs != ''
    pbl_libc_name = 'picolibc (pre-built)'
    libc_args = [default_source_define] + sniprintf_defines
  else
    # Build third_party/picolibc for the current architecture. picolibc is
    # itself a Meson project, but it needs a cross file of its own (no
    # crt0, no semihosting, a 32-bit time_t), so it is configured and built
    # here rather than pulled in as a subproject.
    picolibc_src = pbl_base / 'third_party/picolibc'
    if not fs.exists(picolibc_src / 'meson.build')
      error(
        'third_party/picolibc submodule missing; run '
        + '`git submodule update --init third_party/picolibc`',
      )
    endif
    meson_prog = find_program('meson')
    ninja_prog = find_program('ninja')

    picolibc_out = meson.current_build_dir()
    picolibc_build = picolibc_out / 'build'
    picolibc_install = picolibc_out / 'install'

    # picolibc hardcodes a 64-bit __time_t. The firmware wants 32-bit, to
    # match its RTC/storage and %ld formats, so the same machine override
    # is force included into both the picolibc build and the firmware;
    # that keeps the ABI of the time_t-taking functions consistent.
    picolibc_override = picolibc_out / 'picolibc_machine_override.h'

    picolibc_arch = []
    foreach flag : pbl_arch_args
      if flag.startswith('-mthumb') or flag.startswith('-mcpu=') or flag.startswith('-mfloat-abi=') or flag.startswith('-mfpu=') or flag.startswith('-march=')
        picolibc_arch += flag
      endif
    endforeach

    run_command(
      py, files('picolibc.py'), 'setup',
      '--source', picolibc_src,
      '--build', picolibc_build,
      '--install', picolibc_install,
      '--override', picolibc_override,
      '--cross', picolibc_out / 'cross.txt',
      '--meson', meson_prog.full_path(),
      '--ninja', ninja_prog.full_path(),
      '--cc', cc.cmd_array(),
      '--arch', picolibc_arch,
      check: true,
    )
    picolibc_specs = picolibc_install / 'lib/picolibc.specs'
    pbl_libc_name = 'picolibc (from source)'
    # The firmware needs the same 32-bit time_t override picolibc built with.
    libc_args = ['-include', picolibc_override, default_source_define] + sniprintf_defines
  endif

  pbl_libc_link_args = ['-nostartfiles']
  # picolibc.specs supplies both the headers (cpp -isystem) and the libc
  # link, so it belongs on the compile and the link line.
  libc_args += '-specs=' + picolibc_specs
else
  error('No C library selected (see lib/c/Kconfig)')
endif

pbl_libc_libs = ['-lm', '-lgcc']
pbl_c_args += libc_args
pbl_link_args += libc_args
