# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

# files() only reaches the directory it is called from, so the shared QEMU
# and stub implementations the per-peripheral directories pull in are
# handed to them from here.
driver_shared_srcs = {}
foreach rel : run_command(
  py, glob_py, '--base', meson.current_source_dir(), 'qemu/*.c', 'stubs/*.c',
  check: true,
).stdout().strip().split('\n')
  driver_shared_srcs += {rel: files(rel)}
endforeach

foreach driver : [
  ['CONFIG_SPEAKER', 'speaker'],
  ['CONFIG_AMBIENT_LIGHT', 'ambient'],
  ['CONFIG_TEMPERATURE', 'temperature'],
  ['CONFIG_BATTERY', 'battery'],
  ['', 'imu'],
  ['CONFIG_DISPLAY', 'display'],
  ['CONFIG_EXTI', 'exti'],
  ['CONFIG_FLASH', 'flash'],
  ['CONFIG_GPIO', 'gpio'],
  ['CONFIG_I2C', 'i2c'],
  ['CONFIG_BACKLIGHT', 'backlight'],
  ['CONFIG_PRESSURE', 'pressure'],
  ['CONFIG_MIC', 'mic'],
  ['CONFIG_OTP', 'otp'],
  ['CONFIG_PMIC', 'pmic'],
  ['CONFIG_PWM', 'pwm'],
  ['CONFIG_RNG', 'rng'],
  ['CONFIG_RTC', 'rtc'],
  ['CONFIG_TOUCH', 'touch'],
  ['CONFIG_UART', 'uart'],
  ['CONFIG_VIBE', 'vibe'],
  ['CONFIG_WATCHDOG', 'watchdog'],
  ['CONFIG_HRM', 'hrm'],
]
  if driver[0] == '' or cfg.get(driver[0], false)
    subdir(driver[1])
  endif
endforeach

if cfg.get('CONFIG_MPU_TYPE_ARMV8M', false)
  mpu_arch_source = 'mpu/mpu_armv8m.c'
else
  mpu_arch_source = 'mpu/mpu_armv7m.c'
endif

# The drivers this directory builds itself, one library each: a name, and
# the sources that go into it.
drivers = [
  ['driver_mpu', ['mpu.c', mpu_arch_source]],
  ['driver_task_watchdog', ['task_watchdog.c']],
]

if cfg.get('CONFIG_QEMU', false)
  drivers += [
    ['driver_button', ['qemu/qemu_button.c']],
    ['driver_mcu', ['stubs/mcu.c']],
    ['driver_qemu_serial', ['qemu/qemu_serial.c', 'qemu/qemu_serial_util.c']],
    ['driver_qemu_settings', ['qemu/qemu_settings.c']],
    [
      'driver_qemu_stubs',
      ['stubs/ambient_light.c', 'qemu/qemu_vibe.c', 'stubs/otp.c'],
    ],
  ]
  if cfg.get('CONFIG_MAG', false)
    drivers += [['driver_mag3110', ['imu/mag3110/mag3110.c']]]
  endif
elif cfg.get('CONFIG_SOC_NRF52', false)
  drivers += [
    ['driver_button', ['nrf5/button.c', 'nrf5/debounced_button.c']],
    ['driver_mcu', ['stubs/mcu.c']],
    ['driver_qspi', ['nrf5/qspi.c']],
    ['driver_nrf5_hfxo', ['nrf5/hfxo.c']],
  ]
elif cfg.get('CONFIG_SOC_SF32LB52', false)
  drivers += [
    ['driver_button', ['sf32lb52/button.c', 'sf32lb52/debounced_button.c']],
    ['driver_mcu', ['sf32lb52/mcu.c']],
    ['driver_qspi', ['sf32lb52/qspi.c']],
    ['driver_rc10k', ['sf32lb52/rc10k.c']],
  ]
endif

foreach driver : drivers
  pbl_libs += [
    {
      'name': driver[0],
      'kind': 'objects',
      'srcs': files(driver[1]),
      'inc': [],
      'args': [],
      'defs': [],
    },
  ]
endforeach
