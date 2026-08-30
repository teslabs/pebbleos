# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

subdir('cmsis_core')
subdir('freertos')
subdir('nanopb')
subdir('tinymt')
subdir('qr_code_generator')
if cfg.get('CONFIG_SPEEX', false)
  subdir('speex')
endif
subdir('nonfree')
if cfg.get('CONFIG_MODDABLE_XS', false)
  subdir('moddable')
endif
if cfg.get('CONFIG_HAL_NORDIC', false)
  subdir('hal_nordic')
endif
if cfg.get('CONFIG_HAL_SIFLI', false)
  subdir('hal_sifli')
endif
if cfg.get('CONFIG_MBEDTLS', false)
  subdir('mbedtls')
endif
if cfg.get('CONFIG_BT_FW_NIMBLE', false)
  subdir('nimble')
endif
