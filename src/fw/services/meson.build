# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_SERVICE_ACCEL_MANAGER', false)
  subdir('accel_manager')
endif
if cfg.get('CONFIG_SERVICE_ACTIVITY', false)
  subdir('activity')
endif
if cfg.get('CONFIG_SERVICE_ALARMS', false)
  subdir('alarms')
endif
if cfg.get('CONFIG_SERVICE_ANALYTICS', false)
  subdir('analytics')
endif
if cfg.get('CONFIG_SERVICE_ANIMATION_SERVICE', false)
  subdir('animation_service')
endif
if cfg.get('CONFIG_SERVICE_APP_CACHE', false)
  subdir('app_cache')
endif
if cfg.get('CONFIG_SERVICE_APP_FETCH_ENDPOINT', false)
  subdir('app_fetch_endpoint')
endif
if cfg.get('CONFIG_SERVICE_APP_GLANCES', false)
  subdir('app_glances')
endif
if cfg.get('CONFIG_SERVICE_APP_INBOX_SERVICE', false)
  subdir('app_inbox_service')
endif
if cfg.get('CONFIG_SERVICE_APP_MESSAGE', false)
  subdir('app_message')
endif
if cfg.get('CONFIG_SERVICE_APP_ORDER_ENDPOINT', false)
  subdir('app_order_endpoint')
endif
if cfg.get('CONFIG_SERVICE_APP_OUTBOX_SERVICE', false)
  subdir('app_outbox_service')
endif
if cfg.get('CONFIG_SERVICE_AUDIO_ENDPOINT', false)
  subdir('audio_endpoint')
endif
if cfg.get('CONFIG_SERVICE_BATTERY', false)
  subdir('battery')
endif
if cfg.get('CONFIG_SERVICE_BLOB_DB', false)
  subdir('blob_db')
endif
if cfg.get('CONFIG_SERVICE_BLUETOOTH', false)
  subdir('bluetooth')
endif
if cfg.get('CONFIG_SERVICE_BOOT_SPLASH', false)
  subdir('boot_splash')
endif
if cfg.get('CONFIG_SERVICE_CLOCK', false)
  subdir('clock')
endif
if cfg.get('CONFIG_SERVICE_COMM_SESSION', false)
  subdir('comm_session')
endif
if cfg.get('CONFIG_SERVICE_COMPOSITOR', false)
  subdir('compositor')
endif
if cfg.get('CONFIG_SERVICE_CONTACTS', false)
  subdir('contacts')
endif
if cfg.get('CONFIG_SERVICE_CRON', false)
  subdir('cron')
endif
if cfg.get('CONFIG_SERVICE_DATA_LOGGING', false)
  subdir('data_logging')
endif
if cfg.get('CONFIG_SERVICE_DEBOUNCED_CONNECTION_SERVICE', false)
  subdir('debounced_connection_service')
endif
if cfg.get('CONFIG_SERVICE_ECOMPASS', false)
  subdir('ecompass')
endif
if cfg.get('CONFIG_SERVICE_EVENT_SERVICE', false)
  subdir('event_service')
endif
if cfg.get('CONFIG_SERVICE_EVENTED_TIMER', false)
  subdir('evented_timer')
endif
if cfg.get('CONFIG_SERVICE_FILESYSTEM', false)
  subdir('filesystem')
endif
if cfg.get('CONFIG_SERVICE_FIRMWARE_UPDATE', false)
  subdir('firmware_update')
endif
if cfg.get('CONFIG_SERVICE_GET_BYTES', false)
  subdir('get_bytes')
endif
if cfg.get('CONFIG_SERVICE_HEALTH_SYNC_ENDPOINT', false)
  subdir('health_sync_endpoint')
endif
if cfg.get('CONFIG_SERVICE_HRM', false)
  subdir('hrm')
endif
if cfg.get('CONFIG_SERVICE_I18N', false)
  subdir('i18n')
endif
subdir('imaging')
if cfg.get('CONFIG_SERVICE_IDLE_WATCHDOG', false)
  subdir('idle_watchdog')
endif
if cfg.get('CONFIG_SERVICE_LIGHT', false)
  subdir('light')
endif
if cfg.get('CONFIG_SERVICE_MUSIC', false)
  subdir('music')
endif
if cfg.get('CONFIG_SERVICE_NEW_TIMER', false)
  subdir('new_timer')
endif
if cfg.get('CONFIG_SERVICE_NOTIFICATIONS', false)
  subdir('notifications')
endif
if cfg.get('CONFIG_SERVICE_ORIENTATION_MANAGER', false)
  subdir('orientation_manager')
endif
if cfg.get('CONFIG_SERVICE_PERSIST', false)
  subdir('persist')
endif
if cfg.get('CONFIG_SERVICE_PHONE_CALL', false)
  subdir('phone_call')
endif
if cfg.get('CONFIG_SERVICE_PHONE_PP', false)
  subdir('phone_pp')
endif
if cfg.get('CONFIG_SERVICE_PING', false)
  subdir('ping')
endif
if cfg.get('CONFIG_SERVICE_POLL_REMOTE', false)
  subdir('poll_remote')
endif
if cfg.get('CONFIG_SERVICE_PRF_UPDATE', false)
  subdir('prf_update')
endif
if cfg.get('CONFIG_SERVICE_PROCESS_MANAGEMENT', false)
  subdir('process_management')
endif
if cfg.get('CONFIG_SERVICE_PROTOBUF_LOG', false)
  subdir('protobuf_log')
endif
if cfg.get('CONFIG_SERVICE_PUT_BYTES', false)
  subdir('put_bytes')
endif
if cfg.get('CONFIG_SERVICE_REGISTRY_ENDPOINT', false)
  subdir('registry_endpoint')
endif
if cfg.get('CONFIG_SERVICE_REGULAR_TIMER', false)
  subdir('regular_timer')
endif
if cfg.get('CONFIG_SERVICE_SEND_TEXT_SERVICE', false)
  subdir('send_text_service')
endif
if cfg.get('CONFIG_SERVICE_SERVICES_COMMON', false)
  subdir('services_common')
endif
if cfg.get('CONFIG_SERVICE_SERVICES_NORMAL', false)
  subdir('services_normal')
endif
if cfg.get('CONFIG_SERVICE_SETTINGS', false)
  subdir('settings')
endif
if cfg.get('CONFIG_SERVICE_SHARED_PRF_STORAGE', false)
  subdir('shared_prf_storage')
endif
if cfg.get('CONFIG_SERVICE_SPEAKER', false)
  subdir('speaker')
endif
if cfg.get('CONFIG_SERVICE_STATIONARY', false)
  subdir('stationary')
endif
if cfg.get('CONFIG_SERVICE_SYSTEM_TASK', false)
  subdir('system_task')
endif
if cfg.get('CONFIG_SERVICE_TICK_TIMER', false)
  subdir('tick_timer')
endif
if cfg.get('CONFIG_SERVICE_TIMELINE', false)
  subdir('timeline')
endif
if cfg.get('CONFIG_SERVICE_TIMEZONE_DATABASE', false)
  subdir('timezone_database')
endif
if cfg.get('CONFIG_SERVICE_TOUCH', false)
  subdir('touch')
endif
if cfg.get('CONFIG_SERVICE_VIBE_PATTERN', false)
  subdir('vibe_pattern')
endif
if cfg.get('CONFIG_SERVICE_VIBES', false)
  subdir('vibes')
endif
if cfg.get('CONFIG_SERVICE_VOICE', false)
  subdir('voice')
endif
if cfg.get('CONFIG_SERVICE_VOICE_ENDPOINT', false)
  subdir('voice_endpoint')
endif
if cfg.get('CONFIG_SERVICE_WAKEUP', false)
  subdir('wakeup')
endif
if cfg.get('CONFIG_SERVICE_WEATHER', false)
  subdir('weather')
endif


# Every service that has translatable strings contributes a catalog.
if pbl_services_pot.length() > 0
  services_pot = custom_target(
    'services.pot',
    output: 'services.pot',
    input: pbl_services_pot,
    command: [msgcat, '@INPUT@', '-o', '@OUTPUT@'],
  )
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files('services.c')}]
