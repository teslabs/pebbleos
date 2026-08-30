# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

# The Pebble protocol endpoint table: which endpoint id goes to which
# handler, on which task.
endpoints_args = []
if cfg.get('CONFIG_RECOVERY_FW', false)
  endpoints_args += '--recovery'
endif

pbl_generated += custom_target(
  'protocol_endpoints_table.auto.h',
  output: 'protocol_endpoints_table.auto.h',
  input: files('protocol_endpoints_table.json'),
  command: [
    py, generate_py, 'endpoints-table',
    '--input', '@INPUT@', '--output', '@OUTPUT@',
    endpoints_args,
  ],
)

sources = [
  'default_kernel_receiver.c',
  'default_kernel_sender.c',
  'meta_endpoint.c',
  'session.c',
  'session_analytics.c',
  'session_receive_router.c',
  'session_remote_version.c',
  'session_send_queue.c',
]
if not cfg.get('CONFIG_RECOVERY_FW', false)
  sources += ['app_session_capabilities.c']
endif

pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]
