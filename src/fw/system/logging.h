/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "console/dbgserial.h"
#include "system/die.h"
#include "system/reboot_reason.h"
#include "system/status_codes.h"


#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define SPLIT_64_BIT_ARG(x) (uint32_t)((x >> 32) & 0xFFFFFFFF), (uint32_t)(x & 0xFFFFFFFF)

#define LOG_BUFFER_LENGTH 128

// Minimum amount of stack space required for vsniprintf
#define LOGGING_MIN_STACK_FOR_SPRINTF  240
#define LOGGING_STACK_FULL_MSG  ((const char *)" [STK FULL]")

void pbl_log_hashed_async(const char *module, const uint32_t packed_loghash, ...);

void pbl_log_hashed_sync(const char *module, const uint32_t packed_loghash, ...);

// Core Number must be shifted to the correct position.
void pbl_log_hashed_core(const uint32_t core_number, const char *module,
                         const uint32_t packed_loghash, ...);

// Core Number must be shifted to the correct position.
void pbl_log_hashed_vargs(const bool async, const uint32_t core_number, const char *module,
                          const uint32_t packed_loghash, va_list fmt_args);

void pbl_log_vargs(uint8_t log_level, const char *module, const char *fmt, va_list args);

void pbl_log(uint8_t log_level, const char *module, const char *fmt, ...)
    FORMAT_PRINTF(3, 4);

void pbl_log_sync(uint8_t log_level, const char *module, const char *fmt, ...)
    FORMAT_PRINTF(3, 4);

int pbl_log_binary_format(char *buffer, int buffer_len, const uint8_t log_level,
                          const char *module, const char *fmt, va_list args);

int pbl_log_get_bin_format(char *buffer, int buffer_len, const uint8_t log_level,
                           const char *module, const char *fmt, ...);

#define LOG_LEVEL_ALWAYS 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARNING 50
#define LOG_LEVEL_INFO 100
#define LOG_LEVEL_DEBUG 200
#define LOG_LEVEL_DEBUG_VERBOSE 255

#ifdef PBL_LOGS_HASHED
  #include <logging/log_hashing.h>
#endif

#if MEMFAULT && defined(PBL_LOGS_HASHED) && __has_include("memfault/core/log.h")
  #include "memfault/core/log.h"

  #define PBL_TO_MFLT_LOG_LEVEL(level) \
    ((level) <= LOG_LEVEL_ERROR ? kMemfaultPlatformLogLevel_Error : \
     (level) <= LOG_LEVEL_WARNING ? kMemfaultPlatformLogLevel_Warning : \
     (level) <= LOG_LEVEL_INFO ? kMemfaultPlatformLogLevel_Info : \
     kMemfaultPlatformLogLevel_Debug)

  // Memfault compact_log_helpers.h uses (arg)+0 in _Generic which triggers
  // -Wpointer-arith when log arguments include function pointers. Suppress
  // this warning around the SDK macro expansion.
  #define PBL_MFLT_LOG_SAVE(level, fmt, ...) \
    do { \
      _Pragma("GCC diagnostic push") \
      _Pragma("GCC diagnostic ignored \"-Wpointer-arith\"") \
      MEMFAULT_COMPACT_LOG_SAVE(PBL_TO_MFLT_LOG_LEVEL(level), fmt, ## __VA_ARGS__); \
      _Pragma("GCC diagnostic pop") \
    } while (0)
#else
  #define PBL_MFLT_LOG_SAVE(level, fmt, ...) ((void)0)
#endif

#define LOG_COLOR_BLACK          "BLACK"        // Not so useful in general
#define LOG_COLOR_RED            "RED"
#define LOG_COLOR_GREEN          "GREEN"
#define LOG_COLOR_YELLOW         "YELLOW"
#define LOG_COLOR_BLUE           "BLUE"
#define LOG_COLOR_MAGENTA        "MAGENTA"
#define LOG_COLOR_CYAN           "CYAN"
#define LOG_COLOR_GREY           "GREY"
// Reserved for bold. Use sparingly
#define LOG_COLOR_LIGHT_GREY     "LIGHT_GREY"
#define LOG_COLOR_LIGHT_RED      "LIGHT_RED"
#define LOG_COLOR_LIGHT_GREEN    "LIGHT_GREEN"
#define LOG_COLOR_LIGHT_YELLOW   "LIGHT_YELLOW"
#define LOG_COLOR_LIGHT_BLUE     "LIGHT_BLUE"
#define LOG_COLOR_LIGHT_MAGENTA  "LIGHT_MAGENTA"
#define LOG_COLOR_LIGHT_CYAN     "LIGHT_CYAN"
#define LOG_COLOR_WHITE          "WHITE"

// Level-to-color mapping (fixed per level)
#define LOG_COLOR_FOR_ALWAYS  LOG_COLOR_BLUE
#define LOG_COLOR_FOR_ERROR   LOG_COLOR_RED
#define LOG_COLOR_FOR_WARNING LOG_COLOR_YELLOW
#define LOG_COLOR_FOR_INFO    LOG_COLOR_GREEN
#define LOG_COLOR_FOR_DEBUG   LOG_COLOR_GREY
#define LOG_COLOR_FOR_VERBOSE LOG_COLOR_GREY

#ifndef STRINGIFY
  #define STRINGIFY_NX(a) #a
  #define STRINGIFY(a) STRINGIFY_NX(a)
#endif // STRINGIFY

#define STATUS_STRING(s) STRINGIFY(s)

// Per-module logging.
//
// Every translation unit that calls a PBL_LOG_* macro must register or declare
// a module. REGISTER defines the module's runtime name (exactly one TU per
// module, otherwise the linker reports a multiple-definition error). DECLARE
// references an already-registered module from another TU, allowing a logical
// module to span multiple files. Each TU sets its own compile-time level.
//
// Usage:
//   PBL_LOG_MODULE_REGISTER(bt, LOG_LEVEL_DEBUG);  // owner TU
//   PBL_LOG_MODULE_DECLARE(bt,  LOG_LEVEL_INFO);   // companion TU
//
// `name` is an unquoted identifier, lower-case by convention. The TU-local
// `__pbl_log_module_level` is initialized from a literal so PBL_SHOULD_LOG()
// constant-folds and dead branches are eliminated (including hashed-log
// .log_strings entries for filtered-out levels).

#define PBL_LOG_MODULE_REGISTER(name, level)                                   \
  const char *const __pbl_log_module_##name##_name                             \
      __attribute__((used, section(".log_modules"))) = #name;                  \
  static const char *const __pbl_log_module_name __attribute__((unused))       \
      = __pbl_log_module_##name##_name;                                        \
  static const int __pbl_log_module_level __attribute__((unused))              \
      = (level)

#define PBL_LOG_MODULE_DECLARE(name, level)                                    \
  extern const char *const __pbl_log_module_##name##_name;                     \
  static const char *const __pbl_log_module_name __attribute__((unused))       \
      = __pbl_log_module_##name##_name;                                        \
  static const int __pbl_log_module_level __attribute__((unused))              \
      = (level)

#define PBL_SHOULD_LOG(level) ((level) <= __pbl_log_module_level)


// Internal implementation macros (use level-named macros below instead)
#ifdef PBL_LOG_ENABLED
  #ifdef PBL_LOGS_HASHED
    #define PBL_LOG_COLOR(level, color, fmt, ...) \
      do { \
        if (PBL_SHOULD_LOG(level)) { \
          NEW_LOG_HASH(pbl_log_hashed_async, level, color, fmt, ## __VA_ARGS__); \
          PBL_MFLT_LOG_SAVE(level, fmt, ## __VA_ARGS__); \
        } \
      } while (0)

    #define PBL_LOG_COLOR_SYNC(level, color, fmt, ...) \
      do { \
        if (PBL_SHOULD_LOG(level)) { \
          NEW_LOG_HASH(pbl_log_hashed_sync, level, color, fmt, ## __VA_ARGS__); \
        } \
      } while (0)
  #else
    #define PBL_LOG_COLOR(level, color, fmt, ...) \
      do { \
        if (PBL_SHOULD_LOG(level)) { \
          pbl_log(level, __pbl_log_module_name, fmt, ## __VA_ARGS__); \
        } \
      } while (0)

    #define PBL_LOG_COLOR_SYNC(level, color, fmt, ...) \
      do { \
        if (PBL_SHOULD_LOG(level)) { \
          pbl_log_sync(level, __pbl_log_module_name, fmt, ## __VA_ARGS__); \
        } \
      } while (0)
  #endif
#else // !PBL_LOG_ENABLED
  #define PBL_LOG_COLOR(level, color, fmt, ...)
  #define PBL_LOG_COLOR_SYNC(level, color, fmt, ...)
#endif // PBL_LOG_ENABLED

// Level-named macros (async)
#define PBL_LOG_ALWAYS(fmt, ...) \
  PBL_LOG_COLOR(LOG_LEVEL_ALWAYS, LOG_COLOR_FOR_ALWAYS, fmt, ## __VA_ARGS__)
#define PBL_LOG_ERR(fmt, ...) \
  PBL_LOG_COLOR(LOG_LEVEL_ERROR, LOG_COLOR_FOR_ERROR, fmt, ## __VA_ARGS__)
#define PBL_LOG_WRN(fmt, ...) \
  PBL_LOG_COLOR(LOG_LEVEL_WARNING, LOG_COLOR_FOR_WARNING, fmt, ## __VA_ARGS__)
#define PBL_LOG_INFO(fmt, ...) \
  PBL_LOG_COLOR(LOG_LEVEL_INFO, LOG_COLOR_FOR_INFO, fmt, ## __VA_ARGS__)
#define PBL_LOG_DBG(fmt, ...) \
  PBL_LOG_COLOR(LOG_LEVEL_DEBUG, LOG_COLOR_FOR_DEBUG, fmt, ## __VA_ARGS__)
#define PBL_LOG_VERBOSE(fmt, ...) \
  PBL_LOG_COLOR(LOG_LEVEL_DEBUG_VERBOSE, LOG_COLOR_FOR_VERBOSE, fmt, ## __VA_ARGS__)

// Level-named sync macros
#define PBL_LOG_SYNC_ALWAYS(fmt, ...) \
  PBL_LOG_COLOR_SYNC(LOG_LEVEL_ALWAYS, LOG_COLOR_FOR_ALWAYS, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_ERR(fmt, ...) \
  PBL_LOG_COLOR_SYNC(LOG_LEVEL_ERROR, LOG_COLOR_FOR_ERROR, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_WRN(fmt, ...) \
  PBL_LOG_COLOR_SYNC(LOG_LEVEL_WARNING, LOG_COLOR_FOR_WARNING, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_INFO(fmt, ...) \
  PBL_LOG_COLOR_SYNC(LOG_LEVEL_INFO, LOG_COLOR_FOR_INFO, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_DBG(fmt, ...) \
  PBL_LOG_COLOR_SYNC(LOG_LEVEL_DEBUG, LOG_COLOR_FOR_DEBUG, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_VERBOSE(fmt, ...) \
  PBL_LOG_COLOR_SYNC(LOG_LEVEL_DEBUG_VERBOSE, LOG_COLOR_FOR_VERBOSE, fmt, ## __VA_ARGS__)

#ifdef PBL_LOG_ENABLED
  #define RETURN_STATUS(st) \
    do { \
      if (FAILED(st)) { \
        PBL_LOG_WRN("%d", (int)(st)); \
      } \
      return st; \
    } while (0)

  #define RETURN_STATUS_UP(st) \
    return ((st) != E_INVALID_ARGUMENT ? (st) : E_INTERNAL)
#else // PBL_LOG_ENABLED
  #define RETURN_STATUS(st) return (st)
  #define RETURN_STATUS_UP(st) \
    return ((st) == E_INVALID_ARGUMENT ? E_INTERNAL : (st))
#endif // PBL_LOG_ENABLED
