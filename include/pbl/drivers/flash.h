/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <pbl/device.h>
#include <pbl/kernel/mutex.h>
#include <pbl/kernel/sem.h>
#include <pbl/kernel/thread.h>
#include <pbl/services/new_timer/new_timer.h>

/**
 * @defgroup drivers_flash Flash
 * @ingroup drivers
 * @brief NOR flash class.
 *
 * A flash is a struct pbl_flash device. Addresses are absolute: they start at the device's
 * @c base, which is where the flash is memory-mapped on SoCs that map it. Writes only clear bits,
 * so the target range must be erased first. Erases work on whole subsectors: the class erases a
 * range with as few sector and subsector erases as it can, skipping the blank ones.
 *
 * @code{.c}
 * static void prv_erased(int status, void *arg) {
 *   // Runs on the timer task: keep it short
 * }
 *
 * ret = pbl_flash_erase(FLASH, addr, len);
 * if (ret != 0) {
 *   return ret;
 * }
 *
 * ret = pbl_flash_write(FLASH, addr, data, sizeof(data));
 * if (ret != 0) {
 *   return ret;
 * }
 *
 * ret = pbl_flash_erase_async(FLASH, other_addr, other_len, prv_erased, nullptr);
 * @endcode
 *
 * Calls are serialized per device and must not be issued from ISRs. A read or write issued while
 * an erase is in progress suspends the erase, if the driver can, and the class resumes it shortly
 * after. Only one erase runs at a time: starting another one blocks until the first is done.
 *
 * Calls return 0 or a negative errno: @c -EINVAL for a range outside the flash or an unaligned
 * erase, @c -EACCES for a write or erase in the protected range and @c -ENODEV if the flash is
 * not ready. A hardware failure while reading, writing or erasing is fatal.
 *
 * After pbl_flash_coredump_init() the class runs without OS services: no locking, no timers, no
 * sleeping, and erases are synchronous. That is not reversible.
 * @{
 */

struct pbl_flash;

/**
 * @brief Completion callback of pbl_flash_erase_async().
 *
 * Runs from the timer task, or from the caller if there is nothing to erase, with no lock held:
 * it may start another erase.
 *
 * @param status 0 or a negative errno.
 * @param arg Callback argument.
 */
typedef void (*pbl_flash_erase_cb_t)(int status, void *arg);

/** @brief Flash geometry. */
struct pbl_flash_geometry {
  /** Size, in bytes. */
  uint32_t size;
  /** Program page size, in bytes. */
  uint32_t page_size;
  /** Large erase unit, in bytes. */
  uint32_t sector_size;
  /** Small erase unit, in bytes. */
  uint32_t subsector_size;
  /** Typical sector erase time, to pace erase polling. */
  uint16_t sector_erase_ms;
  /** Typical subsector erase time, to pace erase polling. */
  uint16_t subsector_erase_ms;
};

/** @brief One-time programmable security registers. */
struct pbl_flash_sec_regs {
  /** Address of each register. */
  const uint32_t *addrs;
  /** Number of registers. */
  uint8_t count;
  /** Size of each register, in bytes. */
  uint16_t size;
};

/**
 * @brief Flash driver operations.
 *
 * Called with the device lock held, or without any OS service in coredump mode, which drivers
 * check with pbl_flash_in_coredump(): they must then poll instead of blocking.
 */
struct pbl_flash_ops {
  /**
   * Optional. Bring the hardware up. Also called by pbl_flash_coredump_init(), possibly after a
   * previous initialization. Returns 0 or a negative errno.
   */
  int (*init)(const struct pbl_flash *flash);
  /** Read @p len bytes. Returns 0 or a negative errno. */
  int (*read)(const struct pbl_flash *flash, uint32_t addr, void *buf, size_t len);
  /** Program @p len bytes, returning once they are programmed. Returns 0 or a negative errno. */
  int (*write)(const struct pbl_flash *flash, uint32_t addr, const void *buf, size_t len);
  /**
   * Start erasing the sector or subsector of @p size bytes at @p addr. Returns 0 or a negative
   * errno. Without @c erase_status, returns once the erase is done.
   */
  int (*erase)(const struct pbl_flash *flash, uint32_t addr, uint32_t size);
  /**
   * Optional. Returns 0 once the erase is done, @c -EBUSY while it is in progress, @c -EAGAIN
   * while it is suspended, or another negative errno if it failed.
   */
  int (*erase_status)(const struct pbl_flash *flash);
  /**
   * Optional, with @c erase_status. Suspend the erase in progress. Returns 0, 1 if the erase had
   * already completed, or a negative errno.
   */
  int (*erase_suspend)(const struct pbl_flash *flash);
  /** With @c erase_suspend. Resume the suspended erase. Returns 0 or a negative errno. */
  int (*erase_resume)(const struct pbl_flash *flash);
  /** Optional. Power down before MCU stop mode, with interrupts disabled. */
  void (*power_down)(const struct pbl_flash *flash);
  /** Optional. Power up after MCU stop mode, with interrupts disabled. */
  void (*power_up)(const struct pbl_flash *flash);
  /** With @c FLASH_SEC_REGS. Read a security register byte. */
  int (*sec_reg_read)(const struct pbl_flash *flash, uint32_t addr, uint8_t *val);
  /** With @c FLASH_SEC_REGS. Program a security register byte. */
  int (*sec_reg_write)(const struct pbl_flash *flash, uint32_t addr, uint8_t val);
  /** With @c FLASH_SEC_REGS. Erase the security register containing @p addr. */
  int (*sec_reg_erase)(const struct pbl_flash *flash, uint32_t addr);
  /** With @c FLASH_SEC_REGS. Check whether the register containing @p addr is locked. */
  int (*sec_reg_is_locked)(const struct pbl_flash *flash, uint32_t addr, bool *locked);
  /** Optional. Lock the register containing @p addr forever. */
  int (*sec_reg_lock)(const struct pbl_flash *flash, uint32_t addr);
};

/** @cond INTERNAL_HIDDEN */
struct pbl_flash_state {
  struct pbl_mutex lock;
  struct pbl_sem erase_sem;
  bool coredump;
  struct {
    bool enabled;
    uint32_t start;
    uint32_t end;
  } protect;
  struct {
    bool in_progress;
    bool suspended;
    bool done;
    uint8_t retries;
    uint32_t next;
    uint32_t end;
    uint32_t unit;
    uint32_t expected_ms;
    struct pbl_thread *thread;
    pbl_flash_erase_cb_t cb;
    void *arg;
  } erase;
  TimerID poll_timer;
  TimerID resume_timer;
};

extern const struct pbl_device_class pbl_flash_class;
int pbl_flash_dev_init(const struct pbl_device *dev);
/** @endcond */

/** @brief A flash. */
struct pbl_flash {
  /** Device. */
  struct pbl_device dev;
  /** Driver operations. */
  const struct pbl_flash_ops *ops;
  /** Runtime state. */
  struct pbl_flash_state *state;
  /** Address of the first byte. */
  uint32_t base;
  /** Geometry. */
  const struct pbl_flash_geometry *geometry;
  /** Security registers, or NULL. */
  const struct pbl_flash_sec_regs *sec_regs;
};

/** @brief The board's storage flash. */
extern const struct pbl_flash *const FLASH;

/**
 * @brief Define the class state of flash @p sym, for the @c PBL_FLASH_*_DEFINE() macro of a
 * driver.
 *
 * @param sym Symbol of the flash instance.
 */
#define PBL_FLASH_STATE_DEFINE(sym)                   \
  PBL_DEVICE_STATE_DEFINE(sym);                       \
  static struct pbl_flash_state sym##_flash_state = { \
    .lock = PBL_MUTEX_INITIALIZER,                    \
    .erase_sem = PBL_SEM_INITIALIZER(1, 1),           \
  }

/**
 * @brief Initializer for the struct pbl_flash of flash @p sym.
 *
 * @param sym Symbol of the flash instance, with its state defined by PBL_FLASH_STATE_DEFINE().
 * @param _name Name.
 * @param _parent Parent device, e.g. the memory controller, or NULL.
 * @param _deps Dependencies from PBL_DEVICE_DEPS(), or NULL.
 * @param _ops Driver operations.
 * @param _base Address of the first byte.
 * @param _geometry Geometry.
 * @param _sec_regs Security registers, or NULL.
 */
#define PBL_FLASH_INIT(sym, _name, _parent, _deps, _ops, _base, _geometry, _sec_regs)            \
  {                                                                                              \
    .dev =                                                                                       \
        PBL_DEVICE_INIT_CLASS(sym, _name, pbl_flash_dev_init, _parent, _deps, &pbl_flash_class), \
    .ops = (_ops),                                                                               \
    .state = &sym##_flash_state,                                                                 \
    .base = (_base),                                                                             \
    .geometry = (_geometry),                                                                     \
    .sec_regs = (_sec_regs),                                                                     \
  }

/**
 * @brief Check whether the flash runs in coredump mode, for drivers.
 *
 * @param flash Flash.
 * @return True after pbl_flash_coredump_init().
 */
static inline bool pbl_flash_in_coredump(const struct pbl_flash *flash) {
  return flash->state->coredump;
}

/**
 * @brief Read data.
 *
 * @param flash Flash.
 * @param addr Address.
 * @param[out] buf Buffer.
 * @param len Number of bytes.
 * @return 0 or a negative errno.
 */
int pbl_flash_read(const struct pbl_flash *flash, uint32_t addr, void *buf, size_t len);

/**
 * @brief Program data into an erased range.
 *
 * @param flash Flash.
 * @param addr Address.
 * @param buf Data.
 * @param len Number of bytes.
 * @return 0 or a negative errno.
 */
int pbl_flash_write(const struct pbl_flash *flash, uint32_t addr, const void *buf, size_t len);

/**
 * @brief Erase a range.
 *
 * Feeds the caller's task watchdog while it waits, unless a single erase unit takes unreasonably
 * long.
 *
 * @param flash Flash.
 * @param addr Address, aligned to a subsector.
 * @param len Number of bytes, rounded up to a subsector.
 * @return 0 or a negative errno.
 */
int pbl_flash_erase(const struct pbl_flash *flash, uint32_t addr, size_t len);

/**
 * @brief Erase a range, returning once the erase has started.
 *
 * @param flash Flash.
 * @param addr Address, aligned to a subsector.
 * @param len Number of bytes, rounded up to a subsector.
 * @param cb Completion callback.
 * @param arg Callback argument.
 * @return 0 if @p cb will be called, or a negative errno.
 */
int pbl_flash_erase_async(const struct pbl_flash *flash, uint32_t addr, size_t len,
                          pbl_flash_erase_cb_t cb, void *arg);

/**
 * @brief Check whether a range is erased.
 *
 * @param flash Flash.
 * @param addr Address.
 * @param len Number of bytes.
 * @return True if the flash is ready and every byte reads as erased.
 */
bool pbl_flash_is_erased(const struct pbl_flash *flash, uint32_t addr, size_t len);

/**
 * @brief Refuse writes and erases in a range, replacing the previous one.
 *
 * @param flash Flash.
 * @param addr Address.
 * @param len Number of bytes.
 * @return 0 or a negative errno.
 */
int pbl_flash_protect(const struct pbl_flash *flash, uint32_t addr, size_t len);

/**
 * @brief Allow writes and erases everywhere.
 *
 * @param flash Flash.
 * @return 0 or a negative errno.
 */
int pbl_flash_unprotect(const struct pbl_flash *flash);

/**
 * @brief Wait for the erase in progress, if any, before a reset.
 *
 * @param flash Flash.
 */
void pbl_flash_stop(const struct pbl_flash *flash);

/**
 * @brief Re-initialize the flash for use from a fault handler.
 *
 * @param flash Flash.
 * @return 0 or a negative errno.
 */
int pbl_flash_coredump_init(const struct pbl_flash *flash);

/**
 * @brief Power the flash down before MCU stop mode, with interrupts disabled.
 *
 * @param flash Flash.
 */
void pbl_flash_power_down(const struct pbl_flash *flash);

/**
 * @brief Power the flash up after MCU stop mode, with interrupts disabled.
 *
 * @param flash Flash.
 */
void pbl_flash_power_up(const struct pbl_flash *flash);

/**
 * @brief CRC-32 of a range.
 *
 * @param flash Flash.
 * @param addr Address.
 * @param len Number of bytes.
 * @return CRC-32.
 */
uint32_t pbl_flash_crc32(const struct pbl_flash *flash, uint32_t addr, size_t len);

/**
 * @brief Legacy checksum of a range, as used by firmware images and resources.
 *
 * @param flash Flash.
 * @param addr Address.
 * @param len Number of bytes.
 * @return Checksum.
 */
uint32_t pbl_flash_crc32_legacy(const struct pbl_flash *flash, uint32_t addr, size_t len);

/**
 * @name Security registers
 *
 * Provided by drivers that select @c FLASH_SEC_REGS, at the addresses of their
 * struct pbl_flash_sec_regs.
 * @{
 */

/**
 * @brief Read a security register byte.
 *
 * @param flash Flash.
 * @param addr Address.
 * @param[out] val Value.
 * @return 0 or a negative errno.
 */
int pbl_flash_sec_reg_read(const struct pbl_flash *flash, uint32_t addr, uint8_t *val);

/**
 * @brief Program a security register byte.
 *
 * @param flash Flash.
 * @param addr Address.
 * @param val Value.
 * @return 0 or a negative errno.
 */
int pbl_flash_sec_reg_write(const struct pbl_flash *flash, uint32_t addr, uint8_t val);

/**
 * @brief Erase the security register containing an address.
 *
 * @param flash Flash.
 * @param addr Address.
 * @return 0 or a negative errno.
 */
int pbl_flash_sec_reg_erase(const struct pbl_flash *flash, uint32_t addr);

/**
 * @brief Check whether the security register containing an address is locked.
 *
 * @param flash Flash.
 * @param addr Address.
 * @param[out] locked Lock state.
 * @return 0 or a negative errno.
 */
int pbl_flash_sec_reg_is_locked(const struct pbl_flash *flash, uint32_t addr, bool *locked);

/**
 * @brief Lock the security register containing an address forever.
 *
 * Only the recovery firmware's drivers lock registers.
 *
 * @param flash Flash.
 * @param addr Address.
 * @return 0, @c -ENOTSUP if locking is not available, or another negative errno.
 */
int pbl_flash_sec_reg_lock(const struct pbl_flash *flash, uint32_t addr);

/** @} */

/** @} */
