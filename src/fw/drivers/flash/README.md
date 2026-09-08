Flash Memory Drivers
--------------------

The stack follows Linux MTD / spi-nor / spi-mem:

- `include/pbl/drivers/flash.h` is the storage API (the MTD equivalent). It
  operates on a `struct pbl_flash_device`; the generic layer (`flash.c`) owns
  locking, write protection, blank checks and the erase engine, while
  drivers implement `struct pbl_flash_ops`. After `pbl_flash_coredump_init()`
  the same API works without locks, timers or sleeping.
- `spi_nor.c` is the JEDEC protocol layer (spi-nor). It discovers the part
  through SFDP, falls back to the Kconfig-selected part table in
  `nor_part.h` for what SFDP does not describe (security registers,
  latencies), picks the widest read/program commands the bus supports, and
  talks to hardware only through `spi_mem`.
- `include/pbl/drivers/spi_mem.h` is the controller bus (spi-mem):
  command/address/dummy/data transactions, `supports_op()` for controllers
  with a fixed command set, and an optional direct-mapped read.

Controllers that sequence the flash protocol in hardware or in a vendor HAL
that must run from RAM (SF32 MPI, where the flash is also the XIP code
source) implement `pbl_flash_ops` directly, like Linux `spi-intel`. QEMU's
memory-mapped flash does the same.

Devices are instantiated by the drivers from Kconfig (`NRF5_QSPI_*`,
`FLASH_SF32LB52_MPI_*`, `FLASH_QEMU_*`, the `FLASH_NOR_PART` choice) and
exported as `FLASH`; a spi_mem controller exports the bus the NOR sits on as
`SPI_MEM_NOR`. On nRF5, `FLASH_SPI_NOR` selects the layered stack and
`FLASH_NRF5_QSPI` the direct driver.
