Flash Memory Drivers
--------------------

`include/pbl/drivers/flash.h` is the only flash API. It operates on a
`struct pbl_flash_device`, which boards define and export as `FLASH`. The
generic layer (`flash.c`) owns locking, write protection, blank checks and the
erase engine; drivers implement `struct pbl_flash_ops` for a controller and
never use OS services beyond what the generic layer hands them.

After `pbl_flash_coredump_init()` the same API works without locks, timers or
sleeping so that core dumps can be written from a fault handler.
