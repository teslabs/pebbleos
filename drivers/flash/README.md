Flash Drivers
-------------

Flash is a device class, `struct pbl_flash` in `include/pbl/drivers/flash.h`.
The class core in `flash.c` serializes access, enforces the protected range,
runs erases (blocking or asynchronous, suspended around reads and writes) and
has a coredump mode that relies on no OS service. Drivers only move data and
start erases through `struct pbl_flash_ops`.

The board defines its flash with the `PBL_FLASH_*_DEFINE()` macro of its driver
and exports it as `FLASH`. NOR parts are described in `nor.h` and defined in
their own file.
