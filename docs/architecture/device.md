# Device model

`include/pbl/device.h` defines `struct pbl_device`, the common core of every
hardware device the firmware drives: an on-chip controller such as a GPIO
port or an I2C bus, a peripheral behind a bus such as a PMIC, or a controller
a peripheral exposes in turn, such as the GPIOs of that PMIC. The core is
implemented in `subsys/device/`.

## Structure

The model follows the Linux kernel: a device class embeds `struct pbl_device`
in its own struct, a driver embeds the class struct in a driver-specific one,
and `PBL_CONTAINER_OF` walks back out.

```
struct pbl_device            name, init, deps, state          include/pbl/device.h
  struct pbl_gpio_port       class: ops vtable                include/pbl/drivers/gpio.h
    struct pbl_gpio_nrf5     driver: registers, port index    include/pbl/drivers/gpio/nrf5.h
```

Devices are `const` and live in flash. The only RAM is what the device points
to: `struct pbl_device_state` for the core, plus whatever the class or driver
needs (an I2C bus points to its transfer state, for example).

Hardware relationships are captured in C. A pin is a `struct pbl_gpio`, a
port plus pin number plus flags, whether the port is an SoC one
(`PBL_GPIO(NRF_GPIO_P0, 2, 0)`) or the nPM1300's
(`PBL_GPIO(NPM1300_GPIO, 2, PBL_GPIO_ACTIVE_LOW)`); an I2C peripheral is a
`struct pbl_i2c_dev`, a bus plus address. A board file instantiates devices with the driver's
`PBL_*_DEFINE` macro and points the peripherals at them; SoC-fixed devices,
such as GPIO ports, are instantiated by the SoC driver itself. Nothing is
generated from a markup description today, but the instance definitions are
regular enough that a generator could emit them later.

## Instantiation

A driver provides a `PBL_<DRIVER>_DEFINE(sym, ...)` macro built from three
core pieces:

- `PBL_DEVICE_STATE_DEFINE(sym)` allocates the runtime state;
- `PBL_DEVICE_INIT(sym, name, init, parent, deps)` initialises the embedded
  `struct pbl_device`;
- `PBL_DEVICE_REGISTER(sym, &sym.<class>.dev)` puts a pointer to the device
  in the `.pbl_devices` linker section (`src/fw/linker/sections/devices.ld`).

Devices that are not registered (a pin, an I2C address) are plain structs
with no init of their own.

## Initialisation

`pbl_device_init_all()`, called from `init_drivers()` in `src/fw/main.c`,
walks the device table and calls `pbl_device_init()` on each entry. Ordering
is by dependency, not by table position:

- a device's `parent` is initialised first: the I2C bus a sensor sits on, or
  the multi-function device (a PMIC) whose GPIO port or regulator it is. A
  parent may also bring its children up from its own init, as an MFD does,
  and a child initialised that way does not re-enter its parent;
- a driver's `init` calls `pbl_device_init()` on the other devices it needs
  (an I2C bus initialises the GPIO ports of its pins), so structural
  dependencies never have to be written down twice;
- a board adds anything else, such as a power rail that has to be up first,
  through `PBL_DEVICE_DEPS()` in the instance definition.

`pbl_device_init()` is idempotent, and a device is initialised exactly once,
however many others depend on it. A failed init marks the device
`PBL_DEVICE_FAILED`; dependents fail with `-ENODEV` without running their
own init. A dependency cycle asserts.

Runtime APIs assert `pbl_device_is_ready()` where a stale reference would
otherwise fail confusingly, and never initialise on demand: bring-up is
deterministic and happens in one place.

## Peripherals

A sensor or other peripheral embeds `struct pbl_device` in its config struct
(`LSM6DSOConfig`, `TouchSensor`, `struct pbl_mmc5603nj`, ...), with the bus it
sits on as `parent` and its supply as a dep, and the board instantiates it:

```c
PBL_DEVICE_STATE_DEFINE(s_bmp390);
static const struct pbl_bmp390 s_bmp390 = {
    .dev = PBL_DEVICE_INIT(s_bmp390, "bmp390", bmp390_init, &s_i2c_iic2.bus.dev,
                           PBL_DEVICE_DEPS(&s_pmic.dev)),
    .i2c = PBL_I2C_DEV(&s_i2c_iic2.bus, 0x76),
};
PBL_DEVICE_REGISTER(s_bmp390, &s_bmp390.dev);
```

The driver's init probes and configures the part and returns a negative errno
on failure, which the core logs; nothing in `main.c` has to know the sensor
exists. The subsystem APIs on top (`accel_*`, `mag_*`, `ambient_light_*`,
`touch_sensor_*`) are still singletons implemented by the one driver a board
compiles in; giving them ops vtables is the natural next step.

## Multi-function devices

A chip with several unrelated functions, such as the nPM1300 PMIC (charger,
GPIOs, regulators), is one parent device plus one child device per function,
as in the Linux MFD layer. The parent (`struct pbl_npm1300`) owns the bus
access and a lock, and exposes register read/write/update helpers that every
function goes through, so a read-modify-write in one function cannot
interleave with another. The children embed their class struct (`struct
pbl_gpio_port`, `struct pbl_regulator`) with `parent` pointing at the PMIC,
and the parent's init ends with `pbl_device_init_children()`, so a device that
depends on the PMIC can rely on its rails and pins being configured.

Board-level facts live in the board: each rail is a
`PBL_NPM1300_REGULATOR_DEFINE` with its voltage, mode and whether it is
always on, and consumers reference the regulator (`MicDevice.vdd`) and
enable it through the `struct pbl_regulator` use count rather than through
PMIC-specific calls.

## Adding a driver

1. Define the driver struct embedding the class struct, its `ops`
   implementation and its `init`, which recovers the driver struct with
   `PBL_CONTAINER_OF(dev, const struct <driver>, <class>.dev)`.
2. Provide the `PBL_<DRIVER>_DEFINE` macro.
3. Instantiate in the board (or SoC) file; consumers take a pointer to the
   class struct and never see the driver struct.
