# Build system

The firmware is built with [Meson](https://mesonbuild.com), following a
model borrowed from Zephyr:

```shell
./pbl configure --board asterix
./pbl build
```

`./pbl configure` writes the cross file naming the toolchain, runs
`meson setup` into `build/`, and passes Kconfig overrides through:

```shell
./pbl configure --board obelix@pvt --variant prf -DCONFIG_RELEASE=y
```

Meson has to know its compiler before it reads `meson.build`, and a cross
file can neither look one up nor refer to the source tree, so `./pbl`
generates `build/cross.txt` first. Plain Meson works once that file
exists:

```shell
python3 tools/meson/cross.py --output build/cross.txt
meson setup build --cross-file build/cross.txt -Dboard=asterix
ninja -C build
```

The unit tests still build with waf, out of their own build directory; see
{doc}`testing`.

## The library model

Each directory that contributes code appends one entry to `pbl_libs`:

```meson
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files('service.c')}]
```

- Every registered library ends up in the single firmware image, so
  nothing has to list them. Dependencies between firmware libraries do not
  need declaring either: they resolve at the final link.
- The library is named after its directory
  (`src/fw/services/timeline` → `src__fw__services__timeline`). Give it a
  `'name'` of its own instead of a `'dir'` when something refers to it, or
  when one directory builds more than one library.

Meson has no user-defined functions, so there is no `pbl_library()` to
call: a directory declares a dict, and the root `meson.build` turns every
entry into a `static_library()` once the whole tree has been walked. The
keys are:

| key     | meaning                                                  |
| ------- | -------------------------------------------------------- |
| `dir`   | the directory the library is named after                  |
| `name`  | an explicit name, instead of `dir`                        |
| `srcs`  | the sources, as `files()` objects                         |
| `kind`  | `objects` (default) or `stlib`                            |
| `inc`   | private include directories, relative to `dir`            |
| `defs`  | private macros, without the `-D`                          |
| `args`  | private compiler flags                                    |

`'kind': 'stlib'` builds a static library the linker pulls objects from on
demand, instead of the default `objects`, whose every object file is
linked. Use it for third-party code that ships more than the firmware
references.

Conditional pieces key off the Kconfig symbols, which arrive in two
dictionaries: `cfg` for `if`, `cfgval` for the value itself:

```meson
if cfg.get('CONFIG_ACCEL_LSM6DSO', false)
  sources += files('lsm6dso/lsm6dso.c')
endif
```

`subdir()` shares Meson's scope with its parent, so a directory that adds
subdirectories in the middle of building a source list has to name that
list after its library rather than use the plain `srcs`.

## Exposing headers

Register the directory the headers live in:

```meson
pbl_global_inc += meson.current_source_dir() / 'include'
pbl_global_defs += ['FOO_ENABLED=1']
```

Both the source directory and the matching build directory end up on the
include path, so generated headers are found next to their hand-written
neighbours.

Headers are global; there is no per-consumer opt-in. A library that is not
always needed is gated behind a Kconfig symbol instead, as Zephyr does:
its `subdir()` call is wrapped in an `if`, and consumers `select` (or
`imply`) the symbol in their own Kconfig, e.g. `BT_FW_NIMBLE` selects
`MBEDTLS` and `SERVICE_VOICE` selects `SPEEX`.

Prebuilt vendor archives are added to `pbl_imported_libs` by path.

## Sources by pattern

Meson has no globbing, by design. Directories that pick their sources up
by pattern shell out to `tools/meson/glob.py` at configure time:

```meson
sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--recurse', '*.c',
  check: true,
).stdout().strip().split('\n')
```

Unlike CMake's `CONFIGURE_DEPENDS`, this is not re-evaluated on its own:
after adding or removing a file, run `./pbl configure` again.

## Linker script fragments

Libraries that need linker script additions append them to the array for
one of the master script's hook points, as `"<sort key>|<path>"`:

```meson
pbl_linker_memory += 'default|' + meson.current_source_dir() / 'linker.ld'
```

See `meson/linker/meson.build` for the available hooks.

## Generated sources

Generated headers are `custom_target`s collected in `pbl_generated`, which
becomes a dependency every library inherits. Only headers may go in it: a
`.c` file there would be compiled into every library. Generators live in
`tools/meson/`:

- `kconfig.py` — runs Kconfig, writes `.config` and `autoconf.h`, and
  prints the configuration for `meson.build` to parse.
- `resources.py` — resolves the board's resource maps into one target per
  resource, and runs the resource, pbpack and resource-table steps.
- `generate.py` — the remaining source generators (version header, app
  registry, protocol endpoint table, applib allocator tables, log string
  hashing, stored apps).
- `firmware.py` — image steps: size checks, bundling, QEMU flash images.
- `glob.py` — source globbing.
- `cross.py` — the cross file naming the toolchain.
- `ccwrap.sh` — the compiler wrapper that gives every object the name of
  the source it was built from. Meson cannot set a per-source compile
  definition, and ccache cannot be used through the wrapper.

`autoconf.h` is force-included into every compilation, so Kconfig symbols
reach the sources, the headers and the linker script alike.

## Directory layout

```
meson.build                 the firmware image: what is built and linked
meson.options               board, variant and Kconfig overrides
meson/libc/                 Kconfig-driven C library selection
meson/linker/               linker script assembly
meson/native_sdk/           the SDK shims the firmware and apps share
tools/meson/                the generators the build shells out to
```
