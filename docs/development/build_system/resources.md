# Resources

:::{note}
This page is a proposal. It describes the layout and format the resource
tree is moving to; the build still reads the `resource_map.json` files
until the migration below has landed.
:::

System resources are the fonts, images, animations, vibration patterns and
data blobs the firmware ships in its resource pack. Every one of them is
declared in a YAML file next to the files it is built from, and a
per-platform file picks which of those declarations a firmware carries.

## Why the tree is changing

The current tree grew by copying. It has 14 `resource_map.json` files in
three parallel trees (`common/`, `normal/` and `prf/`), each with a `base`
directory and one directory per *board*, and the build merges them by
resource name at configure time:

- The per-board maps are copies of each other. `qemu_emery` and `obelix`
  are byte-for-byte identical, `qemu_gabbro` and `getafix` differ in a
  comment, and `native_*` boards exist only through `CONFIG_RESOURCES_BOARD`
  pointing them at another board's maps. Of the 608 resources a normal
  firmware can declare, 559 are identical on every platform.
- Files live under the names of boards that no longer exist: `snowy`,
  `silk`, `spalding`, `robert`, and a dangling `calculus` symlink. An
  emery firmware builds its modal animations from `normal/obelix/` and its
  large emoji and regulatory marks from `normal/snowy/`.
- Overrides are implicit. A board map re-declares a resource to flip its
  `builtin` flag or point it at another file, and the override is found by
  matching names across files. The round-only weather icon sequence is
  shipped to rectangular platforms as a zero-byte stub so the merge keeps
  working.
- Display variants are encoded in file names (`checkbox-checked~color~round.png`)
  and resolved by globbing against the platform's tags.
- 27 of the 668 files are referenced by nothing: leftover `spalding`
  vibes and icons, a stray `tintin.po~`, source SVGs of animations that are
  now checked in as `.pdc`, four fonts, and `.DS_Store`. Another dozen
  licences and scripts sit among the assets with nothing marking them as
  such.
- JSON cannot carry comments, so the maps hold `_comment` arrays, and every
  file path is spelled out from the root of the tree.

## Layout

```text
resources/
├── platforms/          one file per platform and variant
│   ├── emery.yaml        emery.prf.yaml
│   ├── gabbro.yaml       gabbro.prf.yaml
│   └── flint.yaml        flint.prf.yaml
├── fonts/              resources.yaml, gothic/, bitham/, leco/, emoji/, ttf/
├── images/
│   ├── system/           action bar, arrows, checkboxes, generic dialogs, results
│   ├── bluetooth/        pairing dialogs (normal and PRF)
│   ├── battery/
│   ├── notifications/    NOTIFICATION_*: one SVG per size, brand logos from third_party
│   ├── timeline/         pin icons, the timeline table, layouts.json.in
│   ├── emoji/
│   ├── music/
│   ├── settings/
│   ├── launcher/
│   ├── regulatory/       SYSTEM_*_MARK
│   └── recovery/         PRF-only screens
├── animations/         PDC sequences: modal transitions, results, alarm, incoming call
├── vibes/
├── data/               globe cube map and star field, weather icon sequence
├── tzdata/
└── lang/               language pack layout
```

A directory that holds a `resources.yaml` is a **fragment**: the YAML
declares resources, and the files it names live in that directory or
below it. Files are grouped by what they are, not by which board first
used them; an app or service that needs an icon finds it under the
category it belongs to. The `common`, `normal` and `prf` trees are gone:
a resource shared by PRF and the normal firmware is one fragment that
both platform files include.

Within a fragment, files that differ per display go in `bw/`, `color/`
and `round/` subdirectories under the same name, instead of carrying tags
in the file name. Fonts keep one directory per family, with the colour
emoji faces in `emoji/color/`.

File names do not change in the move. Renaming the `Pebble_80x80_*.svg`
style names to match their resource names is worth doing, but as its own
change on top of a tree that git already follows through the move.

## Declaring resources

A fragment's `resources.yaml` lists its resources under `media`, with the
same keys an app declares its resources with in `package.json`, so the
format is already familiar from third-party apps and the generators under
`tools/resources/` need no change:

```yaml
# resources/images/system/resources.yaml
media:
  - name: ACTION_BAR_ICON_CHECK
    type: pbi8
    file: action_bar_icon_check.png
    builtin: [emery, gabbro]

  - name: CHECKBOX_CHECKED
    type: png
    file:
      bw: bw/checkbox_checked.png
      color: color/checkbox_checked.png
      gabbro: round/checkbox_checked.png

  # Animated: a directory of SVG frames, one PDC sequence.
  - name: RESULT_SENT_LARGE
    type: pdc
    file: ../../animations/result_sent
    builtin: [emery, gabbro]
```

`file` is relative to the fragment's directory. The keys:

| Key | Meaning |
| --- | --- |
| `name` | The resource; `RESOURCE_ID_<name>` in C. Unique across every fragment a firmware includes. |
| `type` | `font`, `png`, `pbi`, `pbi8`, `pdc`, `raw`, `vibe`, `mo`, `js`, as today. |
| `file` | A path, or a map from platform or display tag to a path (below). |
| `targetPlatforms` | Platforms that get the resource; all of them when absent. |
| `builtin` | `true` to link the resource into the firmware image on every platform, or the list of platforms where it is. |
| `config` | A Kconfig symbol; the resource is only built when it is enabled. |
| `aliases` | Other names resolving to the same id. |
| `characterRegex`, `characterList`, `compatibility`, `extended`, `trackingAdjust`, `pixelHeight`, `compress` | Font options, unchanged. |

A `file` map is keyed by platform name (`emery`, `gabbro`, `flint`) or by
one of the platform's tags (`bw`, `color`, `rect`, `round`). A platform
name beats a tag, and two matching tags are an error rather than a guess,
so the three-way `~color~round` cases name the platform. This replaces
the `~tag` file name convention and the glob that resolved it.

Platform differences are expressed in one place, the entry itself:
`targetPlatforms` for a resource some platforms do not have,
`builtin` for the ones a platform keeps in flash, `config` for a resource
a feature brings along. The eight `MULTIWATCH_*` backgrounds become
`targetPlatforms: [gabbro]`; the three `BLE_HRM_*` dialogs become
`config: SERVICE_BLE_HRM` instead of being left out of the `asterix` copy
of the `qemu_flint` map; the weather icon sequence becomes
`targetPlatforms: [gabbro]` and the stub file goes away. Comments are
comments.

Besides `media`, a fragment may carry the two tables that are not
resources themselves: `timeline`, the published timeline icon ids
(`resources/images/timeline/resources.yaml`), and `files`, the layout of
the language pack written to the filesystem (`resources/lang/resources.yaml`).
Both keep their current shape.

## Platform files

`resources/platforms/<platform>.yaml` is the normal firmware's resource
set for a platform, and `<platform>.prf.yaml` the recovery firmware's. A
platform file lists the fragments it includes, in the order their ids are
assigned, and the defaults the variant applies to every entry:

```yaml
# resources/platforms/flint.yaml
include:
  - fonts
  - images/system
  - images/bluetooth
  - images/battery
  - images/notifications
  - images/timeline
  - images/emoji
  - images/music
  - images/settings
  - images/launcher
  - images/regulatory
  - animations
  - vibes
  - data
  - lang
```

```yaml
# resources/platforms/flint.prf.yaml
defaults:
  builtin: true    # PRF has no resource pack
  extended: false
include:
  - fonts/recovery
  - images/bluetooth
  - images/recovery
```

Whole features a platform does not have are left out of its `include`
list; single resources use `targetPlatforms`. The PRF rule that every
resource is built in, today a special case in `resources.py`, is the
`defaults` block.

Boards no longer appear anywhere under `resources/`. The build picks the
platform file from `CONFIG_PLATFORM_*` and `VARIANT`, so `obelix`,
`qemu_emery` and `native_emery` build the same resources by construction
and `CONFIG_RESOURCES_BOARD` is retired.

## Resource ids

Ids are assigned in order: the platform file's fragments in `include`
order, their entries in file order, then the fragments registered from
CMake in the order their directories are added, then the resources other
build steps produce (stored apps, the timezone database). Resources stored
in the pack come before the built-in ones, as today.

Ids are an artefact of one build. The firmware and its pack are produced
and flashed together, timeline ids are explicit in the `timeline` table,
and fonts are looked up by key, so nothing persists a numeric id. The
weather app's `resource_ids.pin.h` records what happened the one time a
build relied on the numbers; it is the reason the stub override exists,
and the reason it is safe to drop it.

## Resources owned by an app

Most resources are used by exactly one directory. Counting the
`RESOURCE_ID_` references in the tree, the weather app is the only user of
50 resources, settings of 26, workout of 18, the launcher of 11, the
Tictoc and Kickstart faces of 8 and 5, health of 6, and a dozen smaller
owners of a handful each: about 140 of the 612 declared names, before
the 20 `MUSIC_*` icons of the single-file music app. Those
belong with the code, the way a third-party app keeps its icons in its
own `resources/` directory.

An app fragment is the same thing as a fragment under `resources/`, in
the app's directory:

```text
fw/apps/system/workout/
├── resources/
│   ├── resources.yaml
│   ├── detected.svg
│   ├── run_tiny.svg
│   └── ...
├── workout.c
└── ...
```

and is registered from the directory's `CMakeLists.txt`, next to the
sources it serves:

```cmake
pbl_library()
pbl_library_sources(${source})
pbl_resource_fragment(workout/resources)
```

`pbl_resource_fragment(dir)` appends the directory to a global property,
and `pbl_resources()`, which already runs once after every directory has
been added, hands the list to `resources.py manifest` after the platform
file. A fragment is dropped with its code: the heart rate sharing dialogs
in `fw/popups/ble_hrm/` are registered with
`pbl_resource_fragment_ifdef(CONFIG_SERVICE_BLE_HRM ble_hrm/resources)`,
mirroring `pbl_library_sources_ifdef()`, and leave the pack with the
feature.

Only resources with a single owner move. A resource two apps share, or
one the timeline table names, stays under `resources/`, and a fragment
cannot re-declare a name another one already uses: duplicates are an
error, not an override.

## Build integration

`tools/cmake/resources.py manifest` takes `--platform-file` and a list of
`--fragment` directories instead of a board name. It expands each
fragment into the list of dictionaries the generators consume today,
with `file` resolved against the fragment directory and the platform, so
`definitions_from_dict()` and the generators, which the SDK shares, are
untouched. The `.reso` of a resource lands under
`<build>/resources/<path relative to the repository>.<NAME>.reso`, which
also works for fragments outside `resources/`.

Every YAML file read is added to `CMAKE_CONFIGURE_DEPENDS`, as the JSON
maps are now. A `file` that does not exist is a configure error. So is
a file under a fragment directory that no entry names, directly or as a
`characterList`, unless it is a licence, so the tree cannot grow orphans
again. Both checks run in CI as part of the configure step.

## Migration

The move is mechanical and is verified against the current build before
anything is cleaned up.

1. **Loader.** Teach `resources.py` the platform files and fragments, and
   add `pbl_resource_fragment()` to `cmake/modules/resources.cmake`. No
   content changes.
2. **Convert.** A one-off script reads the merged maps of every
   `(board, variant)` pair, writes the fragments and platform files, and
   `git mv`s each file into its category. For every board in the CI
   matrix, normal and PRF, the resource ball before and after holds the
   same set of `(name, type, storage, aliases, data)` and the generated
   timeline, font and language pack tables are identical. Ids may be
   renumbered on a platform where the old per-board order cannot be
   reproduced by one include list; nothing depends on them.
3. **Delete.** The 27 unreferenced files, the `calculus` symlink, the
   `~tag` file names and the `CONFIG_RESOURCES_BOARD` symbol, plus the
   `is_recovery` special case and the name-merging code in `resources.py`.
   Update `tests/overrides/font_compression/README.md` and
   `docs/reference/formats/timezone.md`, which cite the old paths.
4. **App fragments.** Move the single-owner resources into their apps, one
   app per commit, starting with weather, settings and workout. Each
   commit is verified the same way as step 2.

Out of scope here: renaming the files to match their resource names,
changing the SDK's `package.json` resource format, and a `pbl resources`
command to list or inspect a build's pack. All three become easier once
the tree has one format and one place per resource.
