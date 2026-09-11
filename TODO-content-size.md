# Content size follow-ups — scratch notes

**Temporary file. Drop this commit before opening the PR.**

Branch `feat/content-size-followups`, continuing the follow-up list from #1911 /
PR #1912 (`fw: honor the user's Text Size in system menus`).

## Done on this branch

| Commit | What |
|---|---|
| `4f199580f` | `system_theme_get_content_size_for_process()` / `..._font_for_process()` |
| `ff19782ab` | Option menu follows the process content size, ExtraLarge style |
| `7554cf15d` | Notification list, alarm list, Bluetooth pairing hint |
| `dfcf856b2` | Remaining ExtraLarge rows in the theme table |
| `db3fd15b5` | `send_text` no-contacts screen, timeline day separator |
| `9dd7c836e` | `notifTextStyle` pref + `SystemThemeContentSizeFollowSystem` |
| `058becf49` | Notification card pins its text nodes to the notification size |
| `7f4fa50af` | Settings → Notifications → Text Size restored |

## Not yet done

### 1. Validation

- No QEMU pass yet on the new tiers. Check `qemu_emery` (same platform as
  PT2) at Smaller / Default / Larger: settings menus, notification card,
  timeline day separator, send text no-contacts screen.
- No goldens for the notification card at a pinned notification size.
  `tests/fw/ui/test_notification_window*` is the place.
- No test for the `SystemThemeContentSizeFollowSystem` resolve path.
  `tests/fw/shell/test_system_theme.c` already fakes the process identity,
  so extending it there is cheap.
- No on-device check. `uaparit` validated PR #1912 on `obelix_pvt`; worth
  asking again for this one.

### 2. Notification list still follows the system size

`apps/system/notifications.c` draws cells at the *system* content size while
the notification card now follows the notification one. Deliberate for now:
the list's row heights come from `menu_cell_basic_cell_height()`, which reads
the system size, so sourcing the fonts from the notification size would clip
cells. Either leave it (the list is a menu like any other) or teach the cell
geometry helpers to take an explicit size. Needs a decision.

### 3. Screens deliberately left on the compile-time default

These look like they belong in the sweep but must not be rerouted as-is —
their per-tier tables are indexed by content size while the values encode
*display geometry*, not text scale. `prv_config()` reads the compile-time
`PreferredContentSizeDefault` on purpose. Rerouting to the runtime preference
would put Emery-geometry numbers on a 144x168 screen.

- `apps/system/music.c` — `s_music_size_config_large` puts the progress bar
  at y=168; a Basalt display is 168 rows tall.
- `applib/ui/time_selection_window.c`, `date_selection_window.c`,
  `time_range_selection_window.c` — absolute `top_offset_*` / `range_origin_y`
  per tier.
- `applib/ui/selection_layer.c` — Large already uses `GOTHIC_36_BOLD`, the
  largest Gothic bold in the resource map, so an ExtraLarge tier needs a new
  font resource. The whole effort has been "zero new resources" so far.
- `applib/ui/action_menu_layer.c` — separator config is `{162, 2}` at Large,
  wider than a 144px screen. Note `prv_get_item_font()` already uses the raw
  `system_theme_get_font()`, so third-party app action menus *do* follow the
  user size today, unlike everything else. Pre-existing; left alone.
- `applib/ui/crumbs_layer.c` — gutter width, not text. Growing it at larger
  text sizes takes room *away* from the text.

Doing these properly means designing per-display-size layouts, which is real
design work, not a mechanical reroute.

### 4. `applib/ui/text_layer.c` default font

`text_layer_init()` picks its default font from the platform default. Routing
it through the process size would resize every system `TextLayer` created
without an explicit font, most of which sit in hand-tuned fixed frames. High
blast radius, low reward. Left alone.

### 5. Untouched items from #1911

- Marquee / scrolling for truncated menu titles (forum #561 post 10). No
  marquee exists in `menu_layer` / `text_layer` today.
- Status bar font and height. `STATUS_BAR_LAYER_HEIGHT` is compile-time and
  every menu window insets by it.
- Round displays keep the fixed `MENU_CELL_ROUND_*` constants.
- The `Small` tier design (#1462).

## PR body should mention

- Third-party apps are unaffected: everything new goes through
  `system_theme_get_font_for_process()`, except the notification card, which
  only kernel/system tasks render.
- `notifTextStyle` defaults to follow-system, so an untouched watch renders
  byte-identically to before.
- `TextStyleFont_ParagraphHeader` keeps its ExtraLarge value. It carried no
  "same as Large" note, so it reads as deliberate rather than unfinished.
- Section 3 above, so reviewers know the omissions are considered.
