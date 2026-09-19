# iPhone UI control for Bluetooth testing

A small Apache-2.0 XCUITest runner drives an already installed CoreApp. It can
read the accessibility tree, capture the screen, activate an app, tap and swipe.
It does not rebuild CoreApp or create telephone calls. Xcode signs and installs
a separate helper app and test runner on the phone.

Requires macOS, Xcode, a signing team, and an unlocked, trusted iPhone with
Developer Mode enabled. Obtain its UDID from Xcode or `xcrun devicectl list devices`.

```sh
python3 tools/ios_ui_control/build.py --team TEAM_ID --device IPHONE_UDID
xcodebuild test-without-building \
  -xctestrun build-ios-ui-control/build/Build/Products/HfpDriver_iphoneos*.xctestrun \
  -destination 'platform=iOS,id=IPHONE_UDID' -parallel-testing-enabled NO
```

Once the test prints `HFP_CONTROL_READY`, use a second terminal:

```sh
python3 tools/ios_ui_control/control.py --device IPHONE_UDID
python3 tools/ios_ui_control/control.py --device IPHONE_UDID \
  '{"action":"tap","label":"Add a Pebble"}'
python3 tools/ios_ui_control/control.py --device IPHONE_UDID \
  '{"action":"snapshot","bundle":"com.apple.springboard"}'
python3 tools/ios_ui_control/control.py --device IPHONE_UDID \
  '{"action":"stop"}'
```

Each command defaults to `coredevices.coreapp`. Supply `bundle` to inspect or
operate a system dialog or another test app. `activate` brings the chosen app
forward. `tap` requires one hittable label/identifier match and prefers buttons;
ambiguous matches fail. `coordinate` takes normalized `x`/`y` values from 0 to 1;
check a fresh screen first because CoreApp reorders device rows. `swipe` takes
`direction` (`up`, `down`, `left`, `right`). A coordinate tap only confirms that
the event was sent; verify the resulting screen or watch state separately.

The helper writes `tree.txt`, `screen.png`, and the command/response JSON files
to `build-ios-ui-control/session` (override with `--output`). Screens and trees
can contain personal information; leave them in the ignored build directory.
Commands travel through the runner's app data container over the Xcode device
connection; there is no network listener. Use one command client at a time.
The runner exits after 30 minutes or a `stop` command.

For Bluetooth numeric comparison, inspect the fresh code on both the phone
and watch before accepting either confirmation. Small watch fonts can confuse
OCR; a failed or uncertain comparison must not automatically approve pairing.
The first live test used this runner to pair Obelix through CoreApp and verified
an authenticated CTKD bond and encrypted BLE/HFP links on iPhone. Calls and
acoustic quality require separate testing.
