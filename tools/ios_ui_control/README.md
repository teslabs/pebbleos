# iPhone UI control for Bluetooth testing

A small Apache-2.0 XCUITest runner drives an already installed CoreApp. It can
read the accessibility tree, capture the screen, activate an app, tap and swipe.
It does not rebuild CoreApp. A separate helper app generates local CallKit
calls for HFP testing without dialing a telephone number or contacting a
server. Xcode signs the helper app and test runner on the phone.

Requires macOS, Xcode, a signing team, and an unlocked, trusted iPhone with
Developer Mode enabled. Obtain its UDID from Xcode or `xcrun devicectl list devices`.

```sh
python3 tools/ios_ui_control/build.py --team TEAM_ID --device IPHONE_UDID --install
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

## Local calls

Activate `com.teslabs.hfpdriver` and grant microphone permission on first use.
The **Incoming** button reports a simulated incoming call to CallKit; answer
or reject it from the watch. **Outgoing** starts a local outgoing call and
connects it after one second. Neither operation dials a real number. The
reserved display number is `+12025550100` and the caller name is **Local HFP
test**. Only one test call can run at a time, and it ends after three minutes.
**End call** ends only the call owned by this app.

```sh
python3 tools/ios_ui_control/control.py --device IPHONE_UDID \
  '{"action":"activate","bundle":"com.teslabs.hfpdriver"}'
python3 tools/ios_ui_control/control.py --device IPHONE_UDID \
  '{"action":"tap","bundle":"com.teslabs.hfpdriver","label":"Incoming"}'
```

When CallKit activates audio, the app plays a quiet 440 Hz tone and displays
the input sample count and RMS level. It does not save microphone audio.
**Use watch** selects the HFP input when exactly one is available. The route
and sample counters let a test check audio delivery and watch microphone
mute; allow route changes to settle before evaluating them. These local
calls do not establish cellular interoperability or acoustic quality.

Use `--install` after changing the helper. `test-without-building` can leave
an older helper installed, including stale background-audio capabilities.
The app declares audio and VoIP background modes for the local CallKit test.

With the UI runner active, the helper app visible and idle, and BLE/HFP
connected, reproduce the watch answer/hangup sequence with:

```sh
python tools/ios_ui_control/incoming_smoke.py \
  --device IPHONE_UDID --tty WATCH_SERIAL_PORT --gain 7 --duration 5
```

Use the project's Python environment for the watch serial libraries. The
runner queues a volume update immediately before answering, verifies the
active call/audio state, and hangs up. A lost serial connection or increased
HFP error count fails the test. It leaves global watch volume unchanged and
sets call gain to 7/15 by default. Logs and screenshots remain under the
ignored build directory. Avoid other UI/serial clients during the run.
This is a state/transport smoke test, not an acoustic-quality measurement.

To inspect the helper's synthetic downlink tone, add
`--capture-pcm /tmp/hfp-tone.h4`. After the active interval, the runner arms a
bounded capture, waits one second, and exports it after hangup. This contains
received PCM before playback processing; microphone audio is not exported.
Keep captures outside version control.

The optional analyzer requires NumPy (`python -m pip install numpy`):

```sh
python tools/hfp_tone_quality.py /tmp/hfp-tone.h4 --frequency 440
```

It reports packet-status counts, sample loss and tone-to-error ratios for
valid samples and the entire capture. It fits amplitude, phase and DC offset
using valid samples at the known tone frequency. These measurements compare
digital transport/decoder behavior; they do not score speech quality, echo,
speaker acoustics or subsequent loss concealment.

## Pairing

For Bluetooth numeric comparison, inspect the fresh code on both the phone
and watch before accepting either confirmation. Small watch fonts can confuse
OCR; a failed or uncertain comparison must not automatically approve pairing.
The first live test used this runner to pair Obelix through CoreApp and verified
an authenticated CTKD bond and encrypted BLE/HFP links on iPhone.
