# Bluetooth calling

## Scope and constraints

Obelix is the first target for a watch acting as a Bluetooth Hands-Free (HF)
device. The phone is the Audio Gateway (AG) and places the cellular call.
The integrated feature combines call control, simultaneous microphone/speaker
audio, and Pebble's BLE phone connection. It is opt-in while the remaining
release acceptance gates below are completed.

All new host/profile code must be Apache-2.0 or implemented in this project.
Do not integrate BTstack or SiFli's proprietary Bluetooth host libraries.
Controller boot firmware and patches already used by PebbleOS remain below
the HCI boundary. No new proprietary host dependency is needed for these
experiments.

Keep the following independent of the controller vendor:

- BR/EDR connection management, security and bonding;
- L2CAP, SDP, RFCOMM and HFP;
- call state and UI;
- codecs, buffering and acoustic echo cancellation.

Controller initialization, patches and physical transport belong below HCI.
Commands, events, ACL and synchronous (SCO) data cross that boundary. A future
controller should require a transport/initialization port, without changes
to the call service or profiles.

## Hardware evidence

On 2026-09-17, the controller probe ran on a physical Obelix with an
`obelix@pvt` development build (`CONFIG_RELEASE=n`). All five read commands
succeeded:

| Item | Observed value |
| --- | --- |
| HCI/LMP version | 12 / 12 |
| HCI revision / LMP subversion | 906 / 906 |
| Manufacturer ID | 2636 |
| LMP feature bytes | `bfeecdfedbff7b87` |
| BR/EDR and LE | Both advertised |
| SCO, eSCO, CVSD, transparent synchronous data | Advertised |
| ACL buffers | 4 packets, 1021 bytes each |
| SCO buffers | 4 packets, 255 bytes each |
| Read Voice Setting | `0x0000` |

Supported-command bytes, in controller order:

```text
bfffff03ccffefff3ffffc1ff20fe8fe
3ff78fff1c00040061f7ffff7f380000
feffffffffffffffffffffffff3f0000
00000000000000000000000000000000
```

Bumble 0.0.234 subsequently initialized this controller through the external
HCI bridge and enabled Classic discoverability/connectability with an HFP HF
service.

On 2026-09-18, an iPhone paired, connected over RFCOMM and completed the HFP
service-level connection. An outgoing call progressed through dialing,
ringing, answered and ended indications. The controller accepted Enhanced
Accept Synchronous Connection with HCI input/output paths and established
a CVSD synchronous link (handle 384). No SCO payloads reached the host
during that call, including its answered interval. The probe consequently
sent no uplink packets because transmission is paced by received packets.
This verified call signaling and synchronous-link establishment, but left
voice transport unresolved. Repeating with the legacy acceptance command
also established CVSD and reported an answered call, with zero received
SCO packets. Neither acceptance method alone delivered voice over HCI.

A third iPhone call with the shared-audio diagnostic enabled established
the same CVSD link and demonstrated activity in IPC queue 6 and the native
downlink ring:

| Native audio observation | Value |
| --- | --- |
| Shared link status / handle / type | 0 / 384 / 2 (eSCO) |
| Transmit interval | 6 slots (3.75 ms) |
| Negotiated receive / transmit packet lengths | 30 / 30 bytes |
| Air mode | 2 (CVSD) |
| Software-CVSD configuration | `0x00000000` |
| Downlink capacity / read index / write index | 460 / 0 / 448 bytes |
| Mailbox notification samples, about five seconds apart | 1320, 2657, 3994 |
| Uplink read / write index | 0 / 0 |
| Host HCI SCO packets | 0 |

The downlink stopped advancing because the diagnostic deliberately did not
consume data. This confirmed the controller used the native audio path in
the tested configuration. The SDK's non-software-CVSD path suggested 60-byte
PCM payloads plus four-byte headers for this interval; the observed 448-byte
occupancy was consistent with seven such frames.

The first live adapter run subsequently validated 60-byte native payloads
and delivered 16,620 standard HCI SCO packets (997,200 payload bytes) to
Bumble, with 101 nonzero packet-status indications and no malformed frames.
The controller consumed 14,376 uplink packets carrying silence. HCI
disconnect retired the stream and cleared both rings. This establishes
host/controller data movement in both directions.

That run used three 120-byte host-to-controller buffers. The desktop probe
skipped 2,242 uplink silence packets for lack of credits. The revised adapter
advertises seven 60-byte buffers (420 bytes within the 460-byte native ring)
to tolerate a longer desktop round trip. In the subsequent tone test, the
user confirmed hearing the generated 440 Hz tone at the other end of the
call. This establishes audible uplink delivery with signed 16-bit, 8 kHz PCM.
That run received 9,120 packets (82 nonzero statuses), consumed 8,222 uplink
packets, skipped 891 desktop transmissions for lack of credits and reported
no malformed native frames. Seven queued transmissions were dropped when
the link ended.

The subsequent watch-audio call confirmed the Obelix microphone was audible
at the other phone. The user could not hear the watch speaker despite an
unmuted watch at 100% volume. Every received byte was accepted by the speaker
queue, and initial capture had no drops or start failures. The controller
later emitted Hardware Error with payload `0x45`; downlink packets became lost and eventually
mailbox activity stopped. This run does not establish speaker output or
stable full-duplex operation.

After removing the prototype's extra playback attenuation, the user confirmed
hearing speech from the watch speaker as well as watch-microphone speech at
the other phone. That call received 9,630 packets (117 error indications,
about 1.2%), passed all 577,800 received bytes to the speaker queue, and
reported no microphone drops, startup failures or controller hardware errors.
Disconnect cleared both native rings and stopped local audio. Received PCM
peaks ranged from 937 to 2,429 in the five-second diagnostic windows. The
user described playback as faint and somewhat chirpy. This establishes
basic two-way calling on Obelix, with audio quality still needing work.

With the project-owned software CVSD codec, increased watch playback gain
and increased iPhone call volume, the user subsequently described playback
as much better, still somewhat robotic but acceptable for the experiment.
This is the current listening baseline; remaining quality faults and the
desktop-host limitation are detailed below.

The [SF32LB52x datasheet](https://downloads.sifli.com/user%20manual/DS5201-SF32LB52x-Datasheet%20V2p5.pdf),
section 3.1, specifies BR/EDR, SCO/eSCO and CVSD capabilities. The checked-out
SDK also includes `example/bt/hfp` and `example/bt/HCI_over_uart` for SF32LB52.
These are supporting evidence, not substitutes for the live experiment.

Audio routing is controller-specific. The SDK's
`middleware/audio/audio_bt_voice/audio_bt_voice.c` uses shared-memory voice
rings and IPC queue 6, while the existing HCI mailbox uses queue 0. Read
Voice Setting does **not** identify that route. Advertised SCO buffers alone
do **not** prove that controller firmware will deliver voice over HCI.

## Read-only probe

The generic probe lives in `lib/btutil/hci_probe.c`; its callback contract
does not depend on NimBLE or SiFli. The NimBLE adapter serializes commands
through the running host. Rejected queries are reported rather than decoded
as capabilities; backend failures abort the probe.

```sh
pbl configure -b build-obelix-hfp --board obelix@pvt \
  -DCONFIG_RELEASE=n -DCONFIG_BT_CONTROLLER_PROBE=y
pbl build -b build-obelix-hfp
pbl flash -b build-obelix-hfp --tty /dev/tty.wchusbserial5B7A1355001
pbl console -b build-obelix-hfp --tty /dev/tty.wchusbserial5B7A1355001
```

At the console, run `bt controller probe`. Bluetooth must be enabled and
the host synchronized. The command reads version, supported commands,
features, buffer sizes and voice setting. It does not enable Classic,
change pairing state, reset the controller or establish an audio link.

## Independent HFP host experiment

The development-only `BT_FW_HCI_BRIDGE` backend gives an external host
exclusive ownership of the controller. Normal watch BLE services are
unavailable in this build. The watch still runs PebbleOS and its PULSE
console. The bridge does not load SiFli's host stack or audio middleware.

```sh
pbl configure -b build-obelix-hci --board obelix@pvt \
  -DCONFIG_RELEASE=n -DCONFIG_BT_FW_HCI_BRIDGE=y
pbl build -b build-obelix-hci
pbl flash -b build-obelix-hci --tty /dev/tty.wchusbserial5B7A1355001
python tools/hci_bridge.py --tty /dev/tty.wchusbserial5B7A1355001
```

Use the project's Python virtual environment. The bridge listens on
`127.0.0.1:12346` for one host, carrying an unchanged H4 byte stream over
PULSE reliable protocol `0x3e23`. PULSE and TCP fragment boundaries are not
HCI packet boundaries. Restart the bridge after the host disconnects;
the new host must initialize/reset the controller. Flash the normal NimBLE
build again to restore the watch's phone connection.

In another terminal, install the optional desktop test dependencies into
the diagnostic build directory and start the host:

```sh
python -m pip install --target build-obelix-hfp/host-deps bumble==0.0.234
PYTHONPATH=build-obelix-hfp/host-deps python tools/hfp_probe.py --duration 600
```

[Bumble](https://github.com/google/bumble) is an Apache-2.0 desktop test host,
not a proposed Python runtime on the watch. The probe advertises
**Obelix HFP test**. Pair from the phone, initiate a test call there, and
select the test device as the call audio output. The probe reports ACL,
RFCOMM, HFP and SCO progress separately. It supports CVSD only, counts
received SCO packets/bytes and packet-status errors, and sends silence
back paced by received packets. It does not dial or answer automatically.

The microphone and speaker are not connected in this experiment. Zero SCO
packets after a successful SCO connection indicate that the audio route
needs investigation; they do not by themselves prove a silicon limitation.
The counters also cannot demonstrate acoustic quality or successful
delivery of uplink audio to the remote person.

The host prefers Enhanced Accept Synchronous Connection when advertised,
requesting HCI input/output paths. `--legacy-sco` tests the legacy command
with 16-bit signed PCM and CVSD (`voice_setting=0x0060`). Pairing keys are
not persisted by the test host; the phone may need to forget its old test
pairing before reconnecting to a new host process.

`--trace PATH` optionally records a private btsnoop capture. It contains
pairing material and audio payloads; keep it in the ignored build directory.
Without this option the probe does not save audio. The PULSE bridge is a
diagnostic transport: its buffering and acknowledgements must not be used
to infer final on-watch audio latency.

### SiFli audio mailbox diagnostic

Enable `CONFIG_BT_HCI_AUDIO_PROBE=y` in the bridge build to investigate the
separate shared-memory audio route. This opt-in diagnostic initializes the
two SF32LB52 audio-ring descriptors and subscribes to IPC queue 6. It leaves
the controller's software-CVSD setting unchanged. It neither consumes nor
produces voice payloads, so a working downlink ring will eventually fill.

Run the bridge with `--audio-probe` to poll `bt audio probe` over the same
PULSE link every five seconds. The output reports mailbox notifications,
ring cursors and capacity, the software-CVSD configuration, and the shared
synchronous-link metadata. Ring snapshots are asynchronous; compare several
samples. The link record is uninitialized until the controller populates it;
do not interpret its fields before a successful synchronous connection.
The probe holds the LCPU awake while copying metadata because this
RAM is inaccessible when its power domain sleeps. Nonzero notifications and
changed ring cursors would demonstrate
activity in this route, not successful voice decoding or bidirectional audio.

### Experimental native audio adapter

`CONFIG_BT_HCI_AUDIO_ADAPTER=y` additionally enables a project-owned
Apache-2.0 adapter below HCI. It accepts one CVSD connection through legacy
Setup/Accept Synchronous Connection with `voice_setting=0x0060`. It supports
the observed 30- or 60-byte air
packets with symmetric transmit/receive lengths and 6- or 12-slot intervals.
Other codec paths and enhanced setup/accept are deliberately unsupported;
the corresponding enhanced-command capability bits are cleared.

The adapter validates native frame length/status before forwarding payloads
as HCI SCO. It zeros unavailable audio for lost/partial packets while
preserving their HCI status. A malformed native header disables the stream
until another connection. Host SCO packets go into the headerless native
uplink only for the active handle. The desktop host continues using standard
HCI and contains no SiFli-specific audio logic.

Read Buffer Size reports seven SCO buffers of 60 bytes, fitting within the
460-byte native ring. Host-to-controller SCO flow control is implemented:
completion events follow native-ring consumption (or explicit packet drops),
not merely insertion into the ring. Controller-to-host SCO credit control is
unsupported and requests to enable it are rejected. ACL accounting is
preserved. The bridge uses one controller task to serialize events, SCO and
complete H4 packets; partial H4 packets are never interleaved with audio.
Its bounded transport queue fails closed on overflow rather than silently
discarding command/ACL bytes.

```sh
pbl configure -b build-obelix-hci --board obelix@pvt \
  -DCONFIG_RELEASE=n -DCONFIG_BT_FW_HCI_BRIDGE=y \
  -DCONFIG_BT_HCI_AUDIO_PROBE=y -DCONFIG_BT_HCI_AUDIO_ADAPTER=y
pbl build -b build-obelix-hci
pbl flash -b build-obelix-hci --tty /dev/tty.wchusbserial5B7A1355001
python tools/hci_bridge.py --tty /dev/tty.wchusbserial5B7A1355001 --audio-probe
# In another terminal:
PYTHONPATH=build-obelix-hfp/host-deps python tools/hfp_probe.py \
  --legacy-sco --sco-flow-control --duration 1800
```

The probe counts controller completions and skips late silence packets when
no SCO credit is available, keeping its backlog bounded. Watch diagnostics
report the observed native header, malformed-frame count, received packets,
queued uplink packets, native consumption and drops. Consumption is not proof
that the remote person received audio. Add `--tone` to generate quiet 440 Hz
PCM, one second on and one second off, for a person at the other end to
confirm. The tone is disabled by default. Larger PCM payloads are split to
respect the controller's reported SCO MTU. The audible tone test passed on
2026-09-18; repeated disconnect/reconnect behavior still needs validation.

Twelve host-side C tests cover fragmentation/coalescing, malformed H4 framing,
native ring wrapping and descriptor bounds, partial/malformed native frames,
packet-loss status, partial uplink consumption, bounded credits, 120-byte
native frames, full-size ACL packets and stale handles across disconnect/reset.
They do not establish real controller audio
behavior or transport latency. Six optional Bumble tests additionally check
credit isolation, packet splitting, bounded tone amplitude and continuity,
and suppression of desktop transmission in watch-audio mode.

### Watch microphone and speaker experiment

Add `-DCONFIG_BT_HCI_LOCAL_AUDIO=y` to the bridge configuration, rebuild and
flash, then run the bridge as above and start the host with:

```sh
PYTHONPATH=build-obelix-hfp/host-deps python tools/hfp_probe.py \
  --legacy-sco --watch-audio --duration 1800
```

This experimental endpoint uses standard HCI SCO packets and the existing
board microphone and speaker services. Call control remains in the desktop
Apache-2.0 host; audio packets and synchronous credits stay on the watch.
The microphone's 16 kHz PCM passes through a 31-tap low-pass filter before
decimation to 8 kHz. Capture and playback queues are bounded. Capture uses
the advertised SCO credits, and disconnect/reset retires queued samples
before asynchronous driver shutdown. Speaker writes require ownership so
another service's preempting stream cannot receive call audio.

Sixteen local-audio tests cover HCI delivery, credit/backlog bounds, forwarding
of ACL events, disconnect generations, microphone ownership, required flow
control, resampling, hardware-error shutdown, diagnostic-tone isolation,
saturating playback gain and fades across missing packets, including negative
sample polarity through both fade-out and recovery, and the bounded local PCM
diagnostic and cancellation of its pending refill when a call starts.
The explicit raw capture is tested for silence gating and its fixed size limit.
A speaker-service test checks stream ownership after
preemption. These tests and the Obelix firmware build pass. The
`bt audio probe` diagnostics include local state, received/queued playback
bytes, microphone transmissions, capture drops and start failures. It also
reports the received PCM peak since the last query (without recording
audio), watch mute/volume settings and the last hardware error.

Call playback now follows the watch volume setting without the initial
prototype's additional 35% multiplier (about 23 dB attenuation at a 100%
watch setting). The `bt audio speaker test` command requests a short local
tone while no call is active. Controller Hardware Error stops native audio
and microphone/speaker use; restart the controller before another test.
The user confirmed both the local diagnostic tone and received speech after
flashing these changes.

The playback revision adds 4x (12 dB) saturating speech gain and counts
clipped samples. Frames marked erroneous fade to silence over 1 ms; recovery
fades in over 1 ms. This reduces hard discontinuities but does not reconstruct
missing speech. One 32 ms silent refill primes playback to provide headroom
for packet timing variations, adding 32 ms to the existing pipeline latency.
The gain and loss handling remain above HCI and contain no SiFli dependencies.
The user confirmed increased loudness but reported very chirpy playback.
That run received 8,836 frames with 87 error indications and no microphone
drops or controller hardware error. Inspection found an unsigned arithmetic
bug in the new fades: negative samples became positive full-scale spikes,
accounting for clipping despite low input peaks. A regression test reproduced
the incorrect +32767 output where -3500 was expected. Both fade expressions
now use signed arithmetic, and the test passes. The subsequent call had no
clipping or controller hardware error, but the user still reported chirpiness.
It received 6,421 frames with 111 error indications and no microphone drops.

For playback isolation, `bt audio pcm test` streams a five-second, 1 kHz tone through
the same 8 kHz PCM speaker stream used by calls, without Bluetooth or call
DSP. It runs only while no call is active. The audio probe also reports DAC
DMA refill count, missing playback bytes and driver write drops. These
counters reset at playback start; initial pipeline fill can cause an underrun,
so inspect whether the counter continues growing during steady playback.
The original 400 ms PCM diagnostic completed with 17 DMA refills, 2,048
missing bytes and no write drops: 15,360 bytes reached DMA, matching the
400 ms tone plus 80 ms of service drain padding at 16 kHz/16-bit. The user
reported a clean, steady beep and requested a longer test. The five-second
version feeds bounded chunks with 200 ms initial headroom and 100 ms refill
timers. It retires stale callbacks on call startup and closes only its own
speaker stream. Live-call DMA counters still need measurement.

The five-second test recorded 47 then 110 DMA refills during playback with
the underrun counter unchanged at 1,024 bytes and zero write drops. At the
end it recorded 161 refills and 2,304 missing bytes, accounting for exactly
162,560 delivered bytes: five seconds at 16 kHz/16-bit plus 80 ms of drain
padding. There was no continuing underrun during the sampled steady interval.
The user confirmed that the extended tone worked, with only a little crispness
at the very start. This supports correct sustained PCM playback in the isolated
test; it does not yet rule out underruns or other faults under live-call load.

The next live call also had no continuing DMA underrun or write drop: the
missing-byte counter remained at 1,024 through 97, 254, 411 and 439 refills.
Across one five-second interval the native packet-error count stayed at 38.
Nevertheless the user heard robotic ringback before the remote phone answered.
Playback starvation and packet loss alone do not explain that observation.

For waveform inspection, local-audio firmware supports `bt audio capture`
and `bt audio dump`. Capture is off by default. Explicit arming collects up to
8,192 bytes of H4 SCO packets before playback processing, beginning with a
non-silent, error-free packet; this is about 480 ms for the observed link.
Capture stops when full or disconnected. The bridge's optional
`--capture-pcm build-obelix-hci/ringback.h4` arms and retrieves one capture
to the specified local file after the audio link closes. Export is deferred
to avoid blocking playback with diagnostic output. Use an unanswered call to inspect ringback;
the capture can otherwise contain received speech. Normal metadata polling
does not arm or export audio. No uplink microphone samples are captured.

The first ringback capture contained 128 error-free native PCM packets.
Every packet's last sample was zero, and the following packet's first sample
almost exactly repeated the preceding packet's penultimate sample. The 425 Hz
ringback had strong sidebands spaced at the 266.67 Hz packet rate. This
locates a periodic fault before local playback processing, despite the native
packet status reporting no error. It does not yet distinguish a controller
decoder fault from an undocumented native PCM layout.

`CONFIG_BT_HCI_SOFTWARE_CVSD=y` provides an experimental bypass. It sets
`HAL_LCPU_CONFIG_SOFT_CVSD` to the SDK's raw-CVSD mode (`0x5a5aa5a5`) and
converts native CVSD bytes to/from the same signed 16-bit, 8 kHz HCI PCM
interface. Each byte contains eight CVSD bits, earliest bit first in bit 0;
the observed 30-byte air packet therefore produces 60 PCM bytes. Codec
state persists across packets and resets with the synchronous link. HCI
credits still count PCM packets; the native ring accounts for encoded bytes.

The codec in `lib/btutil/cvsd.c` is a project-owned Apache-2.0 implementation
of [Bluetooth Core, Vol 2, Part B, section 9.2](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-62/out/en/br-edr-controller/baseband-specification.html).
It uses a fixed-point predictor and a 128-tap low-pass filter for 8:1
interpolation/decimation. It does not incorporate the SDK's codec source.
Host tests compare the predictor with the specification's equations, check
saturation and bit order, and require at least 20 dB tone-to-distortion ratio
at 425, 1,000 and 2,500 Hz. Adapter tests cover native ring wrap, continuous
codec state, partial frames, loss status, byte-based consumption and reset.
The first hardware run confirmed raw native frames of 30 bytes and no
malformed frames. Its capture no longer had the repeated zero and duplicate
samples at packet boundaries. The 425 Hz component dominated the spectrum;
121 of 128 captured packets were error-free, with seven unavailable packets.
RMS level was 365 PCM counts versus 388 for the earlier native-PCM capture.
However, the user heard mostly silence and cutouts in that first run.
The DMA missing-byte count remained at 2,048
between refills 163 and 320, with no write drops.

The next diagnostic build uses 16x playback gain instead of 4x, bringing
the captured steady ringback's approximately 520-count peaks close to the
8,192-count standalone test tone. `bt audio gain 1..16` adjusts this gain;
normal watch mute/volume controls still apply, and clipping remains counted.
Metadata now records quiet/lost input bytes and the peak and number of
samples above 256 counts in the final DMA buffer. This separates a quiet
received signal from silence inserted farther along the playback path.
The next answered iPhone call received 7,146 packets, including 71 flagged
packets (0.99%, including connection startup/shutdown). All 428,760 received
PCM bytes entered the speaker queue. There were no microphone queue drops,
speaker write drops, malformed native frames or reported hardware errors.
The user increased phone volume and confirmed much better playback, still
somewhat robotic but acceptable for the experiment. The sampled clipping
counter reached 48 before disconnect; incoming peaks reached 2,406, which
exceeds signed 16-bit headroom at 16x gain. DMA underrun bytes rose in
1,024-byte steps during the call, reaching 5,120 across 837 refills. Each
step represents 32 ms of missing 16 kHz output, including startup gaps.

This provides a useful POC baseline, not a clean-audio acceptance result.
Next quality work should address gain headroom at higher phone volume,
intermittent playback underruns and loss concealment. The current evidence
does not attribute all remaining robotic sound to any one of those causes.
The SiFli speaker driver's circular buffer is also shared between task writes
and DMA-interrupt reads without synchronization; its write helper publishes
the new length before copying samples. That race needs correction and
validation, but has not been established as the cause of these underruns.

This stage has no acoustic echo cancellation or clock-drift compensation.
It does not provide a standalone embedded HFP host or concurrent Pebble BLE.

## Standalone embedded demo

`CONFIG_BT_FW_CLASSIC_DEMO=y` replaces the desktop host with a small
project-owned Apache-2.0 implementation in `src/bluetooth-fw/classic`, with the standalone
service adapter in `src/bluetooth-fw/classic_demo`.
Its portable core takes complete H4 packets and an output callback. The
existing SiFli transport and native-audio adapter remain below that boundary.
It supports one incoming BR/EDR connection, Just Works pairing, basic L2CAP,
an HFP SDP service on RFCOMM channel 1, credit-based RFCOMM, the mandatory
HFP service-level exchange, CVSD audio, dialing, answering and hanging up.
The phone initiates the connection from its Bluetooth settings.

This deliberately small host is separate from NimBLE. It does not yet
implement concurrent BLE, outgoing device discovery, multipoint, phonebook
download, three-way calls, codec negotiation or production reconnection.
One pairing key is kept in RAM for reconnects during the same boot. After
a restart, forget the watch in the phone's Bluetooth settings and pair again.
No vendor host stack is linked; the existing controller firmware stays below
HCI as in the earlier experiment.

Build and flash with the existing SDK tools:

```sh
CCACHE_DISABLE=1 pbl configure -b build-obelix-embedded-hfp \
  --board obelix@pvt -DCONFIG_RELEASE=n -DCONFIG_BT_FW_CLASSIC_DEMO=y
CCACHE_DISABLE=1 pbl build -b build-obelix-embedded-hfp
sftool -c SF32LB52 -p /dev/tty.wchusbserial5B7A1355001 \
  write_flash build-obelix-embedded-hfp/pebbleos.hex
```

Pair with **Pebble HFP Demo** from the phone, then open **Phone** in the
watch launcher. Until HFP is ready, the app shows a Bluetooth connection
screen. Once connected, swipe horizontally between **Dialer** and **Contacts**,
or tap their tabs. Holding Select switches pages without touch.

The dialer has a 3-column number pad, a red **×** beside the number, a `+` key,
and a green **Call** button. Tap digits to append them; Up/Down moves focus
and Select activates the focused key. The × clears the number. Only Call
starts dialing. During a call, the app offers a red **End call** button.
Saved contacts appear by name on the call screen, with the number as a fallback.
The app can close while a call continues.

**Contacts** shows names and numbers in a vertically scrollable list.
Selecting a contact dials it when the phone is ready. The Phone app reads
up to eight entries from the existing companion-synced contact favorites
used by Send Text. Those entries survive reboot and update when contact or
preference blobs change. Common visual number separators are removed before
dialing; unsupported address formats are skipped.

For development, console commands can add temporary contacts in RAM. These
are not stored in firmware configuration and disappear on reboot:

```sh
.venv/bin/python tools/hfp.py \
  --tty /dev/tty.wchusbserial5B7A1355001 \
  --contact-name Test --contact-number YOUR_NUMBER --launch
```

Contact names may contain spaces; quote them in the shell, for example
`--contact-name "Test Contact"`. Names must fit in 23 UTF-8 bytes. The helper
encodes the name for the console transport. Numbers support up to 32 dial
characters with an optional leading `+`. Select the
contact's row on the **Contacts** page to call it. This helper only configures the watch;
it does not dial or supply a Bluetooth host. `--monitor` polls host and audio
metadata. Close it before using another serial tool. Calls do not require a
USB cable or a running desktop process. Console commands are `bt hfp status`,
`bt hfp contact NAME NUMBER`, `bt hfp contacts`, `bt hfp dial NUMBER`, `bt hfp answer` and
`bt hfp hangup`.

The initial embedded build uses 229,216 bytes of the 305 KB RAM region and
1,983,808 bytes of flash. Nine portable-host tests exercise startup, SDP
decoding/continuations, ACL fragmentation, RFCOMM framing, split AT responses,
call control, number validation, timeouts and RAM-only key lifecycle using
Bumble's independent packet encoders. A 100,000-input address/undefined-behavior
sanitizer run found no parser faults. Obelix booted this host, completed all
controller initialization commands, and reported discoverable with zero
host errors. A subsequent phone session reached HFP ready with zero host errors and
completed a CVSD audio link: 4,568 received packets and 274,080 PCM bytes,
with no microphone queue drops or speaker write drops. Hardware framebuffer
captures verified the connection screen and dialer. Live incoming-call popup
behavior and longer touch interaction sessions remain to be checked.

Incoming HFP calls use the existing system phone popup even when the app is
closed. Its sidebar answers/rejects through the embedded HFP host; call start,
end and Bluetooth disconnection update that same popup. HFP does not depend
on a Pebble mobile-app session or ANCS, and unrelated BLE disconnections do
not dismiss its calls. Caller identification is not yet queried by this minimal
host. Seven phone-service tests cover existing PP/ANCS behavior and the HFP
answer, reject, end and disconnect paths.

## Dual-mode host direction

The chosen architecture extends the existing NimBLE integration
into one BLE/BR/EDR host. Keep ATT/GATT, SMP and the Pebble BLE services;
add separate BR/EDR link management, L2CAP signaling, SDP, RFCOMM and HFP
modules above a common controller core. The initial implementation has
validated concurrent HFP signaling and GATT traffic; the acceptance gates
below cover the remaining work before production use.

The implementation is selected by `CONFIG_BT_CLASSIC=y` alongside the normal
`CONFIG_BT_FW_NIMBLE` backend. `CONFIG_BT_HFP` enables the shared phone service
and demo app for either the NimBLE extension or the standalone experiment.
The portable protocol files live in `src/bluetooth-fw/classic`; `service.c`
adapts their state to Pebble phone events and the existing demo API.

The NimBLE fork adds `BLE_CLASSIC` (off by default), a typed BR/EDR and SCO
handle registry, separate/shared ACL accounting, Classic event dispatch and
command-credit gating. It checks controller BR/EDR support before enabling
Classic events. Existing LE connection, GATT and SMP implementations remain
in place. The Classic adapter runs on the NimBLE event queue and submits
commands through `ble_hs_hci_cmd_tx()`. Its managed protocol mode skips its
own controller reset, event-mask setup and buffer discovery.

The initial SiFli dual-mode transport uses fixed H4 framing and a bounded
pending packet queue instead of waiting for ACL mbufs on the receive task.
Command responses bypass that queue and can use the returned command buffer
when event pools are exhausted. Audio processing runs separately from host
protocol work, through the existing native-to-HCI adapter. Exhausting the
pending reliable-packet queue fails closed and captures a crash rather than
silently dropping ACL data. This limit still needs sustained-load testing.

The first transport is SiFli-specific; the NimBLE hooks and Classic profiles
contain no SiFli APIs. Both development and console-free release configurations
are supported. Another controller
needs an HCI transport and synchronous-audio endpoint before enabling the
option on that platform.

Build without private contacts:

```sh
CCACHE_DISABLE=1 pbl configure -b build-obelix-dual --board obelix@pvt \
  -DCONFIG_RELEASE=n -DCONFIG_BT_CLASSIC=y -DCONFIG_APP_PHONE=y
CCACHE_DISABLE=1 pbl build -b build-obelix-dual
pbl flash -b build-obelix-dual --tty /dev/tty.wchusbserial5B7A1355001 --resources
```

`bt dual status` reports NimBLE synchronization and simultaneous LE, BR/EDR,
HFP and SCO state. The existing `bt hfp status` and `bt audio probe` commands
provide profile and audio diagnostics. In dual-mode builds, pair over BLE:
CTKD derives the Classic key from the authenticated Secure Connections bond.
The bond stores the negotiated CTKD/CT2 flags alongside the LE keys, without
changing its on-disk size. Classic key lookup uses that same record; deleting
it revokes Classic access too. Separate Classic pairing is rejected in this
mode. The standalone feasibility host retains its RAM-only pairing policy.

Existing BLE-only bonds require one fresh BLE pairing: deriving a key locally
without the phone negotiating CTKD would not establish a shared bond. The
phone must provide its public identity address during pairing. Random-address
LE-only peers remain usable for BLE but cannot supply a Classic identity.
Both h6 and h7 derivation are implemented; the Pixel negotiated h7 (CT2).
Only authenticated, 128-bit Secure Connections bonds authorize HFP. RFCOMM
is blocked until Classic encryption is enabled and the controller reports a
16-byte encryption key. A weaker repeat pairing cannot replace a shared bond.

CoreApp's existing Android `createBond()` path can initiate this flow; the
watch pairing service can also request LE security. Android handles HFP and
call audio. Initial pairing tests used nRF Connect to establish GATT and
request bonding, compared the numeric codes on both screens, then confirmed
them. Android connected HFP automatically with no additional Classic pairing.

On 2026-09-19, the installed CoreApp connected through reverse PPOG V2 using
that shared bond, exchanged watch protocol and blob-database traffic, and
reconnected after a firmware restart while HFP also recovered automatically.
nRF Connect was stopped for this check. No CoreApp source changes were needed.
This validates an existing-bond Android session; full first-time CoreApp setup,
notification load and iPhone interoperability remain separate release gates.

Classic uses the normal watch name, including subsequent name changes. The
NimBLE adapter copies changes to the host task, which updates the controller
local name and extended inquiry response through standard HCI commands.

### Reconnection and bond lifecycle

The NimBLE adapter selects the single authenticated, encrypted BLE peer with a
valid CTKD bond. The portable Classic host initiates the ACL link, authenticates
it, discovers the phone's HFP Audio Gateway service through SDP, and opens
RFCOMM and the HFP service-level connection. It can also start discovery on an
existing encrypted ACL link. No controller-specific connection API is used.

The initial delay is two seconds; failed attempts back off to at most one per
minute. Connection setup has a 30-second deadline, and Bluetooth shutdown
cancels pending paging before stopping the host. Losing the eligible BLE peer
cancels paging but does not interrupt an already-established call. An explicit
phone-side profile disconnect suppresses retries until a new eligible BLE
session. Multiple eligible phones do not trigger an arbitrary choice.

SDP discovery accepts continuation fragments within a 512-byte buffer and eight
transactions. It validates the protocol descriptors and RFCOMM server channel.
The original phone-initiated RFCOMM path remains supported.

Classic authorization snapshots the shared key at connection initiation and
checks it during authentication and while encrypted. Deleting or replacing the
bond revokes the old session; late encryption events cannot revive it.

On 2026-09-19, Obelix and Pixel 8a passed BLE-triggered HFP reconnection after a
firmware restart and Bluetooth off/on. Diagnostics confirmed watch-initiated
RFCOMM and encryption on both links. Forgetting the Pixel in watch settings
disconnected both transports. Fresh BLE numeric comparison negotiated CT2 and
restored encrypted HFP automatically without a second pairing prompt. These
tests used nRF Connect, not a full CoreApp session.

HFP enables calling-line identification with `AT+CLIP=1`. The service validates
the number, handles international numbering and withheld identities, and
updates the existing incoming-call popup. Exact normalized matches in the
companion-synced favorites supply a contact name. Lookup runs on the phone
service task, outside the Bluetooth host task. On the Pixel, the local Telecom
test's reserved display number resolved to a temporary contact in the popup.

### Initial integration validation

On 2026-09-18, a non-release Obelix build completed NimBLE and Classic
startup, then paired with a Pixel 8a and reached HFP ready with zero profile
errors. With HFP still connected, nRF Connect opened a separate LE link,
discovered the standard and Pebble GATT services, and read Battery Level.
The watch reported `LE=1 BR=1 HFP=1 SCO=0 errors=0`; Android independently
reported both BR/EDR and LE ACL links to the same controller address.
That initial test validated concurrent HFP signaling and unencrypted GATT.
A subsequent CTKD test completed one BLE numeric-comparison pairing, then
reported `LE=1 BR=1 HFP=1 SCO=0 errors=0`, authenticated/encrypted BLE with
`CTKD=1 CT2=1 key_size=16`, and encrypted Classic. It does not yet establish
a full companion-app session or concurrent call audio.
A watch-UI Bluetooth off/on cycle with an active LE connection also passed:
the host disabled, links cleared and local audio remained stopped; startup
then restored Classic scan mode 3 with zero profile errors. Full companion-app
validation additionally requires successful LE bonding and Android
companion-device association.

The standalone HFP, BLE-only and dual-mode firmware configurations build.
The portable profile suite runs in both standalone and managed modes,
including ACL backpressure and shutdown during connection establishment.
The actual NimBLE Classic extension is tested with OS/transport substitutes
under address and undefined-behavior sanitizers: shared/separate buffers,
mixed completions, excess completions, malformed input, disconnect/reset,
handle reuse, LE-only capabilities and cross-transport handle collisions.
The existing phone-service tests also pass.

For a live session, use:

```sh
.venv/bin/python tools/hfp.py \
  --tty /dev/tty.wchusbserial5B7A1355001 --dual --monitor
```

### Options considered

| Approach | Reuse | Main work and tradeoff |
| --- | --- | --- |
| Extend NimBLE into a dual-mode host (preferred) | Existing Pebble BLE integration, NPL port, GATT services and SMP; portable Classic modules | Extend common controller startup, command scheduling, typed connection dispatch and buffer accounting. Keep a small, explicit NimBLE patch set. Classic security and protocol coverage still need hardening. |
| Keep two hosts behind an HCI broker | Both current hosts initially | The broker must coordinate resets, event masks, command responses, shared ACL credits and shutdown. Merely routing Classic events is insufficient. A transitional test harness is possible, but this duplicates host lifecycle state. |
| Port Zephyr's combined host | Upstream BLE and Classic protocols, including RFCOMM/HFP | Port the host's kernel/workqueue/buffer/settings dependencies and implement a replacement Pebble BLE backend. Measure memory and revalidate all existing BLE services and bonding. Keep this as the fallback if the NimBLE extension proves too invasive. |

The comparison is based on the checked-out NimBLE revision
`9ed683d0f8e2c3976d3e113b1c3015777198b2d7` and SiFli SDK revision
`bfee83c7adc0b19c2923f1788238def50f0a9dce`, plus the upstream Zephyr sources
reviewed for this proposal.

SiFli's upstream Zephyr driver is Apache-2.0, uses the controller mailbox,
and recognizes Classic/SCO framing. The inspected driver still rejects SCO
receive-buffer allocation, so its presence alone does not establish working
HFP audio. Zephyr's host does provide Classic RFCOMM and HFP options, marked
experimental in the inspected Kconfig. These are useful protocol and driver
references, with our validated audio adapter remaining a separate concern.
See the [upstream SiFli HCI driver](https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/bluetooth/hci/hci_sf32lb.c)
and [Classic host configuration](https://github.com/zephyrproject-rtos/zephyr/blob/main/subsys/bluetooth/host/classic/Kconfig).

The SDK's `middleware/bluetooth/zephyr_bt` also contains Classic sources and
an RT-Thread compatibility layer. Its `sf_port/zbt_hci_sf.c` has a
`BSP_BLE_SIBLES` coexistence path that forwards traffic to a separate BT
stack using `hl_hci_*` helpers; definitions for those helpers were not found
in the checked-out C sources. Its SCons host build substitutes
`sf_port/zbt_hci.c` for upstream `hci_core.c`. Therefore we should not treat
that vendor integration as a drop-in, vendor-neutral open host. Any reused
code needs file-level license and dependency review; no vendor host library
is part of this proposal.

### Common controller ownership

The desired module arrangement is:

```text
Pebble BLE services                  Phone service / audio endpoint
        |                                      |
NimBLE ATT/GATT/SMP                  HFP / RFCOMM / SDP / BR L2CAP
        |                                      |
        +---- one controller core / link registry ----+
                             |
                   standard HCI packet interface
                             |
            controller transport and SCO adaptation
                 SiFli IPC now; UART/other later
```

NimBLE's `ble_hs_hci_cmd_tx()` already serializes synchronous command
transactions. That is an extension point, not permission to call it from
the HCI receive task: waiting there would block the acknowledgement needed
to complete the command. Classic control work must run in the host execution
context or use an asynchronous command interface. The common owner must also
honor zero command credits, route Command Status versus Command Complete,
and distinguish command acceptance from later connection completion.

NimBLE's current `ble_hs_hci_avail_pkts` and connection completion handling
assume LE traffic. Extend the accounting before allowing Classic ACL writes.
Controllers may expose separate BR/EDR and LE pools or one shared pool;
LE Read Buffer Size returning zero selects the shared-pool case. Track
outstanding packets per typed connection and release credits exactly once
on completion or disconnect. Never give both protocol paths the full count
of a shared pool. These requirements follow the
[HCI flow-control and buffer-size specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-60/out/en/host-controller-interface/host-controller-interface-functional-specification.html).

| Resource | Sole owner | Required behavior |
| --- | --- | --- |
| Controller reset, startup and event masks | Common host core | One reset/start sequence; union of required events; capability checks before enabling Classic |
| HCI command queue and credits | Common host core | Bounded queue, exact transaction ownership, timeout recovery for both transports |
| Connection handles and ACL credits | Common host core | Explicit LE/BR/EDR/SCO type; mixed completion events; stale-handle protection |
| LE connections, SMP and GATT | NimBLE BLE modules | Preserve existing Pebble services and LE bonding |
| BR/EDR links and SSP | Classic modules | Separate link keys, encryption state and pairing policy |
| SCO packets and deadlines | Audio endpoint plus HCI adapter | Separate bounded audio buffers; control traffic and BLE allocation stalls must not block audio |
| Radio disable and recovery | Common host lifecycle | Close both link types, stop scanning/advertising, stop microphone/speaker, cancel queued work |
| Caller UI and contact lookup | Pebble phone service/app | Host-independent events and actions; no controller access from the UI |

The SiFli-specific mailbox ABI, codec conversion and synthetic synchronous
credits stay below standard HCI. The portable host must not select that ABI
or infer handle types from vendor-specific numeric ranges. A future controller
with standard HCI SCO should use the same profile and audio interfaces.

### Implementation and acceptance sequence

The main integration points are
`third_party/nimble/transport/hci_sf32lb52.c`, NimBLE
`nimble/transport/common/hci_h4`, and the host files `ble_hs_startup.c`,
`ble_hs_hci.c` and `ble_hs_hci_evt.c`. Keep additions in project-owned
Apache-2.0 modules and the NimBLE changes small enough to review separately.

1. **Shared controller core and NimBLE hooks.** Move the demo's reset,
   event-mask and buffer discovery out of its Classic state machine. Add
   explicit Classic hooks to the NimBLE fork for startup, event dispatch,
   ACL submission/completion and reset. Keep Classic disabled by default
   and compile it out on LE-only controllers. Tests must cover command
   credits reaching zero, mixed completion events, separate/shared ACL
   pools, backpressure, disconnect with packets outstanding, malformed
   events and handle reuse after reset.
2. **Concurrent BLE plus Classic signaling on Obelix.** Retain the normal
   NimBLE BLE backend and turn HFP into an optional capability rather than
   an alternative `BT_FW` choice. First demonstrate a normal Pebble BLE
   connection remaining active while the same phone establishes BR/EDR,
   completes SDP/RFCOMM and reaches HFP ready. Verify Bluetooth off/on and
   controller recovery affect both transports coherently. This is the
   first integration gate before expanding the profile implementation.
3. **Concurrent calling and BLE traffic.** Route CVSD through the existing
   audio endpoint while running notifications, GATT operations and normal
   app traffic. Verify incoming/outgoing calls, answer/reject, disconnect,
   audio starvation counters, peak queue occupancy and memory use. Exercise
   RX pool exhaustion while an HCI command is outstanding. Hardware evidence
   of simultaneous links is required; sequential BLE and Classic tests do
   not pass this gate.
4. **Service and security hardening.** The Phone system app uses an internal
   HFP service API and existing companion-synced contact favorites. LE Secure
   Connections pairing negotiates CTKD and stores one shared bonding record;
   Classic requires an authenticated 128-bit derived key and encryption.
   Stateless Classic pairing remains confined to the standalone development
   backend. HFP owns the call lifecycle when companion notifications race,
   while PP/ANCS can supply caller identity. Protocol timeout, malformed-line,
   and queue-exhaustion handling have regression coverage. BLE-triggered
   reconnect and shared-bond revocation are implemented and hardware tested.
   Basic HFP caller identification and favorite-contact lookup are implemented;
   multi-call handling remains open.
5. **Portability and release gate.** Run the common controller tests against
   simulated standard HCI controllers with both buffer layouts, and exercise
   a second controller transport when hardware is available. Measure flash,
   RAM, latency and power with BLE-only and dual-mode configurations. Keep
   the standalone experiment usable until the integrated path passes these
   checks.

If the first integration gate requires replacing most of NimBLE's connection
or scheduling internals, revisit the Zephyr-host option before growing a
large private fork. The decision should follow the size of the reviewed
patch set and measured behavior, rather than the existence of an HFP sample.

## Release acceptance status

Validated on Obelix and Pixel 8a:

- Authenticated BLE Secure Connections pairing negotiates CT2 and supplies
  the Classic link key through CTKD, without a second pairing prompt.
- Both encrypted links reconnect with the same stored bond after flashing.
  Connecting BLE automatically restores HFP, including after Bluetooth off/on.
  Watch-side bond deletion and a fresh BLE pairing also passed on the Pixel.
- A local Android Telecom incoming call can be answered and hung up from the
  watch. A three-minute run retained encrypted BLE and HFP links and moved
  bidirectional SCO audio. Android reported Bluetooth SCO for both audio
  directions and no playback underruns. BLE was connected through nRF Connect;
  this does not establish full CoreApp traffic coexistence.
- Audio notifications exposed a semaphore wait-queue race: opening the SCO
  mailbox changed the shared IRQ priority above the kernel mask. The SiFli
  IPC port now consistently uses a kernel-safe interrupt priority. The
  three-minute call above passed with the normal watchdog restored.
- The default software playback gain is unity. The earlier 16x setting
  clipped the test tone heavily; unity did not clip it. Acoustic loudness,
  voice quality and echo still require measurement.
- Two cycles of incoming, rejected and outgoing local Telecom calls passed,
  including watch answer/hangup and audio teardown. These short transitions
  complement the longer active-call tests; they do not replace soak testing.
- The reconnect build passed incoming/rejected/outgoing calls with audio-aware
  checks. One earlier incoming call produced only missing SCO payloads; repeats
  passed, but that intermittent failure remains part of the audio-quality gate.
- The Phone app is a system app (`APP_PHONE`), with companion favorites and
  explicitly temporary console contacts. No personal contacts are compiled
  into firmware. Routine bonding logs no longer emit key material.
- Host regression tests cover shared command/ACL ownership, CTKD vectors and
  negotiation, security gating, malformed responses, timeout recovery, and
  bounded output. Phone-service tests cover notification ordering; contact
  tests cover normalization, capacity, duplicate numbers and invalid addresses.
- CoreApp connected using the existing shared bond without source changes,
  exchanged reverse PPOG V2 traffic and restored its session after flashing.
  HFP remained available alongside the companion connection. Two cycles of
  incoming, rejected and outgoing local Telecom calls passed with CoreApp
  connected, including four one-minute active audio intervals at 30% watch
  volume. CoreApp and HFP also recovered after Bluetooth off/on. During a
  subsequent local call, CoreApp logs confirmed reception of periodic Pebble
  protocol pings. These are coexistence smoke checks, not a notification-load
  soak.

On 2026-09-19, timed audio samples showed mid-call DMA underruns even after
removing a redundant speaker refill callback. A two-minute local call recorded
5,120 underrun bytes. Call playback now uses an internal live PCM stream mode:
packet arrival feeds complete driver blocks directly, with bounded work and
backpressure, while incomplete blocks wait for more samples. This keeps speaker
feeding independent of background-task refill delays. The same two-minute test
at 50% watch volume recorded zero DMA underruns and zero driver write drops,
with CoreApp protocol pings enabled. SCO loss indications remained; this result
does not establish acoustic quality, echo cancellation or clock-drift tolerance.

Explicit `ble host reset` requests now use NimBLE's normal HCI reset and
resynchronization path instead of rebooting the watch. The `resets` field in
`bt dual status` counts host resets within the current watch boot. Two cycles
of idle and local incoming-call resets passed on the Pixel: both encrypted
links returned with the existing bond, ringing state was restored, and the
watch could reject the call. Repeated local audio stop/reset requests coalesce
into one pending synchronization callback, retiring old capture immediately.

This covers requested host resets, not an unresponsive or faulty controller.
SiFli controller faults and HCI timeouts still capture a crash dump and reboot
for cold recovery; bounded controller-only recovery remains a release gate.

A console-free release build can be checked independently of the debug
firmware used for flashing and diagnostics:

```sh
pbl configure -b build-obelix-hfp-release --board obelix@pvt \
  -DCONFIG_RELEASE=y -DCONFIG_PROMPT=n -DCONFIG_BT_CLASSIC=y \
  -DCONFIG_APP_PHONE=y
pbl build -b build-obelix-hfp-release
```

Keep the hardware debug build at `CONFIG_RELEASE=n`; deep sleep powers down
the debug UART. Building the release configuration is not evidence of release
readiness. Before enabling calling by default, complete these gates:

1. First-time CoreApp setup and sustained notification/GATT load during calls,
   on Android and iPhone, including deleting and re-establishing the shared bond.
   Existing-bond Android connection and firmware-restart recovery have passed.
2. Reconnect and caller-identification interoperability on iPhone, volume/mute synchronization,
   audio transfer, and call waiting/multiple-call behavior.
3. Measured speaker/microphone latency, packet-loss recovery and clock drift;
   acoustic echo cancellation using the actual playback reference.
4. Repeated incoming/outgoing/rejected calls, radio disable during calls,
   controller faults, and long-duration stress with no resource leaks or resets.
5. Release-mode power/current and CPU measurements, Bluetooth conformance and
   interoperability tests, and a second standard-HCI controller port.

The SiFli shared-memory audio adapter remains below HCI. Other controllers
can deliver standard SCO packets to the same portable host/audio boundary.

## Local Android call testing

`tools/android_hfp_test` is an Apache-2.0 Android Telecom `ConnectionService`
for testing HFP without placing a cellular or network call. It uses a local
SIP address under `.invalid`, has no network permission, and does not save
audio. Active calls send a 1 kHz tone and report microphone RMS, routed audio
devices, sample counts and playback underruns to the `HfpTest` logcat tag.
A call ends automatically after five minutes. Keep the activity visible;
this diagnostic app does not run a background foreground service.

Build with Android SDK platform 36, build-tools 36.0.0, and a JDK supporting
`javac --release 8`. The builder defaults to the macOS Android Studio SDK
and JDK locations; `--sdk` and `--java-home` override them. No Gradle or
third-party Java dependencies are downloaded.

```sh
python tools/android_hfp_test/build.py
adb install -r -g build-android-hfp-test/hfp-test.apk
adb shell am start -n com.teslabs.hfptest/.MainActivity \
  --es device WATCH_BLUETOOTH_ADDRESS --es command incoming
adb logcat -s HfpTest
```

Replace `WATCH_BLUETOOTH_ADDRESS` with the already bonded watch's address.
`-g` grants the test app microphone and nearby-device permissions. Answer
or reject on the watch to exercise HFP. The activity also supports
`outgoing`, `active`, `bluetooth`, and `hangup` commands; `outgoing` creates a
local dialing call and `active` simulates the remote party answering it.
Always end the test explicitly when finished:

```sh
adb shell am start -n com.teslabs.hfptest/.MainActivity --es command hangup
```

This tests Android Telecom routing and HFP signaling, including VoIP-style
calls. It does not replace cellular-call, iPhone, acoustic quality, or
long-duration coexistence testing. Android can establish SCO while ringing;
missing-packet indications before answer must be distinguished from loss
while active. Check that both logged audio routes use Bluetooth SCO before
interpreting microphone RMS as watch capture.

With both encrypted links connected, the repeatable hardware check runs
incoming, rejected, and local outgoing calls, then verifies audio teardown:

Set the watch speaker volume to a comfortable level under Settings > Sounds &
Haptics before running audible tests. The runner preserves that setting.

```sh
python tools/hfp_smoke.py --tty WATCH_SERIAL_PORT --android-serial ANDROID_SERIAL \
  --device WATCH_BLUETOOTH_ADDRESS --duration 60 --repeat 3
```

For a CoreApp coexistence check, add `--companion-ping` to send a Pebble
protocol ping every five seconds during active audio. Verify reception in
CoreApp's `PebbleProtocolRunner` logs for the watch address; generating pings
alone does not verify their delivery. This option does not send notifications
or messages to another person.

`--scenario incoming|reject|outgoing` selects one scenario. The runner checks
both encrypted links throughout active calls and stops its local test call
on failure. It refuses to begin while the watch already reports a call.
Active calls must consume transmit packets and receive a majority of valid SCO
payloads; an audio connection containing only missing packets fails the test.

For timed audio-counter samples, add `--audio-interval 10` to the call runner.
`--max-underrun-bytes 0` additionally fails if DMA underruns increase during
an active call. Inspect the initial sample too: the delta check deliberately
separates startup behavior from subsequent playback.

To exercise idle and ringing-call host resets without placing a real call:

```sh
python tools/hfp_recovery.py --tty WATCH_SERIAL_PORT --android-serial ANDROID_SERIAL \
  --device WATCH_BLUETOOTH_ADDRESS --repeat 2
```

CoreApp must already be connected. The runner checks the reset counter so a
whole-watch reboot cannot pass as host recovery, and requires restoration of
both encrypted links without re-pairing. Its incoming calls remain inside the
local Android test app, are never answered, and are cleaned up on failure.
