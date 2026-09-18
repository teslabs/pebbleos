# Bluetooth calling proof of concept

## Scope and constraints

Obelix is the first target for a watch acting as a Bluetooth Hands-Free (HF)
device. The phone is the Audio Gateway (AG) and places the cellular call.
The eventual feature needs call control, simultaneous microphone/speaker
audio, and coexistence with Pebble's BLE phone connection.

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

## Extending NimBLE

Extending the existing Apache-2.0 NimBLE host is a first-class implementation
option. It preserves Pebble's BLE integration, but requires more than
adding HFP AT commands:

| Existing area | Required addition |
| --- | --- |
| H4 framing | Packet type `0x03`, synchronous packet allocation and delivery |
| HCI dispatcher | Classic connection, pairing and synchronous-link events |
| Connections | BR/EDR link type, addresses, role and shared ACL accounting |
| L2CAP | Classic signaling on CID `0x0001`, connection/configuration requests and dynamic channels |
| Security | SSP/link keys and BR/EDR bonding, distinct from LE SMP |
| Profiles | SDP server/client, RFCOMM and HFP HF state machines |
| Audio | SCO pacing, codec negotiation, frame loss and shutdown |

The current H4 parser has no SCO allocator. The L2CAP signaling dispatcher
contains Classic response opcodes as no-ops; that is not an implementation
of Classic channel setup. Controller command credits, event masks, resets
and ACL credits must have one owner. Adding a second independent host on
the same mailbox is insufficient.

After the controller audio experiment, compare the size of a focused
NimBLE extension with a port of upstream Zephyr's Apache-2.0 host. Keep
HFP and Classic protocol logic separate from Pebble UI and SiFli code in
either case. No embedded host replacement has been selected by the
desktop experiment.

### Proposed NimBLE implementation sequence

Prefer extending the existing integration for the first embedded prototype,
subject to the HCI audio gate. Put new Classic modules in project-owned
Apache-2.0 code and keep the changes to the NimBLE submodule explicit and
reviewable. Reusing Apache-2.0 protocol implementations is an option after
checking the licenses of the particular files and their dependencies.

1. Extend transport and host ownership together. Add SCO framing and a
   separate bounded synchronous-data pool; do not consume ACL buffers for
   voice. Teach startup to enable the required Classic events and initialize
   both BR/EDR and LE buffer accounting. Keep one command scheduler and one
   reset/recovery path. Validate fragmented and malformed H4 packets and
   mixed Classic/LE completion events before adding profiles.
2. Add BR/EDR connection records and handle-based ACL dispatch, then SSP,
   encryption and link-key storage. Reuse the watch's pairing UI with
   explicit user confirmation. Keep Classic keys separate from LE bonds;
   cross-transport key derivation is outside the first prototype.
3. Implement Classic L2CAP basic mode, signaling/configuration, SDP and
   RFCOMM sufficient for an HFP HF. Support the phone's service discovery
   and connection setup as well as reconnection. Test protocol state
   machines against the desktop host before exercising phones.
4. Implement the HFP service-level connection and mandatory call control,
   starting with CVSD. Expose call state, answer/hang-up and audio-link
   events to a Pebble call service; keep AT commands inside the profile.
   Defer mSBC and optional HFP features until narrowband calls work.
5. Connect SCO PCM to the audio service through bounded capture/playback
   queues. CVSD uses 8 kHz speech; adapt the board's 16 kHz audio explicitly
   and verify the actual controller payload format. Measure underflows,
   overflows and drift while BLE traffic continues, then add echo control.

The concrete integration points in the current checkout are
`third_party/nimble/transport/hci_sf32lb52.c`, upstream
`nimble/transport/common/hci_h4`, and the host's `ble_hs_startup.c`,
`ble_hs_hci.c`, `ble_hs_hci_evt.c` and `ble_l2cap_sig.c`. In particular,
startup currently installs an LE-oriented event mask and chooses a single
ACL pool. Merely registering Classic event handlers would therefore leave
both event delivery and dual-mode flow control incomplete.

This is a host-stack development project, not a small HFP feature patch.
The first two steps provide an early decision point: retain the extension
if BLE regression tests and resource measurements remain acceptable;
otherwise evaluate an Apache-2.0 dual-mode host port using the same HCI
transport and call-service boundary. Proving the SiFli audio path first
avoids committing that effort before its main hardware dependency is known.

## Remaining acceptance gates

1. Pair a phone; complete SDP, RFCOMM and the HFP service-level connection.
   Passed with an iPhone on 2026-09-18; Android remains untested.
2. Establish SCO/eSCO and observe bidirectional voice payloads over HCI.
   Confirm uplink reception at the peer, not merely successful writes.
   Passed with CVSD and a user-confirmed audible tone on 2026-09-18.
3. Exercise Obelix's microphone and speaker simultaneously. The board
   configures 16 kHz audio on separate DMA channels. Basic two-way speech
   passed on 2026-09-18; quality, latency and long-term stability remain open.
4. Implement the embedded host and integrate call state, deduplicating
   HFP versus PP/ANCS events for the same phone.
5. Bound audio queues, handle clock drift and packet loss, and add acoustic
   echo cancellation with the actual playback reference.
6. Demonstrate incoming/outgoing calls, mute/volume, clean disconnects,
   reconnection, and a ten-minute call with concurrent Pebble BLE traffic
   on Android and iPhone. Measure CPU, RAM, latency and battery current.

The tested SiFli controller uses native shared-memory audio. Its adapter
keeps this detail below HCI, preserving packet status, handles, codec format
and pacing. Other controllers can provide standard SCO directly through the
same audio boundary. Vendor voice APIs do not belong in the call service.
