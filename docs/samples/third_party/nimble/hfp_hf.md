# NimBLE: hands-free calls

This sample takes the calls of a phone, as the Hands-Free side of the
Hands-Free Profile over Bluetooth Classic, while it advertises and accepts
connections over Low Energy: it needs a dual-mode controller, and a microphone
and a speaker for the audio of the call.

A phone pairs with it over Low Energy. The pairing uses Secure Connections and
asks for the Classic link key to be derived from its keys, so the phone then
connects over Classic and starts the hands-free service without pairing
again. The sample is connectable over Classic, but phones do not find it
there: `hfp discoverable` lets them, to pair over Classic instead.

The calls are driven from the shell:

| Command | Does |
| --- | --- |
| `hfp answer` | Answers the incoming call |
| `hfp hangup` | Rejects the incoming call, or ends the one in course |
| `hfp dial <number>` | Places a call |
| `hfp volume <0-15>` | Tells the phone the speaker gain |
| `hfp audio on`, `hfp audio off` | Takes the audio of the call, or gives it back to the phone |
| `hfp connect`, `hfp disconnect` | Connects to the phone seen last, or disconnects from it |
| `hfp status` | Shows the connections and the counters of the audio |

The audio of a call is 16-bit samples, passed between the synchronous
connection and the audio streams through two small queues. The phone picks the
codec of each call: narrowband speech at 8 kHz, which the controller codes, or
wideband speech at 16 kHz, whose frames the sample codes itself. The
speaker is kept below the level where it distorts, and the microphone gain
below the one where the speaker saturates it.

The speaker is right next to the microphone, so the caller would hear
themselves back, louder than whoever talks to the device. An echo canceller
takes what the speaker plays out of what the microphone captures. An adaptive
filter learns the path between the two from the samples given to the speaker.
What it leaves, which includes all that the speaker distorts, is then turned
down frequency by frequency wherever the echo expected dominates, so that
someone talking over the caller still gets through. `hfp echo off` sends the
microphone as it is, to compare, and `hfp status` shows how much the canceller
takes out, and how much of that is the filter alone.

A few more commands check the audio without anyone talking: `hfp tone <Hz>`
sends a tone in place of the microphone, `hfp beep <Hz>` plays one on the
speaker, `hfp keep received` and `hfp keep mic` keep half a second of what the
phone sends or of what the microphone captures, which `hfp dump` prints, and
`hfp gain` sets the gain of the speaker or of the microphone.

A connection that carries nothing is put in sniff mode, in which the two
devices only meet twice a second, so that the radio rests in between. The
phone usually asks for it; the sample does after a few seconds if it has not.

Bonds and link keys are kept in a small file system on the flash, so they
outlive a reset: the device then connects to the phone it has a link key of,
without pairing again.

```{pebbleos-sample}
:path: samples/third_party/nimble/hfp_hf
:sources: src/main.c src/aec.c prj.conf
:output:
```
