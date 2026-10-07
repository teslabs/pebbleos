# Accelerometer recordings and FSM programs

The LSM6DSO can run small programs, finite state machines (FSM), on its own
accelerometer data and raise an interrupt when a motion pattern is seen, e.g. a
wrist flick. `tools/accel_fsm` helps develop such programs offline: it records
raw accelerometer data on a watch, and runs FSM programs on those recordings in
a simulator.

## Recording

Firmware built without `CONFIG_RELEASE` includes the `accelrec` shell command
(`CONFIG_SERVICE_ACCEL_MANAGER_RECORDER`). It records every accelerometer
sample, at the driver rate closest to the requested one, to a file in the
filesystem, so the watch can be unplugged while recording. Up to 10
recordings are kept.

Drive it from the host over PULSE, with the debug cable connected:

```shell
export ACCEL_FSM_URL=/dev/cu.usbserial-XXXX   # or socket://localhost:12345 for QEMU
python -m tools.accel_fsm record start flick   # label; --rate 52, --max-kib 1024 by default
# ... unplug, perform the activity, plug back in ...
python -m tools.accel_fsm record stop
python -m tools.accel_fsm record pull -o recordings/
python -m tools.accel_fsm record remove all
```

At 52 Hz, a recording takes about 19 KiB per minute. Label recordings by
activity (`flick`, `raise`, `walk`, `run`, `bed`, ...): the simulator reports
events per label, i.e. detections on recordings of the gesture and false
positives on everything else.

`python -m tools.accel_fsm info` describes a recording. Samples are stored in
milli-g, in watch axes. If the recorder falls behind (e.g. the background task
is blocked for seconds), part of the data is lost; `info` reports these gaps
and the simulator restarts the programs after each of them.

## Programs

Programs are written in the textual form the disassembler produces:

```
thresh 1 +1.5000
thresh 2 -1.5000
mask A +X -X
timer TI3 8
code:
  NOP|GNTH1
  TI3|LNTH2
  CONTREL
```

Each line of code is either a command (`CONTREL`, `SELMB`, ...) or a pair of
`RESET|NEXT` conditions: the program moves on when `NEXT` holds and goes back
to the reset point when `RESET` holds. Thresholds are in g. `disasm` also
reads ST's `.ucf` configuration files, e.g. the
[ST examples](https://github.com/STMicroelectronics/STMems_Finite_State_Machine),
and `asm` prints the program bytes to load on the sensor.

```shell
python -m tools.accel_fsm disasm tools/accel_fsm/tests/st/lsm6dso_wrist_tilt_xl.ucf
python -m tools.accel_fsm sim flick.fsm recordings/*.bin --odr 26
```

`sim` converts the samples to the sensor axes of the recording's board,
decimates them to `--odr` and clips them to `--fs`.

The simulator follows ST AN5226 (LSM6DSO: Finite State Machine) and is checked
against ST's example programs (`pytest tools/accel_fsm/tests`). Programs that
need inputs other than the accelerometer, the long counter (`INCR`) or `SETP`
are rejected.
