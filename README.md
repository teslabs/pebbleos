# Pebble Workout redesign

Edited QEMU feature walkthrough for commit `d87a9317b`. All watch pixels are native firmware output. Video captions sit outside the display. Workout data is synthetic and injected into the emulator service memory.

- [Full captioned recording (MP4)](workout-features.mp4)
- [Full native-screen recording (GIF)](workout-features.gif)
- [Screenshot gallery](screenshots.png)

| Time | Features |
| --- | --- |
| 0:00 | Selection, countdown and Run tracking |
| 1:30 | Recap and retained HR history |
| 1:47 | Finish using buttons |
| 2:00 | Return to the picker |
| 2:09 | Walk metrics |
| 2:25 | Finish using touch |
| 2:37 | Choose Open workout |
| 2:45 | Open workout metrics and completion |
| 3:09 | Complete and return to selection |

The captures cover Run, Walk and Open workouts; touch/button selection; startup and cancellation; all metric pages; rolling HR history; pause/resume; cancelled end confirmation; touch/button completion; and browsable recap/history.

Validation: color and monochrome firmware builds, all nine focused CTest entries, and gitlint for both signed commits. Rectangular and round native render fixtures are included in the code PR.
