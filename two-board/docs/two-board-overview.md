# Two-board setup

Adds to [`system-overview.md`](../../docs/system-overview.md), which is unchanged. This describes the setup with a separate controller (handheld) and motor board (receiver).

## Current Board Setup (Genesis Mini)

There are now two Genesis Mini boards linked by ESP-NOW (channel 1). Module manifests are in [`two-board/axiometa.handheld.json`](../axiometa.handheld.json) and [`two-board/axiometa.receiver.json`](../axiometa.receiver.json); the repo-root [`axiometa.json`](../../axiometa.json) is the handheld's.

### Handheld (controller)

| Port | Module | Part | Pins used | Role |
|---|---|---|---|---|
| P1 | AX22-0003 | Rotary encoder | `IO0` button, `IO1` CLK, `IO2` DT | Turn = speed, press = listen for a voice command |
| P2 | AX22-0050 | LED button | `IO1` button, `IO2` LED | Emergency stop. LED on = stopped |
| P3 | AX22-0018 | Buzzer | `IO1` | Feedback beeps; 600 Hz stop tone |
| P4 | AX22-0044 | PDM microphone | `IO1` data, `IO2` clock, `IO0` driven low | 2 s voice clips sent to the computer over USB serial |

### Receiver (chassis, motors)

| Port | Module | Part | Pins used | Role |
|---|---|---|---|---|
| P2 | AX22-0032 | Motor driver | `IO1`, `IO2` | Left motor |
| P3 | AX22-0032 | Motor driver | `IO1`, `IO2` | Right motor |

The receiver obeys only the handheld's MAC address, and stops its motors if no command arrives for 400 ms. The handheld sends a command every 100 ms.

## Software Status

**Firmware** ([`two-board/`](../)) — PlatformIO project.
- `handheld`: encoder sets speed, the stop button and the Studio heartbeat (1.2 s) stop the rover, voice words (`forward`, `reverse`, `left`, `right`, `stop`) from the computer set the direction, with a ramp from 80 to 160. Includes a calibration mode.
- `receiver`: drives two motors from ESP-NOW packets; also accepts `SET:left,right` over USB until the first radio packet.
- `control_test`: the earlier sketch that starts only when the camera reports a clear path (intelligent disobedience).

**Browser** ([`browser/Camera Test.html`](../../browser/Camera%20Test.html)) — runs in Axiometa Studio on a computer connected by USB.
- Object detection with TensorFlow.js COCO-SSD. An object overlapping the centre zone sends `CAM:BLOCK`.

**Navigation** ([`laptop/`](../laptop/)) — paused. Route planning (geocoding and walking routes), a pure-pursuit follower and a simulation. Not connected to the two-board firmware.

### Serial protocol (computer ↔ handheld)

| Direction | Message | Meaning |
|---|---|---|
| Computer → handheld | `PING` | Heartbeat; the handheld stops if it is silent for 1.2 s |
| Computer → handheld | `RECORD` / `VOICE:<epoch>:<word>` / `STOP` | Request a clip, run a recognised word, stop |
| Computer → handheld | `CAL:<move>:<ms>` | Calibration run |
| Handheld → computer | `AUDIO_BEGIN` / `PCM:<hex>` / `AUDIO_END` | A 2 s clip, 16 kHz 16-bit mono |
| Handheld → computer | `STATE:...`, `STOP:...`, `ERROR:...` | Status lines |
| Browser → control_test | `CAM:CLEAR` / `CAM:BLOCK` / `CAM:OFF` | Latest detection frame (about every 200 ms) |

### Not done yet
- Ultrasonic sensor
- Camera steering on the rover (the camera test only gates `control_test` today)
- Route navigation on the two-board firmware
- Phone app (data collection)
- Rejecting replayed packets (the packet `sequence` is sent but not checked)
