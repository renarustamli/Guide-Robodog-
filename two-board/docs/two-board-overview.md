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
| P4 | AX22-0044 | PDM microphone | `IO1` data, `IO2` clock, `IO0` driven low | 3 s voice clips sent to the host (iPhone over Bluetooth, or computer over USB) |

### Receiver (chassis, motors)

| Port | Module | Part | Pins used | Role |
|---|---|---|---|---|
| P2 | AX22-0032 | Motor driver | `IO1`, `IO2` | Left motor |
| P3 | AX22-0032 | Motor driver | `IO1`, `IO2` | Right motor |

The receiver obeys only the handheld's MAC address, and stops its motors if no command arrives for 400 ms. The handheld sends a command every 100 ms.

## Software Status

**Firmware** ([`two-board/`](../)) — PlatformIO project.
- `handheld`: encoder sets speed, the stop button and the host heartbeat (1.2 s) stop the rover, voice words (`forward`, `reverse`, `left`, `right`, `stop`) from the host set the direction, with a ramp from 80 to 160. Forward needs a clear camera frame. Navigation cues buzz the handle. Bluetooth link to the iPhone app. Includes a calibration mode.
- `receiver`: drives two motors from ESP-NOW packets; also accepts `SET:left,right` over USB until the first radio packet.
- `control_test`: the earlier sketch that starts only when the camera reports a clear path (intelligent disobedience).

**Browser** ([`browser/Camera Test.html`](../../browser/Camera%20Test.html)) — runs in Axiometa Studio on a computer connected by USB.
- Object detection with TensorFlow.js COCO-SSD. An object overlapping the centre zone sends `CAM:BLOCK`.

**Navigation** ([`laptop/`](../laptop/)) — paused. Route planning (geocoding and walking routes), a pure-pursuit follower and a simulation. Not connected to the two-board firmware.

**iPhone app** ([`ios/`](../../ios/README.md)) — the handheld's host over Bluetooth: heartbeat, camera obstacle check, voice recognition, walking routes.

### Host protocol (iPhone over Bluetooth, or computer over USB ↔ handheld)

| Direction | Message | Meaning |
|---|---|---|
| Host → handheld | `PING` | Heartbeat; the handheld stops if it is silent for 1.2 s |
| Host → handheld | `RECORD` / `VOICE:<epoch>:<word>` / `STOP` | Request a clip, run a recognised word, stop |
| Host → handheld | `CAL:<move>:<ms>` | Calibration run |
| Host → handheld | `CAM:CLEAR` / `CAM:BLOCK` / `CAM:OFF` | Camera frames; forward needs a clear one in the last 2 s |
| Host → handheld | `NAV:LEFT` / `NAV:RIGHT` / `NAV:STRAIGHT` / `NAV:ARRIVED` | Buzzer turn cue, only while moving |
| Handheld → host | `LISTEN:...` | Encoder clicked; host should send `RECORD` |
| Handheld → host | `AUDIO_BEGIN:<epoch>` / audio / `AUDIO_END:<epoch>:<bytes>` | A 3 s clip, 16 kHz 16-bit mono. USB: `PCM:<hex>` lines. Bluetooth: raw bytes on the audio characteristic |
| Handheld → host | `REFUSED:...` / `NAV:...` | Forward refused (camera), cue played or ignored |
| Handheld → host | `STATE:...`, `STOP:...`, `ERROR:...` | Status lines |

### Not done yet
- Ultrasonic sensor
- Camera steering around obstacles (the camera only stops forward motion today)
- Navigation cues steering the motors (they only buzz the handle today)
- Phone app data collection
- Rejecting replayed packets (the packet `sequence` is sent but not checked)
