# System Overview

Guide-Robodog is a wheeled rover that acts as a robotic guide dog for visually impaired users. It is built on the [Axiometa Genesis Mini Starter Kit](https://www.axiometa.io/products/axiometa-genesis-mini-starter-kit).

The rover lives at ground level (the **chassis**) and the user holds a **handle** attached to it, just like a guide dog harness.

## Workstreams

### Hardware
- [ ] 3D print chassis and handle parts
- [ ] Assemble the rover
- [ ] Connect the boards
- [ ] Test power and wireless features
- [ ] Mount and test the ultrasonic sensor

### Software
- [ ] Phone connectivity
- [ ] Computer vision
- [ ] Voice control
- [ ] Navigation

### Functionality
- [ ] Haptics

## Inputs and Outputs

| Input | Location | Measures / detects | Output / behaviour |
|---|---|---|---|
| Ultrasonic sensor | Chassis | Distance to obstacles | Steers by driving the left or right wheel, or stops. Keeps the user walking in a straight line. |
| Microphone | Handle | Voice commands from the user | Sets the navigation path. Replies through the speaker or haptics (user's choice). |
| Camera + CV | Chassis | Obstacles, including moving ones | Steers and guides around moving obstacles |
| LED button | Handle | User press | Emergency start / stop |
| Rotary encoder | Handle | User rotation | Tuning settings |
| Buzzer | Handle | — (output only) | Haptic feedback for stopping or an approaching obstacle |
| Phone app | — | — | Data collection only, not needed for the rover to work |

## Intelligent Disobedience

Real guide dogs refuse a command when following it would put the user in danger. For example, a dog won't step forward at a pedestrian crossing if a car is coming, even when told to. Blind people rely on hearing to cross roads, so the dog acts as a safety backstop.

The rover follows the same principle:
- If the user tells it to move forward but an obstacle is detected, it **stops and refuses** to move.
- It signals the refusal through the handle's buzzer/haptics so the user knows why it stopped.
- It also keeps the user on a straight line so they don't drift or trip.

## Where the code is

| Part | Folder | Status |
|---|---|---|
| Handheld + receiver firmware (two boards, ESP-NOW, motors) | [`two-board/`](../two-board/README.md) | **Main firmware** |
| iPhone app (host: camera, voice, navigation) | [`ios/`](../ios/README.md) | Main app |
| Single-board Studio test (Bluetooth, camera interlock, cues, no motors) | [`firmware/`](../firmware/) | Earlier test, kept for Axiometa Studio |
| Browser camera test over USB | [`browser/`](../browser/) | Earlier test |

The sections below describe the earlier single-board test. For the two-board setup see [`two-board/docs/two-board-overview.md`](../two-board/docs/two-board-overview.md).

## Current Board Setup (Genesis Mini)

Defined in [`axiometa.json`](../axiometa.json). All four modules sit on the handle side for now.

| Port | Module | Part | Pins used | Role |
|---|---|---|---|---|
| P1 | AX22-0003 | Rotary encoder | `IO0` button, `IO1` CLK, `IO2` DT | Press = start, turn = speed (0–100%, 5% steps) |
| P2 | AX22-0050 | LED button | `IO1` button, `IO2` LED | Emergency stop. LED on = stopped |
| P3 | AX22-0018 | Buzzer | `IO1` | 2 kHz, 300 ms warning beep on stop |
| P4 | AX22-0044 | PDM microphone | `IO1` data, `IO2` clock | Voice control (in progress by a teammate) |

## Software Status

**Firmware** ([`firmware/`](../firmware/)) — control test, no motors connected yet.
- Starts only when the camera reports a clear path (intelligent disobedience).
- Stops and beeps on: stop button, camera `BLOCK`, camera off, or no camera frame for 2 s.
- Plays navigation cues on the handle buzzer while running (see below). Stop beeps cancel any cue.

- Bluetooth LE as "Robodog" ([`firmware/ble.h`](../firmware/ble.h), Nordic UART Service). Accepts the same messages as USB; status lines go to both.

**iPhone app** ([`ios/`](../ios/README.md)) — rides on the chassis and replaces the Mac. Rear-camera obstacle detection, Apple Maps walking route with turn cues, Bluetooth link. See the setup guide in `ios/README.md`.

**Browser** ([`browser/Camera Test.html`](../browser/Camera%20Test.html)) — runs in Axiometa Studio on a computer connected by USB.
- Object detection with TensorFlow.js COCO-SSD. An object overlapping the centre zone sends `CAM:BLOCK`.
- Navigation cue buttons and a timed demo route that send `NAV:` cues to the board.

### Serial protocol

Messages travel over USB serial (Camera Test) or Bluetooth (iPhone app), one per line.

| Direction | Message | Meaning |
|---|---|---|
| Browser → board | `CAM:CLEAR` / `CAM:BLOCK` | Latest detection frame (about every 200 ms) |
| Browser → board | `CAM:OFF` | Camera stopped |
| Browser → board | `NAV:LEFT` / `NAV:RIGHT` / `NAV:STRAIGHT` / `NAV:ARRIVED` | Navigation cue (ignored while stopped) |
| Board → browser | `Running ...` / `Stopped ...` / `Start blocked ...` | Status lines |
| Board → browser | `Nav cue: ...` / `Nav ignored: ...` | Navigation cue result |

### Haptic patterns (handle buzzer)

| Cue | Pattern |
|---|---|
| Stop / obstacle | One 2000 Hz beep, 300 ms. Always overrides other cues. |
| Left | Two short beeps (1500 Hz) |
| Right | One long beep (1500 Hz, 600 ms) |
| Straight | One short low beep (1000 Hz) |
| Arrived | Three rising beeps |

The browser page and the iPhone app both have a timed demo route. The iPhone app also does real routes with Apple Maps. Later the same `NAV:` messages can drive the motors.

### Not done yet
- Motors and steering
- Voice control (teammate)
- Ultrasonic sensor
- Phone app (data collection)
