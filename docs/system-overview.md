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

**Browser** ([`browser/Camera Test.html`](../browser/Camera%20Test.html)) — runs in Axiometa Studio on a computer connected by USB.
- Object detection with TensorFlow.js COCO-SSD. An object overlapping the centre zone sends `CAM:BLOCK`.

### Serial protocol

| Direction | Message | Meaning |
|---|---|---|
| Browser → board | `CAM:CLEAR` / `CAM:BLOCK` | Latest detection frame (about every 200 ms) |
| Browser → board | `CAM:OFF` | Camera stopped |
| Board → browser | `Running ...` / `Stopped ...` / `Start blocked ...` | Status lines |

### Not done yet
- Motors and steering
- Voice control (teammate)
- Ultrasonic sensor
- Phone app (data collection)
