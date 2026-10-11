# Guide rover prototype (Axiometa Genesis Mini)

Prototype only: test with the wheels off the ground first, then with a sighted supervisor holding the stop button.

## Two build targets

Pick one in the VS Code PlatformIO status bar, or use `-e` on the command line.

| Env | What it builds | Command |
|---|---|---|
| `control_test` (default) | Your control sketch: encoder sets speed and starts, LED button stops, buzzer, camera interlock over serial | `pio run -e control_test -t upload` |
| `handheld` | **Controller board**: encoder speed, stop button, buzzer, mic, ESP-NOW to the receiver, Bluetooth to the iPhone app | `pio run -e handheld -t upload` |
| `receiver` | **Motor board** (your Studio Axiometa sketch): ESP-NOW from the handheld drives two AX22-0032 motor modules on P2 and P3 | `pio run -e receiver -t upload` |
| `rover` | Voice + route-following firmware, single-board design. **Paused**, not built with the others | `pio run -e rover -t upload` |

## Layout

| Path | What it is |
|---|---|
| `platformio.ini` | Genesis Mini settings (4 MB flash, 2 MB QSPI PSRAM) and the two envs |
| `include/genesis_pins.h` | Genesis Mini port names (`P1_IO0` ...), copied from the Arduino-ESP32 board definition |
| `include/pins.h` | **Every GPIO the project uses, in one place.** Fails to build if two functions share a pin |
| `src/control_test/control_test.cpp` | Your sketch (logic unchanged) |
| `src/control_test/voice_capture.h` | Your microphone capture (2 s clips sent as hex over serial); pins now read from `pins.h` |
| `src/receiver/receiver.cpp` | Motor board sketch (your `sketch.ino`, logic unchanged) |
| `src/handheld/` | Controller board: `handheld.cpp`, `rover_radio.h` (ESP-NOW sender), `voice_capture.h` (3 s clips to USB or Bluetooth), `ble_link.h` (Bluetooth to the iPhone), `nav_cues.h` (buzzer turn cues) |
| `src/receiver/rover_radio.h` | Receiver side of the same ESP-NOW protocol (channel 1, 12-byte packet, signed PWM -255..255) plus the sender filter |
| `axiometa.json`, `axiometa.handheld.json` | Studio Axiometa module manifests: the receiver (AX22-0032 on ports 2, 3) and the handheld (ports 1 to 4) |
| `src/main.cpp`, `follower.h`, `motors.h` | Paused single-board rover firmware: audio, wake word, Wi-Fi hand-off, route following, motors |
| `laptop/` | Whisper + route planner server, and a simulation of the follower |

## Two boards

- **Handheld controller** (`handheld`): encoder speed, stop button, buzzer, microphone. Sends a `RoverPacket` to the receiver every 100 ms. Its host is the iPhone app over Bluetooth (see below) or the Studio panel over USB; both use the same text commands. `control_test` is the earlier camera-interlock test sketch for the same board.
- **Receiver** (`receiver`): motors only. Left motor P2_IO1/P2_IO2 (GPIO 6/5), right motor P3_IO1/P3_IO2 (GPIO 16/15). It stops if no command arrives for 400 ms, and only obeys the handheld MAC set in `rover_radio.h` (`ROVER_ALLOWED_SENDER`; all zeros = ignore all radio packets).
- Receiver MAC `A0:F2:62:EB:98:BC` is set in `src/handheld/rover_radio.h`. The handheld MAC `A0:F2:62:EB:92:D8` is allowlisted in `src/receiver/rover_radio.h`; if you replace the handheld, update `ROVER_ALLOWED_SENDER` with its startup MAC.
- The pin map below is for the handheld and the paused single-board design. The receiver's pins are set in `receiver.cpp`.

## Handheld with the iPhone app

The handheld advertises over Bluetooth as **Robodog** (Nordic UART Service, plus an audio characteristic `6E400004-…`). The iPhone app in [`../ios/`](../ios/README.md) is its host:

- **Heartbeat**: the app sends `PING` every 0.4 s. Without one for 1.2 s the handheld stops, as before.
- **Voice**: click the encoder → handheld sends `LISTEN` → app sends `RECORD` → handheld records **3 s** and streams raw PCM over Bluetooth, then `AUDIO_END:<epoch>:<bytes>` → the app recognises speech on the phone and sends `VOICE:<epoch>:<word>`. "Guide me to X" starts a walking route in the app instead.
- **Camera (intelligent disobedience)**: the app sends `CAM:CLEAR` / `CAM:BLOCK` about 5 times a second. `forward` is refused (`REFUSED:...`, long low beep) unless the camera reported clear in the last 2 s; an obstacle, `CAM:OFF` or 2 s of silence stops forward motion. Build with `-DREQUIRE_CAMERA=0` to bench-test without a camera.
- **Going around obstacles**: the camera splits its view into left / centre / right and only counts objects that look close. If the centre is blocked but a side is free, the app sends `CAM:AVOID:LEFT` or `CAM:AVOID:RIGHT`. While driving forward the handheld makes a detour that rejoins the path: turn toward the free side until the centre is clear (time T), forward 1.5 s, turn back for 2T, forward 1.5 s, turn for T to face the original heading, then carry on (`AVOID:LEFT|RIGHT`, `AVOID:Passing`, `AVOID:Rejoining`, `AVOID:Done`). It stops instead if both sides are blocked, if turning away takes over 4 s, if something is ahead on a detour leg, or if the camera is lost. Timing based, so approximate: tune `AVOID_PASS_MS` and `AVOID_SPEED` in `handheld.cpp`. Only objects the camera model recognises count; walls, glass and steps are not seen.
- **Turning power**: all turns (voice left/right, detour turns) run at full power (`TURN_SPEED` 255) from the start, because skid-turning on the spot needs it; forward ramps from 120 to the encoder's top speed.
- **Obstacle stops** while driving forward slow down over 0.9 s with three warning beeps instead of cutting power. Every other stop (button, voice, heartbeat, camera off) is immediate.
- **Heartbeat while stopped**: if `PING`s lag while the rover is already stopped, the handheld reports `HOST:Heartbeat lost` but does not stop again, so a voice command already on its way is not rejected.
- **Navigation**: the app sends `NAV:LEFT` / `NAV:RIGHT` / `NAV:STRAIGHT` / `NAV:ARRIVED` near each turn; the handle buzzes (two short / one long / one low / rising). Ignored while stopped. Stop tones cancel any cue.

USB still works the same way for the Studio panel. Clips requested over USB are sent as `PCM:<hex>` lines, now 3 s long.

## Pin map (from `include/pins.h`)

| Function | Port pin | GPIO | Status |
|---|---|---|---|
| Encoder push button | P1_IO0 | 4 | wired (your sketch) |
| Encoder CLK / DT | P1_IO1 / P1_IO2 | 3 / 2 | wired |
| Stop button / its LED | P2_IO1 / P2_IO2 | 6 / 5 | wired |
| Buzzer | P3_IO1 | 16 | wired |
| Microphone SELECT (driven LOW) / data / clock | P4_IO0 / P4_IO1 / P4_IO2 | 1 / 17 / 18 | from your voice_capture.h |
| Motor L fwd / rev, R fwd | P2_IO0, P3_IO0, P3_IO2 | 7, 9, 15 | proposed |
| Motor R rev | SCK | 14 | proposed |
| Wheel encoders L / R (one channel each) | SDA, SCL | 10, 11 | proposed |
| Deadman switch on handle | MOSI | 12 | proposed |
| Free | MISO | 13 | |

## control_test: how it behaves

- Turn the encoder to set speed, press it to start. Start is refused unless the computer's camera test has sent a clear frame in the last 2 s.
- The stop button stops, lights its LED and beeps. A camera block, camera timeout or `MIC:START` also stops.
- Serial commands from the computer: `CAM:CLEAR`, `CAM:BLOCK`, `CAM:OFF`, `MIC:START` (or `MIC:START:<n>` to number the clip), `MIC:STOP` (ignored: clips are a fixed 2 s).
- `MIC:START` stops the rover, then replies `AUDIO_BEGIN:<n>`, `PCM:<hex>` lines and `AUDIO_END:<n>`. Start is refused while a clip is being captured or sent.
- It needs the `RotaryEncoder` library (installed automatically).

## rover: before the first flash

- Set the Wi-Fi name, password and laptop IP in `src/main.cpp`; the microphone pins already match your `voice_capture.h`.
- Sensors in `src/main.cpp` are stubs: `imuGyroZ()`, `compassHeading()`, `obstacleDistanceM()`. A route will not start without a heading and an obstacle sensor. For supervised bench tests only, add `-DDEV_FAKE_HEADING_DEG=90` and `-DALLOW_NO_OBSTACLE_SENSOR` to the `rover` env's `build_flags`.
- Set wheel size, encoder counts and track width in `src/motors.h`.
- The stop button stops the rover and aborts the route; a new route is needed to move again. The encoder button confirms a road crossing. Voice never confirms a crossing.
- Crossing detection is a heuristic on the route text and can miss crossings. Check the `crossing holds` count printed when a route starts.

## Laptop side

```
cd laptop
pip install -r requirements.txt
python planner.py "I want to go to the Tesco on Cromwell Road" --lat 51.4945 --lon -0.1741 --yes
uvicorn server:app --host 0.0.0.0 --port 8000
```

Put your email in `USER_AGENT` in `laptop/planner.py` (Nominatim requires it). The simulation builds with
`g++ -std=c++17 -O2 test_follower.cpp -o test_follower`.
