# Guide rover prototype (Axiometa Genesis Mini)

Prototype only: test with the wheels off the ground first, then with a sighted supervisor holding the stop button.

## Two build targets

Pick one in the VS Code PlatformIO status bar, or use `-e` on the command line.

| Env | What it builds | Command |
|---|---|---|
| `control_test` (default) | Your control sketch: encoder sets speed and starts, LED button stops, buzzer, camera interlock over serial | `pio run -e control_test -t upload` |
| `handheld` | **Controller board** (your Studio Axiometa sketch): encoder speed, stop button, buzzer, mic, ESP-NOW to the receiver | `pio run -e handheld -t upload` |
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
| `src/control_test/voice.h` | Obsolete empty placeholder, safe to delete |
| `src/receiver/receiver.cpp` | Motor board sketch (your `sketch.ino`, logic unchanged) |
| `src/handheld/` | Controller board: `handheld.cpp`, its `rover_radio.h` (sender) and `voice_capture.h` |
| `src/receiver/rover_radio.h` | Receiver side of the same ESP-NOW protocol (channel 1, 12-byte packet, signed PWM -255..255) plus the sender filter |
| `axiometa.json`, `axiometa.handheld.json` | Studio Axiometa module manifests: the receiver (AX22-0032 on ports 2, 3) and the handheld (ports 1 to 4) |
| `include/rover_radio.h` | Obsolete, safe to delete |
| `src/main.cpp`, `follower.h`, `motors.h` | Paused single-board rover firmware: audio, wake word, Wi-Fi hand-off, route following, motors |
| `laptop/` | Whisper + route planner server, and a simulation of the follower |

## Two boards

- **Handheld controller** (`handheld`): encoder speed, stop button, buzzer, microphone, and the Studio serial protocol. Sends a `RoverPacket` to the receiver every 100 ms. `control_test` is the earlier camera-interlock test sketch for the same board.
- **Receiver** (`receiver`): motors only. Left motor P2_IO1/P2_IO2 (GPIO 6/5), right motor P3_IO1/P3_IO2 (GPIO 16/15). It stops if no command arrives for 400 ms, and only obeys the handheld MAC set in `rover_radio.h` (`ROVER_ALLOWED_SENDER`; all zeros = ignore all radio packets).
- Receiver MAC `A0:F2:62:EB:98:BC` is set in `src/handheld/rover_radio.h`. The handheld MAC `A0:F2:62:EB:92:D8` is allowlisted in `src/receiver/rover_radio.h`; if you replace the handheld, update `ROVER_ALLOWED_SENDER` with its startup MAC.
- The pin map below is for the handheld and the paused single-board design. The receiver's pins are set in `receiver.cpp`.

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
