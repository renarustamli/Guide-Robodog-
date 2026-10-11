// Rover: T3902 PDM mic -> wake word -> voice command -> laptop (STT + route) -> waypoint following
// Target: Axiometa Genesis Mini (ESP32-S3-MINI-1), Arduino-ESP32 3.x
//
// Files in src/:  main.cpp  follower.h (path following + safety logic)  motors.h (drive hardware)
//
// Tasks
//   captureTask  (core 0): I2S PDM -> StreamBuffer
//   processTask  (core 1): raw PCM -> wake word/command; conditioned copy -> noise-floor VAD
//   netTask      (core 0): sends the captured command to the laptop, acts on the reply, downloads routes
//   controlTask  (core 0): 50 Hz pose estimate -> follower -> motors, with the safety gate
//
// STATUS: written without access to the hardware. Treat as a starting point. Every TODO below
// (pins, sensors, Wi-Fi) must be filled in, and the first runs must be supervised with the
// wheels off the ground.

#include <Arduino.h>
#include <ESP_I2S.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "pins.h"
#include "follower.h"
#include "motors.h"

#ifdef USE_EDGE_IMPULSE
  #include EI_HEADER
  #ifndef WAKE_LABEL
    #define WAKE_LABEL "hey_rover"
  #endif
#endif

// ---------------------------------------------------------------- configuration
// Pins come from include/pins.h. Check MIC_CLK / MIC_DATA there match where the T3902 is wired.
constexpr int PIN_PDM_CLK  = pins::MIC_CLK;    // T3902 CLOCK
constexpr int PIN_PDM_DATA = pins::MIC_DATA;   // T3902 DATA

// T3902 SELECT pin: tied to GND -> left slot, tied to VDD -> right slot.
constexpr i2s_std_slot_mask_t MIC_SLOT = I2S_STD_SLOT_LEFT;

constexpr uint32_t SAMPLE_RATE   = 16000;
constexpr size_t   FRAME_SAMPLES = 320;                      // 20 ms
constexpr size_t   FRAME_BYTES   = FRAME_SAMPLES * sizeof(int16_t);

constexpr float    GAIN            = 4.0f;    // T3902 is -26 dBFS sensitivity; tune by watching the meter
constexpr float    MIN_SPEECH_RMS  = 300.0f;  // absolute floor for "loud enough to be speech"
constexpr float    SPEECH_RATIO    = 3.0f;    // speech if rms > noiseFloor * ratio

constexpr int      END_SILENCE_FRAMES = 40;   // 800 ms of quiet ends the command
constexpr int      NO_SPEECH_TIMEOUT  = 150;  // 3 s with no speech after wake -> give up
constexpr int      MAX_CMD_SECONDS    = 4;    // raise this if you have PSRAM
constexpr size_t   CMD_MAX_SAMPLES    = SAMPLE_RATE * MAX_CMD_SECONDS;

constexpr float    WAKE_THRESHOLD = 0.80f;    // Edge Impulse confidence
constexpr uint32_t WAKE_COOLDOWN_MS = 1500;

// Wi-Fi + laptop server (run `uvicorn server:app --host 0.0.0.0 --port 8000` on the laptop)
constexpr char WIFI_SSID[]   = "YOUR_WIFI_NAME";            // TODO
constexpr char WIFI_PASS[]   = "YOUR_WIFI_PASSWORD";        // TODO
constexpr char SERVER_HOST[] = "http://192.168.1.50:8000";  // TODO: laptop's LAN IP

// Position sent with each voice command, used as the route start. Hard-coded until a GNSS
// module is added.
float curLat = 51.5074f;   // TODO
float curLon = -0.1278f;   // TODO

// Safety inputs (pins in include/pins.h):
//   E-STOP  the LED stop button on P2. Reads LOW when pressed. A pressed press aborts the route and
//           a new route is needed. A button like this does not fail safe if its wire breaks, and the
//           e-stop should ALSO cut motor power in hardware: this pin is only the software half.
//   DEADMAN normally-OPEN switch to GND on the handle. Held -> LOW. Released or wire broken -> stop.
//   CONFIRM the encoder push button on P1. Reads HIGH when pressed: "I have checked the road, it is
//           safe to cross".
constexpr int PIN_ESTOP   = pins::STOP_BTN;
constexpr int PIN_DEADMAN = pins::DEADMAN;
constexpr int PIN_CONFIRM = pins::CTRL_BTN;

// Starting a route is refused unless these are real. Flip only when the sensors are fitted, or
// build with -DALLOW_NO_OBSTACLE_SENSOR / -DDEV_FAKE_HEADING_DEG=90 for supervised bench tests.
constexpr bool HAVE_OBSTACLE_SENSOR = false;

// ---------------------------------------------------------------- sensors (replace the stubs)
float imuGyroZ()        { return NAN; }        // rad/s, positive = turning left. NAN = no gyro
float compassHeading()  { return NAN; }        // radians counter-clockwise from east. NAN = unknown
bool  gnssFix(double& lat, double& lon, float& accuracyM) { (void)lat; (void)lon; (void)accuracyM; return false; }
float obstacleDistanceM() { return INFINITY; } // TODO ToF/ultrasonic/lidar ahead of the rover, metres

// ---------------------------------------------------------------- globals
I2SClass i2s;
StreamBufferHandle_t audioBuf;
volatile uint32_t droppedBytes = 0;

enum class State { IDLE, LISTEN };
State state = State::IDLE;

int16_t* cmdBuf = nullptr;
size_t   cmdLen = 0;

volatile bool cmdBusy = false;       // true while a captured command is being sent
TaskHandle_t netTaskHandle = nullptr;

rover::Follower      follower;       // touched from netTask and controlTask: hold ctlMutex
rover::PoseEstimator poseEst;
SemaphoreHandle_t    ctlMutex = nullptr;
motors::Drive        drive;
double routeLat0 = 0, routeLon0 = 0;

// ---------------------------------------------------------------- wake word
#ifdef USE_EDGE_IMPULSE
static int16_t eiSlice[EI_CLASSIFIER_SLICE_SIZE];
static size_t  eiFill = 0;

static int eiGetData(size_t offset, size_t length, float* out) {
  numpy::int16_to_float(&eiSlice[offset], out, length);
  return 0;
}

bool wakeInit() {
  const EI_IMPULSE_ERROR err = run_classifier_init();
  if (err != EI_IMPULSE_OK) {
    Serial.printf("[wake] classifier init failed: %d\n", err);
    return false;
  }
  return true;
}

// Feed one frame. Returns true when the wake word is detected.
bool wakeFeed(const int16_t* frame, size_t n) {
  bool hit = false;
  for (size_t i = 0; i < n; i++) {
    eiSlice[eiFill++] = frame[i];
    if (eiFill == EI_CLASSIFIER_SLICE_SIZE) {
      eiFill = 0;
      signal_t sig;
      sig.total_length = EI_CLASSIFIER_SLICE_SIZE;
      sig.get_data = &eiGetData;
      ei_impulse_result_t res = {};
      const EI_IMPULSE_ERROR err = run_classifier_continuous(&sig, &res, false);
      if (err != EI_IMPULSE_OK) {
        Serial.printf("[wake] inference failed: %d\n", err);
      } else {
        for (size_t c = 0; c < EI_CLASSIFIER_LABEL_COUNT; c++) {
          if (strcmp(res.classification[c].label, WAKE_LABEL) == 0 &&
              res.classification[c].value > WAKE_THRESHOLD) {
            hit = true;
          }
        }
      }
    }
  }
  return hit;
}
#else
// Development fallback: type 'w' in the serial monitor to simulate the wake word.
bool wakeInit() { return true; }
bool wakeFeed(const int16_t*, size_t) {
  if (Serial.available() && Serial.read() == 'w') return true;
  return false;
}
#endif

// ---------------------------------------------------------------- command hand-off
void onUtterance(const int16_t* pcm, size_t n) {
  int peak = 0;
  for (size_t i = 0; i < n; i++) peak = max(peak, abs((int)pcm[i]));
  Serial.printf("[cmd] captured %.2f s, peak %d\n", n / (float)SAMPLE_RATE, peak);
  // Hand over to netTask. cmdBuf stays untouched until cmdBusy is cleared.
  cmdBusy = true;
  xTaskNotifyGive(netTaskHandle);
}

// ---------------------------------------------------------------- routes
// Returns an empty string if a route may start, otherwise the reason it may not.
static String preflight(float& heading) {
  heading = compassHeading();
#ifdef DEV_FAKE_HEADING_DEG
  if (!isfinite(heading)) heading = DEV_FAKE_HEADING_DEG * rover::kPi / 180.0f;
#endif
  if (!isfinite(heading)) return "I don't know which way I'm facing (no compass heading).";
#ifndef ALLOW_NO_OBSTACLE_SENSOR
  if (!HAVE_OBSTACLE_SENSOR) return "No obstacle sensor is fitted.";
#endif
  if (digitalRead(PIN_ESTOP) == LOW) return "The stop button is pressed.";
  return "";
}

static bool startRoute() {
  float heading = 0;
  String why = preflight(heading);
  if (why.length()) { Serial.printf("[route] cannot start: %s\n", why.c_str()); return false; }

  HTTPClient http;
  http.begin(String(SERVER_HOST) + "/route/local.bin");
  http.setTimeout(15000);
  int code = http.GET();
  int len = http.getSize();
  if (code != 200 || len <= 0 || len > 400000) {
    Serial.printf("[route] download failed (HTTP %d, %d bytes)\n", code, len);
    http.end();
    return false;
  }
  std::vector<uint8_t> body(len);
  WiFiClient* s = http.getStreamPtr();
  size_t got = 0;
  uint32_t t0 = millis();
  while (http.connected() && got < (size_t)len && millis() - t0 < 15000) {
    size_t avail = s->available();
    if (avail) got += s->readBytes(body.data() + got, min(avail, (size_t)len - got));
    else delay(5);
  }
  http.end();

  rover::RouteData rd;
  if (!rover::parseRoute(body.data(), got, rd)) { Serial.println("[route] bad route data"); return false; }

  xSemaphoreTake(ctlMutex, portMAX_DELAY);
  routeLat0 = rd.lat0; routeLon0 = rd.lon0;
  poseEst.reset(0, 0, heading);
  follower.setPath(rd.pts, rd.holds);
  xSemaphoreGive(ctlMutex);

  Serial.printf("[route] %u waypoints, %u crossing holds, heading %.0f deg\n",
                (unsigned)rd.pts.size(), (unsigned)rd.holds.size(), heading * 180.0f / rover::kPi);
  if (rd.holds.empty())
    Serial.println("[route] WARNING: no crossing holds found - check this route for road crossings yourself");
  return true;
}

// ---------------------------------------------------------------- tasks
static void captureTask(void*) {
  static int16_t buf[FRAME_SAMPLES];
  for (;;) {
    size_t n = i2s.readBytes((char*)buf, FRAME_BYTES);
    if (n == 0) { vTaskDelay(1); continue; }
    size_t sent = xStreamBufferSend(audioBuf, buf, n, 0);
    if (sent != n) droppedBytes += (n - sent);
  }
}

// Sends the captured command to the laptop and acts on the reply. Own task so a slow network
// never stalls audio processing or motor control.
static void netTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[net] Wi-Fi down, reconnecting");
      WiFi.reconnect();
      for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) vTaskDelay(pdMS_TO_TICKS(250));
    }

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(String(SERVER_HOST) + "/command");
      http.setTimeout(30000);   // first Whisper call after start-up can be slow
      http.addHeader("Content-Type", "application/octet-stream");
      http.addHeader("X-Lat", String(curLat, 6));
      http.addHeader("X-Lon", String(curLon, 6));
      int code = http.POST((uint8_t*)cmdBuf, cmdLen * sizeof(int16_t));
      if (code == 200) {
        JsonDocument doc;
        if (deserializeJson(doc, http.getString()) == DeserializationError::Ok) {
          const char* action = doc["action"] | "none";
          Serial.printf("[you said] %s\n", doc["transcript"] | "");
          Serial.printf("[rover]    %s\n", doc["spoken"] | "");
          Serial.printf("[action]   %s\n", action);
          // TODO: speak doc["spoken"] with TTS.
          if (!strcmp(action, "start")) {
            startRoute();
          } else if (!strcmp(action, "stop") || !strcmp(action, "cancel")) {
            xSemaphoreTake(ctlMutex, portMAX_DELAY);
            follower.abort();
            xSemaphoreGive(ctlMutex);
          }
          // Crossings are confirmed ONLY by the physical button, never by voice: a misheard "yes"
          // must not send someone into a road.
        } else {
          Serial.println("[net] bad JSON from server");
        }
      } else {
        Serial.printf("[net] POST failed: %d\n", code);
      }
      http.end();
    } else {
      Serial.println("[net] no Wi-Fi, command dropped");
    }
    cmdBusy = false;
  }
}

// 50 Hz: encoders + gyro -> pose -> follower (with safety gate) -> motors.
static void controlTask(void*) {
  const float dt = 0.02f;
  TickType_t last = xTaskGetTickCount();
  bool prevConfirm = false;
  rover::RouteState prevRoute = rover::RouteState::IDLE;
  rover::Safety prevSafety = rover::Safety::OK;
  uint32_t lastStatus = 0;

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(20));

    float dL, dR;
    drive.readEncoders(dL, dR, dt);

    rover::SafetyInputs si;
    si.estop = digitalRead(PIN_ESTOP) == LOW;
    si.deadman = digitalRead(PIN_DEADMAN) == LOW;
    si.obstacle_m = obstacleDistanceM();
    si.measured_v = 0.5f * (dL + dR) / dt;

    bool confirmNow = digitalRead(PIN_CONFIRM) == HIGH;
    bool confirmPressed = confirmNow && !prevConfirm;
    prevConfirm = confirmNow;

    double lat, lon;
    float acc;
    bool haveFix = gnssFix(lat, lon, acc);
    float gz = imuGyroZ();

    xSemaphoreTake(ctlMutex, portMAX_DELAY);
    poseEst.predict(dL, dR, gz, dt);
    if (haveFix && follower.pathSize() > 0) {
      rover::Vec2 g = rover::gnssToLocal(lat, lon, routeLat0, routeLon0);
      poseEst.gnssUpdate(g.x, g.y, acc);
    }
    if (confirmPressed && follower.confirmCross()) Serial.println("[route] crossing confirmed, continuing");
    rover::Cmd c = follower.update(poseEst.pose(), si, dt);
    rover::RouteState rs = follower.state();
    rover::Safety sf = follower.safety();
    float remaining = follower.remaining();
    rover::Pose p = poseEst.pose();
    xSemaphoreGive(ctlMutex);

    if (si.estop) drive.stop();
    else drive.drive(c.v, c.w, dt);

    if (rs != prevRoute) Serial.printf("[route] %s\n", rover::name(rs));
    if (sf != prevSafety) Serial.printf("[safety] %s\n", rover::name(sf));
    prevRoute = rs; prevSafety = sf;

    if (rs == rover::RouteState::FOLLOWING && millis() - lastStatus > 1000) {
      lastStatus = millis();
      Serial.printf("[follow] x=%.1f y=%.1f th=%.0f deg  v=%.2f w=%.2f  %.0f m to go\n",
                    p.x, p.y, p.theta * 180.0f / rover::kPi, c.v, c.w, remaining);
    }
  }
}

static void processTask(void*) {
  static int16_t rawFrame[FRAME_SAMPLES];
  static int16_t vadFrame[FRAME_SAMPLES];
  float prevX = 0, prevY = 0;          // DC-blocking high-pass state
  float noiseFloor = 200.0f;
  int silentFrames = 0, framesSinceWake = 0;
  bool heardSpeech = false;
  uint32_t lastWake = 0, lastPrint = 0;
  float rms = 0;

  for (;;) {
    size_t got = 0;
    while (got < FRAME_BYTES) {
      got += xStreamBufferReceive(audioBuf, ((uint8_t*)rawFrame) + got, FRAME_BYTES - got, portMAX_DELAY);
    }

    // Keep raw PCM for wake-word inference and the STT hand-off.
    memcpy(vadFrame, rawFrame, FRAME_BYTES);

    // 1) DC block + gain for the VAD meter only.
    double sumSq = 0;
    for (size_t i = 0; i < FRAME_SAMPLES; i++) {
      float x = vadFrame[i];
      float y = x - prevX + 0.995f * prevY;
      prevX = x; prevY = y;
      y *= GAIN;
      if (y > 32767) y = 32767;
      if (y < -32768) y = -32768;
      vadFrame[i] = (int16_t)y;
      sumSq += y * y;
    }
    rms = sqrtf(sumSq / FRAME_SAMPLES);

    // 2) VAD against an adaptive noise floor
    bool loud = rms > max(noiseFloor * SPEECH_RATIO, MIN_SPEECH_RMS);
    if (!loud) noiseFloor = 0.98f * noiseFloor + 0.02f * rms;

    // 3) State machine
    if (state == State::IDLE) {
      bool cooled = (millis() - lastWake > WAKE_COOLDOWN_MS) && !cmdBusy;
      if (wakeFeed(rawFrame, FRAME_SAMPLES) && cooled) {
        Serial.println("[wake] detected, listening...");
        lastWake = millis();
        state = State::LISTEN;
        cmdLen = 0; silentFrames = 0; framesSinceWake = 0; heardSpeech = false;
        // TODO: audible cue (short beep) or LED so the user knows to speak.
      }
    } else {
      framesSinceWake++;
      const size_t remaining = CMD_MAX_SAMPLES - cmdLen;
      const size_t copySamples = min(FRAME_SAMPLES, remaining);
      if (copySamples > 0) {
        memcpy(cmdBuf + cmdLen, rawFrame, copySamples * sizeof(int16_t));
        cmdLen += copySamples;
      }
      if (loud) { heardSpeech = true; silentFrames = 0; } else { silentFrames++; }

      bool full    = cmdLen >= CMD_MAX_SAMPLES;
      bool endSpk  = heardSpeech && silentFrames >= END_SILENCE_FRAMES;
      bool timeout = !heardSpeech && framesSinceWake >= NO_SPEECH_TIMEOUT;

      if (endSpk || full) {
        onUtterance(cmdBuf, cmdLen);
        state = State::IDLE;
        lastWake = millis();
      } else if (timeout) {
        Serial.println("[cmd] no speech heard, cancelled");
        state = State::IDLE;
        lastWake = millis();
      }
    }

    // 4) Debug meter
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.printf("rms=%6.0f floor=%6.0f %s %s dropped=%u\n",
                    rms, noiseFloor, loud ? "SPEECH" : "quiet ",
                    state == State::IDLE ? "IDLE  " : "LISTEN", (unsigned)droppedBytes);
    }
  }
}

// ---------------------------------------------------------------- setup
void setup() {
  // Motors first, and off, so nothing can drive while the rest starts up.
  drive.begin();
  drive.stop();

  Serial.begin(115200);
  delay(1000);
  Serial.println("Rover starting");

  // Command buffer: PSRAM if present, otherwise internal RAM.
  const bool hasPsram = psramFound();
  cmdBuf = (int16_t*)(hasPsram ? ps_malloc(CMD_MAX_SAMPLES * sizeof(int16_t))
                              : malloc(CMD_MAX_SAMPLES * sizeof(int16_t)));
  if (!cmdBuf) { Serial.println("[fatal] out of memory for command buffer"); while (true) delay(1000); }
  Serial.printf("PSRAM: %s, cmd buffer %u bytes\n", hasPsram ? "yes" : "no",
                (unsigned)(CMD_MAX_SAMPLES * sizeof(int16_t)));

  pinMode(PIN_ESTOP, INPUT_PULLUP);   // active LOW
  pinMode(PIN_DEADMAN, INPUT_PULLUP); // active LOW
  pinMode(PIN_CONFIRM, INPUT);        // active HIGH, driven by the encoder module (as in control_test)
  poseEst = rover::PoseEstimator(rover::Params().trackWidth);
  ctlMutex = xSemaphoreCreateMutex();
  if (!ctlMutex) { Serial.println("[fatal] could not create mutex"); while (true) delay(1000); }

  pinMode(pins::MIC_SELECT, OUTPUT); digitalWrite(pins::MIC_SELECT, LOW);   // T3902 SELECT low = left slot, as in voice_capture.h
  i2s.setPinsPdmRx(PIN_PDM_CLK, PIN_PDM_DATA);
  if (!i2s.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, MIC_SLOT)) {
    Serial.println("[fatal] I2S PDM init failed - check pins and Arduino-ESP32 version");
    while (true) delay(1000);
  }

  audioBuf = xStreamBufferCreate(FRAME_BYTES * 16, FRAME_BYTES);
  if (!audioBuf) {
    Serial.println("[fatal] could not allocate audio stream buffer");
    while (true) delay(1000);
  }
  if (!wakeInit()) {
    Serial.println("[fatal] wake-word model initialization failed");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED) Serial.printf("\nWi-Fi OK, IP %s\n", WiFi.localIP().toString().c_str());
  else Serial.println("\nWi-Fi not connected yet (will retry when a command is sent)");

  if (xTaskCreatePinnedToCore(netTask, "net", 8192, nullptr, 3, &netTaskHandle, 0) != pdPASS) {
    Serial.println("[fatal] could not start network task");
    while (true) delay(1000);
  }
  if (xTaskCreatePinnedToCore(controlTask, "control", 6144, nullptr, 6, nullptr, 0) != pdPASS) {
    Serial.println("[fatal] could not start motor control task");
    while (true) delay(1000);
  }

  TaskHandle_t processHandle = nullptr;
  if (xTaskCreatePinnedToCore(processTask, "process", 16384, nullptr, 4, &processHandle, 1) != pdPASS) {
    Serial.println("[fatal] could not start audio processing task");
    while (true) delay(1000);
  }
  if (xTaskCreatePinnedToCore(captureTask, "capture", 4096, nullptr, 5, nullptr, 0) != pdPASS) {
    vTaskDelete(processHandle);
    Serial.println("[fatal] could not start audio capture task");
    while (true) delay(1000);
  }
}

void loop() { vTaskDelay(portMAX_DELAY); }
