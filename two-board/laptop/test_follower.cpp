// Laptop simulation of the rover following routes.   Build and run:
//   g++ -std=c++17 -O2 -Wall -Wextra test_follower.cpp -o test_follower && ./test_follower
#include <cstdio>
#include <random>

#include "../src/follower.h"

using namespace rover;

static int failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) { failures++; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } \
  } while (0)

// ---------------------------------------------------------------- helpers
static std::vector<Vec2> makePath(const std::vector<Vec2>& corners, float step = 1.5f) {
  std::vector<Vec2> out{corners[0]};
  float need = step;
  for (size_t i = 1; i < corners.size(); i++) {
    Vec2 a = corners[i - 1], b = corners[i];
    float seg = std::hypot(b.x - a.x, b.y - a.y), pos = 0;
    while (seg - pos >= need) {
      pos += need;
      float t = pos / seg;
      out.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
      need = step;
    }
    need -= seg - pos;
  }
  if (std::hypot(out.back().x - corners.back().x, out.back().y - corners.back().y) > 0.01f) out.push_back(corners.back());
  return out;
}

static float crossTrack(const std::vector<Vec2>& path, float px, float py) {
  float best = 1e9f;
  for (size_t i = 1; i < path.size(); i++) {
    Vec2 a = path[i - 1], b = path[i];
    float dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    float t = l2 > 0 ? clampf(((px - a.x) * dx + (py - a.y) * dy) / l2, 0, 1) : 0;
    best = std::min(best, std::hypot(px - (a.x + t * dx), py - (a.y + t * dy)));
  }
  return best;
}

struct SimCfg {
  float eL = 0, eR = 0;        // odometry scale error per wheel (0.02 = reads 2% long)
  float gyroBias = 0;          // rad/s
  float gyroNoise = 0;         // rad/s, 1 sigma
  float gnssSigma = 0;         // m, 1 sigma, fix once per second; 0 = no GNSS
  bool useGyro = true;
  float motorTau = 0.15f;      // wheel speed lag
};

struct Sim {
  Params prm;
  Follower f;
  PoseEstimator est;
  Pose truth;
  std::vector<Vec2> path;
  SimCfg cfg;
  SafetyInputs in;
  std::mt19937 rng{42};
  float lw = 0, rw = 0, t = 0, maxCte = 0, maxV = 0;
  bool blocked = false;
  float measuredV = 0;

  Sim(const std::vector<Vec2>& p, const std::vector<uint32_t>& holds, SimCfg c = SimCfg())
      : f(prm), est(prm.trackWidth), path(p), cfg(c) {
    truth = {p[0].x, p[0].y, std::atan2(p[1].y - p[0].y, p[1].x - p[0].x)};
    est.reset(truth.x, truth.y, truth.theta);
    f.setPath(p, holds);
  }

  float gauss(float s) { return s > 0 ? std::normal_distribution<float>(0, s)(rng) : 0.0f; }

  void step(float dt = 0.02f) {
    in.measured_v = measuredV;
    Cmd c = f.update(est.pose(), in, dt);
    float vl, vr;
    wheelSpeeds(c.v, c.w, prm.trackWidth, 1.2f, vl, vr);
    lw += (vl - lw) * dt / cfg.motorTau;
    rw += (vr - rw) * dt / cfg.motorTau;
    if (blocked) lw = rw = 0;
    float dL = lw * dt, dR = rw * dt, ds = 0.5f * (dL + dR), dth = (dR - dL) / prm.trackWidth;
    truth.x += ds * std::cos(truth.theta + 0.5f * dth);
    truth.y += ds * std::sin(truth.theta + 0.5f * dth);
    truth.theta = wrapPi(truth.theta + dth);
    measuredV = ds / dt;

    float gz = dth / dt + cfg.gyroBias + gauss(cfg.gyroNoise);
    est.predict(dL * (1 + cfg.eL), dR * (1 + cfg.eR), cfg.useGyro ? gz : NAN, dt);
    t += dt;
    if (cfg.gnssSigma > 0 && std::fmod(t, 1.0f) < dt)
      est.gnssUpdate(truth.x + gauss(cfg.gnssSigma), truth.y + gauss(cfg.gnssSigma), cfg.gnssSigma);

    maxCte = std::max(maxCte, crossTrack(path, truth.x, truth.y));
    maxV = std::max(maxV, std::fabs(measuredV));
  }

  // Run until pred() is true or timeout. Returns true if pred became true.
  template <class F> bool runUntil(F pred, float timeout) {
    float t0 = t;
    while (t - t0 < timeout) { if (pred()) return true; step(); }
    return pred();
  }
  void run(float seconds) { float t0 = t; while (t - t0 < seconds) step(); }
  float goalErr() const { return std::hypot(truth.x - path.back().x, truth.y - path.back().y); }
};

static const std::vector<Vec2> kLPath = makePath({{0, 0}, {30, 0}, {30, 30}});
static const std::vector<Vec2> kStraight = makePath({{0, 0}, {40, 0}});

// ---------------------------------------------------------------- tests
static void testIdealRoute() {
  std::printf("1. ideal sensors, L-shaped 60 m route\n");
  Sim s(kLPath, {});
  bool ok = s.runUntil([&] { return s.f.state() == RouteState::ARRIVED; }, 200);
  s.run(3);
  std::printf("   arrived=%d time=%.0fs maxCrossTrack=%.2fm goalErr=%.2fm maxSpeed=%.2f\n", ok, s.t, s.maxCte, s.goalErr(), s.maxV);
  CHECK(ok, "never arrived");
  CHECK(s.goalErr() < 1.0f, "goal error %.2f", s.goalErr());
  CHECK(s.maxCte < 1.5f, "cross-track %.2f", s.maxCte);
  CHECK(s.maxV < 0.85f, "speed %.2f over limit", s.maxV);
  CHECK(std::fabs(s.lw) < 0.02f && std::fabs(s.rw) < 0.02f, "still moving after arrival");
}

static void testHold() {
  std::printf("2. crossing hold point at 21 m\n");
  Sim s(kStraight, {14});   // 14 * 1.5 m = 21 m
  bool waiting = s.runUntil([&] { return s.f.state() == RouteState::WAIT_CONFIRM; }, 100);
  s.run(3);                       // let it finish braking
  float xAtStop = s.truth.x;
  s.run(10);
  std::printf("   waiting=%d stopped at x=%.2f (hold at 21.0), drift while waiting=%.3f m\n", waiting, xAtStop, s.truth.x - xAtStop);
  CHECK(waiting, "never reached the hold");
  CHECK(s.truth.x <= 21.0f, "went past the hold: x=%.2f", s.truth.x);
  CHECK(s.truth.x > 19.5f, "stopped too early: x=%.2f", s.truth.x);
  CHECK(s.truth.x - xAtStop < 0.05f, "kept creeping while waiting");
  CHECK(s.f.state() == RouteState::WAIT_CONFIRM, "left the waiting state without confirmation");
  CHECK(!Follower().confirmCross(), "confirm with no route should be ignored");
  CHECK(s.f.confirmCross(), "confirm rejected");
  bool ok = s.runUntil([&] { return s.f.state() == RouteState::ARRIVED; }, 100);
  CHECK(ok, "did not finish after confirmation");
  CHECK(s.f.confirmCross() == false, "late confirm should be ignored");
}

static void testObstacle() {
  std::printf("3. obstacle: slow zone, stop, resume\n");
  Sim s(kStraight, {});
  s.run(10);
  s.in.obstacle_m = 1.2f;                       // inside the slow zone only
  s.run(3);
  float vSlow = std::fabs(s.measuredV);
  s.in.obstacle_m = 0.5f;                       // inside the stop zone
  s.run(3);
  float x0 = s.truth.x;
  s.run(4);
  std::printf("   speed in slow zone=%.2f (limit 0.36), moved while blocked=%.3f m, safety=%s\n", vSlow, s.truth.x - x0, name(s.f.safety()));
  CHECK(vSlow < 0.42f, "slow zone speed %.2f", vSlow);
  CHECK(s.truth.x - x0 < 0.01f, "moved %.3f m with obstacle inside stop distance", s.truth.x - x0);
  CHECK(s.f.safety() == Safety::OBSTACLE, "safety state not OBSTACLE");
  s.in.obstacle_m = INFINITY;
  bool ok = s.runUntil([&] { return s.f.state() == RouteState::ARRIVED; }, 100);
  CHECK(ok, "did not resume after obstacle cleared");
}

static void testDeadman() {
  std::printf("4. deadman released and re-gripped\n");
  Sim s(kStraight, {});
  s.run(15);
  float x0 = s.truth.x, v0 = std::fabs(s.measuredV);
  s.in.deadman = false;
  s.run(3);
  float stopDist = s.truth.x - x0;
  std::printf("   speed at release=%.2f stopping distance=%.2f m\n", v0, stopDist);
  CHECK(stopDist < 0.45f, "stopping distance %.2f", stopDist);
  CHECK(std::fabs(s.measuredV) < 0.01f, "still moving");
  float x1 = s.truth.x;
  s.run(3);
  CHECK(s.truth.x - x1 < 0.01f, "moved while deadman released");
  s.in.deadman = true;
  bool ok = s.runUntil([&] { return s.f.state() == RouteState::ARRIVED; }, 100);
  CHECK(ok, "did not resume after re-gripping");
}

static void testEstop() {
  std::printf("5. e-stop aborts the route and does not self-resume\n");
  Sim s(kStraight, {});
  s.run(12);
  s.in.estop = true;
  s.run(1);
  s.in.estop = false;
  float x0 = s.truth.x;
  s.run(6);
  std::printf("   state after release=%s, moved %.3f m\n", name(s.f.state()), s.truth.x - x0);
  CHECK(s.f.state() == RouteState::ABORTED, "route not aborted");
  CHECK(s.truth.x - x0 < 0.02f, "rover moved after e-stop release");
  s.f.setPath(kStraight);   // an explicit new route is required
  CHECK(s.f.state() == RouteState::FOLLOWING, "new route not accepted");
}

static void testStall() {
  std::printf("6. wheels blocked -> stall fault\n");
  Sim s(kStraight, {});
  s.run(10);
  s.blocked = true;
  bool tripped = s.runUntil([&] { return s.f.safety() == Safety::STALL; }, 6);
  std::printf("   stall detected=%d after %.1fs, route=%s\n", tripped, s.t - 10, name(s.f.state()));
  CHECK(tripped, "stall not detected");
  CHECK(s.f.state() == RouteState::ABORTED, "route not aborted");
  s.blocked = false;
  s.run(3);
  CHECK(std::fabs(s.lw) < 0.01f, "motors still driven after stall");
}

static void testNoisySensors() {
  std::printf("7. realistic sensor errors on the L-shaped route (informational, loose limits)\n");
  struct Case { const char* label; SimCfg cfg; };
  SimCfg a; a.eL = 0.02f; a.eR = -0.02f; a.gyroBias = 0.002f; a.gyroNoise = 0.01f; a.gnssSigma = 2.0f;
  SimCfg b = a; b.useGyro = false;
  SimCfg c = a; c.gnssSigma = 0;
  Case cases[] = {{"odometry+gyro+GNSS(2m)", a}, {"odometry+gyro, no GNSS", c}, {"odometry only (no gyro, no GNSS)", b}};
  for (auto& k : cases) {
    Sim s(kLPath, {}, k.cfg);
    bool ok = s.runUntil([&] { return s.f.state() == RouteState::ARRIVED; }, 250);
    std::printf("   %-34s arrived=%d maxCrossTrack=%5.2fm goalErr(true)=%5.2fm\n", k.label, ok, s.maxCte, s.goalErr());
    if (k.cfg.useGyro && k.cfg.gnssSigma > 0) {
      CHECK(ok, "did not arrive with gyro+GNSS");
      CHECK(s.goalErr() < 4.0f, "goal error %.2f", s.goalErr());
    }
  }
}

static void testPid() {
  std::printf("8. wheel speed PID on a first-order motor model\n");
  PID pid; pid.kp = 1.5f; pid.ki = 4.0f; pid.iLimit = 0.6f; pid.outMin = -1; pid.outMax = 1;
  float speed = 0, target = 0.6f, dt = 0.02f, ff = 0.9f, tau = 0.15f, gain = 1.0f;   // plant: speed -> gain*pwm
  float settled = -1, peak = 0;
  for (int i = 0; i < 300; i++) {
    float pwm = clampf(ff * target + pid.step(target - speed, dt), -1, 1);
    speed += (gain * pwm - speed) * dt / tau;
    peak = std::max(peak, speed);
    if (settled < 0 && std::fabs(speed - target) < 0.03f) settled = i * dt;
  }
  std::printf("   settled within 5%% at %.2fs, overshoot %.0f%%, final error %.3f\n", settled, 100 * (peak - target) / target, target - speed);
  CHECK(settled > 0 && settled < 1.0f, "slow to settle");
  CHECK(std::fabs(target - speed) < 0.02f, "steady-state error");
  CHECK(peak < target * 1.15f, "overshoot too large");
}

int main() {
  testIdealRoute();
  testHold();
  testObstacle();
  testDeadman();
  testEstop();
  testStall();
  testNoisySensors();
  testPid();
  std::printf("\n%s (%d failed check%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
