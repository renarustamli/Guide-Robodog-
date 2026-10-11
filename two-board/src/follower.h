// follower.h - waypoint following for a differential-drive guide rover.
//
// Pure C++ (no Arduino headers) so the same code runs on the ESP32 and in the laptop simulator
// (test_follower.cpp). Place it next to main.cpp in src/.
//
// Frame: local metres, x = east, y = north, theta = radians counter-clockwise from east.
//
// Pieces
//   PoseEstimator  wheel odometry + gyro heading, optional slow GNSS pull
//   Follower       pure-pursuit path following, hold points at crossings, speed profile,
//                  and a safety gate (deadman, e-stop, obstacle, stall) that overrides everything
//   PID            per-wheel speed loop helper, used by motors.h
#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rover {

constexpr float kPi = 3.14159265358979f;

inline float wrapPi(float a) {
  while (a > kPi) a -= 2 * kPi;
  while (a < -kPi) a += 2 * kPi;
  return a;
}
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float slew(float cur, float target, float maxDelta) {
  return cur + clampf(target - cur, -maxDelta, maxDelta);
}

struct Vec2 { float x = 0, y = 0; };
struct Pose { float x = 0, y = 0, theta = 0; };
struct Cmd { float v = 0, w = 0; };   // forward m/s, turn rad/s (positive = left)

inline float dist(const Pose& p, const Vec2& q) { return std::hypot(q.x - p.x, q.y - p.y); }

// ---------------------------------------------------------------- tunables
struct Params {
  float trackWidth = 0.30f;       // m between wheel centres
  float vMax = 0.8f;              // m/s, keep at or below a slow walk
  float vMin = 0.15f;             // m/s minimum while moving along the path
  float wMax = 1.2f;              // rad/s
  float lookMin = 0.8f, lookMax = 2.0f, lookGain = 1.0f;  // lookahead = lookMin + gain*v
  float turnInPlaceAngle = 1.2f;  // rad: rotate on the spot if the target is further off than this
  float goalTol = 0.6f;           // m: arrived
  float holdStopDist = 0.3f;      // m: stop this far (along the path) before a hold point
  float accel = 0.4f;             // m/s^2
  float decel = 1.0f;             // m/s^2 for planned slowing
  float emergencyDecel = 2.5f;    // m/s^2 for deadman / obstacle stops
  float alphaAccel = 2.0f;        // rad/s^2
  float obstacleStop = 0.7f;      // m: stop if something is closer than this
  float obstacleSlow = 1.8f;      // m: start slowing inside this
  float stallV = 0.25f;           // commanded speed above which a stall is suspected...
  float stallMeasured = 0.05f;    // ...if measured speed stays below this...
  float stallTime = 1.5f;         // ...for this long
  float searchWindow = 15.0f;     // m of path searched ahead when finding the closest point
};

// ---------------------------------------------------------------- pose estimate
class PoseEstimator {
 public:
  explicit PoseEstimator(float trackWidth = 0.30f) : track_(trackWidth) {}

  void reset(float x, float y, float theta) { p_ = {x, y, wrapPi(theta)}; }
  void setGyroBias(float b) { gyroBias_ = b; }
  const Pose& pose() const { return p_; }

  // dL, dR: metres each wheel moved since the last call. gyroZ: rad/s, or NAN if no gyro.
  void predict(float dL, float dR, float gyroZ, float dt) {
    float ds = 0.5f * (dL + dR);
    float dthOdo = (dR - dL) / track_;
    float dth = dthOdo;
    if (std::isfinite(gyroZ)) dth = kGyroTrust * (gyroZ - gyroBias_) * dt + (1.0f - kGyroTrust) * dthOdo;
    p_.x += ds * std::cos(p_.theta + 0.5f * dth);
    p_.y += ds * std::sin(p_.theta + 0.5f * dth);
    p_.theta = wrapPi(p_.theta + dth);
  }

  // A GNSS fix in the same local frame. Pulls position gently; it does NOT fix heading and it
  // cannot give pavement-level accuracy (typical error is several metres).
  void gnssUpdate(float x, float y, float accuracyM) {
    if (!(accuracyM > 0.0f) || accuracyM > 8.0f) return;
    float k = clampf(kGnssGain * (3.0f / accuracyM), 0.01f, kGnssGain);
    p_.x += k * (x - p_.x);
    p_.y += k * (y - p_.y);
  }

 private:
  static constexpr float kGyroTrust = 0.98f;
  static constexpr float kGnssGain = 0.05f;
  float track_;
  float gyroBias_ = 0.0f;
  Pose p_;
};

// ---------------------------------------------------------------- wheel speed loop helper
struct PID {
  float kp = 0, ki = 0, iLimit = 1, outMin = -1, outMax = 1;
  float integ = 0;
  void reset() { integ = 0; }
  float step(float err, float dt) {
    integ = clampf(integ + ki * err * dt, -iLimit, iLimit);
    return clampf(kp * err + integ, outMin, outMax);
  }
};

inline void wheelSpeeds(float v, float w, float track, float vWheelMax, float& vl, float& vr) {
  vl = v - 0.5f * w * track;
  vr = v + 0.5f * w * track;
  float m = std::max(std::fabs(vl), std::fabs(vr));
  if (m > vWheelMax) { vl *= vWheelMax / m; vr *= vWheelMax / m; }
}

// ---------------------------------------------------------------- follower
struct SafetyInputs {
  bool estop = false;          // true = emergency stop pressed or circuit broken
  bool deadman = true;         // true = person is holding the handle
  float obstacle_m = INFINITY; // nearest obstacle ahead, metres
  float measured_v = 0;        // from wheel encoders, m/s (absolute value is used)
};

enum class RouteState : uint8_t { IDLE, FOLLOWING, WAIT_CONFIRM, ARRIVED, ABORTED };
enum class Safety : uint8_t { OK, DEADMAN, OBSTACLE, ESTOP, STALL };

inline const char* name(RouteState s) {
  switch (s) {
    case RouteState::IDLE: return "IDLE";
    case RouteState::FOLLOWING: return "FOLLOWING";
    case RouteState::WAIT_CONFIRM: return "WAIT_CONFIRM";
    case RouteState::ARRIVED: return "ARRIVED";
    default: return "ABORTED";
  }
}
inline const char* name(Safety s) {
  switch (s) {
    case Safety::OK: return "ok";
    case Safety::DEADMAN: return "deadman released";
    case Safety::OBSTACLE: return "obstacle";
    case Safety::ESTOP: return "e-stop";
    default: return "stalled";
  }
}

class Follower {
 public:
  explicit Follower(const Params& p = Params()) : prm_(p) {}

  // pts: route in local metres, roughly evenly spaced. holds: indices into pts where the rover must
  // stop and wait for confirmCross() (road crossings).
  void setPath(const std::vector<Vec2>& pts, const std::vector<uint32_t>& holds = {}) {
    pts_ = pts;
    s_.assign(pts_.size(), 0.0f);
    for (size_t i = 1; i < pts_.size(); i++)
      s_[i] = s_[i - 1] + std::hypot(pts_[i].x - pts_[i - 1].x, pts_[i].y - pts_[i - 1].y);
    holds_.clear();
    for (uint32_t h : holds)
      if (h > 0 && h < pts_.size() && (holds_.empty() || h > holds_.back())) holds_.push_back(h);
    nextHold_ = 0;
    idx_ = 0;
    stallT_ = 0;
    stalled_ = false;
    safety_ = Safety::OK;
    route_ = pts_.size() >= 2 ? RouteState::FOLLOWING : RouteState::IDLE;
  }

  void abort() {
    if (route_ == RouteState::FOLLOWING || route_ == RouteState::WAIT_CONFIRM) route_ = RouteState::ABORTED;
  }

  // Person has checked the road and confirmed (button or voice). Only has an effect while waiting.
  bool confirmCross() {
    if (route_ != RouteState::WAIT_CONFIRM) return false;
    nextHold_++;
    route_ = RouteState::FOLLOWING;
    return true;
  }

  RouteState state() const { return route_; }
  Safety safety() const { return safety_; }
  size_t progressIndex() const { return idx_; }
  size_t pathSize() const { return pts_.size(); }
  size_t holdCount() const { return holds_.size(); }
  float remaining() const { return pts_.empty() ? 0.0f : s_.back() - s_[idx_]; }

  Cmd update(const Pose& p, const SafetyInputs& in, float dt) {
    // Hard stops first. They always win.
    if (in.estop) {
      abort();
      safety_ = Safety::ESTOP;
      vOut_ = wOut_ = 0;
      return {0, 0};
    }
    if (stalled_) {
      safety_ = Safety::STALL;
      vOut_ = wOut_ = 0;
      return {0, 0};
    }
    safety_ = Safety::OK;

    Cmd want{0, 0};
    if (route_ == RouteState::FOLLOWING) want = pursue(p);

    bool safetyStop = false;
    if (route_ == RouteState::FOLLOWING) {
      if (!in.deadman) {
        safety_ = Safety::DEADMAN;
        want = {0, 0};
        safetyStop = true;
      } else if (in.obstacle_m < prm_.obstacleStop) {
        safety_ = Safety::OBSTACLE;
        want = {0, 0};
        safetyStop = true;
      } else if (in.obstacle_m < prm_.obstacleSlow) {
        want.v *= (in.obstacle_m - prm_.obstacleStop) / (prm_.obstacleSlow - prm_.obstacleStop);
      }
    }

    // Rate limiting. Stops for safety reasons use the harder deceleration.
    if (want.v > vOut_) vOut_ = std::min(want.v, vOut_ + prm_.accel * dt);
    else vOut_ = std::max(want.v, vOut_ - (safetyStop ? prm_.emergencyDecel : prm_.decel) * dt);
    wOut_ = safetyStop ? 0.0f : slew(wOut_, want.w, prm_.alphaAccel * dt);

    // Stall: we are commanding motion but the wheels are not moving (blocked, motor fault, dead encoder).
    if (std::fabs(vOut_) > prm_.stallV && std::fabs(in.measured_v) < prm_.stallMeasured) stallT_ += dt;
    else stallT_ = std::max(0.0f, stallT_ - dt);
    if (stallT_ > prm_.stallTime) {
      stalled_ = true;
      route_ = RouteState::ABORTED;
      safety_ = Safety::STALL;
      vOut_ = wOut_ = 0;
      return {0, 0};
    }
    return {vOut_, wOut_};
  }

 private:
  Cmd pursue(const Pose& p) {
    const size_t n = pts_.size();

    // Progress: closest point within a window ahead of the last one (never goes backwards).
    float best = 1e9f;
    size_t bi = idx_;
    for (size_t i = idx_; i < n && s_[i] - s_[idx_] <= prm_.searchWindow; i++) {
      float d = dist(p, pts_[i]);
      if (d < best) { best = d; bi = i; }
    }
    idx_ = bi;

    const float remEnd = s_.back() - s_[idx_];
    const bool holdAhead = nextHold_ < holds_.size();
    const size_t h = holdAhead ? holds_[nextHold_] : n - 1;
    const float remHold = holdAhead ? s_[h] - s_[idx_] : 1e9f;

    if (remEnd <= prm_.goalTol) { route_ = RouteState::ARRIVED; return {0, 0}; }
    if (holdAhead && (idx_ >= h || remHold <= prm_.holdStopDist)) { route_ = RouteState::WAIT_CONFIRM; return {0, 0}; }

    // Speed profile: be able to stop before the goal or the next hold point.
    float dStop = remEnd - 0.5f * prm_.goalTol;
    if (holdAhead) dStop = std::min(dStop, remHold - 0.5f * prm_.holdStopDist);
    float vLim = std::min(prm_.vMax, std::sqrt(std::max(0.0f, 1.4f * prm_.decel * dStop)));
    vLim = std::max(vLim, std::min(prm_.vMax, 0.12f));

    // Lookahead target, never past the next hold point.
    const float Ld = clampf(prm_.lookMin + prm_.lookGain * std::fabs(vOut_), prm_.lookMin, prm_.lookMax);
    const size_t lim = holdAhead ? h : n - 1;
    size_t ti = idx_;
    while (ti < lim && dist(p, pts_[ti]) < Ld) ti++;

    const float dx = pts_[ti].x - p.x, dy = pts_[ti].y - p.y;
    const float c = std::cos(p.theta), sn = std::sin(p.theta);
    const float lx = c * dx + sn * dy;    // target in the robot frame, x forward
    const float ly = -sn * dx + c * dy;   // y left
    const float L = std::hypot(lx, ly);
    if (L < 1e-3f) return {0, 0};
    const float alpha = std::atan2(ly, lx);

    Cmd out;
    if (std::fabs(alpha) > prm_.turnInPlaceAngle) {
      out.v = 0;
      out.w = clampf(2.0f * alpha, -prm_.wMax, prm_.wMax);
      if (std::fabs(out.w) < 0.3f) out.w = std::copysign(0.3f, alpha);
      return out;
    }
    const float kappa = 2.0f * ly / (L * L);
    float v = vLim * std::max(0.25f, std::cos(alpha));
    if (std::fabs(kappa) > 1e-3f) v = std::min(v, 0.8f * prm_.wMax / std::fabs(kappa));
    v = clampf(v, std::min(prm_.vMin, vLim), vLim);
    out.v = v;
    out.w = clampf(v * kappa, -prm_.wMax, prm_.wMax);
    return out;
  }

  Params prm_;
  std::vector<Vec2> pts_;
  std::vector<float> s_;           // cumulative path length at each point
  std::vector<uint32_t> holds_;
  size_t nextHold_ = 0, idx_ = 0;
  RouteState route_ = RouteState::IDLE;
  Safety safety_ = Safety::OK;
  float vOut_ = 0, wOut_ = 0, stallT_ = 0;
  bool stalled_ = false;
};

// ---------------------------------------------------------------- route download format
// Matches planner.pack_route_local on the laptop (little-endian):
//   uint32 n, uint32 nHolds, float64 originLat, float64 originLon,
//   n x (float32 x, float32 y), nHolds x uint32 index
struct RouteData {
  double lat0 = 0, lon0 = 0;
  std::vector<Vec2> pts;
  std::vector<uint32_t> holds;
};

inline bool parseRoute(const uint8_t* d, size_t len, RouteData& out) {
  if (len < 24) return false;
  uint32_t n, nh;
  std::memcpy(&n, d, 4);
  std::memcpy(&nh, d + 4, 4);
  if (n < 2 || n > 20000 || nh > 1000) return false;
  if (len != 24 + 8ull * n + 4ull * nh) return false;   // exact size or reject
  std::memcpy(&out.lat0, d + 8, 8);
  std::memcpy(&out.lon0, d + 16, 8);
  out.pts.resize(n);
  for (uint32_t i = 0; i < n; i++) {
    float xy[2];
    std::memcpy(xy, d + 24 + 8ull * i, 8);
    if (!std::isfinite(xy[0]) || !std::isfinite(xy[1])) return false;
    out.pts[i] = {xy[0], xy[1]};
  }
  out.holds.resize(nh);
  if (nh) std::memcpy(out.holds.data(), d + 24 + 8ull * n, 4ull * nh);
  return true;
}

// GNSS lat/lon -> the route's local frame (same flat-earth maths as the laptop).
inline Vec2 gnssToLocal(double lat, double lon, double lat0, double lon0) {
  const double kx = 111320.0 * std::cos(lat0 * 3.14159265358979323846 / 180.0);
  return {(float)((lon - lon0) * kx), (float)((lat - lat0) * 110540.0)};
}

}  // namespace rover
