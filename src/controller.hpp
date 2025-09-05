#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>
#include <utility>
#include <vector>

struct RobotState {
  float x{1.f}, y{1.f}, th{0.f};
};

inline float wrapAngle(float a) {
  while (a > 3.14159265f) a -= 2.f * 3.14159265f;
  while (a < -3.14159265f) a += 2.f * 3.14159265f;
  return a;
}

struct PurePursuit {
  float lookahead{2.0f};
  float targetSpeed{2.0f};

  sf::Vector2f targetPoint(const RobotState& s, const std::vector<sf::Vector2f>& path) const {
    if (path.size() < 2) return {s.x, s.y};
    // Find closest point on path and then the lookahead target
    size_t bestSeg = 0; float bestT = 0.f; float bestDist2 = 1e9f;
    sf::Vector2f pos{s.x, s.y};
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      sf::Vector2f a = path[i], b = path[i+1];
      sf::Vector2f ab = b - a;
      float ab2 = ab.x*ab.x + ab.y*ab.y;
      if (ab2 < 1e-6f) continue;
      float t = ((pos.x - a.x) * ab.x + (pos.y - a.y) * ab.y) / ab2;
      t = std::fmax(0.f, std::fmin(1.f, t));
      sf::Vector2f proj = a + t * ab;
      sf::Vector2f d = pos - proj;
      float d2 = d.x*d.x + d.y*d.y;
      if (d2 < bestDist2) { bestDist2 = d2; bestSeg = i; bestT = t; }
    }

    float Ld = std::fmax(0.1f, lookahead);
    float remain = Ld;
    sf::Vector2f a = path[bestSeg];
    sf::Vector2f b = path[bestSeg + 1];
    sf::Vector2f cur = a + (b - a) * bestT;
    size_t i = bestSeg; float t = bestT;
    while (remain > 0.f && i + 1 < path.size()) {
      a = path[i]; b = path[i+1];
      sf::Vector2f from = a + (b - a) * t;
      sf::Vector2f seg = b - from;
      float segLen = std::sqrt(seg.x*seg.x + seg.y*seg.y);
      if (segLen >= remain) {
        sf::Vector2f dir = (segLen > 1e-6f) ? (seg / segLen) : sf::Vector2f{0.f, 0.f};
        cur = from + dir * remain;
        break;
      } else {
        remain -= segLen;
        ++i; t = 0.f;
        cur = b;
      }
    }
    return cur;
  }

  std::pair<float, float> control(const RobotState& s, const std::vector<sf::Vector2f>& path) const {
    if (path.size() < 2) return {0.f, 0.f};
    sf::Vector2f cur = targetPoint(s, path);

    // Transform target into robot frame
    float dx = cur.x - s.x;
    float dy = cur.y - s.y;
    float ct = std::cos(-s.th), st = std::sin(-s.th);
    float xr = ct * dx - st * dy;
    float yr = st * dx + ct * dy;

    // Unicycle pure pursuit: omega = 2*v*sin(alpha)/Ld, alpha = atan2(yr, xr)
    float v = targetSpeed;
    float alpha = std::atan2(yr, xr);
    float Ld = std::fmax(0.1f, lookahead);
    float omega = (Ld > 1e-4f) ? (2.f * v * std::sin(alpha) / Ld) : 0.f;
    return {v, omega};
  }
};

inline void integrate(RobotState& s, float v, float w, float dt) {
  s.x += v * std::cos(s.th) * dt;
  s.y += v * std::sin(s.th) * dt;
  s.th = wrapAngle(s.th + w * dt);
}

inline float lateralError(const RobotState& s, const std::vector<sf::Vector2f>& path) {
  if (path.size() < 2) return 0.f;
  sf::Vector2f p{s.x, s.y};
  float best = 1e9f; float sign = 1.f;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    sf::Vector2f a = path[i], b = path[i+1];
    sf::Vector2f ab = b - a;
    float ab2 = ab.x*ab.x + ab.y*ab.y;
    if (ab2 < 1e-6f) continue;
    float t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / ab2;
    t = std::fmax(0.f, std::fmin(1.f, t));
    sf::Vector2f proj = a + t * ab;
    sf::Vector2f d = p - proj;
    float dlen = std::sqrt(d.x*d.x + d.y*d.y);
    // Sign via 2D cross product of path tangent and error vector
    float cross = ab.x * d.y - ab.y * d.x;
    float sgn = (cross >= 0.f) ? 1.f : -1.f;
    if (dlen < best) { best = dlen; sign = sgn; }
  }
  return sign * best;
}

struct PIDLateralController {
  float kp{1.5f}, ki{0.0f}, kd{0.3f};
  float targetSpeed{2.0f};
  float integral{0.0f};
  float prevErr{0.0f};

  std::pair<float,float> control(const RobotState& s, const std::vector<sf::Vector2f>& path, float dt) {
    if (path.size() < 2) return {0.f, 0.f};
    float e = lateralError(s, path);
    integral += e * dt;
    float deriv = (dt > 1e-6f) ? ((e - prevErr) / dt) : 0.f;
    prevErr = e;
    float v = targetSpeed;
    float w = kp * e + ki * integral + kd * deriv;
    return {v, w};
  }
};
