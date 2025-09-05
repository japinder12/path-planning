#pragma once

#include <SFML/Graphics.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include "map.hpp"

namespace astar {

inline int idx(int x, int y, int w) { return y * w + x; }

struct Node {
  int x, y; float f;
  bool operator<(const Node& other) const { return f > other.f; } // min-heap
};

inline float heuristic(int x0, int y0, int x1, int y1) {
  float dx = float(x0 - x1);
  float dy = float(y0 - y1);
  return std::sqrt(dx*dx + dy*dy);
}

inline std::vector<sf::Vector2i> plan(const GridMap& map, sf::Vector2i start, sf::Vector2i goal) {
  std::vector<sf::Vector2i> empty;
  if (!map.inBounds(start.x, start.y) || !map.inBounds(goal.x, goal.y)) return empty;
  if (!map.isFree(start.x, start.y) || !map.isFree(goal.x, goal.y)) return empty;

  const int w = map.w, h = map.h;
  std::priority_queue<Node> open;
  std::vector<uint8_t> closed(w * h, 0);
  std::vector<float> g(w * h, std::numeric_limits<float>::infinity());
  std::vector<int> came(w * h, -1);

  int s = idx(start.x, start.y, w), t = idx(goal.x, goal.y, w);
  g[s] = 0.f;
  open.push({start.x, start.y, heuristic(start.x, start.y, goal.x, goal.y)});

  const int dx[8] = {1,1,0,-1,-1,-1,0,1};
  const int dy[8] = {0,1,1,1,0,-1,-1,-1};
  const float cost[8] = {1, std::sqrt(2.f), 1, std::sqrt(2.f), 1, std::sqrt(2.f), 1, std::sqrt(2.f)};

  bool found = false;
  while (!open.empty()) {
    Node n = open.top(); open.pop();
    int id = idx(n.x, n.y, w);
    if (closed[id]) continue;
    closed[id] = 1;
    if (n.x == goal.x && n.y == goal.y) { found = true; break; }

    for (int k = 0; k < 8; ++k) {
      int nx = n.x + dx[k];
      int ny = n.y + dy[k];
      if (!map.inBounds(nx, ny) || !map.isFree(nx, ny)) continue;
      int nid = idx(nx, ny, w);
      float tentative = g[id] + cost[k];
      if (tentative < g[nid]) {
        g[nid] = tentative;
        came[nid] = id;
        float f = tentative + heuristic(nx, ny, goal.x, goal.y);
        open.push({nx, ny, f});
      }
    }
  }

  if (!found) return empty;

  // Reconstruct
  std::vector<sf::Vector2i> path;
  int cur = t;
  while (cur != -1) {
    int cx = cur % w;
    int cy = cur / w;
    path.push_back({cx, cy});
    if (cur == s) break;
    cur = came[cur];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

inline std::vector<sf::Vector2f> toFloatCenter(const std::vector<sf::Vector2i>& p) {
  std::vector<sf::Vector2f> out; out.reserve(p.size());
  for (auto& v : p) out.push_back({v.x + 0.5f, v.y + 0.5f});
  return out;
}

inline std::vector<sf::Vector2f> chaikin(const std::vector<sf::Vector2f>& poly, int iterations = 1) {
  if (poly.size() < 2) return poly;
  std::vector<sf::Vector2f> cur = poly;
  for (int it = 0; it < iterations; ++it) {
    std::vector<sf::Vector2f> next; next.reserve(cur.size() * 2);
    next.push_back(cur.front());
    for (size_t i = 0; i + 1 < cur.size(); ++i) {
      sf::Vector2f P = cur[i];
      sf::Vector2f Q = cur[i + 1];
      sf::Vector2f R = 0.75f * P + 0.25f * Q; // keep closer to original
      sf::Vector2f S = 0.25f * P + 0.75f * Q;
      next.push_back(R);
      next.push_back(S);
    }
    next.push_back(cur.back());
    cur.swap(next);
  }
  return cur;
}

inline float pathLength(const std::vector<sf::Vector2f>& p) {
  float L = 0.f;
  for (size_t i = 1; i < p.size(); ++i) {
    sf::Vector2f d = p[i] - p[i-1];
    L += std::sqrt(d.x*d.x + d.y*d.y);
  }
  return L;
}

} // namespace astar
