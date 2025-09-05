#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Config.hpp>
#include <cstdint>
#include <string>
#include <vector>
#include <random>

struct GridMap {
  int w = 0;
  int h = 0;
  // 0 = free, 1 = obstacle
  std::vector<uint8_t> occ;

  bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
  bool isFree(int x, int y) const { return inBounds(x, y) && occ[y * w + x] == 0; }

  void setOcc(int x, int y, uint8_t val) {
    if (inBounds(x, y)) occ[y * w + x] = val ? 1 : 0;
  }

  void toggle(int x, int y) {
    if (inBounds(x, y)) occ[y * w + x] = occ[y * w + x] ? 0 : 1;
  }

  bool loadPNG(const std::string& path) {
    sf::Image img;
    if (!img.loadFromFile(path)) return false;
    w = static_cast<int>(img.getSize().x);
    h = static_cast<int>(img.getSize().y);
    occ.assign(w * h, 0);
    // Expect white=free, black=obstacle (threshold on luminance)
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
#if SFML_VERSION_MAJOR >= 3
        sf::Color c = img.getPixel(sf::Vector2u{static_cast<unsigned>(x), static_cast<unsigned>(y)});
#else
        sf::Color c = img.getPixel(x, y);
#endif
        float lum = 0.2126f * (c.r / 255.f) + 0.7152f * (c.g / 255.f) + 0.0722f * (c.b / 255.f);
        occ[y * w + x] = (lum < 0.5f) ? 1 : 0;
      }
    }
    return true;
  }

  void makeDemo(int W, int H) {
    w = W; h = H; occ.assign(w * h, 0);
    // Simple demo: border walls + a few blocks and corridors
    for (int x = 0; x < w; ++x) { occ[x] = 1; occ[(h-1) * w + x] = 1; }
    for (int y = 0; y < h; ++y) { occ[y * w + 0] = 1; occ[y * w + (w-1)] = 1; }
    // Blocks
    for (int y = h/4; y < h/4 + h/6; ++y)
      for (int x = w/5; x < w/5 + w/6; ++x) occ[y * w + x] = 1;
    for (int y = h/2; y < h/2 + h/8; ++y)
      for (int x = w/2; x < w/2 + w/8; ++x) occ[y * w + x] = 1;
    // Corridor
    for (int y = h/3; y < h/3 + 2; ++y)
      for (int x = w/5 + w/6; x < w - w/5; ++x) occ[y * w + x] = 0;
  }

  void makeOpen(int W, int H) {
    w = W; h = H; occ.assign(w * h, 0);
    for (int x = 0; x < w; ++x) { occ[x] = 1; occ[(h-1) * w + x] = 1; }
    for (int y = 0; y < h; ++y) { occ[y * w + 0] = 1; occ[y * w + (w-1)] = 1; }
  }

  void makeRandom(int W, int H, int nRects, int minSize, int maxSize, unsigned seed) {
    w = W; h = H; occ.assign(w * h, 0);
    // Borders
    for (int x = 0; x < w; ++x) { occ[x] = 1; occ[(h-1) * w + x] = 1; }
    for (int y = 0; y < h; ++y) { occ[y * w + 0] = 1; occ[y * w + (w-1)] = 1; }
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> wdist(minSize, maxSize);
    std::uniform_int_distribution<int> xdist(1, std::max(1, W - maxSize - 2));
    std::uniform_int_distribution<int> ydist(1, std::max(1, H - maxSize - 2));
    for (int i = 0; i < nRects; ++i) {
      int rw = wdist(rng);
      int rh = wdist(rng);
      int rx = xdist(rng);
      int ry = ydist(rng);
      for (int y = ry; y < ry + rh && y < H - 1; ++y)
        for (int x = rx; x < rx + rw && x < W - 1; ++x)
          occ[y * w + x] = 1;
    }
  }

  bool savePNG(const std::string& path) const {
#if SFML_VERSION_MAJOR >= 3
    sf::Image img(sf::Vector2u{static_cast<unsigned>(w), static_cast<unsigned>(h)}, sf::Color::White);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        bool blocked = occ[y * w + x] != 0;
        img.setPixel(sf::Vector2u{static_cast<unsigned>(x), static_cast<unsigned>(y)}, blocked ? sf::Color::Black : sf::Color::White);
      }
    }
    return img.saveToFile(path);
#else
    sf::Image img; img.create(static_cast<unsigned>(w), static_cast<unsigned>(h), sf::Color::White);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        bool blocked = occ[y * w + x] != 0;
        img.setPixel(static_cast<unsigned>(x), static_cast<unsigned>(y), blocked ? sf::Color::Black : sf::Color::White);
      }
    }
    return img.saveToFile(path);
#endif
  }

  void draw(sf::RenderTarget& target, float scale) const {
    sf::RectangleShape cell({scale, scale});
    cell.setOutlineThickness(0.f);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        uint8_t o = occ[y * w + x];
        cell.setPosition(sf::Vector2f{static_cast<float>(x) * scale, static_cast<float>(y) * scale});
        cell.setFillColor(o ? sf::Color::Black : sf::Color::White);
        target.draw(cell);
      }
    }
  }
};
