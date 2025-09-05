#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Config.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

#include "a_star.hpp"
#include "controller.hpp"
#include "map.hpp"

using Clock = std::chrono::high_resolution_clock;

static std::string nowTimestamp() {
  auto t = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

int main(int argc, char** argv) {
  GridMap map;
  bool loaded = false;
  // CLI flags for random rectangles
  bool flagRandom = false;
  int mapW = 120, mapH = 80;
  int cliRects = 18, cliMin = 3, cliMax = 12;
  unsigned cliSeed = 12345u;
  std::string pngPath;

  auto parseSize = [&](const std::string& s, int& W, int& H) {
    auto xpos = s.find('x');
    if (xpos == std::string::npos) return false;
    try {
      W = std::stoi(s.substr(0, xpos));
      H = std::stoi(s.substr(xpos + 1));
      return (W > 2 && H > 2);
    } catch (...) { return false; }
  };

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--random" || a == "-r") { flagRandom = true; continue; }
    if (a.rfind("--size=", 0) == 0) { std::string v = a.substr(7); parseSize(v, mapW, mapH); continue; }
    if (a == "--size" && i + 1 < argc) { parseSize(argv[++i], mapW, mapH); continue; }
    if (a.rfind("--rects=", 0) == 0) { cliRects = std::max(0, std::atoi(a.c_str() + 8)); continue; }
    if (a == "--rects" && i + 1 < argc) { cliRects = std::max(0, std::atoi(argv[++i])); continue; }
    if (a.rfind("--min=", 0) == 0) { cliMin = std::max(1, std::atoi(a.c_str() + 6)); continue; }
    if (a == "--min" && i + 1 < argc) { cliMin = std::max(1, std::atoi(argv[++i])); continue; }
    if (a.rfind("--max=", 0) == 0) { cliMax = std::max(cliMin, std::atoi(a.c_str() + 6)); continue; }
    if (a == "--max" && i + 1 < argc) { cliMax = std::max(cliMin, std::atoi(argv[++i])); continue; }
    if (a.rfind("--seed=", 0) == 0) { cliSeed = static_cast<unsigned>(std::strtoul(a.c_str() + 7, nullptr, 10)); continue; }
    if (a == "--seed" && i + 1 < argc) { cliSeed = static_cast<unsigned>(std::strtoul(argv[++i], nullptr, 10)); continue; }
    if (!a.empty() && a[0] != '-' && pngPath.empty()) { pngPath = a; }
  }

  if (flagRandom) {
    if (cliMax < cliMin) cliMax = cliMin;
    map.makeRandom(mapW, mapH, cliRects, cliMin, cliMax, cliSeed);
    loaded = true;
    std::cout << "Generated random map: " << mapW << "x" << mapH
              << ", rects=" << cliRects << ", size=[" << cliMin << "," << cliMax
              << "], seed=" << cliSeed << "\n";
  } else if (!pngPath.empty()) {
    loaded = map.loadPNG(pngPath);
    if (!loaded) std::cerr << "Failed to load map from '" << pngPath << "', using demo.\n";
  }
  if (!loaded) map.makeDemo(mapW, mapH);

  const float scale = 8.f; // pixels per cell
#if SFML_VERSION_MAJOR >= 3
  sf::RenderWindow window(sf::VideoMode(sf::Vector2u{static_cast<unsigned>(map.w * scale), static_cast<unsigned>(map.h * scale)}), "Path Planning & Control Sandbox");
#else
  sf::RenderWindow window(sf::VideoMode(static_cast<unsigned>(map.w * scale), static_cast<unsigned>(map.h * scale)), "Path Planning & Control Sandbox");
#endif
  window.setFramerateLimit(60);

  // HUD text removed: no font dependency

  // Sim state
  sf::Vector2i start{1, 1};
  sf::Vector2i goal{map.w - 2, map.h - 2};
  RobotState state{start.x + 0.5f, start.y + 0.5f, 0.f};
  PurePursuit ctrl{2.0f, 2.0f};
  PIDLateralController pid{}; pid.targetSpeed = ctrl.targetSpeed;
  std::vector<sf::Vector2i> gridPath;
  std::vector<sf::Vector2f> smoothPath;
  int smoothingIters = 2;
  bool showLookahead = true;
  bool showRawPath = false;
  bool usePID = false;
  unsigned randSeed = 12345u; bool deterministic = true;
  int rects = 18, rectMin = 3, rectMax = 12;

  // CSV logging
  std::filesystem::create_directories("logs");
  std::string csvPath = std::string("logs/run_") + nowTimestamp() + ".csv";
  std::ofstream csv(csvPath);
  csv << "t,x,y,theta,v,omega,err_lat,path_len,plan_ms\n";
  double simTime = 0.0;

  auto replan = [&](bool reset_pose) {
    auto t0 = Clock::now();
    gridPath = astar::plan(map, start, goal);
    smoothPath.clear();
    if (!gridPath.empty()) {
      auto f = astar::toFloatCenter(gridPath);
      smoothPath = astar::chaikin(f, smoothingIters);
    }
    auto t1 = Clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (reset_pose) {
      state = {start.x + 0.5f, start.y + 0.5f, 0.f};
      pid.integral = 0.f; pid.prevErr = 0.f;
    }
    return ms;
  };

  double lastPlanMs = replan(true);

  // Timing
  const float dt = 1.f / 120.f; // physics step
  sf::Clock clock;
  float accumulator = 0.f;
  bool paused = false;
  // fps EMA removed with HUD text
  double errSumSq = 0.0; long long errCount = 0;

  // Shapes for drawing
  sf::CircleShape robotShape(0.4f * scale);
  robotShape.setFillColor(sf::Color(0, 150, 255));
  robotShape.setOrigin(sf::Vector2f{0.4f * scale, 0.4f * scale});

  sf::RectangleShape startRect({scale, scale});
  startRect.setFillColor(sf::Color(0, 255, 0));
  sf::RectangleShape goalRect({scale, scale});
  goalRect.setFillColor(sf::Color(255, 0, 0));

  auto toPix = [&](sf::Vector2f p) { return sf::Vector2f{p.x * scale, p.y * scale}; };

  // removed unused lastFpsUpdate

  float cmd_v = 0.f, cmd_w = 0.f;
  while (window.isOpen()) {
    // Events
    #if SFML_VERSION_MAJOR >= 3
    while (auto ev = window.pollEvent()) {
      if (ev->is<sf::Event::Closed>()) window.close();
      if (const auto* mb = ev->getIf<sf::Event::MouseButtonPressed>()) {
        int gx = static_cast<int>(mb->position.x / scale);
        int gy = static_cast<int>(mb->position.y / scale);
        if (map.inBounds(gx, gy)) {
          bool shiftDown = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift);
          if (mb->button == sf::Mouse::Button::Left) {
            if (shiftDown) {
              map.toggle(gx, gy);
              for (int x = 0; x < map.w; ++x) { map.setOcc(x, 0, 1); map.setOcc(x, map.h-1, 1); }
              for (int y = 0; y < map.h; ++y) { map.setOcc(0, y, 1); map.setOcc(map.w-1, y, 1); }
              lastPlanMs = replan(false);
            } else if (map.isFree(gx, gy)) {
              start = {gx, gy}; lastPlanMs = replan(true);
            }
          } else if (mb->button == sf::Mouse::Button::Right) {
            if (map.isFree(gx, gy)) { goal = {gx, gy}; lastPlanMs = replan(false); }
          }
        }
      }
      if (const auto* kp = ev->getIf<sf::Event::KeyPressed>()) {
        if (kp->code == sf::Keyboard::Key::R) { lastPlanMs = replan(true); }
        if (kp->code == sf::Keyboard::Key::Space) { paused = !paused; }
        if (kp->code == sf::Keyboard::Key::LBracket) { ctrl.lookahead = std::max(0.5f, ctrl.lookahead - 0.25f); }
        if (kp->code == sf::Keyboard::Key::RBracket) { ctrl.lookahead = std::min(10.f, ctrl.lookahead + 0.25f); }
        if (kp->code == sf::Keyboard::Key::Up) { ctrl.targetSpeed = std::min(10.f, ctrl.targetSpeed + 0.25f); pid.targetSpeed = ctrl.targetSpeed; }
        if (kp->code == sf::Keyboard::Key::Down) { ctrl.targetSpeed = std::max(0.f, ctrl.targetSpeed - 0.25f); pid.targetSpeed = ctrl.targetSpeed; }
        if (kp->code == sf::Keyboard::Key::Semicolon) { smoothingIters = std::max(0, smoothingIters - 1); if (!gridPath.empty()) { auto f = astar::toFloatCenter(gridPath); smoothPath = astar::chaikin(f, smoothingIters); } }
        if (kp->code == sf::Keyboard::Key::Apostrophe) { smoothingIters = std::min(6, smoothingIters + 1); if (!gridPath.empty()) { auto f = astar::toFloatCenter(gridPath); smoothPath = astar::chaikin(f, smoothingIters); } }
        if (kp->code == sf::Keyboard::Key::C) { usePID = !usePID; }
        if (kp->code == sf::Keyboard::Key::P) { showRawPath = !showRawPath; }
        if (kp->code == sf::Keyboard::Key::V) { showLookahead = !showLookahead; }
        if (kp->code == sf::Keyboard::Key::N) {
          unsigned seed = deterministic ? randSeed : static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
          map.makeRandom(map.w, map.h, rects, rectMin, rectMax, seed);
          start = {1,1}; goal = {map.w-2, map.h-2};
          if (!map.isFree(start.x, start.y)) start = {2,2};
          if (!map.isFree(goal.x, goal.y)) goal = {map.w-3, map.h-3};
          lastPlanMs = replan(true);
          if (deterministic) randSeed++;
          errSumSq = 0.0; errCount = 0;
        }
        if (kp->code == sf::Keyboard::Key::T) { deterministic = !deterministic; }
        if (kp->code == sf::Keyboard::Key::Num1) { map.makeOpen(map.w, map.h); lastPlanMs = replan(true); errSumSq = 0.0; errCount = 0; }
        if (kp->code == sf::Keyboard::Key::Num2) { map.makeDemo(map.w, map.h); lastPlanMs = replan(true); errSumSq = 0.0; errCount = 0; }
        if (kp->code == sf::Keyboard::Key::Num3) { map.makeRandom(map.w, map.h, rects*2, 2, rectMax, 42u); lastPlanMs = replan(true); errSumSq = 0.0; errCount = 0; }
        if (kp->code == sf::Keyboard::Key::O) { std::filesystem::create_directories("assets/maps"); map.savePNG("assets/maps/saved.png"); }
        if (kp->code == sf::Keyboard::Key::S || kp->code == sf::Keyboard::Key::G) {
          auto m = sf::Mouse::getPosition(window);
          int gx = static_cast<int>(m.x / scale);
          int gy = static_cast<int>(m.y / scale);
          if (map.inBounds(gx, gy) && map.isFree(gx, gy)) {
            if (kp->code == sf::Keyboard::Key::S) { start = {gx, gy}; lastPlanMs = replan(true); }
            if (kp->code == sf::Keyboard::Key::G) { goal = {gx, gy}; lastPlanMs = replan(false); }
          }
        }
      }
    }
    #else
    sf::Event e;
    while (window.pollEvent(e)) {
      if (e.type == sf::Event::Closed) window.close();
      if (e.type == sf::Event::MouseButtonPressed) {
        sf::Vector2i m = sf::Mouse::getPosition(window);
        int gx = static_cast<int>(m.x / scale);
        int gy = static_cast<int>(m.y / scale);
        if (map.inBounds(gx, gy)) {
          bool shiftDown = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
          if (e.mouseButton.button == sf::Mouse::Left) {
            if (shiftDown) { map.toggle(gx, gy); for (int x = 0; x < map.w; ++x) { map.setOcc(x, 0, 1); map.setOcc(x, map.h-1, 1);} for (int y = 0; y < map.h; ++y) { map.setOcc(0, y, 1); map.setOcc(map.w-1, y, 1);} lastPlanMs = replan(false); }
            else if (map.isFree(gx, gy)) { start = {gx, gy}; lastPlanMs = replan(true); }
          } else if (e.mouseButton.button == sf::Mouse::Right) {
            if (map.isFree(gx, gy)) { goal = {gx, gy}; lastPlanMs = replan(false); }
          }
        }
      }
      if (e.type == sf::Event::KeyPressed) {
        if (e.key.code == sf::Keyboard::R) { lastPlanMs = replan(true); }
        if (e.key.code == sf::Keyboard::Space) { paused = !paused; }
        if (e.key.code == sf::Keyboard::LBracket) { ctrl.lookahead = std::max(0.5f, ctrl.lookahead - 0.25f); }
        if (e.key.code == sf::Keyboard::RBracket) { ctrl.lookahead = std::min(10.f, ctrl.lookahead + 0.25f); }
        if (e.key.code == sf::Keyboard::Up) { ctrl.targetSpeed = std::min(10.f, ctrl.targetSpeed + 0.25f); pid.targetSpeed = ctrl.targetSpeed; }
        if (e.key.code == sf::Keyboard::Down) { ctrl.targetSpeed = std::max(0.f, ctrl.targetSpeed - 0.25f); pid.targetSpeed = ctrl.targetSpeed; }
        if (e.key.code == sf::Keyboard::Semicolon) { smoothingIters = std::max(0, smoothingIters - 1); if (!gridPath.empty()) { auto f = astar::toFloatCenter(gridPath); smoothPath = astar::chaikin(f, smoothingIters); } }
        if (e.key.code == sf::Keyboard::Quote) { smoothingIters = std::min(6, smoothingIters + 1); if (!gridPath.empty()) { auto f = astar::toFloatCenter(gridPath); smoothPath = astar::chaikin(f, smoothingIters); } }
        if (e.key.code == sf::Keyboard::C) { usePID = !usePID; }
        if (e.key.code == sf::Keyboard::P) { showRawPath = !showRawPath; }
        if (e.key.code == sf::Keyboard::V) { showLookahead = !showLookahead; }
        if (e.key.code == sf::Keyboard::N) {
          unsigned seed = deterministic ? randSeed : static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
          map.makeRandom(map.w, map.h, rects, rectMin, rectMax, seed);
          start = {1,1}; goal = {map.w-2, map.h-2};
          if (!map.isFree(start.x, start.y)) start = {2,2};
          if (!map.isFree(goal.x, goal.y)) goal = {map.w-3, map.h-3};
          lastPlanMs = replan(true);
          if (deterministic) randSeed++;
          errSumSq = 0.0; errCount = 0;
        }
        if (e.key.code == sf::Keyboard::T) { deterministic = !deterministic; }
        if (e.key.code == sf::Keyboard::Num1) { map.makeOpen(map.w, map.h); lastPlanMs = replan(true); errSumSq = 0.0; errCount = 0; }
        if (e.key.code == sf::Keyboard::Num2) { map.makeDemo(map.w, map.h); lastPlanMs = replan(true); errSumSq = 0.0; errCount = 0; }
        if (e.key.code == sf::Keyboard::Num3) { map.makeRandom(map.w, map.h, rects*2, 2, rectMax, 42u); lastPlanMs = replan(true); errSumSq = 0.0; errCount = 0; }
        if (e.key.code == sf::Keyboard::O) { std::filesystem::create_directories("assets/maps"); map.savePNG("assets/maps/saved.png"); }
        if (e.key.code == sf::Keyboard::S || e.key.code == sf::Keyboard::G) {
          sf::Vector2i m = sf::Mouse::getPosition(window);
          int gx = static_cast<int>(m.x / scale);
          int gy = static_cast<int>(m.y / scale);
          if (map.inBounds(gx, gy) && map.isFree(gx, gy)) {
            if (e.key.code == sf::Keyboard::S) { start = {gx, gy}; lastPlanMs = replan(true); }
            if (e.key.code == sf::Keyboard::G) { goal = {gx, gy}; lastPlanMs = replan(false); }
          }
        }
      }
    }
    #endif

    // Update timing
    float frame = clock.restart().asSeconds();
    accumulator += frame;
    // FPS/HUD text removed

    // Physics steps
    int steps = 0;
    while (accumulator >= dt) {
      if (!paused && !smoothPath.empty()) {
        auto [v, w] = usePID ? pid.control(state, smoothPath, dt) : ctrl.control(state, smoothPath);
        // Stop near goal
        if (!smoothPath.empty()) {
          sf::Vector2f g = smoothPath.back();
          float dx = g.x - state.x, dy = g.y - state.y;
          float dist = std::sqrt(dx*dx + dy*dy);
          if (dist < 0.5f) { v = 0.f; w = 0.f; }
        }
        cmd_v = v; cmd_w = w;
        integrate(state, cmd_v, cmd_w, dt);

        // Log CSV
        float err = lateralError(state, smoothPath);
        errSumSq += double(err) * double(err);
        ++errCount;
        float plen = astar::pathLength(smoothPath);
        simTime += dt;
        csv << simTime << "," << state.x << "," << state.y << "," << state.th << ","
            << (paused ? 0.f : cmd_v) << "," << (paused ? 0.f : cmd_w) << ","
            << err << "," << plen << "," << lastPlanMs << "\n";
      }
      accumulator -= dt;
      if (++steps > 5) { accumulator = 0.f; break; }
    }

    // Render
    window.clear(sf::Color(30, 30, 30));
    map.draw(window, scale);

    // Start/goal
    startRect.setPosition(sf::Vector2f{start.x * scale, start.y * scale});
    goalRect.setPosition(sf::Vector2f{goal.x * scale, goal.y * scale});
    window.draw(startRect);
    window.draw(goalRect);

    // Path render
    if (showRawPath && gridPath.size() >= 2) {
      auto raw = astar::toFloatCenter(gridPath);
#if SFML_VERSION_MAJOR >= 3
      sf::VertexArray vaRaw(sf::PrimitiveType::LineStrip, raw.size());
#else
      sf::VertexArray vaRaw(sf::LineStrip, raw.size());
#endif
      for (size_t i = 0; i < raw.size(); ++i) {
        vaRaw[i].position = toPix(raw[i]);
        vaRaw[i].color = sf::Color(100, 100, 255);
      }
      window.draw(vaRaw);
    }
    if (smoothPath.size() >= 2) {
#if SFML_VERSION_MAJOR >= 3
      sf::VertexArray va(sf::PrimitiveType::LineStrip, smoothPath.size());
#else
      sf::VertexArray va(sf::LineStrip, smoothPath.size());
#endif
      for (size_t i = 0; i < smoothPath.size(); ++i) {
        va[i].position = toPix(smoothPath[i]);
        va[i].color = sf::Color(255, 140, 0);
      }
      window.draw(va);
    }

    // Robot
    robotShape.setPosition(sf::Vector2f{state.x * scale, state.y * scale});
    #if SFML_VERSION_MAJOR >= 3
    robotShape.setRotation(sf::degrees(state.th * 180.f / 3.14159265f));
    #else
    robotShape.setRotation(state.th * 180.f / 3.14159265f);
    #endif
    window.draw(robotShape);

    // HUD text removed; overlays (path, robot, lookahead) still drawn

    // Lookahead target render
    if (showLookahead && !smoothPath.empty()) {
      sf::Vector2f tpt = ctrl.targetPoint(state, smoothPath);
      sf::CircleShape lh(0.2f * scale);
      lh.setOrigin(sf::Vector2f{0.2f * scale, 0.2f * scale});
      lh.setFillColor(sf::Color(0,255,0,160));
      lh.setPosition(toPix(tpt));
      window.draw(lh);
    }

    window.display();
  }

  return 0;
}
