# Path Planning & Control Sandbox (C++/SFML)

A desktop app where a "robot" plans with A* on a 2D occupancy grid and tracks the smoothed path using a Pure Pursuit controller. Visualized with SFML, with a HUD and CSV logging for metrics.

## Quick start

Dependencies:
- C++17 compiler, CMake >= 3.16
- SFML 2.5 (graphics, window, system)

Install:
- macOS: `brew install cmake sfml`
- Ubuntu: `sudo apt install cmake libsfml-dev`
- Windows: install SFML via vcpkg and pass the vcpkg toolchain to CMake

Build and run:
```bash
mkdir build && cd build
cmake ..
cmake --build . -j
./sandbox ../assets/maps/demo.png   # or no arg to use demo map
```

Note: Place a TTF font at `assets/fonts/DejaVuSans.ttf` to enable HUD text (otherwise the app runs with a minimal HUD fallback). A demo map is optional; without an image the app generates a simple map procedurally.

## Controls
- Mouse: LMB=start, RMB=goal
- Keys: R=replan, Space=pause/resume, [ / ] decrease/increase lookahead

## Metrics / HUD
- FPS (EMA-smoothed)
- Path length (smoothed polyline, in cells)
- Last replan time (ms for A* + smoothing)
- Lookahead (Ld) and speed
- Lateral error (signed)
- Footer hint for controls

## CSV logging
- File: `logs/run_YYYYMMDD_HHMMSS.csv`
- Header: `t,x,y,theta,v,omega,err_lat,path_len,plan_ms`
- Appends every physics tick (dt ~ 1/120s).

## Implementation notes
- GridMap: loads PNG (white=free, black=obstacle) or generates a demo map. Drawn as a grid.
- A*: 8-connected, Euclidean heuristic. Reconstructs grid path.
- Smoothing: Chaikin (1–2 iterations) -> float polyline.
- Controller: Pure Pursuit (unicycle/diff-drive style). `omega = 2*v*sin(alpha)/Ld`.
- Integration: fixed-step (dt ≈ 1/120s), window render at ~60 FPS.

## Portfolio GIF
See `scripts/record_gif.md` for recording instructions.

## Roadmap / stretch goals
- Toggle obstacles (Shift+click) + auto-replan
- Inflate obstacles by robot radius
- PID lateral controller / bicycle vs. diff-drive toggle
- Unit tests for A* on small maps
- ROS2 wrapper