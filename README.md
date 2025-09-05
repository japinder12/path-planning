# Path Planning & Control Sandbox (C++/SFML)

A desktop app where a "robot" plans with A* on a 2D occupancy grid and tracks the smoothed path using a Pure Pursuit controller. Visualized with SFML, with a HUD and CSV logging for metrics.

## Features
- A* on 2D occupancy grid (8-connected, Euclidean heuristic)
- Chaikin path smoothing to produce a drivable polyline
- Two controllers: Pure Pursuit and PID lateral
- On-screen overlays: path, robot pose, lookahead target
- CSV telemetry logging (pose, commands, lateral error, path length, plan time)
- PNG map load/save and interactive obstacle editing

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
# Random rectangles map via CLI flags
./sandbox --random --size=120x80 --rects 24 --min 3 --max 10 --seed 1234
```

Note: A demo map is optional; without an image the app generates a simple map procedurally. Save the current map with `O` to `assets/maps/saved.png`.

## Controls
- Mouse:
  - LMB = set start
  - Shift + LMB = toggle obstacle (auto-replan)
  - RMB = set goal (if available)
- Keys:
  - `S` / `G` = set Start/Goal at mouse cell
  - `R` = replan and reset pose
  - `Space` = pause/resume
  - `[` / `]` = decrease/increase lookahead
  - `;` / `'` = decrease/increase smoothing iterations (Chaikin)
  - `Up` / `Down` = increase/decrease speed
  - `C` = toggle controller (Pure Pursuit / PID lateral)
  - `P` = toggle raw grid path overlay
  - `V` = toggle lookahead target point overlay
- `N` = generate random rectangles map (deterministic seed advances)
- `T` = toggle deterministic vs time-based random seed
- `1`/`2`/`3` = presets (open field / demo / dense random)
- `O` = save current map to `assets/maps/saved.png`

## CLI flags
- `--random` (or `-r`): generate a random rectangles map instead of loading a PNG or the demo map.
- `--size=WxH` or `--size WxH`: grid size (default 120x80).
- `--rects N`: number of rectangles (default 18).
- `--min N`: minimum rectangle side length in cells (default 3).
- `--max N`: maximum rectangle side length in cells (default 12).
- `--seed N`: RNG seed for reproducible maps (default 12345).

Examples:
- `./sandbox -r --size=160x100 --rects 30 --min 2 --max 8 --seed 42`
- `./sandbox --random --size 120x80 --rects 12`

## Metrics
- CSV columns: `t,x,y,theta,v,omega,err_lat,path_len,plan_ms`
- Visual overlays: smoothed path polyline and lookahead target (toggle with `P`/`V`).

## CSV logging
- File: `logs/run_YYYYMMDD_HHMMSS.csv`
- Header: `t,x,y,theta,v,omega,err_lat,path_len,plan_ms`
- Appends every physics tick (dt ~ 1/120s).

## Implementation notes
- GridMap: loads PNG (white=free, black=obstacle), generates demo, open, or random rectangle maps. Obstacles can be toggled per-cell and maps can be saved back to PNG. Drawn as a grid.
- A*: 8-connected, Euclidean heuristic. Reconstructs grid path.
- Smoothing: Chaikin (1–2 iterations) -> float polyline.
- Controller: Pure Pursuit (unicycle/diff-drive style) and a PID option on lateral error. `omega = 2*v*sin(alpha)/Ld` for Pure Pursuit.
- Integration: fixed-step (dt ≈ 1/120s), window render at ~60 FPS.
 - Visualization toggles: show lookahead point and raw path overlay.

## Portfolio GIF
See `scripts/record_gif.md` for recording instructions.

<!-- ## IDE Setup (VS Code)
- Ensure a configure step generates `build/compile_commands.json`:
  - `cmake -B build -S .`
- Point IntelliSense to it (recommended):
  - Settings: set `C_Cpp.default.compileCommands` to `build/compile_commands.json`, or enable the CMake Tools extension which provides configuration automatically.
- If you still see `cannot open source file "SFML/Graphics.hpp"` in the editor, verify SFML is installed and add include fallbacks in your VS Code configuration (platform-dependent):
  - macOS (Homebrew): `/opt/homebrew/include`, `/opt/homebrew/include/SFML` (Apple Silicon) or `/usr/local/include`, `/usr/local/include/SFML` (Intel)
  - Windows (vcpkg): `C:/vcpkg/installed/x64-windows/include`
  - Linux (apt): `/usr/include`, `/usr/include/SFML` -->

## Roadmap / stretch goals
- Bicycle vs. diff-drive model toggle (+ tuning)
- ROS2 wrapper (publish /map, /path, /cmd_vel)

## License
This project is licensed under the MIT License
