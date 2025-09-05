# Developer Guide

This doc contains setup, build/run, CLI, and IDE details that were trimmed from the README for brevity.

## Dependencies
- C++17 compiler
- CMake >= 3.16
- SFML 2.5 (graphics, window, system) or SFML 3 (auto-detected)

## Install
- macOS: `brew install cmake sfml`
- Ubuntu: `sudo apt install cmake libsfml-dev`
- Windows: install SFML via vcpkg and pass the vcpkg toolchain to CMake

Windows (vcpkg) example:
```bash
vcpkg install sfml:x64-windows
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build -j
```

## Build and Run
```bash
cmake -B build -S .
cmake --build build -j
./build/sandbox assets/maps/demo.png   # or no arg to use the procedural demo map

# Random rectangles map via CLI flags
./build/sandbox --random --size=120x80 --rects 24 --min 3 --max 10 --seed 1234
```

Note: You can save the current map with `O` to `assets/maps/saved.png`.

## CLI Flags
- `--random` (or `-r`): generate a random rectangles map instead of loading a PNG or the demo map.
- `--size=WxH` or `--size WxH`: grid size (default 120x80).
- `--rects N`: number of rectangles (default 18).
- `--min N`: minimum rectangle side length in cells (default 3).
- `--max N`: maximum rectangle side length in cells (default 12).
- `--seed N`: RNG seed for reproducible maps (default 12345).

Examples:
- `./build/sandbox -r --size=160x100 --rects 30 --min 2 --max 8 --seed 42`
- `./build/sandbox --random --size 120x80 --rects 12`

## Metrics / CSV
- CSV file: `logs/run_YYYYMMDD_HHMMSS.csv`
- Header: `t,x,y,theta,v,omega,err_lat,path_len,plan_ms`
- Appends every physics tick (dt ~ 1/120s)

## Implementation Notes
- GridMap: loads PNG (white=free, black=obstacle), generates demo, open, or random rectangle maps. Obstacles can be toggled per-cell and maps saved back to PNG. Drawn as a grid.
- A*: 8-connected, Euclidean heuristic. Reconstructs grid path.
- Smoothing: Chaikin (1–2 iterations) -> float polyline.
- Controller: Pure Pursuit (unicycle/diff-drive style) and a PID option on lateral error. `omega = 2*v*sin(alpha)/Ld` for Pure Pursuit.
- Integration: fixed-step (dt ≈ 1/120s), window render at ~60 FPS. Visualization toggles include lookahead target and raw path overlay.

## IDE Setup (VS Code)
- CMake config generates `build/compile_commands.json` (project enables `CMAKE_EXPORT_COMPILE_COMMANDS`).
- Point IntelliSense to it:
  - Settings: set `C_Cpp.default.compileCommands` to `build/compile_commands.json`, or use the CMake Tools extension.
- If the editor shows `cannot open source file "SFML/Graphics.hpp"`, verify SFML is installed and add include fallbacks by platform:
  - macOS (Homebrew): `/opt/homebrew/include`, `/opt/homebrew/include/SFML` (Apple Silicon) or `/usr/local/include`, `/usr/local/include/SFML` (Intel)
  - Windows (vcpkg): `C:/vcpkg/installed/x64-windows/include`
  - Linux (apt): `/usr/include`, `/usr/include/SFML`

## Recording a GIF
See `scripts/record_gif.md` for macOS/Linux instructions. Output goes to `assets/demo.gif`.
