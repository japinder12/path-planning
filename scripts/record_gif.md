Record a GIF of the app on macOS/Linux using ffmpeg + gifski. Output the GIF to `assets/demo.gif` so it can be embedded in the README.

1) Install tools
- macOS (Homebrew): `brew install ffmpeg gifski`
- Ubuntu: `sudo apt install ffmpeg gifski`

2) Record the window (by display or region)
```bash
# Find window id / size (optional)
# ffmpeg -f avfoundation -list_devices true -i ""  # macOS

# Record ~10s at 60 fps to mp4 (adjust duration/size as needed)
# macOS (records entire display 1)
ffmpeg -y -f avfoundation -framerate 60 -i 1 -t 10 out.mp4

# Linux (X11): record a 1280x720 region at +0,0; adjust to your window
ffmpeg -y -video_size 1280x720 -framerate 60 -f x11grab -i :0.0+0,0 -t 10 out.mp4
```

3) Convert to high‑quality GIF with gifski
```bash
mkdir -p assets
gifski -W 960 -o assets/demo.gif out.mp4
```

Tips to showcase in the clip
- Set start (LMB), goal (RMB); toggle an obstacle (Shift+LMB)
- Toggle raw path (P) and lookahead target (V)
- Increase/decrease smoothing (; and ')
- Switch controllers (C) and generate a new map (N)
