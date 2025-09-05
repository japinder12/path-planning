Record a GIF of the app on macOS/Linux using ffmpeg + gifski.

1) Install tools
- macOS (Homebrew): `brew install ffmpeg gifski`
- Ubuntu: `sudo apt install ffmpeg gifski`

2) Record window by title (or use screen coordinates)
```bash
# Find window id / size (optional)
# ffmpeg -f avfoundation -list_devices true -i ""  # macOS

# Record at 60 fps for ~10s to mp4 (change duration as needed)
ffmpeg -y -video_size 1280x720 -framerate 60 -f x11grab -i :0.0+0,0 -t 10 out.mp4   # Linux
# macOS example (records entire display 0):
# ffmpeg -y -f avfoundation -framerate 60 -i 1 -t 10 out.mp4
```

3) Convert to high‑quality GIF with gifski
```bash
gifski -W 960 -o demo.gif out.mp4
```
