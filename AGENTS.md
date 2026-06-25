# AGENTS.md — 相机标定 (Camera Calibration)

## Dual implementations — keep in sync

Both `camera_calibration.cpp` and `camera_calibration.py` implement the same pipeline. Changes to board parameters, calibration logic, or output format should be reflected in **both** files.

## Build & run

```bash
# C++ (CMake >= 3.10, C++14, OpenCV)
cd build && cmake .. && make -j$(nproc) && ./camera_calibration

# Python
python3 camera_calibration.py
```

Both accept image paths as positional args; without args they auto-discover `calibration_images/chessboardNN.jpg` sequentially.

## Hardcoded board parameters

Both files hardcode `boardSize = (6, 4)` (inner corners width × height) and `squareSize = 30.0` (mm). These are **not** passed via CLI — edit the source if your board differs.

## Image naming convention

Images must follow `chessboard01.jpg`, `chessboard02.jpg`, … (zero-padded two-digit numbering).

- `calibration_images/` — intrinsic calibration images (auto-discovered, up to `chessboard43.jpg`)
- `extrinsic_images/` — **separate folder** for extrinsics PnP images (same naming convention, independently loaded)

## Output gitignored files

The following are in `.gitignore` and should never be committed:
- `build/` — CMake build artifacts
- `camera_calibration_result.xml` — intrinsic calibration output
- `camera_extrinsic_result.xml` — extrinsic PnP output
- `undistortion_results/` — batch-exported comparison images (Python only)
- `process.ipynb` — exploratory notebook

## C++ vs Python behavioral difference

| Feature | C++ | Python |
|---|---|---|
| Undistortion output | Interactive `imshow` loop (press any key / ESC) | Batch saves comparison images to `undistortion_results/` |
| GUI detection | Always shows windows | Auto-disables `imshow` when no `DISPLAY`/`WAYLAND_DISPLAY` |

## Jupyter notebook

`process.ipynb` is gitignored. If regenerating it, export to `.py` or add an allowlist entry before committing.
