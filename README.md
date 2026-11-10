# MultiCam Macro & 3D

Free software (GPL-3.0) Android Camera2 instrument for close-up inspection and dual-camera 3D scanning.
Everything runs on the phone; the APK declares **no network permission**.

| Mode | What it does |
|---|---|
| **Inspect** | Captures a 20-frame burst of a close subject, aligns the frames (gyroscope prior + image refinement) and merges them onto a 2× sub-pixel grid, then lightly sharpens. A draggable before/after view compares it with the first single frame. |
| **3D Scan** | Streams two back cameras of a logical multi-camera together, estimates stereo depth (census + semi-global matching), fuses it into a TSDF volume and exports **STL** and **PLY** meshes. |
| **Labs** | Raw dual-stream view, synchronised calibration-pair capture (PGM), mesh export, calibration-file import and a copyable console. |

## What it does *not* claim

* **Resolution.** A bare phone lens is diffraction/aberration limited long before 2× sampling. Super-resolution removes
  Bayer/aliasing artefacts and recovers edge contrast up to that limit; it does not turn a phone into a microscope.
* **Metric accuracy.** Stereo depth is only metric when the baseline is right. Scale comes, in order of preference, from
  an imported `stereo_calibration.json`, the lens pose the phone reports (Camera2 `LENS_POSE_TRANSLATION`), or a generic
  15 mm estimate (the UI then says *UNCALIBRATED*). Handheld camera motion is tracked from the IMU rotation plus a depth-centroid
  translation estimate — good for object-sized scans, **not** SLAM-grade. Meshes are isosurfaces and are usually open shells.
* **Device coverage is unverified on real hardware.** Camera selection uses only documented Camera2 metadata (no hard-coded IDs,
  sizes or focal lengths) and falls back gracefully, but the author has not tested every vendor. See *Verification*.

## How the hardware is chosen (vendor-neutral)

`CameraTopology` inspects every back camera and logical multi-camera:

* **Macro camera** = the lens (or physical sub-camera) with the highest thin-lens magnification at its closest focus distance.
  It opens directly if the framework lists it, otherwise through its logical camera with physical-stream routing.
* **Stereo pair** = two physical sub-cameras of one logical camera that share a YUV size; the narrower-FOV one is "left".
  Baseline/direction come from the reported lens pose and are converted into the camera frame using `SENSOR_ORIENTATION`.
* **Stream sizes** come from the stream-configuration map. RAW is used only for Bayer sensors that advertise it; otherwise YUV.
* **Timing**: if the camera clock is not `REALTIME`, the gyro/camera clock offset is estimated once per run (less precise; logged).
* **Memory**: the 2× output region is a centred crop sized from the device's RAM (≤ ~24 MP out), so 50–200 MP sensors cannot exhaust memory.
* Focus is auto-locked and exposure locked for the burst; OIS/video stabilisation/edge enhancement are switched off when the device allows.

## Stereo calibration file

Import via **Labs → import button** (or put `stereo_calibration.json` in `Android/data/<package>/files`). OpenCV `stereoCalibrate` conventions,
camera 1 = the "left" camera named in the console, `X2 = R·X1 + T`:

```json
{ "image_size": [1440, 1080],
  "K1": [[fx,0,cx],[0,fy,cy],[0,0,1]], "D1": [k1,k2,p1,p2,k3],
  "K2": [[fx,0,cx],[0,fy,cy],[0,0,1]], "D2": [k1,k2,p1,p2,k3],
  "R": [[1,0,0],[0,1,0],[0,0,1]], "T": [-18.5, 0, 0], "T_units": "mm" }
```

## Verification

The computational core (`app/src/main/cpp/engine.hpp`) is pure C++ and is tested on a host against independently generated ground truth:

```bash
./native-tests/run.sh      # needs g++; CI runs it on every push
```

It checks gyro-integration == rotation-vector-derived rotation for all sensor orientations; burst registration sign/magnitude and
that the merged result beats a single frame (PSNR vs. ground truth) with and without gyro; RAW/sRGB exposure; memory-bounded ROI;
marching-tetrahedra output is closed, consistently wound, outward and on the true surface (sphere); stereo depth error on planes at
150/250/400 mm with horizontal and vertical baselines. **Not covered by automated tests:** Camera2 behaviour on real devices,
sensor timestamps, UI rendering. Those need a phone; please report device results.

## Building

Prerequisites: JDK 17 or 21, Android SDK Platform 37 + Build-Tools 37.0.0, NDK 26.1.10909125, CMake 3.22.1.

```bash
git clone https://github.com/Foadsf/android-multicam.git && cd android-multicam
echo "sdk.dir=$ANDROID_HOME" > local.properties
./native-tests/run.sh          # optional: host tests
./gradlew assembleDebug        # app/build/outputs/apk/debug/app-debug.apk (arm64-v8a, armeabi-v7a, x86_64)
adb install -r -g app/build/outputs/apk/debug/app-debug.apk
```

Every merge to `main` is built by CI and published as the rolling [`latest` release](../../releases/tag/latest) (debug-signed). Every push/PR also uploads the APK as a workflow artifact.

## Outputs

`Android/data/<package>/files/`: `macro_<time>.jpg` (2× result), `scan_<time>.stl|.ply` (millimetres), `left_NN.pgm`/`right_NN.pgm` (calibration pairs).
Use the in-app **Share/Open** buttons — recent Android versions hide that folder from file managers.

## Interface

Thumb-reach layout: viewfinder in the middle, all controls in the bottom panel, ≥ 44–48 dp touch targets, per-mode accent colour, haptic feedback,
screen kept awake while capturing, Back closes the top layer first. Portrait only for now (the viewfinder transform for landscape is not implemented).

## License

[GNU General Public License v3.0](LICENSE).
