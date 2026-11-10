# MultiCam Macro & 3D

MultiCam Macro & 3D is an experimental, fully open-source (GPL-3.0) Android Camera2 instrument providing:
- **Handheld Gyro-Assisted Macro Super-Resolution**: Designed for mostly planar subjects (PCB solder fillets, textiles, coins, prints).
- **Dual-Camera Stereo Depth Estimation**: Synchronous Wide (`Camera 0`) and Ultra-Wide (`Camera 2`) streaming under a single logical multi-camera handle.
- **TSDF Voxel Isosurface Reconstruction**: Kuhn Marching Tetrahedra volumetric fusion exporting binary **STL** and **PLY** models.
- **100% On-Device Processing**: Completely offline; **zero network permissions** (`android.permission.INTERNET` is strictly omitted).

> **Note on Calibration & Metric Scale**:
> Uncalibrated stereo scanning operates with empirical baseline and focal estimates (~18.5 mm baseline, central depth centroid translation tracking). Metric accuracy and reconstruction fidelity depend on external stereo calibration (`stereo_calibration.json`) and scene geometry. Mesh export produces a voxel-grid isosurface; topological 2-manifold watertightness depends on scene boundary closure.

---

## Core Capabilities

### 1. Surface Inspection (Macro Super-Resolution)
Exploits natural human hand tremor across a 20-frame burst to eliminate Bayer demosaicing artifacts and increase effective resolving power:
- **Gyroscope-Assisted Alignment**: Integrates angular velocity over sensor exposure periods using Rodrigues rotation kinematics ($H = K R K^{-1}$) to project frames onto a shared reference plane.
- **2× Drizzle Accumulator**: Sub-pixel phase-shift accumulation onto an $8064 \times 6048$ grid.
- **Interactive Before/After Comparison**: Compares the single-frame baseline directly with the reconstructed super-resolution output.
- **Optical Resolution Envelope**:
  With primary focal length $f \approx 4.3\text{ mm}$, minimum focus distance $d_0 \approx 90\text{ mm}$, and sensor pixel pitch $p = 1.4\ \mu\text{m}$, the geometric footprint on the object plane is:
  $$\text{Footprint} = \frac{p}{M} \approx \frac{1.4\ \mu\text{m}}{0.05} \approx 28\ \mu\text{m}$$
  By Nyquist-Shannon criteria ($2 \times 28\ \mu\text{m} = 56\ \mu\text{m}$) and the $f/2.4$ Airy disk ($\approx 64\ \mu\text{m}$), the physical resolving limit of the bare phone lens is **$\sim 60\text{--}80\ \mu\text{m}$**.

### 2. 3D Scanning (Stereo Depth & Mesh Extraction)
Synchronously binds Wide and Ultra-Wide physical camera sub-devices:
- **Logical Multi-Camera Session**: Uses Camera2 `OutputConfiguration.setPhysicalCameraId("0")` (Wide) and `setPhysicalCameraId("2")` (Ultra-Wide).
- **Census + Semi-Global Matching (SGM)**: Stereo disparity calculation across physical perspectives.
- **Real-Time Depth Heatmap**: Live false-color depth preview rendered directly in the viewfinder.
- **TSDF Isosurface Export**: Fuses depth maps into a 3D Truncated Signed Distance Field (TSDF) voxel grid and extracts meshes via Kuhn Marching Tetrahedra into **STL** and **PLY** files.
- **Calibration Pair Capture**: Stores synchronized uncompressed PGM frame pairs for offline calibration.

---

## User Interface & Experience

Designed for one-handed, thumb-reach use: every control sits in the bottom panel, every touch target is at least 44–48 dp, and the active mode is colour-coded (teal = Inspect, violet = 3D Scan, orange = Labs).

- **Undistorted live viewfinder** with framing reticle; the preview buffer is negotiated 4:3 so nothing is stretched.
- **Stability badge**: gyroscope-driven `STEADY` / `HOLD STILL` pill that also recolours the reticle.
- **Mode switcher** (`INSPECT`, `3D SCAN`, `LABS`) with a sliding highlight, haptic tick and disabled while a capture runs.
- **One big shutter** that morphs into a stop button, shows burst progress as a ring, and gives haptic feedback. The screen stays awake while capturing.
- **Before/after reveal**: drag the divider, pinch to zoom (up to 4×), double-tap to reset.
- **3D scan**: live depth heat-map with near/far legend, elapsed timer, one-tap *Finish & build mesh*, then Share STL / Share PLY / Open / New scan.
- **Labs**: standard vs. full-res streams, one-tap calibration-pair capture, mesh export, and an inline console that expands into a copyable log sheet.
- **Guide sheet** explaining each mode and tips; Back always closes the topmost layer first.
- **Direct sharing** of images and meshes through Android `FileProvider`.

---

## Hardware Compatibility

- **Target OS**: Android 10+ (API 29+, compiled with SDK 37).
- **Camera Requirements**: Multi-camera hardware advertising `REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA` with independent physical stream routing.
- **Privacy Assurance**: No internet or external communication permissions are requested or declared.

---

## Building from Source

Prerequisites:
- JDK 17 or JDK 21
- Android SDK Platform 37 (`platforms;android-37.0`) and Build-Tools `37.0.0`
- Android NDK `26.1.10909125`
- CMake `3.22.1`

```bash
# Clone the repository
git clone https://github.com/Foadsf/android-multicam.git
cd android-multicam

# Configure local Android SDK path
echo "sdk.dir=$ANDROID_HOME" > local.properties

# Build debug APK
./gradlew assembleDebug

# Output APK:
# app/build/outputs/apk/debug/app-debug.apk
```

Install via ADB:
```bash
adb install -r -g app/build/outputs/apk/debug/app-debug.apk
```

---

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).
