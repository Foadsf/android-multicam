# MultiCam Macro & 3D

A zero-patch, fully open-source (GPL-3.0) Android multi-camera suite for close-up surface inspection and metric 3D scanning. Built on modern Android Camera2 (API 28+), uncalibrated high-rate NDK sensor fusion, and Kotlin + Jetpack Compose (Material 3).

No root, no vendor binary patching, and **zero network permissions**: all processing runs 100% on-device in memory.

---

## Capabilities

### 1. Handheld Macro Super-Resolution (`Camera 0`)
Exploits natural human hand tremor to achieve sub-pixel resolution enhancement and demosaicing artifact removal on the primary camera:
- **400 Hz Inertial Fusion**: Uncalibrated gyroscope samples read via NDK `ASensorManager` and integrated over exposure intervals via $SO(3)$ Rodrigues rotation kinematics to form an inter-frame homography warp ($H = K R K^{-1}$).
- **2× Drizzle Grid**: Sub-pixel phase-shift accumulation onto an $8064 \times 6048$ grid, eliminating Bayer demosaic blur filters.
- **Richardson-Lucy Deconvolution**: Iterative MTF restoration recovering optical edge contrast.
- **Optical Reality Check**:
  With $f \approx 4.3\text{ mm}$, minimum focus distance $d_0 \approx 90\text{ mm}$, and sensor pixel pitch $p = 1.4\ \mu\text{m}$, the geometric footprint on the object plane is:
  $$\text{Footprint} = \frac{p}{M} \approx \frac{1.4\ \mu\text{m}}{0.05} \approx 28\ \mu\text{m}$$
  By Nyquist-Shannon criteria ($2 \times 28\ \mu\text{m} = 56\ \mu\text{m}$) and the $f/2.4$ Airy disk ($\approx 64\ \mu\text{m}$), the physical resolution limit of the bare lens is **$\sim 60\text{--}80\ \mu\text{m}$**. The pipeline reliably recovers true optical sharpness on PCB solder fillets, surface textures, and fibers, without claiming impossible sub-20 $\mu\text{m}$ cellular microscopy.

### 2. Synchronous Dual-Camera 3D Scanning (`Camera 0` + `Camera 2`)
Streams physical sub-devices under a single logical multi-camera handle:
- **Logical Multi-Camera Binding**: Uses Android Camera2 `OutputConfiguration.setPhysicalCameraId("0")` (Wide) and `setPhysicalCameraId("2")` (Ultra-Wide).
- **Census + Semi-Global Matching (SGM)**: Robust disparity computation across differing camera viewpoints.
- **Live Depth Heatmap**: Real-time false-colour depth preview rendered in the Viewfinder.
- **TSDF & Watertight Mesh Export**: Volumetric Truncated Signed Distance Field (TSDF) fusion exporting watertight triangle meshes in both binary **STL** and **PLY** formats to the app's files directory.
- **Stereo Pair Capture**: Stores synchronised PGM frame pairs for external ChArUco / Bouguet stereo extrinsic calibration.

---

## UI & Architecture

Borrowing the clean, dark instrument design language from [`Foadsf/android-microscope`](https://github.com/Foadsf/android-microscope) and [`Foadsf/parallax`](https://github.com/Foadsf/parallax):
- **Declarative Interface**: Built with Kotlin and Jetpack Compose (Material 3).
- **Decoupled Engines**:
  - `CaptureEngine.java`: Camera2 multi-stream sessions, ImageReader pipelines, frame synchronisation locks.
  - `NativeEngine.java` / `native-lib.cpp`: High-performance C++17 multi-threaded SIMD image processing, uncalibrated gyro ring buffer, Drizzle accumulator, and TSDF meshing.
  - `MultiCamViewModel.kt`: Unified reactive `StateFlow<UiState>` managing modes, shutter animations, and diagnostics.
- **Privacy Contract**: `android.permission.INTERNET` is strictly omitted from `AndroidManifest.xml`. CI automatically verifies that no network permission exists in the compiled APK.

---

## Hardware Compatibility

- **Target Architecture**: Android 10+ (minSdk 29, compileSdk 37).
- **Camera Requirements**: Multi-camera devices exposing `REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA` with independent physical camera streams (e.g. Samsung Galaxy S10 / Note10 / S20 series).
- **Fallback**: If simultaneous physical YUV/RAW streams are refused by the vendor HAL, the UI reports the refusal directly and offers sequential / diagnostic test modes.

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

To install on a connected device via ADB:
```bash
adb install -r -g app/build/outputs/apk/debug/app-debug.apk
```

---

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).
