# Contributing

Contributions to this project are welcome. A few ground rules keep this app small, honest, and free:

- **Zero network access, ever.** No `INTERNET` permission, no analytics, no telemetry, no third-party crash reporting. CI asserts that `android.permission.INTERNET` is absent from the APK.
- **No proprietary dependencies.** Everything must be FLOSS and buildable from source.
- **Hardware honesty.** Do not claim optical performance exceeding diffraction or sampling physics. Super-resolution restores Nyquist edge contrast and removes Bayer demosaic blur; it does not turn bare smartphone lenses into cellular microscopes.
- **No device-specific binary patching.** Camera pipelines must use documented Android Camera2 (API 28+) logical multi-camera features and NDK sensor loops. If a manufacturer HAL refuses concurrent physical streams, fail gracefully with clear diagnostic messages.
- **Clean architecture.** Camera and sensor loops run in dedicated background executors; UI is declarative Kotlin + Jetpack Compose (Material 3).

## Pull Requests

1. Verify that the build succeeds locally:
   ```bash
   ./gradlew assembleDebug
   ```
2. Verify that no network permissions were introduced:
   ```bash
   "$ANDROID_HOME/build-tools/37.0.0/aapt2" dump permissions app/build/outputs/apk/debug/app-debug.apk | grep INTERNET
   ```
   (This must return nothing).
3. If changing camera or native pipelines, describe the test device, sensor IDs, and physical stream formats verified.

All contributions are licensed under the [GNU General Public License v3.0](LICENSE).
