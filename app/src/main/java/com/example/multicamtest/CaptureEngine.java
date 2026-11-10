package com.example.multicamtest;

import android.app.ActivityManager;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.BlackLevelPattern;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.view.Surface;

import java.io.File;
import java.io.FileOutputStream;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Camera2 + sensor orchestration. All camera/lens/size decisions come from {@link CameraTopology}
 * (documented metadata only); all maths lives in the native engine.
 *
 * <p>Threads: {@code cameraThread} runs Camera2 callbacks and state changes; {@code workThread} receives
 * ImageReader frames and runs the native processing so heavy work never blocks camera callbacks.
 */
public final class CaptureEngine {
    public interface Listener {
        int LEVEL_INFO = 0, LEVEL_BUSY = 1, LEVEL_OK = 2, LEVEL_ERROR = 3;
        void onState(String message, int level);
        void onProgress(int value, int max);
        void onLog(String line);
        void onStability(float stability);
        void onDepthFrame(Bitmap bitmap);
        void onMacroResult(Bitmap before, Bitmap after, File file, int outWidth, int outHeight);
        void onScanResult(File stlFile, File plyFile, int triangleCount);
        void onStereoInfo(float baselineMm, float zMinMm, float zMaxMm, String calibration);
        void onCapabilities(boolean stereo, boolean raw);
    }

    private static final String TAG = "MultiCam";
    public static final int MICROSCOPE_BURST_FRAMES = 20;
    private static final long FOCUS_TIMEOUT_MS = 3000;
    private static final int SETTLE_FRAMES_AFTER_LOCK = 3;

    private final Context ctx;
    private final Listener listener;
    private final CameraTopology topo;

    private final HandlerThread cameraThread = new HandlerThread("mc-camera"), workThread = new HandlerThread("mc-work");
    private final Handler cameraHandler, workHandler;
    private final Executor cameraExecutor;
    private final ExecutorService finishExecutor = Executors.newSingleThreadExecutor(r -> new Thread(r, "mc-finish"));

    private SensorManager sensorManager;
    private Sensor gyroSensor;
    private volatile float stability = 1.0f;
    private long lastStabilityEmit;
    private boolean sensorsActive;

    // --- camera state (touched on cameraThread, except where noted) ---
    private Surface previewSurface;
    private boolean busy;
    private CameraDevice cameraDevice;
    private CameraCaptureSession session;
    private ImageReader reader0, reader2;

    // --- work-thread state, guarded by procLock; runId invalidates in-flight frames of a finished run ---
    private final Object procLock = new Object();
    private int runId;
    private int sensorOrientation = 90;
    private boolean timeOffsetKnown;
    private boolean cameraClockRealtime = true;

    // macro
    private volatile boolean macroAccepting;
    private volatile int macroSettle;
    private final AtomicBoolean macroFinishing = new AtomicBoolean(false);
    private int[] macroOut;
    private boolean focusDone, aeDone, locked;
    private long focusStartMs;

    // stereo
    private int stW, stH;
    private int[] heatmap;
    private float[] stereoInfo;
    private long pairSyncNs = 6_000_000L;
    private final AtomicBoolean saveNextPair = new AtomicBoolean(false);
    private int pairCounter, pairFramesLogged;
    private long lastDepthEmitMs;
    private static final class Slot { long ts; int w, h, stride; ByteBuffer y; boolean fresh; }
    private final Slot slotL = new Slot(), slotR = new Slot();

    public CaptureEngine(Context context, Listener l) {
        ctx = context.getApplicationContext();
        listener = l;
        topo = CameraTopology.get(ctx);
        cameraThread.start(); workThread.start();
        cameraHandler = new Handler(cameraThread.getLooper());
        workHandler = new Handler(workThread.getLooper());
        cameraExecutor = cameraHandler::post;

        sensorManager = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager != null) gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        sensorOrientation = topo.macro != null ? topo.macro.sensorOrientation : 90;
        NativeEngine.startImuLooper(ctx.getPackageName(), sensorOrientation);
        for (String n : topo.notes) listener.onLog(n);
        listener.onCapabilities(topo.stereo != null, topo.macro != null && topo.macro.supportsRaw16());
        if (gyroSensor == null) listener.onLog("No gyroscope: stability meter disabled, registration relies on image alignment only.");
    }

    // ------------------------------------------------------------------------------------------
    // lifecycle / public API
    // ------------------------------------------------------------------------------------------

    /** Register motion sensors only while the UI is visible. */
    public void setActive(boolean active) {
        cameraExecutor.execute(() -> {
            if (active == sensorsActive || sensorManager == null) return;
            sensorsActive = active;
            if (active && gyroSensor != null) sensorManager.registerListener(gyroListener, gyroSensor, SensorManager.SENSOR_DELAY_UI, cameraHandler);
            else sensorManager.unregisterListener(gyroListener);
        });
    }

    public File outputDir() { return ctx.getExternalFilesDir(null); }

    public void setPreviewSurface(Surface surface) {
        cameraExecutor.execute(() -> {
            previewSurface = surface;
            if (surface != null && surface.isValid() && !busy) startLivePreview();
            else if (surface == null) cleanupCamera();
        });
    }

    public void resumePreview() {
        cameraExecutor.execute(() -> {
            busy = false;
            if (previewSurface != null && previewSurface.isValid()) startLivePreview();
        });
    }

    public void startScan(boolean fullRes) { cameraExecutor.execute(() -> { busy = true; startStereo(fullRes); }); }
    public void startMacro(boolean raw) { cameraExecutor.execute(() -> { busy = true; startMacroBurst(raw); }); }

    public void stop() {
        cameraExecutor.execute(() -> {
            cleanupCamera();
            busy = false;
            if (previewSurface != null && previewSurface.isValid()) startLivePreview();
        });
    }

    /** Stops everything and releases the camera (called when the UI goes to the background). */
    public void pause() {
        cameraExecutor.execute(() -> { cleanupCamera(); busy = false; });
        setActive(false);
    }

    public void savePair() {
        saveNextPair.set(true);
        listener.onState("Saving next synchronised pair…", Listener.LEVEL_BUSY);
    }

    /** Builds and exports the mesh. Blocking: call from a background thread. */
    public int exportMesh() {
        File dir = outputDir();
        if (dir == null) { listener.onState("Storage unavailable.", Listener.LEVEL_ERROR); return -1; }
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        File stl = new File(dir, "scan_" + stamp + ".stl"), ply = new File(dir, "scan_" + stamp + ".ply");
        int tris = NativeEngine.exportStereoMesh(stl.getAbsolutePath(), ply.getAbsolutePath());
        listener.onLog(tris > 0 ? "Mesh exported: " + stl.getName() + " (" + tris + " triangles)" : "Mesh export produced no surface.");
        listener.onScanResult(tris > 0 ? stl : null, tris > 0 ? ply : null, tris);
        return tris;
    }

    public void release() {
        setActive(false);
        cameraExecutor.execute(this::cleanupCamera);
        NativeEngine.stopImuLooper();
        NativeEngine.releaseMicroscope();
        finishExecutor.shutdown();
        cameraThread.quitSafely();
        workThread.quitSafely();
    }

    private final SensorEventListener gyroListener = new SensorEventListener() {
        @Override public void onSensorChanged(SensorEvent e) {
            float omega = (float) Math.sqrt(e.values[0] * e.values[0] + e.values[1] * e.values[1] + e.values[2] * e.values[2]);
            float inst = Math.max(0f, Math.min(1f, 1f - omega / 0.35f));  // 0.35 rad/s (~20 deg/s) = unusable
            float prev = stability;
            stability = 0.8f * prev + 0.2f * inst;
            long now = SystemClock.elapsedRealtime();
            if (Math.abs(stability - prev) > 0.03f || now - lastStabilityEmit > 250) { lastStabilityEmit = now; listener.onStability(stability); }
        }
        @Override public void onAccuracyChanged(Sensor s, int a) {}
    };

    private void setState(String s, int level) { listener.onState(s, level); }
    private void log(String s) { Log.i(TAG, s); listener.onLog(s); }
    private boolean previewStarting;  // a failed preview must not restart itself forever

    private void fail(String message) {
        Log.e(TAG, message);
        final boolean wasPreview = previewStarting;
        previewStarting = false;
        busy = false;
        cleanupCamera();
        listener.onProgress(0, 0);
        setState(message, Listener.LEVEL_ERROR);
        if (!wasPreview && previewSurface != null && previewSurface.isValid()) startLivePreview();
    }

    // ------------------------------------------------------------------------------------------
    // open + session with fallback plans
    // ------------------------------------------------------------------------------------------

    private interface SessionReady { void onReady(CameraDevice dev, CameraCaptureSession s); }

    private static final class Plan { final String id; final boolean routed; Plan(String id, boolean routed) { this.id = id; this.routed = routed; } }

    private List<Plan> plansFor(CameraTopology.Cam cam) {
        List<Plan> p = new ArrayList<>();
        if (cam.physicalId == null) { p.add(new Plan(cam.openId, false)); return p; }
        if (cam.directId != null) p.add(new Plan(cam.directId, false));  // plain open of a framework-listed physical id
        p.add(new Plan(cam.openId, true));                               // logical camera + physical stream routing
        return p;
    }

    /** Opens {@code cam}, creates a session over {@code surfaces}, trying each open plan until one configures. */
    private void openSession(CameraTopology.Cam cam, List<Surface> surfaces, SessionReady ready) {
        tryPlan(cam, plansFor(cam), 0, surfaces, ready, "");
    }

    private void tryPlan(CameraTopology.Cam cam, List<Plan> plans, int i, List<Surface> surfaces, SessionReady ready, String lastError) {
        if (i >= plans.size()) { fail("Camera unavailable" + (lastError.isEmpty() ? "." : ": " + lastError)); return; }
        final Plan plan = plans.get(i);
        CameraManager manager = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        final int myRun;
        synchronized (procLock) { myRun = runId; }
        try {
            manager.openCamera(plan.id, cameraExecutor, new CameraDevice.StateCallback() {
                private boolean handled;
                private void next(String why) {
                    if (handled) return;
                    handled = true;
                    if (!isCurrent(myRun)) return;
                    closeDeviceOnly();
                    tryPlan(cam, plans, i + 1, surfaces, ready, why);
                }
                @Override public void onOpened(CameraDevice dev) {
                    if (!isCurrent(myRun)) { dev.close(); return; }
                    cameraDevice = dev;
                    try {
                        List<OutputConfiguration> outs = new ArrayList<>();
                        for (Surface s : surfaces) {
                            OutputConfiguration oc = new OutputConfiguration(s);
                            if (plan.routed) oc.setPhysicalCameraId(cam.physicalId);
                            outs.add(oc);
                        }
                        dev.createCaptureSession(new SessionConfiguration(SessionConfiguration.SESSION_REGULAR, outs, cameraExecutor,
                            new CameraCaptureSession.StateCallback() {
                                @Override public void onConfigured(CameraCaptureSession s) {
                                    if (!isCurrent(myRun)) { s.close(); return; }
                                    session = s; handled = true;
                                    ready.onReady(dev, s);
                                }
                                @Override public void onConfigureFailed(CameraCaptureSession s) {
                                    next("session configuration failed on " + plan.id + (plan.routed ? " (routed)" : ""));
                                }
                            }));
                    } catch (Exception e) { next(e.getClass().getSimpleName() + ": " + e.getMessage()); }
                }
                @Override public void onDisconnected(CameraDevice dev) { dev.close(); if (cameraDevice == dev) cameraDevice = null; next("camera disconnected"); }
                @Override public void onError(CameraDevice dev, int err) { dev.close(); if (cameraDevice == dev) cameraDevice = null; next("camera error " + err); }
            });
        } catch (SecurityException e) {
            fail("Camera permission denied.");
        } catch (Exception e) {
            tryPlan(cam, plans, i + 1, surfaces, ready, e.getMessage() == null ? e.getClass().getSimpleName() : e.getMessage());
        }
    }

    private boolean isCurrent(int id) { synchronized (procLock) { return id == runId; } }

    private void closeDeviceOnly() {
        if (session != null) { try { session.close(); } catch (Exception ignored) {} session = null; }
        if (cameraDevice != null) { try { cameraDevice.close(); } catch (Exception ignored) {} cameraDevice = null; }
    }

    /** Closes everything and invalidates frames still in flight. Waits for an in-progress frame to finish. */
    private void cleanupCamera() {
        synchronized (procLock) {
            runId++;
            macroAccepting = false;
            closeDeviceOnly();
            if (reader0 != null) { try { reader0.close(); } catch (Exception ignored) {} reader0 = null; }
            if (reader2 != null) { try { reader2.close(); } catch (Exception ignored) {} reader2 = null; }
            slotL.fresh = slotR.fresh = false;
        }
    }

    // ------------------------------------------------------------------------------------------
    // live preview
    // ------------------------------------------------------------------------------------------

    private void startLivePreview() {
        if (previewSurface == null || !previewSurface.isValid() || topo.macro == null) return;
        cleanupCamera();
        final Surface target = previewSurface;
        final CameraTopology.Cam cam = topo.macro;
        previewStarting = true;
        openSession(cam, java.util.Collections.singletonList(target), (dev, s) -> {
            try {
                CaptureRequest.Builder b = dev.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                b.addTarget(target);
                b.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
                if (has(cam.chars.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES), CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_PICTURE))
                    b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                b.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
                s.setRepeatingRequest(b.build(), null, cameraHandler);
                previewStarting = false;
                setState("Ready — frame your subject.", Listener.LEVEL_OK);
            } catch (Exception e) { fail("Preview failed: " + e.getMessage()); }
        });
    }

    // ------------------------------------------------------------------------------------------
    // macro burst
    // ------------------------------------------------------------------------------------------

    private static boolean has(int[] arr, int v) { if (arr != null) for (int a : arr) if (a == v) return true; return false; }

    private static Size largest(Size[] sizes, double maxPixels) {
        Size best = null;
        for (Size s : sizes) {
            double a = (double) s.getWidth() * s.getHeight();
            if (a <= maxPixels && (best == null || a > (double) best.getWidth() * best.getHeight())) best = s;
        }
        return best;
    }

    private float outputBudgetMegapixels() {
        ActivityManager am = (ActivityManager) ctx.getSystemService(Context.ACTIVITY_SERVICE);
        ActivityManager.MemoryInfo mi = new ActivityManager.MemoryInfo();
        am.getMemoryInfo(mi);
        // ~10 float planes (4 B) live at the peak of reconstruction; keep that under ~10% of total RAM.
        double mp = mi.totalMem * 0.10 / 40.0 / 1e6;
        return (float) Math.max(6.0, Math.min(24.0, mp));
    }

    private void startMacroBurst(boolean wantRaw) {
        cleanupCamera();
        final CameraTopology.Cam cam = topo.macro;
        if (cam == null) { fail("No back camera found."); return; }
        final boolean raw = wantRaw && cam.supportsRaw16();
        if (wantRaw && !raw) log("RAW16 not available on " + cam.label() + " — using processed frames.");
        Size size = raw ? largest(cam.sizes(ImageFormat.RAW_SENSOR), 1e9) : largest(cam.sizes(ImageFormat.YUV_420_888), 20e6);
        if (size == null) { fail("The camera offers no suitable capture size."); return; }
        final int w = size.getWidth(), h = size.getHeight();

        sensorOrientation = cam.sensorOrientation;
        NativeEngine.setSensorOrientation(sensorOrientation);
        Integer ts = cam.chars.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE);
        cameraClockRealtime = ts != null && ts == CameraMetadata.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME;
        timeOffsetKnown = false;
        NativeEngine.setImuTimeOffsetNs(0);
        if (!cameraClockRealtime) log("Camera clock is not REALTIME: gyro alignment uses an estimated clock offset (less precise).");

        float fx, fy;
        float[] k = cam.chars.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION);
        android.graphics.Rect arr = cam.chars.get(CameraCharacteristics.SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE);
        if (arr == null) arr = cam.chars.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
        float scale = arr != null && arr.width() > 0 ? (float) w / arr.width() : 1f;
        if (k != null && k.length >= 2 && k[0] > 0) { fx = k[0] * scale; fy = k[1] * scale; }
        else if (cam.sensorMm != null && cam.sensorMm.getWidth() > 0 && cam.focalMm > 0) { fx = fy = cam.focalMm / cam.sensorMm.getWidth() * w; }
        else { fx = fy = 0.8f * w; }
        Integer cfa = cam.chars.get(CameraCharacteristics.SENSOR_INFO_COLOR_FILTER_ARRANGEMENT);
        Integer white = cam.chars.get(CameraCharacteristics.SENSOR_INFO_WHITE_LEVEL);
        BlackLevelPattern blp = cam.chars.get(CameraCharacteristics.SENSOR_BLACK_LEVEL_PATTERN);
        int black = 0;
        if (blp != null) black = (blp.getOffsetForIndex(0, 0) + blp.getOffsetForIndex(1, 0) + blp.getOffsetForIndex(0, 1) + blp.getOffsetForIndex(1, 1)) / 4;
        NativeEngine.configureMicroscope(fx, fy, cfa != null ? cfa : 0, black, white != null ? white : 1023);
        macroOut = NativeEngine.resetMicroscope(w, h, outputBudgetMegapixels());
        macroFinishing.set(false);
        macroAccepting = false;
        macroSettle = SETTLE_FRAMES_AFTER_LOCK;
        log(String.format(Locale.US, "Macro burst: %s %dx%d on %s → processing a %dx%d output region", raw ? "RAW16" : "YUV", w, h, cam.label(), macroOut[0], macroOut[1]));
        setState("Focusing…", Listener.LEVEL_BUSY);
        listener.onProgress(0, MICROSCOPE_BURST_FRAMES);

        final int fmt = raw ? ImageFormat.RAW_SENSOR : ImageFormat.YUV_420_888;
        final int myRun;
        synchronized (procLock) { myRun = runId; }
        reader0 = ImageReader.newInstance(w, h, fmt, 4);
        reader0.setOnImageAvailableListener(r -> onMacroImage(r, myRun, raw), workHandler);

        openSession(cam, java.util.Collections.singletonList(reader0.getSurface()), (dev, s) -> startMacroRequests(cam, dev, s));
    }

    private void startMacroRequests(CameraTopology.Cam cam, CameraDevice dev, CameraCaptureSession s) {
        try {
            focusDone = aeDone = locked = false;
            focusStartMs = SystemClock.elapsedRealtime();
            final boolean af = has(cam.chars.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES), CameraMetadata.CONTROL_AF_MODE_AUTO) && cam.minFocusDiopters > 0;
            final boolean aeLockOk = Boolean.TRUE.equals(cam.chars.get(CameraCharacteristics.CONTROL_AE_LOCK_AVAILABLE));
            final Surface target = reader0.getSurface();
            if (!af) focusDone = true;

            CaptureRequest.Builder b = macroBuilder(cam, dev, target, af, false);
            CameraCaptureSession.CaptureCallback cb = new CameraCaptureSession.CaptureCallback() {
                @Override public void onCaptureCompleted(CameraCaptureSession sess, CaptureRequest req, TotalCaptureResult res) {
                    if (locked) {
                        int left = macroSettle;
                        if (left > 0) {
                            macroSettle = left - 1;
                            if (left == 1) { macroAccepting = true; setState("Hold steady — capturing…", Listener.LEVEL_BUSY); }
                        }
                        return;
                    }
                    Integer afs = res.get(CaptureResult.CONTROL_AF_STATE), aes = res.get(CaptureResult.CONTROL_AE_STATE);
                    if (af && afs != null && (afs == CaptureResult.CONTROL_AF_STATE_FOCUSED_LOCKED || afs == CaptureResult.CONTROL_AF_STATE_NOT_FOCUSED_LOCKED)) focusDone = true;
                    if (aes == null || aes == CaptureResult.CONTROL_AE_STATE_CONVERGED || aes == CaptureResult.CONTROL_AE_STATE_LOCKED || aes == CaptureResult.CONTROL_AE_STATE_FLASH_REQUIRED) aeDone = true;
                    boolean timeout = SystemClock.elapsedRealtime() - focusStartMs > FOCUS_TIMEOUT_MS;
                    if ((focusDone && aeDone) || timeout) {
                        locked = true;
                        if (!(focusDone && aeDone)) log("Focus/exposure did not settle in time; capturing anyway.");
                        try {
                            CaptureRequest.Builder lb = macroBuilder(cam, dev, target, af, aeLockOk);
                            sess.setRepeatingRequest(lb.build(), this, cameraHandler);
                        } catch (Exception e) { fail("Camera request failed: " + e.getMessage()); }
                    }
                }
            };
            if (af) {
                CaptureRequest.Builder t = macroBuilder(cam, dev, target, true, false);
                t.set(CaptureRequest.CONTROL_AF_TRIGGER, CaptureRequest.CONTROL_AF_TRIGGER_START);
                s.capture(t.build(), cb, cameraHandler);
            }
            s.setRepeatingRequest(b.build(), cb, cameraHandler);
        } catch (Exception e) { fail("Camera request failed: " + e.getMessage()); }
    }

    private CaptureRequest.Builder macroBuilder(CameraTopology.Cam cam, CameraDevice dev, Surface target, boolean af, boolean aeLock) throws CameraAccessException {
        CaptureRequest.Builder b = dev.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
        b.addTarget(target);
        b.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
        b.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
        if (aeLock) b.set(CaptureRequest.CONTROL_AE_LOCK, true);  // constant exposure across the burst
        if (af) b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_AUTO);
        else {
            b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
            if (cam.minFocusDiopters > 0) b.set(CaptureRequest.LENS_FOCUS_DISTANCE, Math.min(cam.minFocusDiopters, 10f));
        }
        // Shortest exposures: the highest frame-rate range (<= 60 fps) the device offers.
        Range<Integer>[] ranges = cam.chars.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
        if (ranges != null && ranges.length > 0) b.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, bestRangeAtMost(ranges, 60));
        // Stabilisation and ISP sharpening move/alter pixels in ways the registration model does not know about.
        if (has(cam.chars.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION), CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF))
            b.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF);
        if (has(cam.chars.get(CameraCharacteristics.CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES), CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF))
            b.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
        if (has(cam.chars.get(CameraCharacteristics.EDGE_AVAILABLE_EDGE_MODES), CameraMetadata.EDGE_MODE_OFF))
            b.set(CaptureRequest.EDGE_MODE, CameraMetadata.EDGE_MODE_OFF);
        int[] nr = cam.chars.get(CameraCharacteristics.NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES);
        if (has(nr, CameraMetadata.NOISE_REDUCTION_MODE_MINIMAL)) b.set(CaptureRequest.NOISE_REDUCTION_MODE, CameraMetadata.NOISE_REDUCTION_MODE_MINIMAL);
        else if (has(nr, CameraMetadata.NOISE_REDUCTION_MODE_OFF)) b.set(CaptureRequest.NOISE_REDUCTION_MODE, CameraMetadata.NOISE_REDUCTION_MODE_OFF);
        return b;
    }

    private static Range<Integer> bestRangeAtMost(Range<Integer>[] ranges, int maxFps) {
        Range<Integer> best = null;
        for (Range<Integer> r : ranges) {
            if (r.getUpper() > maxFps) continue;
            if (best == null || r.getUpper() > best.getUpper() || (r.getUpper().equals(best.getUpper()) && r.getLower() > best.getLower())) best = r;
        }
        return best != null ? best : ranges[0];
    }

    private void onMacroImage(ImageReader r, int myRun, boolean raw) {
        Image img;
        try { img = r.acquireLatestImage(); } catch (Exception e) { return; }  // reader closed under us
        if (img == null) return;
        try {
            synchronized (procLock) {
                if (myRun != runId || !macroAccepting || macroFinishing.get()) return;
                if (!cameraClockRealtime && !timeOffsetKnown) {  // UNKNOWN timestamp source: pin to elapsedRealtime once
                    NativeEngine.setImuTimeOffsetNs(SystemClock.elapsedRealtimeNanos() - img.getTimestamp());
                    timeOffsetKnown = true;
                }
                long ts = img.getTimestamp(); int w = img.getWidth(), h = img.getHeight(), acc;
                if (raw) {
                    Image.Plane p = img.getPlanes()[0];
                    acc = NativeEngine.pushMicroscopeRaw16Frame(p.getBuffer(), p.getRowStride(), w, h, ts);
                } else {
                    Image.Plane py = img.getPlanes()[0], pu = img.getPlanes()[1], pv = img.getPlanes()[2];
                    acc = NativeEngine.pushMicroscopeYuvFrame(py.getBuffer(), py.getRowStride(), pu.getBuffer(), pu.getRowStride(), pu.getPixelStride(),
                        pv.getBuffer(), pv.getRowStride(), pv.getPixelStride(), w, h, ts);
                }
                if (acc < 0) { cameraExecutor.execute(() -> fail("Frame rejected by the engine (size mismatch).")); return; }
                listener.onProgress(acc, MICROSCOPE_BURST_FRAMES);
                setState("Aligning frame " + acc + " of " + MICROSCOPE_BURST_FRAMES + " — hold steady", Listener.LEVEL_BUSY);
                if (acc >= MICROSCOPE_BURST_FRAMES && macroFinishing.compareAndSet(false, true)) {
                    macroAccepting = false;
                    cameraExecutor.execute(() -> { cleanupCamera(); finishExecutor.execute(this::finishMacro); });
                }
            }
        } finally {
            img.close();
        }
    }

    private void finishMacro() {
        setState("Reconstructing 2× image…", Listener.LEVEL_BUSY);
        listener.onProgress(0, 0);
        Bitmap full = null;
        try {
            if (!NativeEngine.finalizeMicroscope()) { failFromWorker("Reconstruction failed."); return; }
            int ow = macroOut[0], oh = macroOut[1];
            full = Bitmap.createBitmap(ow, oh, Bitmap.Config.ARGB_8888);
            if (!NativeEngine.exportMicroscopeToBitmap(full)) { failFromWorker("Could not read the reconstructed image."); return; }
            int[] bs = NativeEngine.baselineSize();
            Bitmap base = Bitmap.createBitmap(bs[0], bs[1], Bitmap.Config.ARGB_8888);
            boolean haveBase = NativeEngine.exportBaselineToBitmap(base);
            NativeEngine.releaseMicroscope();  // free native planes before encoding

            File dir = outputDir();
            File out = new File(dir, "macro_" + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date()) + ".jpg");
            try (FileOutputStream fos = new FileOutputStream(out)) {
                if (!full.compress(Bitmap.CompressFormat.JPEG, 95, fos)) throw new java.io.IOException("JPEG encoder failed");
            }
            // Both previews at the same displayed size so the divider compares like with like.
            float sc = Math.min(1f, 2200f / Math.max(ow, oh));
            int pw = Math.max(1, Math.round(ow * sc)), ph = Math.max(1, Math.round(oh * sc));
            Bitmap after = Bitmap.createScaledBitmap(full, pw, ph, true);
            Bitmap before = haveBase ? Bitmap.createScaledBitmap(base, pw, ph, true) : after;
            base.recycle();
            full.recycle(); full = null;
            log("Saved " + out.getName() + " (" + ow + "×" + oh + ", " + (out.length() / 1024) + " kB)");
            setState("2× super-resolution ready.", Listener.LEVEL_OK);
            listener.onMacroResult(upright(before), upright(after), out, ow, oh);
        } catch (Throwable t) {
            Log.e(TAG, "macro finish failed", t);
            failFromWorker("Reconstruction failed: " + (t instanceof OutOfMemoryError ? "not enough memory" : t.getMessage()));
        } finally {
            if (full != null) full.recycle();
            NativeEngine.releaseMicroscope();
            cameraExecutor.execute(() -> { busy = false; if (previewSurface != null && previewSurface.isValid()) startLivePreview(); });
        }
    }

    private void failFromWorker(String m) { cameraExecutor.execute(() -> fail(m)); }

    /** Rotate a sensor-oriented bitmap so it appears upright with the phone held in portrait. */
    private Bitmap upright(Bitmap src) {
        if (src == null || sensorOrientation % 360 == 0) return src;
        Matrix m = new Matrix(); m.postRotate(sensorOrientation);
        Bitmap r = Bitmap.createBitmap(src, 0, 0, src.getWidth(), src.getHeight(), m, true);
        return r;
    }

    // ------------------------------------------------------------------------------------------
    // stereo scan
    // ------------------------------------------------------------------------------------------

    private void startStereo(boolean fullRes) {
        cleanupCamera();
        final CameraTopology.StereoPair pair = topo.stereo;
        if (pair == null) { fail("3D scanning needs two back cameras exposed as one logical camera — not available on this phone."); return; }
        Size size = CameraTopology.commonStereoSize(pair.left, pair.right, fullRes);
        if (size == null) { fail("The two cameras share no common stream size."); return; }
        stW = size.getWidth(); stH = size.getHeight();

        sensorOrientation = pair.left.sensorOrientation;
        NativeEngine.setSensorOrientation(sensorOrientation);
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        Integer ts = null;
        try { ts = cm.getCameraCharacteristics(pair.logicalId).get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE); } catch (Exception ignored) {}
        cameraClockRealtime = ts != null && ts == CameraMetadata.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME;
        timeOffsetKnown = false;
        NativeEngine.setImuTimeOffsetNs(0);
        pairSyncNs = pair.calibratedSync ? 6_000_000L : 16_000_000L;

        List<String> notes = new ArrayList<>();
        StereoCalibration cal = StereoCalibration.build(outputDir(), pair, stW, stH, notes);
        for (String n : notes) log(n);
        NativeEngine.resetStereoScanner();
        NativeEngine.configureStereo(stW, stH, cal.params);
        stereoInfo = NativeEngine.stereoInfo();
        int rw = (int) stereoInfo[5], rh = (int) stereoInfo[6];
        heatmap = new int[rw * rh];
        listener.onStereoInfo(stereoInfo[0], stereoInfo[1], stereoInfo[2], cal.source);
        log(String.format(Locale.US, "Stereo %dx%d, baseline %.1f mm, depth range %.0f–%.0f mm, calibration: %s", stW, stH, stereoInfo[0], stereoInfo[1], stereoInfo[2], cal.source));
        pairFramesLogged = 0; lastDepthEmitMs = 0;

        final int myRun;
        synchronized (procLock) { myRun = runId; }
        reader0 = ImageReader.newInstance(stW, stH, ImageFormat.YUV_420_888, 4);
        reader2 = ImageReader.newInstance(stW, stH, ImageFormat.YUV_420_888, 4);
        reader0.setOnImageAvailableListener(r -> onStereoImage(r, myRun, true), workHandler);
        reader2.setOnImageAvailableListener(r -> onStereoImage(r, myRun, false), workHandler);

        final Surface s0 = reader0.getSurface(), s2 = reader2.getSurface();
        try {
            cm.openCamera(pair.logicalId, cameraExecutor, new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice dev) {
                    if (!isCurrent(myRun)) { dev.close(); return; }
                    cameraDevice = dev;
                    try {
                        OutputConfiguration c0 = new OutputConfiguration(s0); c0.setPhysicalCameraId(pair.left.physicalId);
                        OutputConfiguration c2 = new OutputConfiguration(s2); c2.setPhysicalCameraId(pair.right.physicalId);
                        dev.createCaptureSession(new SessionConfiguration(SessionConfiguration.SESSION_REGULAR, Arrays.asList(c0, c2), cameraExecutor,
                            new CameraCaptureSession.StateCallback() {
                                @Override public void onConfigured(CameraCaptureSession s) {
                                    if (!isCurrent(myRun)) { s.close(); return; }
                                    session = s;
                                    try {
                                        CaptureRequest.Builder b = dev.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                                        b.addTarget(s0); b.addTarget(s2);
                                        b.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
                                        b.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
                                        // Fixed focus keeps both lenses' focal lengths (and so the calibration) constant.
                                        float near = Math.min(pair.left.minFocusDiopters, pair.right.minFocusDiopters);
                                        if (near > 0f && has(pair.left.chars.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES), CameraMetadata.CONTROL_AF_MODE_OFF)) {
                                            float subjectMm = Math.max(120f, stereoInfo[1] * 1.8f);
                                            b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
                                            b.set(CaptureRequest.LENS_FOCUS_DISTANCE, Math.min(near, 1000f / subjectMm));
                                        }
                                        if (has(pair.left.chars.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION), CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF))
                                            b.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF);
                                        s.setRepeatingRequest(b.build(), null, cameraHandler);
                                        setState(String.format(Locale.US, "Scanning — keep the object %.0f–%.0f cm away and move slowly", stereoInfo[1] / 10f, Math.min(stereoInfo[2], 600f) / 10f), Listener.LEVEL_OK);
                                    } catch (Exception e) { fail("Camera request failed: " + e.getMessage()); }
                                }
                                @Override public void onConfigureFailed(CameraCaptureSession s) {
                                    fail(fullRes ? "The phone refused full-resolution dual streams — try Standard." : "The phone refused to stream both cameras at once.");
                                }
                            }));
                    } catch (Exception e) { fail("Dual stream setup failed: " + e.getMessage()); }
                }
                @Override public void onDisconnected(CameraDevice dev) { dev.close(); if (isCurrent(myRun)) fail("Camera disconnected."); }
                @Override public void onError(CameraDevice dev, int err) { dev.close(); if (isCurrent(myRun)) fail("Camera error " + err + "."); }
            });
        } catch (SecurityException e) { fail("Camera permission denied."); }
        catch (CameraAccessException e) { fail("Camera access failed: " + e.getMessage()); }
        catch (Exception e) { fail("Dual stream setup failed: " + e.getMessage()); }
    }

    private void onStereoImage(ImageReader r, int myRun, boolean isLeft) {
        Image img;
        try { img = r.acquireLatestImage(); } catch (Exception e) { return; }
        if (img == null) return;
        try {
            synchronized (procLock) {
                if (myRun != runId) return;
                if (!cameraClockRealtime && !timeOffsetKnown) {
                    NativeEngine.setImuTimeOffsetNs(SystemClock.elapsedRealtimeNanos() - img.getTimestamp());
                    timeOffsetKnown = true;
                }
                Image.Plane yp = img.getPlanes()[0];
                Slot slot = isLeft ? slotL : slotR;
                int stride = yp.getRowStride(), h = img.getHeight(), cap = stride * h;
                if (slot.y == null || slot.y.capacity() < cap) slot.y = ByteBuffer.allocateDirect(cap);
                ByteBuffer src = yp.getBuffer();
                slot.y.clear(); src.rewind(); slot.y.put(src); slot.y.rewind();
                slot.ts = img.getTimestamp(); slot.w = img.getWidth(); slot.h = h; slot.stride = stride; slot.fresh = true;

                if (!slotL.fresh || !slotR.fresh) return;
                long dNs = Math.abs(slotL.ts - slotR.ts);
                if (dNs > pairSyncNs) {                       // drop the older frame and wait for a better partner
                    if (slotL.ts < slotR.ts) slotL.fresh = false; else slotR.fresh = false;
                    return;
                }
                slotL.fresh = slotR.fresh = false;
                if (++pairFramesLogged % 30 == 1) log(String.format(Locale.US, "Pair #%d  inter-camera skew %d µs", pairFramesLogged, dNs / 1000));
                if (saveNextPair.compareAndSet(true, false)) savePairFiles(slotL, slotR);

                int frames = NativeEngine.processStereoPair(slotL.y, slotL.stride, slotR.y, slotR.stride, slotL.w, slotL.h, slotL.ts, heatmap);
                long now = SystemClock.elapsedRealtime();
                if (frames >= 0 && now - lastDepthEmitMs > 120) {
                    lastDepthEmitMs = now;
                    int rw = (int) stereoInfo[5], rh = (int) stereoInfo[6];
                    Bitmap bm = Bitmap.createBitmap(heatmap, rw, rh, Bitmap.Config.ARGB_8888);  // copies the pixels
                    listener.onDepthFrame(rotateForDisplay(bm, (int) stereoInfo[4] + sensorOrientation));
                }
            }
        } finally {
            img.close();
        }
    }

    private static Bitmap rotateForDisplay(Bitmap src, int degrees) {
        int d = ((degrees % 360) + 360) % 360;
        if (d == 0) return src;
        Matrix m = new Matrix(); m.postRotate(d);
        return Bitmap.createBitmap(src, 0, 0, src.getWidth(), src.getHeight(), m, false);
    }

    private void savePairFiles(Slot l, Slot r) {
        File dir = outputDir();
        if (dir == null) return;
        int idx = pairCounter++;
        writePgm(l, new File(dir, String.format(Locale.US, "left_%02d.pgm", idx)));
        writePgm(r, new File(dir, String.format(Locale.US, "right_%02d.pgm", idx)));
        log("Saved calibration pair #" + idx + " (" + l.w + "×" + l.h + ")");
        setState("Saved calibration pair #" + idx, Listener.LEVEL_OK);
    }

    private static void writePgm(Slot s, File f) {
        try (FileOutputStream fos = new FileOutputStream(f)) {
            fos.write(String.format(Locale.US, "P5\n%d %d\n255\n", s.w, s.h).getBytes(StandardCharsets.US_ASCII));
            byte[] row = new byte[s.w]; ByteBuffer b = s.y.duplicate();
            for (int y = 0; y < s.h; ++y) { b.position(y * s.stride); b.get(row, 0, s.w); fos.write(row); }
        } catch (Exception e) { Log.e(TAG, "PGM write failed", e); }
    }

    // ------------------------------------------------------------------------------------------

    /** Preview buffer size for the viewfinder (sensor aspect ratio, ~2 MP). */
    public static Size choosePreviewSize(Context context) { return CameraTopology.previewSize(context.getApplicationContext()); }
}
