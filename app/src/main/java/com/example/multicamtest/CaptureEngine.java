package com.example.multicamtest;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.*;
import android.hardware.camera2.params.BlackLevelPattern;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Size;
import android.util.SizeF;
import android.view.Surface;
import org.json.JSONArray;
import org.json.JSONObject;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Camera2 + Sensor engine managing live preview, gyro-aligned macro super-resolution bursts,
 * and dual physical camera synchronous stereo depth estimation.
 */
public final class CaptureEngine {
    public interface Listener {
        void onState(String message, int color);
        void onProgress(int value, int max);
        void onLog(String line);
        void onStability(float stability);
        void onImage(Bitmap bitmap);
        void onMacroResult(Bitmap before, Bitmap after, File file);
        void onScanResult(File stlFile, File plyFile, int triangleCount);
    }

    private static final String TAG = "MultiCamTest";
    private static final String DEFAULT_LOGICAL_ID = "20", FALLBACK_LOGICAL_ID = "23";
    private static final String PHYS_ID_WIDE = "0", PHYS_ID_UW = "2";
    public static final int MICROSCOPE_BURST_FRAMES = 20;

    private final Context ctx;
    private final Listener listener;

    private final HandlerThread cameraThread;
    private final Handler cameraHandler;
    private final Executor cameraExecutor;

    /** Sensor mounting angle of the wide camera; used only to rotate display bitmaps upright. */
    private int sensorOrientation = 90;

    private SensorManager sensorManager;
    private Sensor gyroSensor;
    private float currentStability = 1.0f;

    private Surface previewSurface;
    private boolean isPreviewActive = false;
    private boolean isBusyCapturing = false;

    private CameraDevice cameraDevice;
    private CameraCaptureSession activeSession;
    private ImageReader reader0, reader2;
    private int width0 = 1440, height0 = 1080, width2 = 1440, height2 = 1080;

    private final AtomicInteger frameCount0 = new AtomicInteger(0), frameCount2 = new AtomicInteger(0);
    private final AtomicBoolean saveNextCalibrationPair = new AtomicBoolean(false);
    private final AtomicInteger calibrationPairCounter = new AtomicInteger(0);

    private Bitmap baselineBeforeBitmap = null;

    private static final class FrameSlot {
        long timestampNs; int width, height, rowStride; ByteBuffer directY;
    }

    private final Object stereoSyncLock = new Object();
    private FrameSlot latestSlot0, latestSlot2;
    private int[] heatmapPixels;

    private final SensorEventListener gyroListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent event) {
            float gx = event.values[0], gy = event.values[1], gz = event.values[2];
            float omega = (float) Math.sqrt(gx * gx + gy * gy + gz * gz);
            float inst = Math.max(0.0f, Math.min(1.0f, 1.0f - (omega / 0.35f)));
            currentStability = 0.85f * currentStability + 0.15f * inst;
            listener.onStability(currentStability);
        }
        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {}
    };

    public CaptureEngine(Context context, Listener l) {
        ctx = context.getApplicationContext();
        listener = l;
        cameraThread = new HandlerThread("CameraBackgroundThread");
        cameraThread.start();
        cameraHandler = new Handler(cameraThread.getLooper());
        cameraExecutor = cameraHandler::post;

        sensorManager = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager != null) {
            gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
            if (gyroSensor != null) {
                sensorManager.registerListener(gyroListener, gyroSensor, SensorManager.SENSOR_DELAY_UI, cameraHandler);
            }
        }

        try {
            CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
            Integer so = cm.getCameraCharacteristics(PHYS_ID_WIDE).get(CameraCharacteristics.SENSOR_ORIENTATION);
            if (so != null) sensorOrientation = so;
        } catch (Exception e) { Log.w(TAG, "Sensor orientation lookup failed", e); }

        NativeEngine.startImuLooper();
    }

    /**
     * Largest 4:3 SurfaceTexture size of the wide camera not exceeding ~2 MP (falls back to 1440x1080).
     * The viewfinder is laid out with this aspect ratio so the live preview is never stretched.
     */
    public static Size choosePreviewSize(Context context) {
        Size best = new Size(1440, 1080);
        try {
            CameraManager cm = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
            StreamConfigurationMap map = cm.getCameraCharacteristics(PHYS_ID_WIDE).get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            Size[] sizes = map != null ? map.getOutputSizes(SurfaceTexture.class) : null;
            long bestArea = 0;
            if (sizes != null) {
                for (Size sz : sizes) {
                    long area = (long) sz.getWidth() * sz.getHeight();
                    boolean fourThree = Math.abs(sz.getWidth() * 3 - sz.getHeight() * 4) <= 8;
                    if (fourThree && area <= 2_100_000L && area > bestArea) { best = sz; bestArea = area; }
                }
            }
        } catch (Exception e) { Log.w(TAG, "Preview size negotiation fallback", e); }
        return best;
    }

    /** Rotate a sensor-oriented bitmap so it appears upright when the phone is held in portrait. */
    private Bitmap upright(Bitmap src) {
        if (src == null || sensorOrientation == 0) return src;
        Matrix m = new Matrix();
        m.postRotate(sensorOrientation);
        return Bitmap.createBitmap(src, 0, 0, src.getWidth(), src.getHeight(), m, true);
    }

    public File outputDir() { return ctx.getExternalFilesDir(null); }

    public void setPreviewSurface(Surface surface) {
        cameraExecutor.execute(() -> {
            this.previewSurface = surface;
            if (surface != null && surface.isValid() && !isBusyCapturing) {
                startLivePreview();
            } else if (surface == null && isPreviewActive) {
                stopLivePreview();
            }
        });
    }

    public void resumePreview() {
        cameraExecutor.execute(() -> {
            isBusyCapturing = false;
            if (previewSurface != null && previewSurface.isValid()) {
                startLivePreview();
            }
        });
    }

    public void startScan(boolean fullRes) {
        cameraExecutor.execute(() -> {
            isBusyCapturing = true;
            startDualCameraScannerMode(DEFAULT_LOGICAL_ID, fullRes);
        });
    }

    public void startMacro(boolean raw) {
        cameraExecutor.execute(() -> {
            isBusyCapturing = true;
            startMicroscopeMode(raw);
        });
    }

    public void stop() {
        cameraExecutor.execute(() -> {
            cleanupCamera();
            isBusyCapturing = false;
            if (previewSurface != null && previewSurface.isValid()) {
                startLivePreview();
            }
        });
    }

    public void savePair() {
        saveNextCalibrationPair.set(true);
        setState("Saving next synchronized pair...", Color.rgb(255, 213, 79));
    }

    public int exportMesh() {
        File dir = ctx.getExternalFilesDir(null);
        if (dir == null) return -1;
        File stl = new File(dir, "scan_mesh.stl"), ply = new File(dir, "scan_mesh.ply");
        int tris = NativeEngine.exportStereoMesh(stl.getAbsolutePath(), ply.getAbsolutePath());
        appendStatus("Mesh exported -> " + stl.getAbsolutePath() + " (" + tris + " triangles)");
        listener.onScanResult(stl, ply, tris);
        return tris;
    }

    public void release() {
        if (sensorManager != null) {
            sensorManager.unregisterListener(gyroListener);
        }
        cleanupCamera();
        NativeEngine.stopImuLooper();
        cameraThread.quitSafely();
    }

    private void setState(String s, int color) { listener.onState(s, color); }
    private void setProgress(int v, int max) { listener.onProgress(v, max); }
    private void appendStatus(String msg) { listener.onLog(msg); }

    private void startLivePreview() {
        if (previewSurface == null || !previewSurface.isValid()) return;
        cleanupCamera();
        CameraManager manager = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        try {
            manager.openCamera(PHYS_ID_WIDE, cameraExecutor, new CameraDevice.StateCallback() {
                @Override
                public void onOpened(CameraDevice camera) {
                    cameraDevice = camera;
                    try {
                        OutputConfiguration outConf = new OutputConfiguration(previewSurface);
                        SessionConfiguration sc = new SessionConfiguration(
                            SessionConfiguration.SESSION_REGULAR,
                            Arrays.asList(outConf),
                            cameraExecutor,
                            new CameraCaptureSession.StateCallback() {
                                @Override
                                public void onConfigured(CameraCaptureSession session) {
                                    activeSession = session;
                                    isPreviewActive = true;
                                    try {
                                        CaptureRequest.Builder b = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                                        b.addTarget(previewSurface);
                                        b.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
                                        b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                                        b.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
                                        session.setRepeatingRequest(b.build(), null, cameraHandler);
                                        setState("Viewfinder Ready - Align subject.", Color.rgb(0, 230, 118));
                                    } catch (CameraAccessException e) {
                                        Log.e(TAG, "Preview request failed", e);
                                    }
                                }

                                @Override
                                public void onConfigureFailed(CameraCaptureSession session) {
                                    Log.e(TAG, "Preview session configure failed");
                                    isPreviewActive = false;
                                }
                            }
                        );
                        camera.createCaptureSession(sc);
                    } catch (CameraAccessException e) {
                        Log.e(TAG, "Preview session creation failed", e);
                    }
                }

                @Override
                public void onDisconnected(CameraDevice camera) {
                    cleanupCamera();
                    isPreviewActive = false;
                }

                @Override
                public void onError(CameraDevice camera, int error) {
                    cleanupCamera();
                    isPreviewActive = false;
                }
            });
        } catch (Exception e) {
            Log.e(TAG, "Error starting live preview", e);
        }
    }

    private void stopLivePreview() {
        cleanupCamera();
        isPreviewActive = false;
    }

    private void configureStereoCalibration(CameraManager manager, int streamW, int streamH) {
        float fx0 = 0.762f * streamW, fy0 = 0.762f * streamW, cx0 = 0.5f * streamW, cy0 = 0.5f * streamH;
        float fx2 = 0.391f * streamW, fy2 = 0.391f * streamW, cx2 = 0.5f * streamW, cy2 = 0.5f * streamH;
        float[] d0 = new float[5], d2 = new float[5], R = new float[]{1f,0f,0f, 0f,1f,0f, 0f,0f,1f}, T_mm = new float[]{-18.5f, 0f, 0f};
        try {
            CameraCharacteristics c0 = manager.getCameraCharacteristics(PHYS_ID_WIDE);
            float[] f0 = c0.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
            SizeF s0 = c0.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
            if (f0 != null && f0.length > 0 && s0 != null && s0.getWidth() > 0) {
                fx0 = streamW * (f0[0] / s0.getWidth()); fy0 = streamH * (f0[0] / s0.getHeight());
            }
            CameraCharacteristics c2 = manager.getCameraCharacteristics(PHYS_ID_UW);
            float[] f2 = c2.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
            SizeF s2 = c2.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
            if (f2 != null && f2.length > 0 && s2 != null && s2.getWidth() > 0) {
                fx2 = streamW * (f2[0] / s2.getWidth()); fy2 = streamH * (f2[0] / s2.getHeight());
            }
        } catch (Exception e) { Log.w(TAG, "Physical focal lookup fallback", e); }

        File cf = new File(ctx.getExternalFilesDir(null), "stereo_calibration.json");
        if (cf.exists()) {
            try (FileInputStream fis = new FileInputStream(cf)) {
                byte[] data = new byte[(int) cf.length()]; int n = fis.read(data);
                if (n > 0) {
                    JSONObject obj = new JSONObject(new String(data, 0, n, StandardCharsets.UTF_8));
                    JSONArray k1 = obj.getJSONArray("K1"), k2 = obj.getJSONArray("K2");
                    fx0 = (float) k1.getJSONArray(0).getDouble(0); cx0 = (float) k1.getJSONArray(0).getDouble(2);
                    fy0 = (float) k1.getJSONArray(1).getDouble(1); cy0 = (float) k1.getJSONArray(1).getDouble(2);
                    fx2 = (float) k2.getJSONArray(0).getDouble(0); cx2 = (float) k2.getJSONArray(0).getDouble(2);
                    fy2 = (float) k2.getJSONArray(1).getDouble(1); cy2 = (float) k2.getJSONArray(1).getDouble(2);
                    JSONArray d1J = obj.getJSONArray("D1"), d2J = obj.getJSONArray("D2");
                    for (int i = 0; i < Math.min(5, d1J.length()); ++i)
                        d0[i] = (float) (d1J.optJSONArray(i) != null ? d1J.getJSONArray(i).getDouble(0) : d1J.getDouble(i));
                    for (int i = 0; i < Math.min(5, d2J.length()); ++i)
                        d2[i] = (float) (d2J.optJSONArray(i) != null ? d2J.getJSONArray(i).getDouble(0) : d2J.getDouble(i));
                    JSONArray rJ = obj.getJSONArray("R"), tJ = obj.getJSONArray("T");
                    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) R[r * 3 + c] = (float) rJ.getJSONArray(r).getDouble(c);
                    for (int i = 0; i < 3; ++i) {
                        double v = tJ.optJSONArray(i) != null ? tJ.getJSONArray(i).getDouble(0) : tJ.getDouble(i);
                        T_mm[i] = (float) (Math.abs(v) < 1.0 ? v * 1000.0 : v);
                    }
                    appendStatus("Loaded stereo_calibration.json");
                }
            } catch (Exception e) { Log.e(TAG, "Failed parsing stereo_calibration.json", e); }
        }
        NativeEngine.setDualCameraCalibration(fx0, fy0, cx0, cy0, d0, fx2, fy2, cx2, cy2, d2, R, T_mm);
    }

    private void startDualCameraScannerMode(String logicalId, boolean testFullRes) {
        cleanupCamera();
        frameCount0.set(0); frameCount2.set(0);
        NativeEngine.resetStereoScanner();
        CameraManager manager = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        try {
            CameraCharacteristics logicalChars = manager.getCameraCharacteristics(logicalId);
            Set<String> physicalIds = logicalChars.getPhysicalCameraIds();
            Log.i(TAG, "Logical ID: " + logicalId + " | Physical IDs: " + physicalIds);
            appendStatus("Binding Logical " + logicalId + " -> Physical: " + physicalIds);
            if (!physicalIds.contains(PHYS_ID_WIDE) || !physicalIds.contains(PHYS_ID_UW)) {
                setState("Dual camera combination (Wide + Ultra-Wide) unavailable on this device.", Color.rgb(255,138,128));
                isBusyCapturing = false;
                return;
            }

            if (testFullRes) {
                width0 = 4032; height0 = 3024; width2 = 4608; height2 = 3456;
            } else {
                StreamConfigurationMap map0, map2;
                try {
                    map0 = manager.getCameraCharacteristics(PHYS_ID_WIDE).get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                    map2 = manager.getCameraCharacteristics(PHYS_ID_UW).get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                } catch (IllegalArgumentException e) {
                    map0 = logicalChars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP); map2 = map0;
                }
                Size[] s0 = map0.getOutputSizes(ImageFormat.YUV_420_888), s2 = map2.getOutputSizes(ImageFormat.YUV_420_888);
                List<Size> matches = new ArrayList<>();
                if (s0 != null && s2 != null) {
                    Set<Size> set2 = new HashSet<>(Arrays.asList(s2));
                    for (Size sz : s0) if (sz.getWidth() <= 1920 && sz.getHeight() <= 1080 && set2.contains(sz)) matches.add(sz);
                }
                if (matches.isEmpty()) {
                    setState("No common stream size for dual cameras.", Color.rgb(255,138,128));
                    isBusyCapturing = false;
                    return;
                }
                matches.sort((a, b) -> {
                    boolean a43 = Math.abs(a.getWidth()*3 - a.getHeight()*4) <= 16, b43 = Math.abs(b.getWidth()*3 - b.getHeight()*4) <= 16;
                    if (a43 != b43 && a.getHeight() >= 1080 && b.getHeight() >= 1080) return a43 ? -1 : 1;
                    return Integer.compare(b.getWidth()*b.getHeight(), a.getWidth()*a.getHeight());
                });
                Size opt = matches.get(0);
                width0 = width2 = opt.getWidth(); height0 = height2 = opt.getHeight();
                appendStatus("Negotiated dual stream: " + width0 + "x" + height0);
            }

            configureStereoCalibration(manager, width0, height0);
            manager.openCamera(logicalId, cameraExecutor, new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice cam) {
                    cameraDevice = cam;
                    configureDualPhysicalStreams(logicalId, testFullRes);
                }
                @Override public void onDisconnected(CameraDevice cam) { cleanupCamera(); isBusyCapturing = false; }
                @Override public void onError(CameraDevice cam, int err) { cleanupCamera(); isBusyCapturing = false; }
            });
        } catch (Exception e) {
            Log.e(TAG, "Init error", e);
            isBusyCapturing = false;
        }
    }

    private void configureDualPhysicalStreams(String logicalId, boolean testFullRes) {
        try {
            reader0 = ImageReader.newInstance(width0, height0, ImageFormat.YUV_420_888, 4);
            reader2 = ImageReader.newInstance(width2, height2, ImageFormat.YUV_420_888, 4);
            reader0.setOnImageAvailableListener(r -> { Image i = r.acquireLatestImage(); if (i != null) { onPhysicalFrame(i, true); i.close(); } }, cameraHandler);
            reader2.setOnImageAvailableListener(r -> { Image i = r.acquireLatestImage(); if (i != null) { onPhysicalFrame(i, false); i.close(); } }, cameraHandler);

            OutputConfiguration c0 = new OutputConfiguration(reader0.getSurface()); c0.setPhysicalCameraId(PHYS_ID_WIDE);
            OutputConfiguration c2 = new OutputConfiguration(reader2.getSurface()); c2.setPhysicalCameraId(PHYS_ID_UW);

            SessionConfiguration sc = new SessionConfiguration(
                SessionConfiguration.SESSION_REGULAR, Arrays.asList(c0, c2), cameraExecutor,
                new CameraCaptureSession.StateCallback() {
                    @Override public void onConfigured(CameraCaptureSession session) {
                        activeSession = session;
                        setState("3D Scan Active (" + width0 + "x" + height0 + "). Move in an arc around subject.", Color.rgb(0,230,118));
                        try {
                            CaptureRequest.Builder b = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                            b.addTarget(reader0.getSurface()); b.addTarget(reader2.getSurface());
                            b.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
                            b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
                            b.set(CaptureRequest.LENS_FOCUS_DISTANCE, 4.0f);
                            b.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF);
                            b.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                            session.setRepeatingRequest(b.build(), null, cameraHandler);
                        } catch (CameraAccessException e) { Log.e(TAG, "Request error", e); }
                    }
                    @Override public void onConfigureFailed(CameraCaptureSession session) {
                        Log.e(TAG, "CameraCaptureSession onConfigureFailed (" + width0 + "x" + height0 + ")");
                        setState("Dual stream configuration failed on ID " + logicalId, Color.rgb(255,138,128));
                        if (DEFAULT_LOGICAL_ID.equals(logicalId)) startDualCameraScannerMode(FALLBACK_LOGICAL_ID, testFullRes);
                        else isBusyCapturing = false;
                    }
                }
            );
            cameraDevice.createCaptureSession(sc);
        } catch (CameraAccessException e) {
            Log.e(TAG, "Session error", e);
            isBusyCapturing = false;
        }
    }

    private void onPhysicalFrame(Image img, boolean isCam0) {
        long ts = img.getTimestamp();
        Image.Plane yP = img.getPlanes()[0];
        ByteBuffer src = yP.getBuffer();
        int stride = yP.getRowStride(), w = img.getWidth(), h = img.getHeight();
        synchronized (stereoSyncLock) {
            FrameSlot slot = isCam0 ? latestSlot0 : latestSlot2;
            int cap = stride * h;
            if (slot == null || slot.directY == null || slot.directY.capacity() < cap) {
                slot = new FrameSlot(); slot.directY = ByteBuffer.allocateDirect(cap);
                if (isCam0) latestSlot0 = slot; else latestSlot2 = slot;
            }
            slot.timestampNs = ts; slot.width = w; slot.height = h; slot.rowStride = stride;
            slot.directY.clear(); src.rewind(); slot.directY.put(src); slot.directY.rewind();
            if (isCam0) frameCount0.incrementAndGet(); else frameCount2.incrementAndGet();

            if (latestSlot0 != null && latestSlot2 != null) {
                long dNs = Math.abs(latestSlot0.timestampNs - latestSlot2.timestampNs);
                if (dNs < 10_000_000L) {
                    long dUs = dNs / 1000L;
                    int pc = frameCount0.get();
                    if (pc % 15 == 0) {
                        appendStatus("Synced Pair #" + pc + " | Jitter: " + dUs + " \u00b5s");
                    }
                    if (saveNextCalibrationPair.compareAndSet(true, false)) {
                        int idx = calibrationPairCounter.getAndIncrement();
                        File dir = ctx.getExternalFilesDir(null);
                        if (dir != null) {
                            writePgm(latestSlot0, new File(dir, String.format(Locale.US, "left_%02d.pgm", idx)));
                            writePgm(latestSlot2, new File(dir, String.format(Locale.US, "right_%02d.pgm", idx)));
                            appendStatus("Saved calibration pair #" + idx);
                            setState("Saved pair #" + idx + " to storage.", Color.rgb(0, 230, 118));
                        }
                    }
                    if (latestSlot0.width == latestSlot2.width && latestSlot0.height == latestSlot2.height) {
                        int dw = latestSlot0.width / 4, dh = latestSlot0.height / 4;
                        if (heatmapPixels == null || heatmapPixels.length != dw * dh) heatmapPixels = new int[dw * dh];
                        NativeEngine.processStereoPair(latestSlot0.directY, latestSlot0.rowStride, latestSlot2.directY, latestSlot2.rowStride, latestSlot0.width, latestSlot0.height, latestSlot0.timestampNs, heatmapPixels);
                        if (pc % 2 == 0) updateHeatmapUi(dw, dh);
                    }
                    latestSlot0.timestampNs = -100_000_000L;
                }
            }
        }
    }

    private void updateHeatmapUi(int dw, int dh) {
        int[] copy = Arrays.copyOf(heatmapPixels, heatmapPixels.length);
        Bitmap bm = Bitmap.createBitmap(dw, dh, Bitmap.Config.ARGB_8888);
        bm.setPixels(copy, 0, dw, 0, 0, dw, dh);
        listener.onImage(upright(bm));
    }

    private static void writePgm(FrameSlot s, File f) {
        try (FileOutputStream fos = new FileOutputStream(f)) {
            fos.write(String.format(Locale.US, "P5\n%d %d\n255\n", s.width, s.height).getBytes(StandardCharsets.US_ASCII));
            byte[] row = new byte[s.width]; ByteBuffer b = s.directY.duplicate();
            for (int y = 0; y < s.height; ++y) { b.position(y * s.rowStride); b.get(row, 0, s.width); fos.write(row); }
        } catch (Exception e) { Log.e(TAG, "PGM write failed", e); }
    }

    private Bitmap extractDownscaledBaseline(Image img) {
        try {
            int w = img.getWidth(), h = img.getHeight();
            int scale = 8;
            int bw = w / scale, bh = h / scale;
            int[] pixels = new int[bw * bh];
            if (img.getFormat() == ImageFormat.RAW_SENSOR) {
                Image.Plane p0 = img.getPlanes()[0];
                ByteBuffer b = p0.getBuffer().duplicate();
                b.rewind();
                int st = p0.getRowStride();
                byte[] row = new byte[st];
                for (int y = 0; y < bh; ++y) {
                    b.position(y * scale * st);
                    b.get(row, 0, Math.min(row.length, b.remaining()));
                    for (int x = 0; x < bw; ++x) {
                        int idx = x * scale * 2;
                        int val = (row[idx] & 0xFF) | ((row[idx + 1] & 0xFF) << 8);
                        int lum = Math.min(255, val >> 2);
                        pixels[y * bw + x] = 0xFF000000 | (lum << 16) | (lum << 8) | lum;
                    }
                }
            } else {
                Image.Plane pY = img.getPlanes()[0];
                ByteBuffer yBuf = pY.getBuffer().duplicate();
                yBuf.rewind();
                int rowStride = pY.getRowStride();
                byte[] row = new byte[Math.min(rowStride, w)];
                for (int y = 0; y < bh; ++y) {
                    yBuf.position(y * scale * rowStride);
                    yBuf.get(row, 0, Math.min(row.length, yBuf.remaining()));
                    for (int x = 0; x < bw; ++x) {
                        int lum = row[x * scale] & 0xFF;
                        pixels[y * bw + x] = 0xFF000000 | (lum << 16) | (lum << 8) | lum;
                    }
                }
            }
            return Bitmap.createBitmap(pixels, bw, bh, Bitmap.Config.ARGB_8888);
        } catch (Exception e) {
            Log.e(TAG, "Error extracting baseline preview", e);
            return null;
        }
    }

    void startMicroscopeMode(boolean useRaw16) {
        cleanupCamera();
        final int macroW = 4032, macroH = 3024;
        NativeEngine.resetMicroscope(macroW, macroH);
        baselineBeforeBitmap = null;
        CameraManager manager = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        try {
            CameraCharacteristics c0 = manager.getCameraCharacteristics(PHYS_ID_WIDE);
            float[] apertures = c0.get(CameraCharacteristics.LENS_INFO_AVAILABLE_APERTURES);
            float[] focals = c0.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
            SizeF physSz = c0.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
            Integer cfa = c0.get(CameraCharacteristics.SENSOR_INFO_COLOR_FILTER_ARRANGEMENT);
            Integer wl = c0.get(CameraCharacteristics.SENSOR_INFO_WHITE_LEVEL);
            BlackLevelPattern blp = c0.get(CameraCharacteristics.SENSOR_BLACK_LEVEL_PATTERN);

            float fx = (focals != null && focals.length > 0 && physSz != null && physSz.getWidth() > 0) ? (macroW * focals[0] / physSz.getWidth()) : (0.762f * macroW);
            float fy = (focals != null && focals.length > 0 && physSz != null && physSz.getHeight() > 0) ? (macroH * focals[0] / physSz.getHeight()) : (0.762f * macroW);
            NativeEngine.configureMicroscopeOptics(fx, fy, cfa != null ? cfa : 0, blp != null ? blp.getOffsetForIndex(0, 0) : 64, wl != null ? wl : 1023);

            final int fmt = useRaw16 ? ImageFormat.RAW_SENSOR : ImageFormat.YUV_420_888;
            setState("Starting macro burst...", Color.rgb(255,213,79));
            appendStatus("Capturing Macro Super-Res (" + (useRaw16 ? "RAW16 Bayer" : "Color YUV") + " @ 4032x3024)...");

            manager.openCamera(PHYS_ID_WIDE, cameraExecutor, new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice cam) {
                    cameraDevice = cam;
                    try {
                        reader0 = ImageReader.newInstance(macroW, macroH, fmt, 4);
                        reader0.setOnImageAvailableListener(r -> {
                            Image img = r.acquireLatestImage(); if (img == null) return;
                            long ts = img.getTimestamp(); int w = img.getWidth(), h = img.getHeight(), acc;

                            if (baselineBeforeBitmap == null) {
                                baselineBeforeBitmap = extractDownscaledBaseline(img);
                            }

                            if (img.getFormat() == ImageFormat.RAW_SENSOR) {
                                Image.Plane p0 = img.getPlanes()[0]; int st = p0.getRowStride();
                                ByteBuffer copy = cloneDirect(p0.getBuffer()); img.close();
                                acc = NativeEngine.pushMicroscopeRaw16Frame(copy, st, w, h, ts);
                            } else {
                                Image.Plane pY = img.getPlanes()[0], pU = img.getPlanes()[1], pV = img.getPlanes()[2];
                                ByteBuffer cY = cloneDirect(pY.getBuffer()), cU = cloneDirect(pU.getBuffer()), cV = cloneDirect(pV.getBuffer());
                                int yS = pY.getRowStride(), uS = pU.getRowStride(), uP = pU.getPixelStride(), vS = pV.getRowStride(), vP = pV.getPixelStride();
                                img.close();
                                acc = NativeEngine.pushMicroscopeYuvFrame(cY, yS, cU, uS, uP, cV, vS, vP, w, h, ts);
                            }

                            setProgress(acc, MICROSCOPE_BURST_FRAMES);
                            setState("Aligning frame " + acc + " of " + MICROSCOPE_BURST_FRAMES + " \u2014 Hold Steady", Color.rgb(255,213,79));
                            if (acc >= MICROSCOPE_BURST_FRAMES) {
                                cleanupCamera();
                                setState("Reconstructing 2\u00d7 Drizzle Grid...", Color.rgb(255,213,79));
                                File dir = ctx.getExternalFilesDir(null);
                                File bmp = new File(dir, "microscope_2x_superres.bmp");
                                File ppm = new File(dir, "microscope_2x_superres.ppm");
                                File pgm = new File(dir, "microscope_2x_superres.pgm");
                                boolean ok = NativeEngine.exportMicroscopeImages(bmp.getAbsolutePath(), ppm.getAbsolutePath(), pgm.getAbsolutePath());
                                appendStatus(ok ? "Super-resolution saved: " + bmp.getName() : "Reconstruction failed");
                                setProgress(0, 0);
                                isBusyCapturing = false;

                                if (ok && bmp.exists()) {
                                    BitmapFactory.Options o = new BitmapFactory.Options();
                                    o.inSampleSize = 8;
                                    Bitmap afterBm = BitmapFactory.decodeFile(bmp.getAbsolutePath(), o);
                                    setState("Finished! 2\u00d7 Super-Resolution Ready.", Color.rgb(0,230,118));
                                    listener.onMacroResult(upright(baselineBeforeBitmap), upright(afterBm), bmp);
                                } else {
                                    setState("Reconstruction failed.", Color.rgb(255,138,128));
                                }
                            }
                        }, cameraHandler);

                        OutputConfiguration oc = new OutputConfiguration(reader0.getSurface());
                        SessionConfiguration sc = new SessionConfiguration(
                            SessionConfiguration.SESSION_REGULAR, Arrays.asList(oc), cameraExecutor,
                            new CameraCaptureSession.StateCallback() {
                                @Override public void onConfigured(CameraCaptureSession session) {
                                    activeSession = session;
                                    try {
                                        CaptureRequest.Builder b = cam.createCaptureRequest(CameraDevice.TEMPLATE_MANUAL);
                                        b.addTarget(reader0.getSurface());
                                        b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
                                        b.set(CaptureRequest.LENS_FOCUS_DISTANCE, 10.0f);
                                        b.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
                                        b.set(CaptureRequest.SENSOR_EXPOSURE_TIME, 3_500_000L);
                                        b.set(CaptureRequest.SENSOR_SENSITIVITY, 100);
                                        b.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF);
                                        b.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                                        b.set(CaptureRequest.NOISE_REDUCTION_MODE, CameraMetadata.NOISE_REDUCTION_MODE_OFF);
                                        b.set(CaptureRequest.EDGE_MODE, CameraMetadata.EDGE_MODE_OFF);
                                        if (apertures != null) for (float ap : apertures) if (Math.abs(ap - 2.4f) < 0.1f) b.set(CaptureRequest.LENS_APERTURE, 2.4f);
                                        session.setRepeatingRequest(b.build(), null, cameraHandler);
                                    } catch (CameraAccessException e) { Log.e(TAG, "Microscope req error", e); }
                                }
                                @Override public void onConfigureFailed(CameraCaptureSession s) {
                                    Log.e(TAG, "Microscope config failed");
                                    setState("Sensor refused burst format.", Color.rgb(255,138,128));
                                    isBusyCapturing = false;
                                }
                            }
                        );
                        cam.createCaptureSession(sc);
                    } catch (CameraAccessException e) {
                        Log.e(TAG, "Microscope setup error", e);
                        isBusyCapturing = false;
                    }
                }
                @Override public void onDisconnected(CameraDevice c) { cleanupCamera(); isBusyCapturing = false; }
                @Override public void onError(CameraDevice c, int e) { cleanupCamera(); isBusyCapturing = false; }
            });
        } catch (Exception e) {
            Log.e(TAG, "Microscope open error", e);
            isBusyCapturing = false;
        }
    }

    private static ByteBuffer cloneDirect(ByteBuffer src) {
        src.rewind(); ByteBuffer copy = ByteBuffer.allocateDirect(src.remaining());
        copy.put(src); copy.rewind(); return copy;
    }

    private void cleanupCamera() {
        if (activeSession != null) { try { activeSession.close(); } catch (Exception ignored) {} activeSession = null; }
        if (cameraDevice != null) { try { cameraDevice.close(); } catch (Exception ignored) {} cameraDevice = null; }
        if (reader0 != null) { try { reader0.close(); } catch (Exception ignored) {} reader0 = null; }
        if (reader2 != null) { try { reader2.close(); } catch (Exception ignored) {} reader2 = null; }
        isPreviewActive = false;
    }
}
