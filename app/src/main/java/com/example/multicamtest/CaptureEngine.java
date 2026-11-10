package com.example.multicamtest;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.hardware.camera2.*;
import android.hardware.camera2.params.BlackLevelPattern;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Size;
import android.util.SizeF;
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


public final class CaptureEngine {
    public interface Listener {
        void onState(String message, int color);
        void onProgress(int value, int max);
        void onLog(String line);
        void onImage(Bitmap bitmap);
    }

    private static final String TAG = "MultiCamTest";
    private static final String DEFAULT_LOGICAL_ID = "20", FALLBACK_LOGICAL_ID = "23";
    private static final String PHYS_ID_WIDE = "0", PHYS_ID_UW = "2";
    public static final int MICROSCOPE_BURST_FRAMES = 20;

    private final Context ctx;
    private final Listener listener;

    public CaptureEngine(Context context, Listener l) {
        ctx = context.getApplicationContext();
        listener = l;
        cameraThread = new HandlerThread("CameraBackgroundThread");
        cameraThread.start();
        cameraHandler = new Handler(cameraThread.getLooper());
        cameraExecutor = cameraHandler::post;
        NativeEngine.startImuLooper();
    }

    public File outputDir() { return ctx.getExternalFilesDir(null); }

    public void startScan(boolean fullRes) { cameraExecutor.execute(() -> startDualCameraScannerMode(DEFAULT_LOGICAL_ID, fullRes)); }
    public void startMacro(boolean raw) { cameraExecutor.execute(() -> startMicroscopeMode(raw)); }
    public void stop() { cameraExecutor.execute(this::cleanupCamera); }
    public void savePair() { saveNextCalibrationPair.set(true); setState("Saving next synchronized pair...", Color.rgb(255, 213, 79)); }
    public int exportMesh() {
        File dir = ctx.getExternalFilesDir(null); if (dir == null) return -1;
        File stl = new File(dir, "scan_mesh.stl"), ply = new File(dir, "scan_mesh.ply");
        int tris = NativeEngine.exportStereoMesh(stl.getAbsolutePath(), ply.getAbsolutePath());
        appendStatus("Mesh -> " + stl.getAbsolutePath());
        return tris;
    }
    public void release() {
        cleanupCamera();
        NativeEngine.stopImuLooper();
        cameraThread.quitSafely();
    }

    private void setState(String s, int color) { listener.onState(s, color); }
    private void setProgress(int v, int max) { listener.onProgress(v, max); }
    private void appendStatus(String msg) { listener.onLog(msg); }
    private void showImageFile(File f) {
        try {
            BitmapFactory.Options o = new BitmapFactory.Options(); o.inSampleSize = 8;
            Bitmap bm = BitmapFactory.decodeFile(f.getAbsolutePath(), o);
            if (bm != null) listener.onImage(bm);
        } catch (Exception e) { Log.e(TAG, "preview load failed", e); }
    }

    private HandlerThread cameraThread;
    private Handler cameraHandler;
    private Executor cameraExecutor;
    private CameraDevice cameraDevice;
    private CameraCaptureSession activeSession;
    private ImageReader reader0, reader2;
    private int width0 = 1440, height0 = 1080, width2 = 1440, height2 = 1080;

    private final AtomicInteger frameCount0 = new AtomicInteger(0), frameCount2 = new AtomicInteger(0);
    private final AtomicBoolean saveNextCalibrationPair = new AtomicBoolean(false);
    private final AtomicInteger calibrationPairCounter = new AtomicInteger(0);

    private static final class FrameSlot {
        long timestampNs; int width, height, rowStride; ByteBuffer directY;
    }

    private final Object stereoSyncLock = new Object();
    private FrameSlot latestSlot0, latestSlot2;
    private int[] heatmapPixels;

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
            int[] caps = logicalChars.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES);
            boolean isMultiCam = false;
            if (caps != null) for (int c : caps) if (c == CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA) isMultiCam = true;
            Set<String> physicalIds = logicalChars.getPhysicalCameraIds();
            Log.i(TAG, "Logical ID: " + logicalId + " | isLogicalMultiCamera: " + isMultiCam);
            Log.i(TAG, "Advertised Physical IDs: " + physicalIds);
            appendStatus("Logical ID " + logicalId + " | Physical IDs: " + physicalIds);
            if (!physicalIds.contains(PHYS_ID_WIDE) || !physicalIds.contains(PHYS_ID_UW)) { setState("This phone does not expose both cameras together (physical IDs: " + physicalIds + ").", Color.rgb(255,138,128)); return; }

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
                if (matches.isEmpty()) { setState("No common stream size for both cameras.", Color.rgb(255,138,128)); return; }
                matches.sort((a, b) -> {
                    boolean a43 = Math.abs(a.getWidth()*3 - a.getHeight()*4) <= 16, b43 = Math.abs(b.getWidth()*3 - b.getHeight()*4) <= 16;
                    if (a43 != b43 && a.getHeight() >= 1080 && b.getHeight() >= 1080) return a43 ? -1 : 1;
                    return Integer.compare(b.getWidth()*b.getHeight(), a.getWidth()*a.getHeight());
                });
                Size opt = matches.get(0);
                width0 = width2 = opt.getWidth(); height0 = height2 = opt.getHeight();
                Log.i(TAG, String.format(Locale.US, "Optimal symmetric physical stream negotiated: %dx%d (Area: %d px)", width0, height0, width0*height0));
                appendStatus("Negotiated stream: " + width0 + "x" + height0);
            }

            configureStereoCalibration(manager, width0, height0);
            manager.openCamera(logicalId, cameraExecutor, new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice cam) {
                    cameraDevice = cam;
                    Log.i(TAG, "SUCCESS: Camera " + logicalId + " opened by client.");
                    configureDualPhysicalStreams(logicalId, testFullRes);
                }
                @Override public void onDisconnected(CameraDevice cam) { cleanupCamera(); }
                @Override public void onError(CameraDevice cam, int err) { cleanupCamera(); }
            });
        } catch (Exception e) { Log.e(TAG, "Init error", e); }
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
                        Log.i(TAG, "SESSION ACTIVE: Physical stream pipeline successfully bound!");
                        Log.i(TAG, "CAM0 target: " + width0 + "x" + height0 + " | CAM2 target: " + width2 + "x" + height2);
                        appendStatus("SESSION ACTIVE: " + width0 + "x" + height0 + " + " + width2 + "x" + height2); setState("3D scan running (" + width0 + "x" + height0 + "). Move slowly; depth map above.", Color.rgb(0,230,118));
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
                        Log.e(TAG, "CRITICAL: CameraCaptureSession.onConfigureFailed! (" + width0 + "x" + height0 + " + " + width2 + "x" + height2 + ")");
                        appendStatus("FAILED on Logical ID " + logicalId); setState("Camera session refused for ID " + logicalId + (DEFAULT_LOGICAL_ID.equals(logicalId) ? " - trying fallback..." : " - dual streaming not supported here."), Color.rgb(255,138,128));
                        if (DEFAULT_LOGICAL_ID.equals(logicalId)) startDualCameraScannerMode(FALLBACK_LOGICAL_ID, testFullRes);
                    }
                }
            );
            boolean adv = cameraDevice.isSessionConfigurationSupported(sc);
            Log.i(TAG, "CameraDevice.isSessionConfigurationSupported() advisory returned: " + adv);
            cameraDevice.createCaptureSession(sc);
        } catch (CameraAccessException e) { Log.e(TAG, "Session error", e); }
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
                        Log.i(TAG, String.format(Locale.US, "CAM0 [Wide] Frame #%d (%dx%d) | Inter-sensor Sync Jitter: %d us", frameCount0.get(), latestSlot0.width, latestSlot0.height, dUs));
                        Log.i(TAG, String.format(Locale.US, "CAM2 [UW]   Frame #%d (%dx%d) | Inter-sensor Sync Jitter: %d us", frameCount2.get(), latestSlot2.width, latestSlot2.height, dUs));
                        appendStatus("Synced Pair #" + pc + " | Jitter: " + dUs + " us");
                    }
                    if (saveNextCalibrationPair.compareAndSet(true, false)) {
                        int idx = calibrationPairCounter.getAndIncrement();
                        File dir = ctx.getExternalFilesDir(null);
                        if (dir != null) {
                            writePgm(latestSlot0, new File(dir, String.format(Locale.US, "left_%02d.pgm", idx)));
                            writePgm(latestSlot2, new File(dir, String.format(Locale.US, "right_%02d.pgm", idx)));
                            appendStatus("Saved calibration pair #" + idx);
                        }
                    }
                    if (latestSlot0.width == latestSlot2.width && latestSlot0.height == latestSlot2.height) {
                        int dw = latestSlot0.width / 4, dh = latestSlot0.height / 4;
                        if (heatmapPixels == null || heatmapPixels.length != dw * dh) heatmapPixels = new int[dw * dh];
                        NativeEngine.processStereoPair(latestSlot0.directY, latestSlot0.rowStride, latestSlot2.directY, latestSlot2.rowStride, latestSlot0.width, latestSlot0.height, latestSlot0.timestampNs, heatmapPixels);
                        if (pc % 3 == 0) updateHeatmapUi(dw, dh);
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
        listener.onImage(bm);
    }

    private static void writePgm(FrameSlot s, File f) {
        try (FileOutputStream fos = new FileOutputStream(f)) {
            fos.write(String.format(Locale.US, "P5\n%d %d\n255\n", s.width, s.height).getBytes(StandardCharsets.US_ASCII));
            byte[] row = new byte[s.width]; ByteBuffer b = s.directY.duplicate();
            for (int y = 0; y < s.height; ++y) { b.position(y * s.rowStride); b.get(row, 0, s.width); fos.write(row); }
        } catch (Exception e) { Log.e(TAG, "PGM write failed", e); }
    }

    void startMicroscopeMode(boolean useRaw16) {
        cleanupCamera();
        final int macroW = 4032, macroH = 3024;
        NativeEngine.resetMicroscope(macroW, macroH);
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
            setState("Starting macro capture...", Color.rgb(255,213,79)); appendStatus("Starting Macro Microscope (" + (useRaw16 ? "RAW16 Bayer" : "Color YUV") + " @ 4032x3024, f/2.4, 10 dpt, OIS OFF)...");

            manager.openCamera(PHYS_ID_WIDE, cameraExecutor, new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice cam) {
                    cameraDevice = cam;
                    try {
                        reader0 = ImageReader.newInstance(macroW, macroH, fmt, 4);
                        reader0.setOnImageAvailableListener(r -> {
                            Image img = r.acquireLatestImage(); if (img == null) return;
                            long ts = img.getTimestamp(); int w = img.getWidth(), h = img.getHeight(), acc;
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
                            Log.i(TAG, "Microscope Drizzle frame " + acc + "/" + MICROSCOPE_BURST_FRAMES);
                            appendStatus("Microscope SR frame " + acc + "/" + MICROSCOPE_BURST_FRAMES); setProgress(acc, MICROSCOPE_BURST_FRAMES); setState("Capturing frame " + acc + " of " + MICROSCOPE_BURST_FRAMES + " - hold steady.", Color.rgb(255,213,79));
                            if (acc >= MICROSCOPE_BURST_FRAMES) {
                                cleanupCamera();
                                File dir = ctx.getExternalFilesDir(null);
                                File bmp = new File(dir, "microscope_2x_superres.bmp");
                                File ppm = new File(dir, "microscope_2x_superres.ppm");
                                File pgm = new File(dir, "microscope_2x_superres.pgm");
                                boolean ok = NativeEngine.exportMicroscopeImages(bmp.getAbsolutePath(), ppm.getAbsolutePath(), pgm.getAbsolutePath());
                                appendStatus(ok ? "Saved 8064x6048 BMP/PPM/PGM: " + bmp.getAbsolutePath() : "Export failed"); setProgress(0, 0); setState(ok ? "Done. Saved to " + bmp.getAbsolutePath() : "Export failed.", ok ? Color.rgb(0,230,118) : Color.rgb(255,138,128)); if (ok) showImageFile(bmp);
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
                                @Override public void onConfigureFailed(CameraCaptureSession s) { Log.e(TAG, "Microscope config failed"); setState("Camera refused this format - try the other macro button.", Color.rgb(255,138,128)); }
                            }
                        );
                        cam.createCaptureSession(sc);
                    } catch (CameraAccessException e) { Log.e(TAG, "Microscope setup error", e); }
                }
                @Override public void onDisconnected(CameraDevice c) { cleanupCamera(); }
                @Override public void onError(CameraDevice c, int e) { cleanupCamera(); }
            });
        } catch (Exception e) { Log.e(TAG, "Microscope open error", e); }
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
    }
}

