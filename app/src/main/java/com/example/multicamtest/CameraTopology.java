package com.example.multicamtest;

import android.content.Context;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.util.Log;
import android.util.Size;
import android.util.SizeF;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Vendor-neutral discovery of the cameras this app needs, using only documented Camera2 metadata:
 * no hard-coded camera IDs, resolutions or focal lengths.
 *
 * <ul>
 *   <li><b>macro camera</b>: the back-facing camera (or physical sub-camera of a logical one) that can resolve the
 *       smallest area on a close subject, ranked by thin-lens magnification at its minimum focus distance;</li>
 *   <li><b>stereo pair</b>: two physical sub-cameras of one logical multi-camera that share a YUV stream size.</li>
 * </ul>
 */
final class CameraTopology {
    private static final String TAG = "MultiCamTopology";

    /** One usable camera: how to open it and how to route its stream. */
    static final class Cam {
        final String openId;        // id given to CameraManager.openCamera()
        final String physicalId;    // non-null: stream must be routed to this physical sub-camera of {@code openId}
        final String directId;      // non-null: {@code physicalId} is itself listed by the framework and can be opened directly
        final CameraCharacteristics chars;
        final float focalMm, minFocusDiopters;
        final SizeF sensorMm;
        final int sensorOrientation;

        Cam(String openId, String physicalId, String directId, CameraCharacteristics c) {
            this.openId = openId; this.physicalId = physicalId; this.directId = directId; this.chars = c;
            float[] f = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
            focalMm = (f != null && f.length > 0) ? f[0] : 0f;
            Float md = c.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE);
            minFocusDiopters = md != null ? md : 0f;
            sensorMm = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
            Integer so = c.get(CameraCharacteristics.SENSOR_ORIENTATION);
            sensorOrientation = so != null ? so : 90;
        }

        String label() {
            return (physicalId != null ? openId + "/" + physicalId : openId)
                + String.format(java.util.Locale.US, " f=%.1fmm", focalMm)
                + (minFocusDiopters > 0 ? String.format(java.util.Locale.US, " near=%.0fcm", 100f / minFocusDiopters) : " fixed-focus");
        }

        boolean hasCapability(int cap) {
            int[] caps = chars.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES);
            if (caps != null) for (int c : caps) if (c == cap) return true;
            return false;
        }

        boolean supportsRaw16() {
            Integer cfa = chars.get(CameraCharacteristics.SENSOR_INFO_COLOR_FILTER_ARRANGEMENT);
            if (cfa == null || cfa > 3) return false;  // only Bayer mosaics are handled
            StreamConfigurationMap m = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            return hasCapability(CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_RAW) && m != null && m.getOutputSizes(ImageFormat.RAW_SENSOR) != null;
        }

        /** Focal length as a fraction of the sensor width — larger means a narrower field of view. */
        float focalRatio() { return (sensorMm != null && sensorMm.getWidth() > 0) ? focalMm / sensorMm.getWidth() : focalMm; }

        /** Thin-lens magnification at the minimum focus distance (0 for fixed focus). */
        float magnification() {
            if (minFocusDiopters <= 0 || focalMm <= 0) return 0f;
            float d = 1000f / minFocusDiopters;  // mm
            return d > focalMm ? focalMm / (d - focalMm) : 0f;
        }

        Size[] sizes(int format) {
            StreamConfigurationMap m = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            Size[] s = m != null ? m.getOutputSizes(format) : null;
            return s != null ? s : new Size[0];
        }
    }

    static final class StereoPair {
        final String logicalId;
        final Cam left, right;       // left = narrower field of view (its intrinsics define the rectified view)
        final float[] poseDeltaMm;   // right centre minus left centre, Android sensor frame; null when the device reports no pose
        final boolean calibratedSync;
        StereoPair(String logicalId, Cam left, Cam right, float[] poseDeltaMm, boolean calibratedSync) {
            this.logicalId = logicalId; this.left = left; this.right = right; this.poseDeltaMm = poseDeltaMm; this.calibratedSync = calibratedSync;
        }
    }

    final Cam macro;            // may be null on a device without a back camera
    final StereoPair stereo;    // null when no logical multi-camera with two usable physical cameras exists
    final List<String> notes = new ArrayList<>();

    private CameraTopology(Cam macro, StereoPair stereo) { this.macro = macro; this.stereo = stereo; }

    private static CameraTopology cached;

    static synchronized CameraTopology get(Context ctx) {
        if (cached == null) cached = discover(ctx);
        return cached;
    }

    private static CameraTopology discover(Context ctx) {
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        List<Cam> candidates = new ArrayList<>();
        StereoPair bestPair = null; float bestScore = -1f;
        List<String> notes = new ArrayList<>();
        try {
            Set<String> listed = new HashSet<>(java.util.Arrays.asList(cm.getCameraIdList()));
            for (String id : listed) {
                CameraCharacteristics c = cm.getCameraCharacteristics(id);
                Integer facing = c.get(CameraCharacteristics.LENS_FACING);
                if (facing == null || facing != CameraCharacteristics.LENS_FACING_BACK) continue;
                Set<String> phys = c.getPhysicalCameraIds();
                List<Cam> subs = new ArrayList<>();
                for (String pid : phys) {
                    try {
                        subs.add(new Cam(id, pid, listed.contains(pid) ? pid : null, cm.getCameraCharacteristics(pid)));
                    } catch (Exception e) { notes.add("physical " + pid + " not queryable: " + e.getMessage()); }
                }
                if (subs.isEmpty()) { candidates.add(new Cam(id, null, null, c)); continue; }
                candidates.addAll(subs);
                // stereo candidates among the sub-cameras of this logical camera
                Integer sync = c.get(CameraCharacteristics.LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE);
                boolean calibrated = sync != null && sync == CameraMetadata.LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE_CALIBRATED;
                for (int i = 0; i < subs.size(); ++i)
                    for (int j = i + 1; j < subs.size(); ++j) {
                        Cam a = subs.get(i), b = subs.get(j);
                        if (commonStereoSize(a, b, false) == null) continue;
                        Cam left = a.focalRatio() >= b.focalRatio() ? a : b, right = left == a ? b : a;
                        float[] delta = poseDelta(left, right);
                        float baseline = delta != null ? norm(delta) : 15f;
                        float ratio = Math.max(left.focalRatio(), 1e-3f) / Math.max(right.focalRatio(), 1e-3f);
                        float score = baseline / (1f + 2f * Math.abs((float) Math.log(ratio)));
                        if (baseline < 3f) continue;  // two lenses practically on top of each other: no usable parallax
                        if (score > bestScore) { bestScore = score; bestPair = new StereoPair(id, left, right, delta, calibrated); }
                    }
            }
        } catch (Exception e) {
            Log.e(TAG, "camera discovery failed", e);
            notes.add("discovery error: " + e.getMessage());
        }
        Cam macro = null; float bestMag = -1f;
        for (Cam c : candidates) {
            float m = c.magnification();
            if (m > bestMag) { bestMag = m; macro = c; }
        }
        if (macro == null && !candidates.isEmpty()) macro = candidates.get(0);
        CameraTopology t = new CameraTopology(macro, bestPair);
        t.notes.addAll(notes);
        if (macro != null) t.notes.add("Macro camera: " + macro.label() + String.format(java.util.Locale.US, "  (magnification %.3f at closest focus)", macro.magnification()));
        else t.notes.add("No back-facing camera found");
        if (bestPair != null) t.notes.add("Stereo pair: left " + bestPair.left.label() + "  right " + bestPair.right.label() + " via logical " + bestPair.logicalId
            + (bestPair.poseDeltaMm != null ? String.format(java.util.Locale.US, "  baseline %.1f mm (device pose)", norm(bestPair.poseDeltaMm)) : "  baseline unknown"));
        else t.notes.add("Stereo: no logical multi-camera with two usable back cameras");
        return t;
    }

    private static float norm(float[] v) { return (float) Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

    /** Right camera centre minus left camera centre in mm (Android sensor frame), or null. */
    static float[] poseDelta(Cam left, Cam right) {
        float[] tl = left.chars.get(CameraCharacteristics.LENS_POSE_TRANSLATION);
        float[] tr = right.chars.get(CameraCharacteristics.LENS_POSE_TRANSLATION);
        if (tl == null || tr == null || tl.length < 3 || tr.length < 3) return null;
        float[] d = {(tr[0] - tl[0]) * 1000f, (tr[1] - tl[1]) * 1000f, (tr[2] - tl[2]) * 1000f};
        float n = norm(d);
        return (n >= 3f && n <= 150f) ? d : null;  // outside this range the metadata is implausible for a phone
    }

    /**
     * Stereo stream size supported by both cameras. Standard: closest to ~1.5 MP with the sensor's aspect ratio
     * (the guaranteed multi-stream budget); full: the largest common size.
     */
    static Size commonStereoSize(Cam a, Cam b, boolean largest) {
        Set<Size> other = new HashSet<>(java.util.Arrays.asList(b.sizes(ImageFormat.YUV_420_888)));
        Size best = null; double bestScore = Double.MAX_VALUE;
        double aspect = aspectOf(a);
        for (Size s : a.sizes(ImageFormat.YUV_420_888)) {
            if (!other.contains(s) || s.getWidth() % 4 != 0 || s.getHeight() % 4 != 0 || s.getWidth() < s.getHeight()) continue;
            double area = (double) s.getWidth() * s.getHeight();
            if (largest) { if (best == null || area > (double) best.getWidth() * best.getHeight()) best = s; continue; }
            if (s.getWidth() > 1920 || area > 2.1e6) continue;
            double aspectErr = Math.abs((double) s.getWidth() / s.getHeight() - aspect);
            double score = aspectErr * 1e7 + Math.abs(area - 1.5e6);
            if (score < bestScore) { bestScore = score; best = s; }
        }
        return best;
    }

    static double aspectOf(Cam c) {
        android.graphics.Rect r = c.chars.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
        return r != null && r.height() > 0 ? (double) r.width() / r.height() : 4.0 / 3.0;
    }

    /** Preview size for the macro camera: ~2 MP, sensor aspect ratio (so the viewfinder is never stretched). */
    static Size previewSize(Context ctx) {
        CameraTopology t = get(ctx);
        Size fallback = new Size(1440, 1080);
        if (t.macro == null) return fallback;
        double aspect = aspectOf(t.macro);
        Size best = null; double bestScore = Double.MAX_VALUE;
        StreamConfigurationMap m = t.macro.chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        Size[] sizes = m != null ? m.getOutputSizes(SurfaceTexture.class) : null;
        if (sizes == null) return fallback;
        for (Size s : sizes) {
            double area = (double) s.getWidth() * s.getHeight();
            if (s.getWidth() < s.getHeight() || area > 2.2e6 || area < 3e5) continue;
            double score = Math.abs((double) s.getWidth() / s.getHeight() - aspect) * 1e7 + Math.abs(area - 1.6e6);
            if (score < bestScore) { bestScore = score; best = s; }
        }
        return best != null ? best : fallback;
    }
}
