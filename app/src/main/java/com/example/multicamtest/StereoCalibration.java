package com.example.multicamtest;

import android.graphics.Rect;
import android.hardware.camera2.CameraCharacteristics;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * Builds the 31-float parameter block expected by {@link NativeEngine#configureStereo}.
 * Precedence: user file (stereo_calibration.json) > device-reported Camera2 pose/intrinsics > estimates.
 *
 * <p>File format (OpenCV stereoCalibrate conventions, camera 1 = "left" reported in the console, X2 = R*X1 + T):
 * <pre>{ "image_size": [w, h], "K1": [[fx,0,cx],[0,fy,cy],[0,0,1]], "D1": [k1,k2,p1,p2,k3],
 *   "K2": ..., "D2": ..., "R": [[..],[..],[..]], "T": [tx,ty,tz], "T_units": "mm" | "m" }</pre>
 */
final class StereoCalibration {
    private static final String TAG = "MultiCamCalib";
    static final String FILE_NAME = "stereo_calibration.json";

    final float[] params = new float[31];
    String source = "estimated (uncalibrated: scale is approximate)";
    boolean calibrated;

    static StereoCalibration build(File dir, CameraTopology.StereoPair pair, int w, int h, java.util.List<String> log) {
        StereoCalibration c = new StereoCalibration();
        intrinsics(pair.left, w, h, c.params, 0);
        intrinsics(pair.right, w, h, c.params, 9);
        float[] R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        float[] T = {-15f, 0f, 0f};  // generic phone baseline; only used when the device reports no pose
        boolean deviceIntr = true;

        if (pair.poseDeltaMm != null) {
            float[] d = pair.poseDeltaMm;
            float[] m = sensorToCamera(pair.left.sensorOrientation, d);
            float n = (float) Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            if (Math.abs(m[2]) > 0.5f * n) m[2] = 0f;  // lens separation is in the image plane on phones; discard implausible depth offset
            T = new float[]{-m[0], -m[1], -m[2]};      // camera-1 centre in camera-0 frame is -R^T T
            c.calibrated = true;
            c.source = String.format(Locale.US, "device-reported pose (baseline %.1f mm, nominal intrinsics)", n);
            log.add("Stereo extrinsics from Camera2 LENS_POSE_TRANSLATION; rotation assumed parallel.");
        } else {
            log.add("Device reports no lens pose: using a generic 15 mm baseline (depth scale approximate).");
        }

        File f = new File(dir, FILE_NAME);
        if (f.exists()) {
            try {
                if (loadFile(f, w, h, c.params, R, T)) {
                    c.calibrated = true;
                    c.source = "calibration file " + FILE_NAME;
                    log.add("Loaded " + FILE_NAME);
                }
            } catch (Exception e) {
                Log.e(TAG, "calibration file rejected", e);
                log.add("Ignoring " + FILE_NAME + ": " + e.getMessage());
            }
        }
        System.arraycopy(R, 0, c.params, 18, 9);
        System.arraycopy(T, 0, c.params, 27, 3);
        c.params[30] = c.calibrated ? 1f : 0f;
        return c;
    }

    /** Camera-frame vector (x right, y down, z forward) from a device-frame vector, for a back camera. */
    static float[] sensorToCamera(int orientationDeg, float[] d) {
        int o = ((orientationDeg % 360) + 360) % 360;
        int c = o == 0 ? 1 : o == 180 ? -1 : 0, s = o == 90 ? 1 : o == 270 ? -1 : 0;
        return new float[]{c * d[0] - s * d[1], -s * d[0] - c * d[1], -d[2]};
    }

    private static void intrinsics(CameraTopology.Cam cam, int sw, int sh, float[] out, int off) {
        Rect arr = cam.chars.get(CameraCharacteristics.SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE);
        if (arr == null) arr = cam.chars.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
        float aw = arr != null ? arr.width() : sw, ah = arr != null ? arr.height() : sh;
        float s = sw / aw;                                    // stream spans the array width; height is a centred crop
        float[] k = cam.chars.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION);
        float fx, fy, cx, cy;
        if (k != null && k.length >= 4 && k[0] > 0 && k[1] > 0) {
            fx = k[0] * s; fy = k[1] * s; cx = k[2] * s; cy = k[3] * s - (ah * s - sh) / 2f;
        } else if (cam.sensorMm != null && cam.sensorMm.getWidth() > 0 && cam.focalMm > 0) {
            fx = fy = cam.focalMm / cam.sensorMm.getWidth() * sw; cx = sw / 2f; cy = sh / 2f;
        } else {
            fx = fy = 0.8f * sw; cx = sw / 2f; cy = sh / 2f;
        }
        out[off] = fx; out[off + 1] = fy; out[off + 2] = cx; out[off + 3] = cy;
        for (int i = 4; i < 9; ++i) out[off + i] = 0f;      // ISP-corrected streams: treat as a pinhole
    }

    private static boolean loadFile(File f, int w, int h, float[] p, float[] R, float[] T) throws Exception {
        byte[] data = new byte[(int) f.length()];
        try (FileInputStream in = new FileInputStream(f)) { int n = in.read(data); if (n != data.length) throw new Exception("short read"); }
        JSONObject o = new JSONObject(new String(data, StandardCharsets.UTF_8));
        double sx = 1, sy = 1;
        if (o.has("image_size")) {
            JSONArray sz = o.getJSONArray("image_size");
            sx = w / sz.getDouble(0); sy = h / sz.getDouble(1);
        }
        readK(o.getJSONArray("K1"), sx, sy, p, 0); readD(o.getJSONArray("D1"), p, 4);
        readK(o.getJSONArray("K2"), sx, sy, p, 9); readD(o.getJSONArray("D2"), p, 13);
        JSONArray rj = o.getJSONArray("R");
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) R[r * 3 + c] = (float) rj.getJSONArray(r).getDouble(c);
        JSONArray tj = o.getJSONArray("T");
        String units = o.optString("T_units", "");
        for (int i = 0; i < 3; ++i) T[i] = (float) (tj.optJSONArray(i) != null ? tj.getJSONArray(i).getDouble(0) : tj.getDouble(i));
        float n = (float) Math.sqrt(T[0] * T[0] + T[1] * T[1] + T[2] * T[2]);
        boolean meters = units.equalsIgnoreCase("m") || (units.isEmpty() && n < 1.0f);  // legacy: no unit given, |T| < 1 => metres
        if (meters) for (int i = 0; i < 3; ++i) T[i] *= 1000f;
        n = (float) Math.sqrt(T[0] * T[0] + T[1] * T[1] + T[2] * T[2]);
        if (!(n > 1f && n < 500f)) throw new Exception("baseline " + n + " mm is implausible; set \"T_units\"");
        return true;
    }

    private static void readK(JSONArray k, double sx, double sy, float[] p, int off) throws Exception {
        p[off] = (float) (k.getJSONArray(0).getDouble(0) * sx);
        p[off + 1] = (float) (k.getJSONArray(1).getDouble(1) * sy);
        p[off + 2] = (float) (k.getJSONArray(0).getDouble(2) * sx);
        p[off + 3] = (float) (k.getJSONArray(1).getDouble(2) * sy);
    }

    private static void readD(JSONArray d, float[] p, int off) throws Exception {
        for (int i = 0; i < 5; ++i) {
            double v = i < d.length() ? (d.optJSONArray(i) != null ? d.getJSONArray(i).getDouble(0) : d.getDouble(i)) : 0.0;
            p[off + i] = (float) v;
        }
    }
}
