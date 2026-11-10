package com.example.multicamtest;

import android.graphics.Bitmap;
import java.nio.ByteBuffer;

/** JNI surface of libmulticam_native (see cpp/engine.hpp for the algorithms). */
public final class NativeEngine {
    static { System.loadLibrary("multicam_native"); }
    private NativeEngine() {}

    // IMU
    public static native void startImuLooper(String packageName, int sensorOrientationDeg);
    public static native void stopImuLooper();
    public static native void setSensorOrientation(int degrees);
    /** Offset added to camera timestamps to align them with sensor-event time (0 when both are REALTIME). */
    public static native void setImuTimeOffsetNs(long offsetNs);
    /** {gyroHz, rotationVectorHz, gyroSamples, rotationSamples} */
    public static native float[] imuInfo();

    // Microscope burst
    public static native void configureMicroscope(float fxPx, float fyPx, int cfaPattern, int blackLevel, int whiteLevel);
    /** @return {outputWidth, outputHeight}; the processed region is a centred crop bounded by maxOutMegapixels. */
    public static native int[] resetMicroscope(int width, int height, float maxOutMegapixels);
    public static native int pushMicroscopeRaw16Frame(ByteBuffer raw, int rowStrideBytes, int width, int height, long timestampNs);
    public static native int pushMicroscopeYuvFrame(ByteBuffer y, int yStride, ByteBuffer u, int uStride, int uPixStride,
                                                    ByteBuffer v, int vStride, int vPixStride, int width, int height, long timestampNs);
    public static native boolean finalizeMicroscope();
    /** Fills an ARGB_8888 bitmap of exactly the output size. */
    public static native boolean exportMicroscopeToBitmap(Bitmap bitmap);
    /** {width, height} of the single-frame baseline ("before") image. */
    public static native int[] baselineSize();
    public static native boolean exportBaselineToBitmap(Bitmap bitmap);
    public static native void releaseMicroscope();

    // Stereo
    public static native void resetStereoScanner();
    /** params: fxL fyL cxL cyL k1 k2 p1 p2 k3 | fxR fyR cxR cyR k1 k2 p1 p2 k3 | R[9] | T_mm[3] | calibrated(0/1) */
    public static native void configureStereo(int streamWidth, int streamHeight, float[] params);
    /** {baselineMm, zMinMm, zMaxMm, rectFocalPx, rectRotationDeg, rectWidth, rectHeight} */
    public static native float[] stereoInfo();
    public static native int processStereoPair(ByteBuffer left, int leftStride, ByteBuffer right, int rightStride,
                                               int width, int height, long timestampNs, int[] outHeatmapArgb);
    public static native int exportStereoMesh(String stlPath, String plyPath);
}
