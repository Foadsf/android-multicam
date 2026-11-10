package com.example.multicamtest;
import java.nio.ByteBuffer;

public final class NativeEngine {
    static { System.loadLibrary("multicam_native"); }
    public static native void startImuLooper();
    public static native void stopImuLooper();
    public static native void configureMicroscopeOptics(float fxPx, float fyPx, int cfaPattern, int blackLevel, int whiteLevel);
    public static native void resetMicroscope(int width, int height);
    public static native int pushMicroscopeRaw16Frame(ByteBuffer rawBuffer, int rowStrideBytes, int width, int height, long timestampNs);
    public static native int pushMicroscopeYuvFrame(ByteBuffer yBuf, int yStride, ByteBuffer uBuf, int uStride, int uPixStride, ByteBuffer vBuf, int vStride, int vPixStride, int width, int height, long timestampNs);
    public static native boolean exportMicroscopeImages(String bmpPath, String ppmPath, String pgmPath);
    public static native void resetStereoScanner();
    public static native void setDualCameraCalibration(float fx0, float fy0, float cx0, float cy0, float[] d0, float fx2, float fy2, float cx2, float cy2, float[] d2, float[] R_0to2, float[] T_0to2_mm);
    public static native int processStereoPair(ByteBuffer leftYBuf, int leftStride, ByteBuffer rightYBuf, int rightStride, int width, int height, long timestampNs, int[] outHeatmapArgb);
    public static native int exportStereoMesh(String stlPath, String plyPath);
}
