// JNI glue + Android sensor loop. All computation lives in engine.hpp (unit-tested on a host).
#include <android/bitmap.h>
#include <android/log.h>
#include <android/looper.h>
#include <android/sensor.h>
#include <jni.h>

#include <atomic>
#include <string>
#include <thread>

#include "engine.hpp"

#define LOG_TAG "MultiCam"
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define ALOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)

namespace {

mc::ImuFusionBuffer gImu;
mc::MicroscopeEngine gMicro(gImu);
mc::StereoTsdfEngine gStereo(gImu);

std::atomic<bool> gRunning{false};
std::atomic<float> gGyroHz{0.0f}, gRotHz{0.0f};
std::thread gThread;

void sensorLoop(std::string pkg) {
    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ASensorManager* mgr = ASensorManager_getInstanceForPackage(pkg.c_str());
    if (!mgr) { ALOGW("no ASensorManager"); gRunning = false; return; }
    bool uncal = true;
    const ASensor* gyro = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
    if (!gyro) { gyro = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_GYROSCOPE); uncal = false; }
    const ASensor* rot = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_GAME_ROTATION_VECTOR);
    ASensorEventQueue* queue = ASensorManager_createEventQueue(mgr, looper, 1, nullptr, nullptr);
    if (!queue || (!gyro && !rot)) { ALOGW("no usable motion sensor"); if (queue) ASensorManager_destroyEventQueue(mgr, queue); gRunning = false; return; }
    auto enable = [&](const ASensor* s, int targetUs) {
        if (!s) return;
        const int minUs = ASensor_getMinDelay(s);
        ASensorEventQueue_enableSensor(queue, s);
        ASensorEventQueue_setEventRate(queue, s, std::max(targetUs, minUs > 0 ? minUs : targetUs));
    };
    enable(gyro, 2500);   // ask for up to 400 Hz; the device grants what it can (reported via imuInfo)
    enable(rot, 5000);
    ALOGI("IMU: gyro=%s (%s) rotation=%s", gyro ? "yes" : "no", uncal ? "uncalibrated" : "calibrated", rot ? "yes" : "no");

    int64_t winStart = 0; int gyroN = 0, rotN = 0;
    while (gRunning.load(std::memory_order_relaxed)) {
        ALooper_pollOnce(20, nullptr, nullptr, nullptr);
        ASensorEvent ev;
        while (ASensorEventQueue_getEvents(queue, &ev, 1) > 0) {
            if (winStart == 0) winStart = ev.timestamp;
            if (ev.type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
                gImu.pushGyroDevice(ev.timestamp, ev.uncalibrated_gyro.x_uncalib - ev.uncalibrated_gyro.x_bias,
                                    ev.uncalibrated_gyro.y_uncalib - ev.uncalibrated_gyro.y_bias,
                                    ev.uncalibrated_gyro.z_uncalib - ev.uncalibrated_gyro.z_bias);
                ++gyroN;
            } else if (ev.type == ASENSOR_TYPE_GYROSCOPE) {
                gImu.pushGyroDevice(ev.timestamp, ev.vector.x, ev.vector.y, ev.vector.z); ++gyroN;
            } else if (ev.type == ASENSOR_TYPE_GAME_ROTATION_VECTOR) {
                gImu.pushRotationDevice(ev.timestamp, ev.data[3], ev.data[0], ev.data[1], ev.data[2]); ++rotN;
            }
            const int64_t span = ev.timestamp - winStart;
            if (span > 1'000'000'000LL) {
                gGyroHz = gyroN * 1e9f / span; gRotHz = rotN * 1e9f / span;
                winStart = ev.timestamp; gyroN = rotN = 0;
            }
        }
    }
    if (gyro) ASensorEventQueue_disableSensor(queue, gyro);
    if (rot) ASensorEventQueue_disableSensor(queue, rot);
    ASensorManager_destroyEventQueue(mgr, queue);
}

struct Utf {
    JNIEnv* env; jstring js; const char* c;
    Utf(JNIEnv* e, jstring s) : env(e), js(s), c(s ? e->GetStringUTFChars(s, nullptr) : nullptr) {}
    ~Utf() { if (c) env->ReleaseStringUTFChars(js, c); }
};

jfloatArray toJFloats(JNIEnv* env, const float* v, int n) {
    jfloatArray a = env->NewFloatArray(n);
    env->SetFloatArrayRegion(a, 0, n, v);
    return a;
}

}  // namespace

#define JNI(ret, name) extern "C" JNIEXPORT ret JNICALL Java_com_example_multicamtest_NativeEngine_##name

JNI(void, startImuLooper)(JNIEnv* env, jclass, jstring pkg, jint sensorOrientation) {
    gImu.setSensorOrientation(sensorOrientation);
    Utf p(env, pkg);
    if (!gRunning.exchange(true)) {
        if (gThread.joinable()) gThread.join();
        gThread = std::thread(sensorLoop, std::string(p.c ? p.c : ""));
    }
}
JNI(void, stopImuLooper)(JNIEnv*, jclass) {
    gRunning = false;
    if (gThread.joinable()) gThread.join();
}
JNI(void, setSensorOrientation)(JNIEnv*, jclass, jint deg) { gImu.setSensorOrientation(deg); }
JNI(void, setImuTimeOffsetNs)(JNIEnv*, jclass, jlong off) { gImu.setTimeOffsetNs(off); }
JNI(jfloatArray, imuInfo)(JNIEnv* env, jclass) {
    const float v[4] = {gGyroHz.load(), gRotHz.load(), static_cast<float>(gImu.gyroCount()), static_cast<float>(gImu.quatCount())};
    return toJFloats(env, v, 4);
}

// ---- microscope -------------------------------------------------------------------------------
JNI(void, configureMicroscope)(JNIEnv*, jclass, jfloat fx, jfloat fy, jint cfa, jint bl, jint wl) { gMicro.configure(fx, fy, cfa, bl, wl); }
JNI(jintArray, resetMicroscope)(JNIEnv* env, jclass, jint w, jint h, jfloat maxOutMp) {
    gMicro.reset(w, h, maxOutMp);
    const jint v[2] = {gMicro.outputWidth(), gMicro.outputHeight()};
    jintArray a = env->NewIntArray(2); env->SetIntArrayRegion(a, 0, 2, v); return a;
}
JNI(jint, pushMicroscopeRaw16Frame)(JNIEnv* env, jclass, jobject buf, jint stride, jint w, jint h, jlong ts) {
    auto* p = static_cast<const uint8_t*>(env->GetDirectBufferAddress(buf));
    return p ? gMicro.addRaw16(p, stride, w, h, ts) : -1;
}
JNI(jint, pushMicroscopeYuvFrame)(JNIEnv* env, jclass, jobject yB, jint yS, jobject uB, jint uS, jint uP, jobject vB, jint vS, jint vP, jint w, jint h, jlong ts) {
    auto* y = static_cast<const uint8_t*>(env->GetDirectBufferAddress(yB));
    auto* u = uB ? static_cast<const uint8_t*>(env->GetDirectBufferAddress(uB)) : nullptr;
    auto* v = vB ? static_cast<const uint8_t*>(env->GetDirectBufferAddress(vB)) : nullptr;
    return y ? gMicro.addYuv(y, yS, u, uS, uP, v, vS, vP, w, h, ts) : -1;
}
JNI(jboolean, finalizeMicroscope)(JNIEnv*, jclass) { return gMicro.finalize(8) ? JNI_TRUE : JNI_FALSE; }
JNI(jboolean, exportMicroscopeToBitmap)(JNIEnv* env, jclass, jobject bitmap) {
    AndroidBitmapInfo info;
    if (AndroidBitmap_getInfo(env, bitmap, &info) != ANDROID_BITMAP_RESULT_SUCCESS || info.format != ANDROID_BITMAP_FORMAT_RGBA_8888) return JNI_FALSE;
    if (static_cast<int>(info.width) != gMicro.outputWidth() || static_cast<int>(info.height) != gMicro.outputHeight()) return JNI_FALSE;
    void* px = nullptr;
    if (AndroidBitmap_lockPixels(env, bitmap, &px) != ANDROID_BITMAP_RESULT_SUCCESS || !px) return JNI_FALSE;
    const bool ok = gMicro.exportRgba(static_cast<uint8_t*>(px), static_cast<int>(info.stride));
    AndroidBitmap_unlockPixels(env, bitmap);
    return ok ? JNI_TRUE : JNI_FALSE;
}
JNI(jintArray, baselineSize)(JNIEnv* env, jclass) {
    const jint v[2] = {gMicro.baselineWidth(), gMicro.baselineHeight()};
    jintArray a = env->NewIntArray(2); env->SetIntArrayRegion(a, 0, 2, v); return a;
}
JNI(jboolean, exportBaselineToBitmap)(JNIEnv* env, jclass, jobject bitmap) {
    AndroidBitmapInfo info;
    if (AndroidBitmap_getInfo(env, bitmap, &info) != ANDROID_BITMAP_RESULT_SUCCESS || info.format != ANDROID_BITMAP_FORMAT_RGBA_8888) return JNI_FALSE;
    if (static_cast<int>(info.width) != gMicro.baselineWidth() || static_cast<int>(info.height) != gMicro.baselineHeight()) return JNI_FALSE;
    void* px = nullptr;
    if (AndroidBitmap_lockPixels(env, bitmap, &px) != ANDROID_BITMAP_RESULT_SUCCESS || !px) return JNI_FALSE;
    const bool ok = gMicro.exportBaselineRgba(static_cast<uint8_t*>(px), static_cast<int>(info.stride));
    AndroidBitmap_unlockPixels(env, bitmap);
    return ok ? JNI_TRUE : JNI_FALSE;
}
JNI(void, releaseMicroscope)(JNIEnv*, jclass) { gMicro.release(); }

// ---- stereo -----------------------------------------------------------------------------------
JNI(void, resetStereoScanner)(JNIEnv*, jclass) { gStereo.reset(); }
// p: fxL fyL cxL cyL k1 k2 p1 p2 k3 | fxR fyR cxR cyR k1 k2 p1 p2 k3 | R[9] | T_mm[3] | calibrated
JNI(void, configureStereo)(JNIEnv* env, jclass, jint w, jint h, jfloatArray pA) {
    if (!pA || env->GetArrayLength(pA) < 31) return;
    float p[31]; env->GetFloatArrayRegion(pA, 0, 31, p);
    mc::StereoConfig c;
    c.left = {p[0], p[1], p[2], p[3], {p[4], p[5], p[6], p[7], p[8]}};
    c.right = {p[9], p[10], p[11], p[12], {p[13], p[14], p[15], p[16], p[17]}};
    for (int i = 0; i < 9; ++i) c.R[i] = p[18 + i];
    for (int i = 0; i < 3; ++i) c.T_mm[i] = p[27 + i];
    c.calibrated = p[30] > 0.5f;
    gStereo.configure(c, w, h);
}
JNI(jfloatArray, stereoInfo)(JNIEnv* env, jclass) {
    const auto i = gStereo.info();
    const float v[7] = {i.baselineMm, i.zMinMm, i.zMaxMm, i.fRectPx, static_cast<float>(i.rectRotationDeg), static_cast<float>(i.rectW), static_cast<float>(i.rectH)};
    return toJFloats(env, v, 7);
}
JNI(jint, processStereoPair)(JNIEnv* env, jclass, jobject lB, jint lS, jobject rB, jint rS, jint w, jint h, jlong ts, jintArray hmA) {
    auto* l = static_cast<const uint8_t*>(env->GetDirectBufferAddress(lB));
    auto* r = static_cast<const uint8_t*>(env->GetDirectBufferAddress(rB));
    if (!l || !r) return -1;
    jint* hm = nullptr;
    if (hmA && env->GetArrayLength(hmA) >= gStereo.rectWidth() * gStereo.rectHeight()) hm = env->GetIntArrayElements(hmA, nullptr);
    const int f = gStereo.process(l, lS, r, rS, w, h, ts, reinterpret_cast<uint32_t*>(hm));
    if (hm) env->ReleaseIntArrayElements(hmA, hm, 0);
    return f;
}
JNI(jint, exportStereoMesh)(JNIEnv* env, jclass, jstring stlJ, jstring plyJ) {
    Utf stl(env, stlJ), ply(env, plyJ);
    return gStereo.exportMesh(stl.c, ply.c);
}
