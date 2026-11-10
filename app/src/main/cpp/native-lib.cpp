#include <jni.h>
#include <android/sensor.h>
#include <android/log.h>
#include <android/looper.h>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>

#define LOG_TAG "MultiCamTest"
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

namespace {

template <typename Func>
void parallelFor(int start, int end, Func&& fn) {
    int total = end - start; if (total <= 0) return;
    int nt = static_cast<int>(std::min<unsigned>(std::max(1u, std::thread::hardware_concurrency()), 6u));
    if (total < nt * 4) { for (int i = start; i < end; ++i) fn(i); return; }
    std::vector<std::thread> pool; pool.reserve(nt - 1);
    int chunk = (total + nt - 1) / nt;
    for (int t = 0; t < nt; ++t) {
        int s = start + t * chunk, e = std::min(end, s + chunk);
        if (s >= e) break;
        if (t == nt - 1) { for (int i = s; i < e; ++i) fn(i); }
        else pool.emplace_back([s, e, &fn]() { for (int i = s; i < e; ++i) fn(i); });
    }
    for (auto& w : pool) w.join();
}

struct GyroSample { int64_t ts; float wx, wy, wz; };
struct QuatSample { int64_t ts; float w, x, y, z; };
constexpr size_t IMU_RING_SIZE = 4096;

class ImuFusionBuffer {
public:
    void pushGyroDeviceFrame(int64_t ts, float wxImu, float wyImu, float wzImu) {
        std::lock_guard<std::mutex> lk(mtx_);
        gyroRing_[gyroHead_++ % IMU_RING_SIZE] = {ts, -wyImu, -wxImu, -wzImu};
    }
    void pushGameRotDeviceFrame(int64_t ts, float w, float xImu, float yImu, float zImu) {
        std::lock_guard<std::mutex> lk(mtx_);
        float x = -yImu, y = -xImu, z = -zImu, n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n > 1e-6f) { w /= n; x /= n; y /= n; z /= n; } else { w = 1.0f; x = y = z = 0.0f; }
        quatRing_[quatHead_++ % IMU_RING_SIZE] = {ts, w, x, y, z};
    }
    void getInterpolatedQuat(int64_t targetTs, float outQ[4]) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (quatHead_ == 0) { outQ[0] = 1.0f; outQ[1] = outQ[2] = outQ[3] = 0.0f; return; }
        size_t count = std::min(quatHead_, IMU_RING_SIZE), start = (quatHead_ >= IMU_RING_SIZE) ? (quatHead_ - IMU_RING_SIZE) : 0;
        const QuatSample *before = nullptr, *after = nullptr;
        for (size_t i = 0; i < count; ++i) {
            const QuatSample& s = quatRing_[(start + i) % IMU_RING_SIZE];
            if (s.ts <= targetTs) before = &s; else { after = &s; break; }
        }
        if (!before && after) { outQ[0]=after->w; outQ[1]=after->x; outQ[2]=after->y; outQ[3]=after->z; return; }
        if (before && !after) { outQ[0]=before->w; outQ[1]=before->x; outQ[2]=before->y; outQ[3]=before->z; return; }
        double dt = static_cast<double>(after->ts - before->ts);
        float t = (dt > 1.0) ? std::max(0.0f, std::min(1.0f, static_cast<float>((targetTs - before->ts) / dt))) : 0.0f;
        float q1[4] = {before->w, before->x, before->y, before->z}, q2[4] = {after->w, after->x, after->y, after->z};
        float dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
        if (dot < 0.0f) { dot = -dot; for (int k = 0; k < 4; ++k) q2[k] = -q2[k]; }
        if (dot > 0.9995f) { for (int k = 0; k < 4; ++k) outQ[k] = q1[k] + t * (q2[k] - q1[k]); }
        else {
            float th0 = std::acos(dot), th = th0 * t, s0 = std::sin(th0), st = std::sin(th);
            float a = std::cos(th) - dot * st / s0, b = st / s0;
            for (int k = 0; k < 4; ++k) outQ[k] = a * q1[k] + b * q2[k];
        }
        float n = std::sqrt(outQ[0]*outQ[0] + outQ[1]*outQ[1] + outQ[2]*outQ[2] + outQ[3]*outQ[3]);
        if (n > 1e-6f) for (int k = 0; k < 4; ++k) outQ[k] /= n;
    }
    void integrateGyroSO3(int64_t tStartNs, int64_t tEndNs, float R[9]) {
        R[0] = R[4] = R[8] = 1.0f; R[1] = R[2] = R[3] = R[5] = R[6] = R[7] = 0.0f;
        if (tEndNs <= tStartNs) return;
        std::lock_guard<std::mutex> lk(mtx_);
        if (gyroHead_ < 2) return;
        size_t count = std::min(gyroHead_, IMU_RING_SIZE), start = (gyroHead_ >= IMU_RING_SIZE) ? (gyroHead_ - IMU_RING_SIZE) : 0;
        int64_t prevT = tStartNs; float pWx = 0, pWy = 0, pWz = 0; bool hasPrev = false;
        for (size_t i = 0; i < count; ++i) {
            const GyroSample& s = gyroRing_[(start + i) % IMU_RING_SIZE];
            if (s.ts <= tStartNs) { pWx = s.wx; pWy = s.wy; pWz = s.wz; hasPrev = true; continue; }
            int64_t curT = std::min(s.ts, tEndNs);
            float dt = static_cast<float>(curT - prevT) * 1e-9f;
            if (dt > 0.0f && dt < 0.1f) {
                float dR[9], nextR[9];
                rodriguesExp((hasPrev ? 0.5f*(pWx + s.wx) : s.wx)*dt,
                             (hasPrev ? 0.5f*(pWy + s.wy) : s.wy)*dt,
                             (hasPrev ? 0.5f*(pWz + s.wz) : s.wz)*dt, dR);
                matMul3x3(dR, R, nextR); std::memcpy(R, nextR, sizeof(nextR));
            }
            prevT = curT; pWx = s.wx; pWy = s.wy; pWz = s.wz; hasPrev = true;
            if (s.ts >= tEndNs) break;
        }
    }
    static void quatToMatrix(const float q[4], float R[9]) {
        float w = q[0], x = q[1], y = q[2], z = q[3];
        R[0]=1-2*(y*y+z*z); R[1]=2*(x*y-z*w);   R[2]=2*(x*z+y*w);
        R[3]=2*(x*y+z*w);   R[4]=1-2*(x*x+z*z); R[5]=2*(y*z-x*w);
        R[6]=2*(x*z-y*w);   R[7]=2*(y*z+x*w);   R[8]=1-2*(x*x+y*y);
    }
    static void matMul3x3(const float A[9], const float B[9], float C[9]) {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                C[r*3 + c] = A[r*3]*B[c] + A[r*3 + 1]*B[3 + c] + A[r*3 + 2]*B[6 + c];
    }
    static void rodriguesExp(float rx, float ry, float rz, float R[9]) {
        float th = std::sqrt(rx*rx + ry*ry + rz*rz);
        if (th < 1e-7f) { R[0]=1; R[1]=-rz; R[2]=ry; R[3]=rz; R[4]=1; R[5]=-rx; R[6]=-ry; R[7]=rx; R[8]=1; return; }
        float kx = rx/th, ky = ry/th, kz = rz/th, c = std::cos(th), s = std::sin(th), v = 1.0f - c;
        R[0]=kx*kx*v+c;    R[1]=kx*ky*v-kz*s; R[2]=kx*kz*v+ky*s;
        R[3]=ky*kx*v+kz*s; R[4]=ky*ky*v+c;    R[5]=ky*kz*v-kx*s;
        R[6]=kz*kx*v-ky*s; R[7]=kz*ky*v+kx*s; R[8]=kz*kz*v+c;
    }
private:
    std::mutex mtx_;
    GyroSample gyroRing_[IMU_RING_SIZE]{};
    QuatSample quatRing_[IMU_RING_SIZE]{};
    size_t gyroHead_ = 0, quatHead_ = 0;
};

static ImuFusionBuffer gImuBuffer;
static std::atomic<bool> gSensorThreadRunning{false};
static std::thread gSensorThread;

static void sensorWorkerLoop() {
    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ASensorManager* mgr = ASensorManager_getInstanceForPackage("com.example.multicamtest");
    if (!mgr) return;
    const ASensor* gyro = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
    const ASensor* gameRot = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_GAME_ROTATION_VECTOR);
    ASensorEventQueue* queue = ASensorManager_createEventQueue(mgr, looper, 1, nullptr, nullptr);
    int minDelayUs = 2500;
    if (gyro) {
        int d = ASensor_getMinDelay(gyro); if (d > 0) minDelayUs = d;
        ASensorEventQueue_enableSensor(queue, gyro); ASensorEventQueue_setEventRate(queue, gyro, minDelayUs);
    }
    if (gameRot) {
        int d = ASensor_getMinDelay(gameRot);
        ASensorEventQueue_enableSensor(queue, gameRot); ASensorEventQueue_setEventRate(queue, gameRot, d > 0 ? d : 2500);
    }
    ALOGI("Native 400Hz IMU Looper active (interval=%d us)", minDelayUs);
    while (gSensorThreadRunning.load(std::memory_order_relaxed)) {
        ALooper_pollOnce(10, nullptr, nullptr, nullptr);
        ASensorEvent ev;
        while (ASensorEventQueue_getEvents(queue, &ev, 1) > 0) {
            if (ev.type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
                gImuBuffer.pushGyroDeviceFrame(ev.timestamp,
                    ev.uncalibrated_gyro.x_uncalib - ev.uncalibrated_gyro.x_bias,
                    ev.uncalibrated_gyro.y_uncalib - ev.uncalibrated_gyro.y_bias,
                    ev.uncalibrated_gyro.z_uncalib - ev.uncalibrated_gyro.z_bias);
            } else if (ev.type == ASENSOR_TYPE_GAME_ROTATION_VECTOR) {
                gImuBuffer.pushGameRotDeviceFrame(ev.timestamp, ev.data[3], ev.data[0], ev.data[1], ev.data[2]);
            }
        }
    }
    if (gyro) ASensorEventQueue_disableSensor(queue, gyro);
    if (gameRot) ASensorEventQueue_disableSensor(queue, gameRot);
    ASensorManager_destroyEventQueue(mgr, queue);
}

static bool write24BitBmp(const char* path, int w, int h, const float* R, const float* G, const float* B, const float* Lorig, const float* Ldeconv) {
    FILE* fp = std::fopen(path, "wb"); if (!fp) return false;
    int rowStride = (w * 3 + 3) & ~3;
    uint32_t dataSize = static_cast<uint32_t>(rowStride) * h, fileSize = 54 + dataSize, offset = 54, dibSize = 40;
    int32_t negH = -h; uint16_t planes = 1, bpp = 24;
    uint8_t hdr[54]{}; hdr[0] = 'B'; hdr[1] = 'M';
    std::memcpy(hdr + 2, &fileSize, 4); std::memcpy(hdr + 10, &offset, 4); std::memcpy(hdr + 14, &dibSize, 4);
    std::memcpy(hdr + 18, &w, 4); std::memcpy(hdr + 22, &negH, 4); std::memcpy(hdr + 26, &planes, 2);
    std::memcpy(hdr + 28, &bpp, 2); std::memcpy(hdr + 34, &dataSize, 4);
    std::fwrite(hdr, 1, 54, fp);
    std::vector<uint8_t> row(rowStride, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            size_t i = static_cast<size_t>(y) * w + x;
            float g = (Lorig[i] > 1.0f) ? (Ldeconv[i] / Lorig[i]) : 1.0f;
            row[x*3 + 0] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, std::round(B[i] * g))));
            row[x*3 + 1] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, std::round(G[i] * g))));
            row[x*3 + 2] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, std::round(R[i] * g))));
        }
        std::fwrite(row.data(), 1, rowStride, fp);
    }
    std::fclose(fp); return true;
}

class MicroscopeDrizzleEngine {
public:
    void configureSensorOptics(float fxPx, float fyPx, int cfaPattern, int blackLevel, int whiteLevel) {
        std::lock_guard<std::mutex> lk(mtx_);
        fxPx_ = fxPx; fyPx_ = fyPx; cfaPattern_ = cfaPattern;
        blackLevel_ = blackLevel; whiteLevel_ = std::max(blackLevel + 1, whiteLevel);
    }
    void reset(int width, int height) { std::lock_guard<std::mutex> lk(mtx_); allocateBuffers(width, height); }

    int addRaw16BayerFrame(const uint8_t* rawBytes, int rowStrideBytes, int width, int height, int64_t timestampNs) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (width != inW_ || height != inH_ || inW_ == 0) allocateBuffers(width, height);
        const float scale = 255.0f / static_cast<float>(whiteLevel_ - blackLevel_);
        std::vector<float> rawNorm(static_cast<size_t>(inW_) * inH_, 0.0f);
        double sumCh[3] = {0, 0, 0}; size_t cntCh[3] = {1, 1, 1};
        for (int y = 0; y < inH_; ++y) {
            const auto* row = reinterpret_cast<const uint16_t*>(rawBytes + y * rowStrideBytes);
            for (int x = 0; x < inW_; ++x) {
                float v = std::max(0.0f, std::min(255.0f, (static_cast<int>(row[x]) - blackLevel_) * scale));
                rawNorm[y * inW_ + x] = v; int c = getCfaChannel(x, y, cfaPattern_);
                sumCh[c] += v; cntCh[c]++;
            }
        }
        float mR = static_cast<float>(sumCh[0]/cntCh[0]), mG = static_cast<float>(sumCh[1]/cntCh[1]), mB = static_cast<float>(sumCh[2]/cntCh[2]);
        float wb[3] = {mR > 1.0f ? std::max(0.5f, std::min(3.0f, mG/mR)) : 1.0f, 1.0f, mB > 1.0f ? std::max(0.5f, std::min(3.0f, mG/mB)) : 1.0f};

        std::vector<float> curG(static_cast<size_t>(inW_) * inH_, 0.0f);
        parallelFor(1, inH_ - 1, [&](int y) {
            for (int x = 1; x < inW_ - 1; ++x) {
                size_t idx = static_cast<size_t>(y) * inW_ + x;
                if (getCfaChannel(x, y, cfaPattern_) == 1) curG[idx] = rawNorm[idx];
                else {
                    float gh = 0.5f*(rawNorm[idx - 1] + rawNorm[idx + 1]), gv = 0.5f*(rawNorm[idx - inW_] + rawNorm[idx + inW_]);
                    float dh = std::abs(rawNorm[idx - 1] - rawNorm[idx + 1]), dv = std::abs(rawNorm[idx - inW_] - rawNorm[idx + inW_]);
                    curG[idx] = (dh < dv) ? gh : (dv < dh ? gv : 0.5f*(gh + gv));
                }
            }
        });

        float Hinv[9], dx = 0, dy = 0;
        if (frameCount_ == 0) {
            refLuma_ = curG; refTimestampNs_ = timestampNs;
            Hinv[0] = Hinv[4] = Hinv[8] = 1.0f; Hinv[1] = Hinv[2] = Hinv[3] = Hinv[5] = Hinv[6] = Hinv[7] = 0.0f;
        } else computeGyroHomographyAndRefine(curG.data(), timestampNs, Hinv, dx, dy);

        for (int y = 2; y < inH_ - 2; ++y)
            for (int x = 2; x < inW_ - 2; ++x) {
                float w = Hinv[6]*x + Hinv[7]*y + Hinv[8]; if (std::abs(w) < 1e-5f) continue;
                float gx = ((Hinv[0]*x + Hinv[1]*y + Hinv[2])/w + dx)*2.0f, gy = ((Hinv[3]*x + Hinv[4]*y + Hinv[5])/w + dy)*2.0f;
                int ix = static_cast<int>(std::floor(gx)), iy = static_cast<int>(std::floor(gy));
                if (ix < 1 || ix + 2 >= outW_ || iy < 1 || iy + 2 >= outH_) continue;
                int c = getCfaChannel(x, y, cfaPattern_);
                float val = std::min(255.0f, rawNorm[y * inW_ + x] * wb[c]);
                for (int ky = 0; ky <= 1; ++ky)
                    for (int kx = 0; kx <= 1; ++kx) {
                        float rx = gx - (ix + kx), ry = gy - (iy + ky), wt = std::exp(-1.1f*(rx*rx + ry*ry));
                        size_t idx = static_cast<size_t>(iy + ky)*outW_ + (ix + kx);
                        accumRgb_[c][idx] += val * wt; weightRgb_[c][idx] += wt;
                    }
            }
        return ++frameCount_;
    }

    int addYuv420Frame(const uint8_t* yP, int yS, const uint8_t* uP, int uS, int uPS, const uint8_t* vP, int vS, int vPS, int width, int height, int64_t ts) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (width != inW_ || height != inH_ || inW_ == 0) allocateBuffers(width, height);
        std::vector<float> curL(static_cast<size_t>(inW_) * inH_);
        for (int y = 0; y < inH_; ++y) for (int x = 0; x < inW_; ++x) curL[y*inW_ + x] = yP[y*yS + x];

        float Hinv[9], dx = 0, dy = 0;
        if (frameCount_ == 0) {
            refLuma_ = curL; refTimestampNs_ = ts;
            Hinv[0] = Hinv[4] = Hinv[8] = 1.0f; Hinv[1] = Hinv[2] = Hinv[3] = Hinv[5] = Hinv[6] = Hinv[7] = 0.0f;
        } else computeGyroHomographyAndRefine(curL.data(), ts, Hinv, dx, dy);

        bool hasC = (uP != nullptr && vP != nullptr);
        for (int y = 1; y < inH_ - 1; ++y) {
            const uint8_t *rY = yP + y*yS, *rU = hasC ? uP + (y>>1)*uS : nullptr, *rV = hasC ? vP + (y>>1)*vS : nullptr;
            for (int x = 1; x < inW_ - 1; ++x) {
                float w = Hinv[6]*x + Hinv[7]*y + Hinv[8]; if (std::abs(w) < 1e-5f) continue;
                float gx = ((Hinv[0]*x + Hinv[1]*y + Hinv[2])/w + dx)*2.0f, gy = ((Hinv[3]*x + Hinv[4]*y + Hinv[5])/w + dy)*2.0f;
                int ix = static_cast<int>(std::floor(gx)), iy = static_cast<int>(std::floor(gy));
                if (ix < 0 || ix + 1 >= outW_ || iy < 0 || iy + 1 >= outH_) continue;
                float Y = rY[x], R = Y, G = Y, B = Y;
                if (hasC) {
                    float U = static_cast<float>(rU[(x>>1)*uPS]) - 128.0f, V = static_cast<float>(rV[(x>>1)*vPS]) - 128.0f;
                    R = std::max(0.0f, std::min(255.0f, Y + 1.402f*V));
                    G = std::max(0.0f, std::min(255.0f, Y - 0.344136f*U - 0.714136f*V));
                    B = std::max(0.0f, std::min(255.0f, Y + 1.772f*U));
                }
                float fx = gx - ix, fy = gy - iy, vals[3] = {R, G, B};
                float w00 = (1-fx)*(1-fy), w10 = fx*(1-fy), w01 = (1-fx)*fy, w11 = fx*fy;
                size_t b = static_cast<size_t>(iy)*outW_ + ix;
                for (int c = 0; c < 3; ++c) {
                    accumRgb_[c][b] += vals[c]*w00; weightRgb_[c][b] += w00;
                    accumRgb_[c][b+1] += vals[c]*w10; weightRgb_[c][b+1] += w10;
                    accumRgb_[c][b+outW_] += vals[c]*w01; weightRgb_[c][b+outW_] += w01;
                    accumRgb_[c][b+outW_+1] += vals[c]*w11; weightRgb_[c][b+outW_+1] += w11;
                }
            }
        }
        return ++frameCount_;
    }

    bool finalizeAndExport(const char* bmpPath, const char* ppmPath, const char* pgmPath) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (frameCount_ == 0 || outW_ == 0 || outH_ == 0) return false;
        size_t total = static_cast<size_t>(outW_) * outH_;
        std::vector<float> R(total, 0.0f), G(total, 0.0f), B(total, 0.0f);
        for (size_t i = 0; i < total; ++i) {
            if (weightRgb_[0][i] > 1e-4f) R[i] = accumRgb_[0][i] / weightRgb_[0][i];
            if (weightRgb_[1][i] > 1e-4f) G[i] = accumRgb_[1][i] / weightRgb_[1][i];
            if (weightRgb_[2][i] > 1e-4f) B[i] = accumRgb_[2][i] / weightRgb_[2][i];
        }
        for (int p = 0; p < 3; ++p) {
            fillSparseHoles(G, weightRgb_[1], outW_, outH_);
            fillSparseHoles(R, weightRgb_[0], outW_, outH_);
            fillSparseHoles(B, weightRgb_[2], outW_, outH_);
        }
        std::vector<float> L(total, 0.0f);
        for (size_t i = 0; i < total; ++i) L[i] = 0.299f*R[i] + 0.587f*G[i] + 0.114f*B[i];
        std::vector<float> Lorig = L;
        richardsonLucyDeconv(L, outW_, outH_, 10);

        if (bmpPath && bmpPath[0]) write24BitBmp(bmpPath, outW_, outH_, R.data(), G.data(), B.data(), Lorig.data(), L.data());
        if (ppmPath && ppmPath[0]) {
            FILE* fp = std::fopen(ppmPath, "wb");
            if (fp) {
                std::fprintf(fp, "P6\n%d %d\n255\n", outW_, outH_);
                std::vector<uint8_t> row(outW_ * 3);
                for (int y = 0; y < outH_; ++y) {
                    for (int x = 0; x < outW_; ++x) {
                        size_t i = static_cast<size_t>(y)*outW_ + x;
                        float g = (Lorig[i] > 1.0f) ? (L[i] / Lorig[i]) : 1.0f;
                        row[x*3+0] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, std::round(R[i]*g))));
                        row[x*3+1] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, std::round(G[i]*g))));
                        row[x*3+2] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, std::round(B[i]*g))));
                    }
                    std::fwrite(row.data(), 1, row.size(), fp);
                }
                std::fclose(fp);
            }
        }
        if (pgmPath && pgmPath[0]) {
            FILE* fp = std::fopen(pgmPath, "wb");
            if (fp) {
                std::fprintf(fp, "P5\n%d %d\n255\n", outW_, outH_);
                std::vector<uint8_t> row(outW_);
                for (int y = 0; y < outH_; ++y) {
                    for (int x = 0; x < outW_; ++x) row[x] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, std::round(L[static_cast<size_t>(y)*outW_ + x]))));
                    std::fwrite(row.data(), 1, outW_, fp);
                }
                std::fclose(fp);
            }
        }
        return true;
    }

private:
    void allocateBuffers(int width, int height) {
        inW_ = width; inH_ = height; outW_ = width * 2; outH_ = height * 2;
        size_t totalOut = static_cast<size_t>(outW_) * outH_;
        refLuma_.assign(static_cast<size_t>(inW_) * inH_, 0.0f);
        for (int c = 0; c < 3; ++c) { accumRgb_[c].assign(totalOut, 0.0f); weightRgb_[c].assign(totalOut, 0.0f); }
        frameCount_ = 0; refTimestampNs_ = 0;
    }
    static int getCfaChannel(int x, int y, int pattern) {
        int px = x & 1, py = y & 1;
        switch (pattern) {
            case 0: return (py == 0) ? (px == 0 ? 0 : 1) : (px == 0 ? 1 : 2);
            case 1: return (py == 0) ? (px == 0 ? 1 : 0) : (px == 0 ? 2 : 1);
            case 2: return (py == 0) ? (px == 0 ? 1 : 2) : (px == 0 ? 0 : 1);
            default: return (py == 0) ? (px == 0 ? 2 : 1) : (px == 0 ? 1 : 0);
        }
    }
    void computeGyroHomographyAndRefine(const float* curLuma, int64_t timestampNs, float Hinv[9], float& outDx, float& outDy) {
        float R[9]; gImuBuffer.integrateGyroSO3(refTimestampNs_, timestampNs, R);
        float fx = (fxPx_ > 100.0f) ? fxPx_ : (0.762f*inW_), fy = (fyPx_ > 100.0f) ? fyPx_ : (0.762f*inW_), cx = 0.5f*inW_, cy = 0.5f*inH_;
        float K[9] = {fx,0,cx, 0,fy,cy, 0,0,1}, Kinv[9] = {1/fx,0,-cx/fx, 0,1/fy,-cy/fy, 0,0,1};
        float Rt[9] = {R[0],R[3],R[6], R[1],R[4],R[7], R[2],R[5],R[8]}, tmp[9];
        ImuFusionBuffer::matMul3x3(K, Rt, tmp); ImuFusionBuffer::matMul3x3(tmp, Kinv, Hinv);

        outDx = 0.0f; outDy = 0.0f;
        int roiW = std::min(320, inW_ / 2), roiH = std::min(320, inH_ / 2), x0 = (inW_ - roiW) / 2, y0 = (inH_ - roiH) / 2;
        for (int iter = 0; iter < 5; ++iter) {
            double sXX = 0, sXY = 0, sYY = 0, sXT = 0, sYT = 0;
            for (int y = y0; y < y0 + roiH; y += 2)
                for (int x = x0; x < x0 + roiW; x += 2) {
                    float ixG = 0.5f*(refLuma_[y*inW_ + x + 1] - refLuma_[y*inW_ + x - 1]);
                    float iyG = 0.5f*(refLuma_[(y + 1)*inW_ + x] - refLuma_[(y - 1)*inW_ + x]);
                    if (ixG*ixG + iyG*iyG < 25.0f) continue;
                    float w = Hinv[6]*x + Hinv[7]*y + Hinv[8];
                    float u = (Hinv[0]*x + Hinv[1]*y + Hinv[2])/w + outDx, v = (Hinv[3]*x + Hinv[4]*y + Hinv[5])/w + outDy;
                    int iu = static_cast<int>(std::floor(u)), iv = static_cast<int>(std::floor(v));
                    if (iu < 0 || iu + 1 >= inW_ || iv < 0 || iv + 1 >= inH_) continue;
                    float fu = u - iu, fv = v - iv;
                    float cV = (1-fu)*(1-fv)*curLuma[iv*inW_+iu] + fu*(1-fv)*curLuma[iv*inW_+iu+1] + (1-fu)*fv*curLuma[(iv+1)*inW_+iu] + fu*fv*curLuma[(iv+1)*inW_+iu+1];
                    float itD = cV - refLuma_[y*inW_ + x];
                    sXX += ixG*ixG; sXY += ixG*iyG; sYY += iyG*iyG; sXT += ixG*itD; sYT += iyG*itD;
                }
            double det = sXX*sYY - sXY*sXY; if (std::abs(det) < 1e-3) break;
            float ddx = static_cast<float>((sYY*sXT - sXY*sYT)/det), ddy = static_cast<float>((sXX*sYT - sXY*sXT)/det);
            outDx = std::max(-3.0f, std::min(3.0f, outDx - ddx)); outDy = std::max(-3.0f, std::min(3.0f, outDy - ddy));
            if (ddx*ddx + ddy*ddy < 1e-4f) break;
        }
    }
    static void fillSparseHoles(std::vector<float>& plane, std::vector<float>& weights, int w, int h) {
        std::vector<float> next = plane;
        parallelFor(1, h - 1, [&](int y) {
            for (int x = 1; x < w - 1; ++x) {
                size_t idx = static_cast<size_t>(y) * w + x;
                if (weights[idx] <= 1e-4f) {
                    float sum = 0.0f; int cnt = 0;
                    for (int dy = -1; dy <= 1; ++dy)
                        for (int dx = -1; dx <= 1; ++dx) {
                            size_t nIdx = static_cast<size_t>(y + dy)*w + (x + dx);
                            if (weights[nIdx] > 1e-4f) { sum += plane[nIdx]; cnt++; }
                        }
                    if (cnt > 0) { next[idx] = sum / cnt; weights[idx] = 0.5f; }
                }
            }
        });
        plane.swap(next);
    }
    static void convolveSeparable5(const std::vector<float>& src, std::vector<float>& dst, int w, int h) {
        static constexpr float k[5] = {0.06136f, 0.24477f, 0.38774f, 0.24477f, 0.06136f};
        std::vector<float> tmp(static_cast<size_t>(w) * h, 0.0f);
        parallelFor(0, h, [&](int y) {
            for (int x = 2; x < w - 2; ++x)
                tmp[y*w + x] = k[0]*src[y*w+x-2] + k[1]*src[y*w+x-1] + k[2]*src[y*w+x] + k[3]*src[y*w+x+1] + k[4]*src[y*w+x+2];
        });
        parallelFor(2, h - 2, [&](int y) {
            for (int x = 2; x < w - 2; ++x)
                dst[y*w + x] = k[0]*tmp[(y-2)*w+x] + k[1]*tmp[(y-1)*w+x] + k[2]*tmp[y*w+x] + k[3]*tmp[(y+1)*w+x] + k[4]*tmp[(y+2)*w+x];
        });
    }
    static void richardsonLucyDeconv(std::vector<float>& est, int w, int h, int iterations) {
        std::vector<float> obs = est, blur(est.size(), 0.0f), rat(est.size(), 1.0f), corr(est.size(), 1.0f);
        for (int it = 0; it < iterations; ++it) {
            convolveSeparable5(est, blur, w, h);
            for (size_t i = 0; i < est.size(); ++i) rat[i] = (blur[i] > 1.0f) ? (obs[i] / blur[i]) : 1.0f;
            convolveSeparable5(rat, corr, w, h);
            for (size_t i = 0; i < est.size(); ++i) est[i] = std::max(0.0f, std::min(255.0f, est[i] * corr[i]));
        }
    }
    std::mutex mtx_;
    int inW_ = 0, inH_ = 0, outW_ = 0, outH_ = 0, frameCount_ = 0;
    int64_t refTimestampNs_ = 0;
    float fxPx_ = 3080.0f, fyPx_ = 3080.0f;
    int cfaPattern_ = 0, blackLevel_ = 64, whiteLevel_ = 1023;
    std::vector<float> refLuma_, accumRgb_[3], weightRgb_[3];
};

static MicroscopeDrizzleEngine gMicroscopeEngine;

struct Vertex3D { float p[3]; float n[3]; };
struct IndexedTriangle { uint32_t i0, i1, i2; };

class StereoTsdfScannerEngine {
public:
    StereoTsdfScannerEngine() { resetTsdfVolume(); }
    void resetTsdfVolume() {
        std::lock_guard<std::mutex> lk(mtx_);
        size_t total = static_cast<size_t>(VOXEL_DIM) * VOXEL_DIM * VOXEL_DIM;
        tsdfDist_.assign(total, 1.0f); tsdfWeight_.assign(total, 0.0f);
        integratedFrames_ = 0; lutReady_ = false; verticalAligned_ = false;
        refCentroidMm_[0] = 0.0f; refCentroidMm_[1] = 0.0f; refCentroidMm_[2] = VOLUME_CENTER_Z_MM;
    }
    void setDualCameraCalibration(
            float fx0, float fy0, float cx0, float cy0, const float d0[5],
            float fx2, float fy2, float cx2, float cy2, const float d2[5],
            const float R_0to2[9], const float T_0to2_mm[3]) {
        std::lock_guard<std::mutex> lk(mtx_);
        fx0_ = fx0; fy0_ = fy0; cx0_ = cx0; cy0_ = cy0;
        fx2_ = fx2; fy2_ = fy2; cx2_ = cx2; cy2_ = cy2;
        if (d0) std::memcpy(dist0_, d0, 5 * sizeof(float));
        if (d2) std::memcpy(dist2_, d2, 5 * sizeof(float));
        if (R_0to2) std::memcpy(R_ext_, R_0to2, 9 * sizeof(float));
        if (T_0to2_mm) {
            std::memcpy(T_ext_mm_, T_0to2_mm, 3 * sizeof(float));
            baselineMm_ = std::sqrt(T_0to2_mm[0]*T_0to2_mm[0] + T_0to2_mm[1]*T_0to2_mm[1] + T_0to2_mm[2]*T_0to2_mm[2]);
        }
        lutReady_ = false;
    }
    int processSynchronizedStereoPair(
            const uint8_t* leftY, int leftStride, const uint8_t* rightY, int rightStride,
            int width, int height, int64_t timestampNs, uint32_t* outHeatmapArgb) {
        std::lock_guard<std::mutex> lk(mtx_);
        int dw = width / 4, dh = height / 4;
        if (dw < 48 || dh < 48) return integratedFrames_;
        if (!lutReady_ || dw != rectW_ || dh != rectH_) buildBouguetRectificationLut(width, height, dw, dh);

        std::vector<uint8_t> rectL(dw * dh, 0), rectR(dw * dh, 0);
        remapBilinear(leftY, leftStride, width, height, mapLx_.data(), mapLy_.data(), dw, dh, rectL.data());
        remapBilinear(rightY, rightStride, width, height, mapRx_.data(), mapRy_.data(), dw, dh, rectR.data());

        if (!verticalAligned_) {
            int bestDy = estimateVerticalEpipolarOffset(rectL.data(), rectR.data(), dw, dh);
            if (bestDy != 0) {
                cy2_ += bestDy * (static_cast<float>(height) / dh);
                buildBouguetRectificationLut(width, height, dw, dh);
                remapBilinear(rightY, rightStride, width, height, mapRx_.data(), mapRy_.data(), dw, dh, rectR.data());
            }
            verticalAligned_ = true;
        }

        std::vector<float> depthMm(dw * dh, 0.0f), dispSubPx(dw * dh, 0.0f);
        computeCensusSgmDepth(rectL.data(), rectR.data(), dw, dh, rectFx_, baselineMm_, dispSubPx.data(), depthMm.data());
        if (outHeatmapArgb) renderHeatmapArgb(rectL.data(), dispSubPx.data(), dw, dh, outHeatmapArgb);

        float q[4], curC[3];
        gImuBuffer.getInterpolatedQuat(timestampNs, q);
        bool validC = computeDepthCentroid(depthMm.data(), dw, dh, rectFx_, rectFy_, rectCx_, rectCy_, curC);
        if (integratedFrames_ == 0) {
            std::memcpy(refQuat_, q, sizeof(q));
            if (validC) std::memcpy(refCentroidMm_, curC, sizeof(curC));
        }

        float Rcur[9], Rref[9], RrefT[9], Rrel[9];
        ImuFusionBuffer::quatToMatrix(q, Rcur); ImuFusionBuffer::quatToMatrix(refQuat_, Rref);
        RrefT[0]=Rref[0]; RrefT[1]=Rref[3]; RrefT[2]=Rref[6]; RrefT[3]=Rref[1]; RrefT[4]=Rref[4]; RrefT[5]=Rref[7]; RrefT[6]=Rref[2]; RrefT[7]=Rref[5]; RrefT[8]=Rref[8];
        ImuFusionBuffer::matMul3x3(Rcur, RrefT, Rrel);

        float tRel[3] = {0, 0, 0};
        if (validC) for (int k = 0; k < 3; ++k) tRel[k] = std::max(-40.0f, std::min(40.0f, curC[k] - refCentroidMm_[k]));
        integrateTsdf(depthMm.data(), dw, dh, rectFx_, rectFy_, rectCx_, rectCy_, Rrel, tRel);
        return ++integratedFrames_;
    }
    int exportWatertightMesh(const char* stlPath, const char* plyPath) {
        std::lock_guard<std::mutex> lk(mtx_);
        std::vector<Vertex3D> verts; std::vector<IndexedTriangle> tris;
        extractKuhnMarchingTetrahedra(verts, tris);
        if (stlPath && stlPath[0]) writeBinaryStl(stlPath, verts, tris);
        if (plyPath && plyPath[0]) writeIndexedPly(plyPath, verts, tris);
        return static_cast<int>(tris.size());
    }

private:
    static constexpr int VOXEL_DIM = 96, MAX_DISP = 40;
    static constexpr float VOXEL_SIZE_MM = 2.0f, TRUNC_DIST_MM = 8.0f, VOLUME_CENTER_Z_MM = 250.0f;

    static int estimateVerticalEpipolarOffset(const uint8_t* rectL, const uint8_t* rectR, int w, int h) {
        std::vector<float> pL(h, 0.0f), pR(h, 0.0f);
        for (int y = 4; y < h - 4; ++y) {
            float sL = 0, sR = 0;
            for (int x = w / 4; x < (3 * w) / 4; ++x) {
                sL += std::abs(static_cast<int>(rectL[(y+1)*w+x]) - static_cast<int>(rectL[(y-1)*w+x]));
                sR += std::abs(static_cast<int>(rectR[(y+1)*w+x]) - static_cast<int>(rectR[(y-1)*w+x]));
            }
            pL[y] = sL; pR[y] = sR;
        }
        int bestDy = 0; float bestS = -1.0f;
        for (int dy = -6; dy <= 6; ++dy) {
            float s = 0; for (int y = 12; y < h - 12; ++y) s += pL[y] * pR[y + dy];
            if (s > bestS) { bestS = s; bestDy = dy; }
        }
        return bestDy;
    }

    void buildBouguetRectificationLut(int srcW, int srcH, int dstW, int dstH) {
        rectW_ = dstW; rectH_ = dstH;
        float sx = static_cast<float>(srcW) / dstW, sy = static_cast<float>(srcH) / dstH;
        float fx0 = fx0_ > 100 ? fx0_ : 0.762f*srcW, fy0 = fy0_ > 100 ? fy0_ : 0.762f*srcW, cx0 = cx0_ > 10 ? cx0_ : 0.5f*srcW, cy0 = cy0_ > 10 ? cy0_ : 0.5f*srcH;
        float fx2 = fx2_ > 100 ? fx2_ : 0.391f*srcW, fy2 = fy2_ > 100 ? fy2_ : 0.391f*srcW, cx2 = cx2_ > 10 ? cx2_ : 0.5f*srcW, cy2 = cy2_ > 10 ? cy2_ : 0.5f*srcH;
        rectFx_ = fx0 / sx; rectFy_ = fy0 / sy; rectCx_ = cx0 / sx; rectCy_ = cy0 / sy;

        float tr = std::max(-1.0f, std::min(1.0f, (R_ext_[0] + R_ext_[4] + R_ext_[8] - 1.0f) * 0.5f));
        float th = std::acos(tr), om[3] = {0, 0, 0};
        if (th > 1e-5f) {
            float s = th / (2.0f * std::sin(th));
            om[0] = (R_ext_[7] - R_ext_[5])*s; om[1] = (R_ext_[2] - R_ext_[6])*s; om[2] = (R_ext_[3] - R_ext_[1])*s;
        }
        float r_l[9], r_r[9];
        ImuFusionBuffer::rodriguesExp(0.5f*om[0], 0.5f*om[1], 0.5f*om[2], r_l);
        ImuFusionBuffer::rodriguesExp(-0.5f*om[0], -0.5f*om[1], -0.5f*om[2], r_r);

        float t[3] = {
            r_r[0]*T_ext_mm_[0] + r_r[1]*T_ext_mm_[1] + r_r[2]*T_ext_mm_[2],
            r_r[3]*T_ext_mm_[0] + r_r[4]*T_ext_mm_[1] + r_r[5]*T_ext_mm_[2],
            r_r[6]*T_ext_mm_[0] + r_r[7]*T_ext_mm_[1] + r_r[8]*T_ext_mm_[2]
        };
        float normT = std::sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
        float e1[3] = {1, 0, 0};
        if (normT > 1e-4f) { baselineMm_ = normT; e1[0] = -t[0]/normT; e1[1] = -t[1]/normT; e1[2] = -t[2]/normT; }
        float xyN = std::hypot(e1[0], e1[1]);
        float e2[3] = {xyN > 1e-5f ? -e1[1]/xyN : 0.0f, xyN > 1e-5f ? e1[0]/xyN : 1.0f, 0.0f};
        float e3[3] = {e1[1]*e2[2] - e1[2]*e2[1], e1[2]*e2[0] - e1[0]*e2[2], e1[0]*e2[1] - e1[1]*e2[0]};
        float R_rect[9] = {e1[0],e1[1],e1[2], e2[0],e2[1],e2[2], e3[0],e3[1],e3[2]}, R1[9], R2[9];
        ImuFusionBuffer::matMul3x3(R_rect, r_l, R1); ImuFusionBuffer::matMul3x3(R_rect, r_r, R2);
        float R1i[9] = {R1[0],R1[3],R1[6], R1[1],R1[4],R1[7], R1[2],R1[5],R1[8]};
        float R2i[9] = {R2[0],R2[3],R2[6], R2[1],R2[4],R2[7], R2[2],R2[5],R2[8]};

        mapLx_.resize(dstW * dstH); mapLy_.resize(dstW * dstH); mapRx_.resize(dstW * dstH); mapRy_.resize(dstW * dstH);
        auto projectCam = [](float xR, float yR, const float Ri[9], const float d[5], float fx, float fy, float cx, float cy, float& mx, float& my) {
            float rx = Ri[0]*xR + Ri[1]*yR + Ri[2], ry = Ri[3]*xR + Ri[4]*yR + Ri[5], rz = Ri[6]*xR + Ri[7]*yR + Ri[8];
            float xn = std::abs(rz) > 1e-5f ? rx/rz : xR, yn = std::abs(rz) > 1e-5f ? ry/rz : yR;
            float r2 = xn*xn + yn*yn, rad = 1.0f + d[0]*r2 + d[1]*r2*r2 + d[4]*r2*r2*r2;
            mx = (xn*rad + 2*d[2]*xn*yn + d[3]*(r2 + 2*xn*xn))*fx + cx;
            my = (yn*rad + d[2]*(r2 + 2*yn*yn) + 2*d[3]*xn*yn)*fy + cy;
        };
        for (int v = 0; v < dstH; ++v)
            for (int u = 0; u < dstW; ++u) {
                float xR = (u - rectCx_) / rectFx_, yR = (v - rectCy_) / rectFy_;
                projectCam(xR, yR, R1i, dist0_, fx0, fy0, cx0, cy0, mapLx_[v*dstW + u], mapLy_[v*dstW + u]);
                projectCam(xR, yR, R2i, dist2_, fx2, fy2, cx2, cy2, mapRx_[v*dstW + u], mapRy_[v*dstW + u]);
            }
        lutReady_ = true;
    }

    static void remapBilinear(const uint8_t* src, int srcS, int srcW, int srcH, const float* mX, const float* mY, int dstW, int dstH, uint8_t* dst) {
        parallelFor(0, dstH, [&](int y) {
            for (int x = 0; x < dstW; ++x) {
                float u = mX[y*dstW + x], v = mY[y*dstW + x];
                int iu = static_cast<int>(std::floor(u)), iv = static_cast<int>(std::floor(v));
                if (iu >= 0 && iu + 1 < srcW && iv >= 0 && iv + 1 < srcH) {
                    float fu = u - iu, fv = v - iv;
                    const uint8_t *r0 = src + iv*srcS + iu, *r1 = r0 + srcS;
                    dst[y*dstW + x] = static_cast<uint8_t>(std::round((1-fu)*(1-fv)*r0[0] + fu*(1-fv)*r0[1] + (1-fu)*fv*r1[0] + fu*fv*r1[1]));
                } else dst[y*dstW + x] = 0;
            }
        });
    }

    static void computeCensusSgmDepth(const uint8_t* left, const uint8_t* right, int w, int h, float fx, float baselineMm, float* outDisp, float* outDepth) {
        std::vector<uint32_t> cL(w * h, 0), cR(w * h, 0);
        parallelFor(2, h - 2, [&](int y) {
            for (int x = 2; x < w - 2; ++x) {
                uint8_t vL = left[y*w + x], vR = right[y*w + x];
                uint32_t bL = 0, bR = 0; int bit = 0;
                for (int dy = -2; dy <= 2; ++dy)
                    for (int dx = -2; dx <= 2; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int idx = (y + dy)*w + (x + dx);
                        if (left[idx] < vL) bL |= (1u << bit);
                        if (right[idx] < vR) bR |= (1u << bit);
                        bit++;
                    }
                cL[y*w + x] = bL; cR[y*w + x] = bR;
            }
        });

        const size_t volSz = static_cast<size_t>(w) * h * MAX_DISP;
        std::vector<uint8_t> baseC(volSz, 24);
        std::vector<uint16_t> aggC(volSz, 0);
        parallelFor(2, h - 2, [&](int y) {
            for (int x = 2; x < w - 2; ++x) {
                uint32_t lB = cL[y*w + x]; size_t b = (static_cast<size_t>(y)*w + x)*MAX_DISP;
                for (int d = 0; d < MAX_DISP; ++d)
                    if (x - d >= 2) baseC[b + d] = static_cast<uint8_t>(__builtin_popcount(lB ^ cR[y*w + (x - d)]));
            }
        });

        auto stepSgm = [&](uint16_t* path, uint16_t& prevMin, size_t idx, int grad) {
            constexpr int P1 = 6, P2_B = 24;
            int P2 = std::max(P1 + 2, P2_B - (grad >> 2));
            uint16_t nxtMin = 0xffff, np[MAX_DISP];
            for (int d = 0; d < MAX_DISP; ++d) {
                uint16_t v = path[d];
                if (d > 0) v = std::min<uint16_t>(v, path[d - 1] + P1);
                if (d + 1 < MAX_DISP) v = std::min<uint16_t>(v, path[d + 1] + P1);
                v = std::min<uint16_t>(v, prevMin + P2);
                np[d] = static_cast<uint16_t>(baseC[idx + d] + v - prevMin);
                aggC[idx + d] += np[d]; nxtMin = std::min(nxtMin, np[d]);
            }
            std::memcpy(path, np, sizeof(np)); prevMin = nxtMin;
        };

        parallelFor(2, h - 2, [&](int y) {
            uint16_t pLR[MAX_DISP]{}, pRL[MAX_DISP]{}, minLR = 0, minRL = 0;
            for (int x = 2; x < w - 2; ++x)
                stepSgm(pLR, minLR, (static_cast<size_t>(y)*w + x)*MAX_DISP, std::abs(static_cast<int>(left[y*w+x]) - static_cast<int>(left[y*w+x-1])));
            for (int x = w - 3; x >= 2; --x)
                stepSgm(pRL, minRL, (static_cast<size_t>(y)*w + x)*MAX_DISP, std::abs(static_cast<int>(left[y*w+x]) - static_cast<int>(left[y*w+x+1])));
        });

        parallelFor(2, w - 2, [&](int x) {
            uint16_t pTB[MAX_DISP]{}, pBT[MAX_DISP]{}, minTB = 0, minBT = 0;
            for (int y = 2; y < h - 2; ++y)
                stepSgm(pTB, minTB, (static_cast<size_t>(y)*w + x)*MAX_DISP, std::abs(static_cast<int>(left[y*w+x]) - static_cast<int>(left[(y-1)*w+x])));
            for (int y = h - 3; y >= 2; --y)
                stepSgm(pBT, minBT, (static_cast<size_t>(y)*w + x)*MAX_DISP, std::abs(static_cast<int>(left[y*w+x]) - static_cast<int>(left[(y+1)*w+x])));
        });

        parallelFor(2, h - 2, [&](int y) {
            for (int x = MAX_DISP; x < w - 2; ++x) {
                size_t idx = (static_cast<size_t>(y)*w + x)*MAX_DISP;
                int bestD = -1; uint16_t bestC = 0xffff, secC = 0xffff;
                for (int d = 2; d < MAX_DISP - 1; ++d) {
                    uint16_t c = aggC[idx + d];
                    if (c < bestC) { secC = bestC; bestC = c; bestD = d; } else if (c < secC) secC = c;
                }
                if (bestD < 2 || bestC > 48 || bestC * 10 > secC * 8) continue;
                int xr = x - bestD, bestDr = -1; uint16_t minCr = 0xffff;
                for (int dr = 2; dr < MAX_DISP - 1; ++dr)
                    if (xr + dr < w - 2) {
                        uint16_t cr = aggC[(static_cast<size_t>(y)*w + (xr + dr))*MAX_DISP + dr];
                        if (cr < minCr) { minCr = cr; bestDr = dr; }
                    }
                if (std::abs(bestD - bestDr) > 1) continue;
                float cP = aggC[idx + bestD - 1], c0 = bestC, cN = aggC[idx + bestD + 1], den = 2.0f*(cP - 2.0f*c0 + cN);
                float dSub = bestD + (std::abs(den) > 1e-3f ? std::max(-0.5f, std::min(0.5f, (cP - cN)/den)) : 0.0f);
                float z = (fx * baselineMm) / dSub;
                if (z >= 80.0f && z <= 600.0f) { outDisp[y*w + x] = dSub; outDepth[y*w + x] = z; }
            }
        });
    }

    static bool computeDepthCentroid(const float* depthMm, int dw, int dh, float fx, float fy, float cx, float cy, float outC[3]) {
        double sx = 0, sy = 0, sz = 0; int cnt = 0;
        for (int y = dh / 4; y < (3 * dh) / 4; ++y)
            for (int x = dw / 4; x < (3 * dw) / 4; ++x) {
                float z = depthMm[y*dw + x];
                if (z > 100.0f && z < 400.0f) { sx += (x - cx)*z/fx; sy += (y - cy)*z/fy; sz += z; cnt++; }
            }
        if (cnt < 50) return false;
        outC[0] = static_cast<float>(sx / cnt); outC[1] = static_cast<float>(sy / cnt); outC[2] = static_cast<float>(sz / cnt);
        return true;
    }

    static void renderHeatmapArgb(const uint8_t* grayL, const float* disp, int w, int h, uint32_t* outArgb) {
        for (int i = 0; i < w * h; ++i) {
            float d = disp[i];
            if (d <= 1.0f) { uint8_t g = grayL[i]; outArgb[i] = (0xFFu << 24) | (g << 16) | (g << 8) | g; }
            else {
                float t = std::max(0.0f, std::min(1.0f, (d - 2.0f) / (MAX_DISP - 4.0f)));
                uint8_t r = static_cast<uint8_t>(255.0f * std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f*t - 3.0f))));
                uint8_t g = static_cast<uint8_t>(255.0f * std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f*t - 2.0f))));
                uint8_t b = static_cast<uint8_t>(255.0f * std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f*t - 1.0f))));
                outArgb[i] = (0xFFu << 24) | (r << 16) | (g << 8) | b;
            }
        }
    }

    void integrateTsdf(const float* depthMm, int dw, int dh, float fx, float fy, float cx, float cy, const float Rrel[9], const float tRel[3]) {
        const float halfVol = 0.5f * VOXEL_DIM * VOXEL_SIZE_MM;
        parallelFor(0, VOXEL_DIM, [&](int vz) {
            float pwZ = (vz * VOXEL_SIZE_MM - halfVol) + VOLUME_CENTER_Z_MM;
            for (int vy = 0; vy < VOXEL_DIM; ++vy) {
                float pwY = vy * VOXEL_SIZE_MM - halfVol;
                for (int vx = 0; vx < VOXEL_DIM; ++vx) {
                    float pwX = vx * VOXEL_SIZE_MM - halfVol;
                    float dx = pwX - refCentroidMm_[0], dy = pwY - refCentroidMm_[1], dz = pwZ - refCentroidMm_[2];
                    float pcX = Rrel[0]*dx + Rrel[1]*dy + Rrel[2]*dz + refCentroidMm_[0] + tRel[0];
                    float pcY = Rrel[3]*dx + Rrel[4]*dy + Rrel[5]*dz + refCentroidMm_[1] + tRel[1];
                    float pcZ = Rrel[6]*dx + Rrel[7]*dy + Rrel[8]*dz + refCentroidMm_[2] + tRel[2];
                    if (pcZ <= 50.0f) continue;
                    int u = static_cast<int>(std::round((pcX * fx) / pcZ + cx)), v = static_cast<int>(std::round((pcY * fy) / pcZ + cy));
                    if (u < 0 || u >= dw || v < 0 || v >= dh) continue;
                    float mZ = depthMm[v * dw + u]; if (mZ <= 0.0f) continue;
                    float sdf = mZ - pcZ; if (sdf < -TRUNC_DIST_MM) continue;
                    float tsdf = std::min(1.0f, sdf / TRUNC_DIST_MM);
                    size_t idx = (static_cast<size_t>(vz) * VOXEL_DIM + vy) * VOXEL_DIM + vx;
                    float wOld = tsdfWeight_[idx], wNew = std::min(64.0f, wOld + 1.0f);
                    tsdfDist_[idx] = (tsdfDist_[idx] * wOld + tsdf) / wNew; tsdfWeight_[idx] = wNew;
                }
            }
        });
    }

    void extractKuhnMarchingTetrahedra(std::vector<Vertex3D>& outV, std::vector<IndexedTriangle>& outT) const {
        static constexpr int CV[8][3] = {{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1}};
        static constexpr int KT[6][4] = {{0,5,1,6},{0,1,2,6},{0,2,3,6},{0,3,7,6},{0,7,4,6},{0,4,5,6}};
        static constexpr int TE[6][2] = {{0,1},{1,2},{2,0},{0,3},{1,3},{2,3}};
        static constexpr int TT[16][6] = {
            {-1,-1,-1,-1,-1,-1},{0,3,2,-1,-1,-1},{0,1,4,-1,-1,-1},{1,4,2,2,4,3},
            {1,2,5,-1,-1,-1},{0,3,5,0,5,1},{0,2,5,0,5,4},{3,5,4,-1,-1,-1},
            {3,4,5,-1,-1,-1},{0,4,5,0,5,2},{0,1,5,0,5,3},{1,5,2,-1,-1,-1},
            {1,2,4,2,3,4},{0,4,1,-1,-1,-1},{0,2,3,-1,-1,-1},{-1,-1,-1,-1,-1,-1}
        };
        const float halfVol = 0.5f * VOXEL_DIM * VOXEL_SIZE_MM;
        std::unordered_map<uint64_t, uint32_t> cache;
        auto gIdx = [](int x, int y, int z) -> size_t { return (static_cast<size_t>(z)*VOXEL_DIM + y)*VOXEL_DIM + x; };
        auto getVert = [&](size_t iA, size_t iB, const int pA[3], const int pB[3], float dA, float dB) -> uint32_t {
            uint64_t key = (static_cast<uint64_t>(std::min(iA, iB)) << 32) | std::max(iA, iB);
            auto it = cache.find(key); if (it != cache.end()) return it->second;
            float t = std::abs(dA - dB) > 1e-6f ? std::max(0.0f, std::min(1.0f, dA / (dA - dB))) : 0.5f;
            float gx = pA[0] + t*(pB[0] - pA[0]), gy = pA[1] + t*(pB[1] - pA[1]), gz = pA[2] + t*(pB[2] - pA[2]);
            int ix = std::max(1, std::min(VOXEL_DIM - 2, static_cast<int>(std::round(gx))));
            int iy = std::max(1, std::min(VOXEL_DIM - 2, static_cast<int>(std::round(gy))));
            int iz = std::max(1, std::min(VOXEL_DIM - 2, static_cast<int>(std::round(gz))));
            float nx = tsdfDist_[gIdx(ix+1, iy, iz)] - tsdfDist_[gIdx(ix-1, iy, iz)];
            float ny = tsdfDist_[gIdx(ix, iy+1, iz)] - tsdfDist_[gIdx(ix, iy-1, iz)];
            float nz = tsdfDist_[gIdx(ix, iy, iz+1)] - tsdfDist_[gIdx(ix, iy, iz-1)];
            float len = std::sqrt(nx*nx + ny*ny + nz*nz);
            Vertex3D v{};
            v.p[0] = gx*VOXEL_SIZE_MM - halfVol; v.p[1] = gy*VOXEL_SIZE_MM - halfVol; v.p[2] = (gz*VOXEL_SIZE_MM - halfVol) + VOLUME_CENTER_Z_MM;
            if (len > 1e-6f) { v.n[0] = -nx/len; v.n[1] = -ny/len; v.n[2] = -nz/len; } else v.n[2] = -1.0f;
            uint32_t idx = static_cast<uint32_t>(outV.size());
            outV.push_back(v); cache.emplace(key, idx); return idx;
        };

        for (int z = 1; z < VOXEL_DIM - 2; ++z)
            for (int y = 1; y < VOXEL_DIM - 2; ++y)
                for (int x = 1; x < VOXEL_DIM - 2; ++x) {
                    int cP[8][3], neg = 0; size_t cI[8]; float cV[8]; bool ok = true;
                    for (int i = 0; i < 8; ++i) {
                        cP[i][0] = x + CV[i][0]; cP[i][1] = y + CV[i][1]; cP[i][2] = z + CV[i][2];
                        cI[i] = gIdx(cP[i][0], cP[i][1], cP[i][2]);
                        if (tsdfWeight_[cI[i]] < 1.0f) { ok = false; break; }
                        cV[i] = tsdfDist_[cI[i]]; if (cV[i] < 0.0f) neg++;
                    }
                    if (!ok || neg == 0 || neg == 8) continue;
                    for (int t = 0; t < 6; ++t) {
                        int tv[4] = {KT[t][0], KT[t][1], KT[t][2], KT[t][3]};
                        int m = (cV[tv[0]]<0?1:0)|(cV[tv[1]]<0?2:0)|(cV[tv[2]]<0?4:0)|(cV[tv[3]]<0?8:0);
                        const int* row = TT[m];
                        for (int i = 0; i < 6 && row[i] != -1; i += 3) {
                            uint32_t ti[3];
                            for (int k = 0; k < 3; ++k) {
                                int a = tv[TE[row[i+k]][0]], b = tv[TE[row[i+k]][1]];
                                ti[k] = getVert(cI[a], cI[b], cP[a], cP[b], cV[a], cV[b]);
                            }
                            if (ti[0] != ti[1] && ti[1] != ti[2] && ti[2] != ti[0]) outT.push_back({ti[0], ti[1], ti[2]});
                        }
                    }
                }
    }

    static bool writeBinaryStl(const char* path, const std::vector<Vertex3D>& verts, const std::vector<IndexedTriangle>& tris) {
        FILE* fp = std::fopen(path, "wb"); if (!fp) return false;
        char hdr[80]{}; std::snprintf(hdr, sizeof(hdr), "Metric 3D Kuhn Marching Tetrahedra TSDF Mesh (mm)");
        std::fwrite(hdr, 1, 80, fp);
        uint32_t cnt = static_cast<uint32_t>(tris.size()); std::fwrite(&cnt, 4, 1, fp);
        uint16_t attr = 0;
        for (const auto& t : tris) {
            const Vertex3D &a = verts[t.i0], &b = verts[t.i1], &c = verts[t.i2];
            float n[3] = {(a.n[0]+b.n[0]+c.n[0])/3.0f, (a.n[1]+b.n[1]+c.n[1])/3.0f, (a.n[2]+b.n[2]+c.n[2])/3.0f};
            std::fwrite(n, 4, 3, fp); std::fwrite(a.p, 4, 3, fp); std::fwrite(b.p, 4, 3, fp); std::fwrite(c.p, 4, 3, fp);
            std::fwrite(&attr, 2, 1, fp);
        }
        std::fclose(fp); return true;
    }

    static bool writeIndexedPly(const char* path, const std::vector<Vertex3D>& verts, const std::vector<IndexedTriangle>& tris) {
        FILE* fp = std::fopen(path, "w"); if (!fp) return false;
        std::fprintf(fp, "ply\nformat ascii 1.0\nelement vertex %zu\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face %zu\nproperty list uchar int vertex_indices\nend_header\n", verts.size(), tris.size());
        for (const auto& v : verts) std::fprintf(fp, "%.4f %.4f %.4f %.4f %.4f %.4f\n", v.p[0], v.p[1], v.p[2], v.n[0], v.n[1], v.n[2]);
        for (const auto& t : tris) std::fprintf(fp, "3 %u %u %u\n", t.i0, t.i1, t.i2);
        std::fclose(fp); return true;
    }

    std::mutex mtx_;
    float fx0_ = 1098.0f, fy0_ = 1098.0f, cx0_ = 720.0f, cy0_ = 540.0f;
    float fx2_ = 563.0f,  fy2_ = 563.0f,  cx2_ = 720.0f, cy2_ = 540.0f;
    float dist0_[5]{}, dist2_[5]{};
    float R_ext_[9] = {1.0f,0.0f,0.0f, 0.0f,1.0f,0.0f, 0.0f,0.0f,1.0f};
    float T_ext_mm_[3] = {-18.5f, 0.0f, 0.0f}, baselineMm_ = 18.5f;
    bool lutReady_ = false, verticalAligned_ = false;
    int rectW_ = 0, rectH_ = 0;
    float rectFx_ = 274.5f, rectFy_ = 274.5f, rectCx_ = 180.0f, rectCy_ = 135.0f;
    std::vector<float> mapLx_, mapLy_, mapRx_, mapRy_, tsdfDist_, tsdfWeight_;
    float refQuat_[4] = {1.0f, 0.0f, 0.0f, 0.0f}, refCentroidMm_[3] = {0.0f, 0.0f, VOLUME_CENTER_Z_MM};
    int integratedFrames_ = 0;
};

static StereoTsdfScannerEngine gStereoScanner;

} // namespace

extern "C" {
JNIEXPORT void JNICALL Java_com_example_multicamtest_NativeEngine_startImuLooper(JNIEnv*, jclass) {
    if (!gSensorThreadRunning.exchange(true)) gSensorThread = std::thread(sensorWorkerLoop);
}
JNIEXPORT void JNICALL Java_com_example_multicamtest_NativeEngine_stopImuLooper(JNIEnv*, jclass) {
    if (gSensorThreadRunning.exchange(false) && gSensorThread.joinable()) gSensorThread.join();
}
JNIEXPORT void JNICALL Java_com_example_multicamtest_NativeEngine_configureMicroscopeOptics(JNIEnv*, jclass, jfloat fx, jfloat fy, jint cfa, jint bl, jint wl) {
    gMicroscopeEngine.configureSensorOptics(fx, fy, cfa, bl, wl);
}
JNIEXPORT void JNICALL Java_com_example_multicamtest_NativeEngine_resetMicroscope(JNIEnv*, jclass, jint w, jint h) {
    gMicroscopeEngine.reset(w, h);
}
JNIEXPORT jint JNICALL Java_com_example_multicamtest_NativeEngine_pushMicroscopeRaw16Frame(JNIEnv* env, jclass, jobject buf, jint stride, jint w, jint h, jlong ts) {
    auto* ptr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(buf));
    return ptr ? gMicroscopeEngine.addRaw16BayerFrame(ptr, stride, w, h, ts) : -1;
}
JNIEXPORT jint JNICALL Java_com_example_multicamtest_NativeEngine_pushMicroscopeYuvFrame(JNIEnv* env, jclass, jobject yB, jint yS, jobject uB, jint uS, jint uP, jobject vB, jint vS, jint vP, jint w, jint h, jlong ts) {
    auto* pY = static_cast<const uint8_t*>(env->GetDirectBufferAddress(yB));
    auto* pU = uB ? static_cast<const uint8_t*>(env->GetDirectBufferAddress(uB)) : nullptr;
    auto* pV = vB ? static_cast<const uint8_t*>(env->GetDirectBufferAddress(vB)) : nullptr;
    return pY ? gMicroscopeEngine.addYuv420Frame(pY, yS, pU, uS, uP, pV, vS, vP, w, h, ts) : -1;
}
JNIEXPORT jboolean JNICALL Java_com_example_multicamtest_NativeEngine_exportMicroscopeImages(JNIEnv* env, jclass, jstring bmpJ, jstring ppmJ, jstring pgmJ) {
    const char *bmp = env->GetStringUTFChars(bmpJ, nullptr), *ppm = env->GetStringUTFChars(ppmJ, nullptr), *pgm = env->GetStringUTFChars(pgmJ, nullptr);
    bool ok = gMicroscopeEngine.finalizeAndExport(bmp, ppm, pgm);
    env->ReleaseStringUTFChars(bmpJ, bmp); env->ReleaseStringUTFChars(ppmJ, ppm); env->ReleaseStringUTFChars(pgmJ, pgm);
    return ok ? JNI_TRUE : JNI_FALSE;
}
JNIEXPORT void JNICALL Java_com_example_multicamtest_NativeEngine_resetStereoScanner(JNIEnv*, jclass) {
    gStereoScanner.resetTsdfVolume();
}
JNIEXPORT void JNICALL Java_com_example_multicamtest_NativeEngine_setDualCameraCalibration(JNIEnv* env, jclass, jfloat fx0, jfloat fy0, jfloat cx0, jfloat cy0, jfloatArray d0A, jfloat fx2, jfloat fy2, jfloat cx2, jfloat cy2, jfloatArray d2A, jfloatArray rA, jfloatArray tA) {
    jfloat *d0 = d0A ? env->GetFloatArrayElements(d0A, nullptr) : nullptr;
    jfloat *d2 = d2A ? env->GetFloatArrayElements(d2A, nullptr) : nullptr;
    jfloat *r  = rA  ? env->GetFloatArrayElements(rA, nullptr)  : nullptr;
    jfloat *t  = tA  ? env->GetFloatArrayElements(tA, nullptr)  : nullptr;
    gStereoScanner.setDualCameraCalibration(fx0, fy0, cx0, cy0, d0, fx2, fy2, cx2, cy2, d2, r, t);
    if (d0) env->ReleaseFloatArrayElements(d0A, d0, JNI_ABORT);
    if (d2) env->ReleaseFloatArrayElements(d2A, d2, JNI_ABORT);
    if (r)  env->ReleaseFloatArrayElements(rA, r, JNI_ABORT);
    if (t)  env->ReleaseFloatArrayElements(tA, t, JNI_ABORT);
}
JNIEXPORT jint JNICALL Java_com_example_multicamtest_NativeEngine_processStereoPair(JNIEnv* env, jclass, jobject lB, jint lS, jobject rB, jint rS, jint w, jint h, jlong ts, jintArray hmA) {
    auto *pL = static_cast<const uint8_t*>(env->GetDirectBufferAddress(lB)), *pR = static_cast<const uint8_t*>(env->GetDirectBufferAddress(rB));
    if (!pL || !pR) return -1;
    jint* hm = hmA ? env->GetIntArrayElements(hmA, nullptr) : nullptr;
    int f = gStereoScanner.processSynchronizedStereoPair(pL, lS, pR, rS, w, h, ts, reinterpret_cast<uint32_t*>(hm));
    if (hm) env->ReleaseIntArrayElements(hmA, hm, 0);
    return f;
}
JNIEXPORT jint JNICALL Java_com_example_multicamtest_NativeEngine_exportStereoMesh(JNIEnv* env, jclass, jstring stlJ, jstring plyJ) {
    const char *stl = env->GetStringUTFChars(stlJ, nullptr), *ply = env->GetStringUTFChars(plyJ, nullptr);
    int tris = gStereoScanner.exportWatertightMesh(stl, ply);
    env->ReleaseStringUTFChars(stlJ, stl); env->ReleaseStringUTFChars(plyJ, ply);
    return tris;
}
} // extern "C"
