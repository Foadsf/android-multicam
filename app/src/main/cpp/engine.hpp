// Pure C++17 computational core (no Android / JNI dependencies) so it can be unit-tested on a host.
//
//  * ImuFusionBuffer        gyro + rotation-vector ring buffers, device->camera axis mapping
//  * MicroscopeEngine       gyro-seeded, image-refined 2x drizzle accumulator (RAW16 Bayer or YUV)
//  * StereoTsdfEngine       rectification, census-SGM disparity, TSDF fusion, marching-tetrahedra mesh
//
// Conventions: camera frame = x right, y down, z forward (OpenCV). Pixel (0,0) = centre of top-left pixel.
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace mc {

// ------------------------------------------------------------------------------------------------
// helpers
// ------------------------------------------------------------------------------------------------

template <typename Func>
void parallelFor(int start, int end, Func&& fn) {
    const int total = end - start;
    if (total <= 0) return;
    const int nt = static_cast<int>(std::min<unsigned>(std::max(1u, std::thread::hardware_concurrency()), 8u));
    if (nt == 1 || total < nt * 2) { for (int i = start; i < end; ++i) fn(i); return; }
    std::vector<std::thread> pool;
    pool.reserve(nt - 1);
    const int chunk = (total + nt - 1) / nt;
    for (int t = 0; t < nt; ++t) {
        const int s = start + t * chunk, e = std::min(end, s + chunk);
        if (s >= e) break;
        if (t == nt - 1) { for (int i = s; i < e; ++i) fn(i); }
        else pool.emplace_back([s, e, &fn]() { for (int i = s; i < e; ++i) fn(i); });
    }
    for (auto& w : pool) w.join();
}

inline void matMul3(const float A[9], const float B[9], float C[9]) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
}
inline void transpose3(const float A[9], float T[9]) {
    T[0] = A[0]; T[1] = A[3]; T[2] = A[6];
    T[3] = A[1]; T[4] = A[4]; T[5] = A[7];
    T[6] = A[2]; T[7] = A[5]; T[8] = A[8];
}
inline void identity3(float R[9]) { for (int i = 0; i < 9; ++i) R[i] = (i % 4 == 0) ? 1.0f : 0.0f; }

/// exp([r]x) for a rotation vector r (radians).
inline void rodriguesExp(float rx, float ry, float rz, float R[9]) {
    const float th = std::sqrt(rx * rx + ry * ry + rz * rz);
    if (th < 1e-7f) { R[0] = 1; R[1] = -rz; R[2] = ry; R[3] = rz; R[4] = 1; R[5] = -rx; R[6] = -ry; R[7] = rx; R[8] = 1; return; }
    const float kx = rx / th, ky = ry / th, kz = rz / th, c = std::cos(th), s = std::sin(th), v = 1.0f - c;
    R[0] = kx * kx * v + c;      R[1] = kx * ky * v - kz * s; R[2] = kx * kz * v + ky * s;
    R[3] = ky * kx * v + kz * s; R[4] = ky * ky * v + c;      R[5] = ky * kz * v - kx * s;
    R[6] = kz * kx * v - ky * s; R[7] = kz * ky * v + kx * s; R[8] = kz * kz * v + c;
}

inline void quatToMatrix(const float q[4], float R[9]) {  // q = (w,x,y,z), rotates body -> world
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - z * w);     R[2] = 2 * (x * z + y * w);
    R[3] = 2 * (x * y + z * w);     R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - x * w);
    R[6] = 2 * (x * z - y * w);     R[7] = 2 * (y * z + x * w);     R[8] = 1 - 2 * (x * x + y * y);
}

// ------------------------------------------------------------------------------------------------
// IMU
// ------------------------------------------------------------------------------------------------

/**
 * Stores raw device-frame IMU samples (Android sensor frame: x right, y up, z out of the screen) and
 * expresses rotations in the *camera* frame using the camera's SENSOR_ORIENTATION (back-facing lens).
 *
 *   camera x (image right) = ( cos t, -sin t, 0) . device
 *   camera y (image down)  = (-sin t, -cos t, 0) . device
 *   camera z (forward)     = ( 0, 0, -1)         . device           with t = sensor orientation
 *
 * All rotations returned satisfy  d_start = R * d_end  (R = orientation of the camera at "end"
 * expressed in the camera frame at "start").
 */
class ImuFusionBuffer {
public:
    static constexpr size_t kRing = 8192;

    void setSensorOrientation(int degrees) {
        std::lock_guard<std::mutex> lk(mtx_);
        const float t = static_cast<float>(((degrees % 360) + 360) % 360) * 3.14159265358979f / 180.0f;
        const float c = std::round(std::cos(t)), s = std::round(std::sin(t));  // multiples of 90 deg
        M_[0] = c;  M_[1] = -s; M_[2] = 0;
        M_[3] = -s; M_[4] = -c; M_[5] = 0;
        M_[6] = 0;  M_[7] = 0;  M_[8] = -1;
    }
    /// Offset added to camera timestamps to bring them into the sensor-event time base.
    void setTimeOffsetNs(int64_t off) { timeOffsetNs_.store(off); }
    int64_t timeOffsetNs() const { return timeOffsetNs_.load(); }

    void clear() { std::lock_guard<std::mutex> lk(mtx_); gyroHead_ = quatHead_ = 0; }

    void pushGyroDevice(int64_t ts, float wx, float wy, float wz) {
        std::lock_guard<std::mutex> lk(mtx_);
        gyro_[gyroHead_++ % kRing] = {ts, wx, wy, wz};
    }
    void pushRotationDevice(int64_t ts, float w, float x, float y, float z) {
        std::lock_guard<std::mutex> lk(mtx_);
        const float n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-6f) return;
        quat_[quatHead_++ % kRing] = {ts, w / n, x / n, y / n, z / n};
    }
    size_t gyroCount() const { std::lock_guard<std::mutex> lk(mtx_); return std::min(gyroHead_, kRing); }
    size_t quatCount() const { std::lock_guard<std::mutex> lk(mtx_); return std::min(quatHead_, kRing); }

    /// Integrates the gyroscope (body-frame rates, trapezoid rule) between two camera timestamps.
    /// Returns false (R = identity) when there is no usable data.
    bool integrateGyro(int64_t camStartNs, int64_t camEndNs, float R[9]) const {
        identity3(R);
        const int64_t t0 = camStartNs + timeOffsetNs_.load(), t1 = camEndNs + timeOffsetNs_.load();
        if (t1 <= t0) return false;
        std::lock_guard<std::mutex> lk(mtx_);
        if (gyroHead_ < 2) return false;
        const size_t count = std::min(gyroHead_, kRing), first = gyroHead_ - count;
        // Samples are time-ordered; find the first sample strictly after t0.
        size_t lo = 0, hi = count;
        while (lo < hi) { size_t mid = (lo + hi) / 2; if (g(first + mid).ts <= t0) lo = mid + 1; else hi = mid; }
        if (lo >= count) return false;
        float pw[3];
        bool have = false;
        if (lo > 0) { camRate(g(first + lo - 1), pw); have = true; }
        int64_t prevT = t0;
        bool any = false;
        for (size_t i = lo; i < count; ++i) {
            const GyroSample& s = g(first + i);
            float cw[3]; camRate(s, cw);
            const int64_t curT = std::min(s.ts, t1);
            const float dt = static_cast<float>(curT - prevT) * 1e-9f;
            if (dt > 0.0f && dt < 0.1f) {
                const float ax = have ? 0.5f * (pw[0] + cw[0]) : cw[0], ay = have ? 0.5f * (pw[1] + cw[1]) : cw[1], az = have ? 0.5f * (pw[2] + cw[2]) : cw[2];
                float dR[9], nR[9];
                rodriguesExp(ax * dt, ay * dt, az * dt, dR);
                matMul3(R, dR, nR);  // body-frame increments multiply on the right
                std::memcpy(R, nR, sizeof(nR));
                any = true;
            }
            prevT = curT; pw[0] = cw[0]; pw[1] = cw[1]; pw[2] = cw[2]; have = true;
            if (s.ts >= t1) break;
        }
        return any;
    }

    /// Same quantity from the fused rotation-vector (absolute orientation) samples.
    bool relativeRotation(int64_t camRefNs, int64_t camCurNs, float R[9]) const {
        identity3(R);
        float qa[4], qb[4];
        if (!interpolate(camRefNs + timeOffsetNs_.load(), qa) || !interpolate(camCurNs + timeOffsetNs_.load(), qb)) return false;
        float Ra[9], Rb[9], RaT[9], Rd[9], tmp[9], MT[9];
        quatToMatrix(qa, Ra); quatToMatrix(qb, Rb); transpose3(Ra, RaT);
        matMul3(RaT, Rb, Rd);                // d_dev_ref = Ra^T Rb d_dev_cur
        float M[9]; { std::lock_guard<std::mutex> lk(mtx_); std::memcpy(M, M_, sizeof(M)); }
        transpose3(M, MT);
        matMul3(M, Rd, tmp); matMul3(tmp, MT, R);
        return true;
    }

private:
    struct GyroSample { int64_t ts; float x, y, z; };
    struct QuatSample { int64_t ts; float w, x, y, z; };
    const GyroSample& g(size_t absIdx) const { return gyro_[absIdx % kRing]; }
    void camRate(const GyroSample& s, float out[3]) const {
        out[0] = M_[0] * s.x + M_[1] * s.y + M_[2] * s.z;
        out[1] = M_[3] * s.x + M_[4] * s.y + M_[5] * s.z;
        out[2] = M_[6] * s.x + M_[7] * s.y + M_[8] * s.z;
    }
    bool interpolate(int64_t t, float out[4]) const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (quatHead_ == 0) return false;
        const size_t count = std::min(quatHead_, kRing), first = quatHead_ - count;
        size_t lo = 0, hi = count;  // first sample with ts > t
        while (lo < hi) { size_t mid = (lo + hi) / 2; if (quat_[(first + mid) % kRing].ts <= t) lo = mid + 1; else hi = mid; }
        const QuatSample* after = lo < count ? &quat_[(first + lo) % kRing] : nullptr;
        const QuatSample* before = lo > 0 ? &quat_[(first + lo - 1) % kRing] : nullptr;
        if (!before && !after) return false;
        if (!before) { out[0] = after->w; out[1] = after->x; out[2] = after->y; out[3] = after->z; return true; }
        if (!after) { out[0] = before->w; out[1] = before->x; out[2] = before->y; out[3] = before->z; return true; }
        const double dt = static_cast<double>(after->ts - before->ts);
        const float a = dt > 1.0 ? static_cast<float>(std::max(0.0, std::min(1.0, (t - before->ts) / dt))) : 0.0f;
        float q1[4] = {before->w, before->x, before->y, before->z}, q2[4] = {after->w, after->x, after->y, after->z};
        float dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
        if (dot < 0.0f) { dot = -dot; for (float& v : q2) v = -v; }
        if (dot > 0.9995f) { for (int k = 0; k < 4; ++k) out[k] = q1[k] + a * (q2[k] - q1[k]); }
        else {
            const float th0 = std::acos(dot), th = th0 * a, s0 = std::sin(th0), st = std::sin(th);
            const float c1 = std::cos(th) - dot * st / s0, c2 = st / s0;
            for (int k = 0; k < 4; ++k) out[k] = c1 * q1[k] + c2 * q2[k];
        }
        const float n = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
        if (n > 1e-6f) for (int k = 0; k < 4; ++k) out[k] /= n;
        return true;
    }

    mutable std::mutex mtx_;
    GyroSample gyro_[kRing]{};
    QuatSample quat_[kRing]{};
    size_t gyroHead_ = 0, quatHead_ = 0;
    float M_[9] = {0, -1, 0, -1, 0, 0, 0, 0, -1};  // sensor orientation 90 until configured
    std::atomic<int64_t> timeOffsetNs_{0};
};

inline const std::vector<uint8_t>& srgbLut() {  // linear 0..255 (1/16 steps) -> sRGB 8-bit
    static const std::vector<uint8_t> lut = [] {
        std::vector<uint8_t> l(255 * 16 + 1);
        for (size_t i = 0; i < l.size(); ++i) {
            const double v = static_cast<double>(i) / (255.0 * 16.0);
            const double s = v <= 0.0031308 ? 12.92 * v : 1.055 * std::pow(v, 1.0 / 2.4) - 0.055;
            l[i] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, s * 255.0 + 0.5)));
        }
        return l;
    }();
    return lut;
}

// ------------------------------------------------------------------------------------------------
// Microscope (burst super-resolution)
// ------------------------------------------------------------------------------------------------

class MicroscopeEngine {
public:
    explicit MicroscopeEngine(ImuFusionBuffer& imu) : imu_(imu) {}

    /// fx, fy: focal length in pixels of the *full* frame. cfa: SENSOR_INFO_COLOR_FILTER_ARRANGEMENT (0..3).
    void configure(float fx, float fy, int cfa, int blackLevel, int whiteLevel) {
        std::lock_guard<std::mutex> lk(mtx_);
        fx_ = fx; fy_ = fy; cfa_ = std::max(0, std::min(3, cfa));
        black_ = blackLevel; white_ = std::max(blackLevel + 1, whiteLevel);
    }

    /// Prepares accumulators. The processed region is a centred crop whose 2x output has at most
    /// `maxOutMegapixels` pixels, which bounds native memory on any device.
    void reset(int fullW, int fullH, float maxOutMegapixels) {
        std::lock_guard<std::mutex> lk(mtx_);
        releaseLocked();
        fullW_ = fullW; fullH_ = fullH;
        const double budgetIn = std::max(0.25, static_cast<double>(maxOutMegapixels)) * 1e6 / 4.0;
        const double full = static_cast<double>(fullW) * fullH;
        const double s = full <= budgetIn ? 1.0 : std::sqrt(budgetIn / full);
        roiW_ = std::max(16, (static_cast<int>(fullW * s) / 4) * 4);
        roiH_ = std::max(16, (static_cast<int>(fullH * s) / 4) * 4);
        roiW_ = std::min(roiW_, fullW & ~3); roiH_ = std::min(roiH_, fullH & ~3);
        roiX_ = ((fullW - roiW_) / 2) & ~1; roiY_ = ((fullH - roiH_) / 2) & ~1;
        outW_ = roiW_ * 2; outH_ = roiH_ * 2;
        const size_t n = static_cast<size_t>(outW_) * outH_;
        for (int c = 0; c < 3; ++c) { acc_[c].assign(n, 0.0f); wgt_[c].assign(n, 0.0f); }
        refLuma_.clear(); base_.clear(); frames_ = 0; wbReady_ = false; finalized_ = false; rawLinear_ = false;
    }

    int outputWidth() const { return outW_; }
    int outputHeight() const { return outH_; }
    int frames() const { return frames_; }
    /// Last estimated residual image shift (pixels, input grid) — useful diagnostics / tests.
    float lastShiftX() const { return lastDx_; }
    float lastShiftY() const { return lastDy_; }
    bool imageRefine = true;  // test hook: false => gyro-only registration

    int addRaw16(const uint8_t* raw, int strideBytes, int width, int height, int64_t tsNs) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!ready(width, height) || finalized_) return -1;
        rawLinear_ = true;
        const float scale = 255.0f / static_cast<float>(white_ - black_);
        std::vector<float> norm(static_cast<size_t>(roiW_) * roiH_);
        double sum[3] = {0, 0, 0}; size_t cnt[3] = {1, 1, 1};
        for (int y = 0; y < roiH_; ++y) {
            const auto* row = reinterpret_cast<const uint16_t*>(raw + static_cast<size_t>(y + roiY_) * strideBytes) + roiX_;
            for (int x = 0; x < roiW_; ++x) {
                const float v = std::max(0.0f, std::min(255.0f, (static_cast<int>(row[x]) - black_) * scale));
                norm[static_cast<size_t>(y) * roiW_ + x] = v;
                const int c = cfaChannel(x + roiX_, y + roiY_);
                sum[c] += v; cnt[c]++;
            }
        }
        if (!wbReady_) {  // gray-world white balance from the first frame, reused for the whole burst
            const float mR = static_cast<float>(sum[0] / cnt[0]), mG = static_cast<float>(sum[1] / cnt[1]), mB = static_cast<float>(sum[2] / cnt[2]);
            wb_[0] = mR > 1.0f ? std::max(0.5f, std::min(4.0f, mG / mR)) : 1.0f; wb_[1] = 1.0f;
            wb_[2] = mB > 1.0f ? std::max(0.5f, std::min(4.0f, mG / mB)) : 1.0f; wbReady_ = true;
        }
        // Green-channel estimate (edge-directed) used only for registration.
        std::vector<float> green(norm.size(), 0.0f);
        parallelFor(1, roiH_ - 1, [&](int y) {
            for (int x = 1; x < roiW_ - 1; ++x) {
                const size_t i = static_cast<size_t>(y) * roiW_ + x;
                if (cfaChannel(x + roiX_, y + roiY_) == 1) green[i] = norm[i];
                else {
                    const float gh = 0.5f * (norm[i - 1] + norm[i + 1]), gv = 0.5f * (norm[i - roiW_] + norm[i + roiW_]);
                    const float dh = std::abs(norm[i - 1] - norm[i + 1]), dv = std::abs(norm[i - roiW_] - norm[i + roiW_]);
                    green[i] = dh < dv ? gh : (dv < dh ? gv : 0.5f * (gh + gv));
                }
            }
        });
        if (frames_ == 0) makeBaselineFromBayer(norm);
        float Hacc[9];
        register_(green, tsNs, Hacc);
        splatBayer(norm, Hacc);
        return ++frames_;
    }

    /// uP/vP may be null for luma-only operation.
    int addYuv(const uint8_t* yP, int yS, const uint8_t* uP, int uS, int uPS, const uint8_t* vP, int vS, int vPS,
               int width, int height, int64_t tsNs) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!ready(width, height) || finalized_) return -1;
        rawLinear_ = false;
        std::vector<float> luma(static_cast<size_t>(roiW_) * roiH_);
        for (int y = 0; y < roiH_; ++y) {
            const uint8_t* r = yP + static_cast<size_t>(y + roiY_) * yS + roiX_;
            for (int x = 0; x < roiW_; ++x) luma[static_cast<size_t>(y) * roiW_ + x] = r[x];
        }
        const bool hasC = uP && vP;
        if (frames_ == 0) makeBaselineFromYuv(luma, uP, uS, uPS, vP, vS, vPS, hasC);
        float Hacc[9];
        register_(luma, tsNs, Hacc);
        for (int y = 1; y < roiH_ - 1; ++y) {
            const int ay = y + roiY_;
            const uint8_t* rU = hasC ? uP + static_cast<size_t>(ay >> 1) * uS : nullptr;
            const uint8_t* rV = hasC ? vP + static_cast<size_t>(ay >> 1) * vS : nullptr;
            for (int x = 1; x < roiW_ - 1; ++x) {
                float gx, gy;
                if (!mapToRef(Hacc, x, y, gx, gy)) continue;
                const int ix = static_cast<int>(std::floor(gx)), iy = static_cast<int>(std::floor(gy));
                if (ix < 0 || ix + 1 >= outW_ || iy < 0 || iy + 1 >= outH_) continue;
                const float Y = luma[static_cast<size_t>(y) * roiW_ + x];
                float val[3] = {Y, Y, Y};
                if (hasC) {
                    const int ax = x + roiX_;
                    const float U = static_cast<float>(rU[(ax >> 1) * uPS]) - 128.0f, V = static_cast<float>(rV[(ax >> 1) * vPS]) - 128.0f;
                    val[0] = std::max(0.0f, std::min(255.0f, Y + 1.402f * V));
                    val[1] = std::max(0.0f, std::min(255.0f, Y - 0.344136f * U - 0.714136f * V));
                    val[2] = std::max(0.0f, std::min(255.0f, Y + 1.772f * U));
                }
                const float fx = gx - ix, fy = gy - iy;
                const float w00 = (1 - fx) * (1 - fy), w10 = fx * (1 - fy), w01 = (1 - fx) * fy, w11 = fx * fy;
                const size_t b = static_cast<size_t>(iy) * outW_ + ix;
                for (int c = 0; c < 3; ++c) {
                    acc_[c][b] += val[c] * w00;          wgt_[c][b] += w00;
                    acc_[c][b + 1] += val[c] * w10;      wgt_[c][b + 1] += w10;
                    acc_[c][b + outW_] += val[c] * w01;  wgt_[c][b + outW_] += w01;
                    acc_[c][b + outW_ + 1] += val[c] * w11; wgt_[c][b + outW_ + 1] += w11;
                }
            }
        }
        return ++frames_;
    }

    /// Normalises, fills holes and applies a light Richardson-Lucy sharpen. Memory-lean: peak 8 planes.
    bool finalize(int deconvIterations = 8) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (frames_ == 0 || outW_ == 0) return false;
        if (finalized_) return true;
        const size_t n = static_cast<size_t>(outW_) * outH_;
        for (int c = 0; c < 3; ++c)
            for (size_t i = 0; i < n; ++i) acc_[c][i] = wgt_[c][i] > 1e-4f ? acc_[c][i] / wgt_[c][i] : 0.0f;
        std::vector<float> tmp(n);
        for (int pass = 0; pass < 3; ++pass)
            for (int c = 0; c < 3; ++c) fillHoles(acc_[c], wgt_[c], tmp);
        for (auto& w : wgt_) { std::vector<float>().swap(w); }
        luma_.resize(n);
        for (size_t i = 0; i < n; ++i) luma_[i] = 0.299f * acc_[0][i] + 0.587f * acc_[1][i] + 0.114f * acc_[2][i];
        {
            std::vector<float> obs = luma_, blur(n), corr(n);
            for (int it = 0; it < deconvIterations; ++it) {
                blurSeparable(luma_, blur, tmp);
                for (size_t i = 0; i < n; ++i) blur[i] = blur[i] > 1.0f ? obs[i] / blur[i] : 1.0f;
                blurSeparable(blur, corr, tmp);
                for (size_t i = 0; i < n; ++i) luma_[i] = std::max(0.0f, std::min(255.0f, luma_[i] * corr[i]));
            }
        }
        finalized_ = true;
        return true;
    }

    /// Writes RGBA8888 (byte order R,G,B,A == Android ARGB_8888 Bitmap memory layout).
    bool exportRgba(uint8_t* dst, int strideBytes) const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!finalized_) return false;
        const std::vector<uint8_t>& lut = srgbLut();
        parallelFor(0, outH_, [&](int y) {
            uint8_t* o = dst + static_cast<size_t>(y) * strideBytes;
            for (int x = 0; x < outW_; ++x) {
                const size_t i = static_cast<size_t>(y) * outW_ + x;
                const float r = acc_[0][i], g = acc_[1][i], b = acc_[2][i];
                const float l0 = 0.299f * r + 0.587f * g + 0.114f * b;
                const float gain = l0 > 1.0f ? luma_[i] / l0 : 1.0f;  // sharpen luminance, keep chroma
                const float v[3] = {r * gain, g * gain, b * gain};
                for (int c = 0; c < 3; ++c) {
                    const float cl = std::max(0.0f, std::min(255.0f, v[c]));
                    o[x * 4 + c] = rawLinear_ ? lut[static_cast<size_t>(cl * 16.0f + 0.5f)] : static_cast<uint8_t>(cl + 0.5f);
                }
                o[x * 4 + 3] = 255;
            }
        });
        return true;
    }

    /// Single-frame reference ("before"): the first burst frame, demosaiced / converted at 1x on the same crop.
    int baselineWidth() const { return roiW_; }
    int baselineHeight() const { return roiH_; }
    bool exportBaselineRgba(uint8_t* dst, int strideBytes) const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (base_.empty()) return false;
        for (int y = 0; y < roiH_; ++y) {
            uint8_t* o = dst + static_cast<size_t>(y) * strideBytes;
            const uint8_t* s = base_.data() + static_cast<size_t>(y) * roiW_ * 3;
            for (int x = 0; x < roiW_; ++x) { o[x * 4] = s[x * 3]; o[x * 4 + 1] = s[x * 3 + 1]; o[x * 4 + 2] = s[x * 3 + 2]; o[x * 4 + 3] = 255; }
        }
        return true;
    }

    void release() { std::lock_guard<std::mutex> lk(mtx_); releaseLocked(); }

private:
    bool ready(int w, int h) const { return outW_ > 0 && w == fullW_ && h == fullH_; }
    void releaseLocked() {
        for (int c = 0; c < 3; ++c) { std::vector<float>().swap(acc_[c]); std::vector<float>().swap(wgt_[c]); }
        std::vector<float>().swap(luma_); std::vector<float>().swap(refLuma_); std::vector<uint8_t>().swap(base_);
        outW_ = outH_ = 0; frames_ = 0; finalized_ = false;
    }
    int cfaChannel(int x, int y) const {  // 0=R 1=G 2=B for RGGB, GRBG, GBRG, BGGR
        const int px = x & 1, py = y & 1;
        switch (cfa_) {
            case 0: return py == 0 ? (px == 0 ? 0 : 1) : (px == 0 ? 1 : 2);
            case 1: return py == 0 ? (px == 0 ? 1 : 0) : (px == 0 ? 2 : 1);
            case 2: return py == 0 ? (px == 0 ? 1 : 2) : (px == 0 ? 0 : 1);
            default: return py == 0 ? (px == 0 ? 2 : 1) : (px == 0 ? 1 : 0);
        }
    }

    /// Maps a current-frame pixel (ROI coords) into reference-frame coordinates, then to the 2x output grid.
    static bool mapToRef(const float H[9], int x, int y, float& gx, float& gy) {
        const float w = H[6] * x + H[7] * y + H[8];
        if (std::abs(w) < 1e-5f) return false;
        gx = ((H[0] * x + H[1] * y + H[2]) / w) * 2.0f + 0.5f;  // +0.5: pixel centres onto the 2x grid
        gy = ((H[3] * x + H[4] * y + H[5]) / w) * 2.0f + 0.5f;
        return true;
    }

    /**
     * Estimates how the current frame relates to the reference frame.
     * Forward model (ref -> cur):  u = Hf * x + d,   Hf = K Rb^T K^-1  (Rb from the gyroscope),
     * d refined by Gauss-Newton on the central region. Output Hacc maps cur -> ref:  x = Hr * (u - d).
     */
    void register_(const std::vector<float>& cur, int64_t tsNs, float Hacc[9]) {
        float Rb[9];
        const float cx = 0.5f * fullW_ - roiX_, cy = 0.5f * fullH_ - roiY_;
        const float fx = fx_ > 50.0f ? fx_ : 0.8f * fullW_, fy = fy_ > 50.0f ? fy_ : fx;
        std::vector<float> curB = smooth3(cur);
        if (frames_ == 0 || refLuma_.empty()) {
            refLuma_ = smooth3(cur); refTs_ = tsNs; lastDx_ = lastDy_ = 0;  // registration works on a lightly smoothed copy
            identity3(Hacc);
            return;
        }
        if (!imu_.integrateGyro(refTs_, tsNs, Rb)) identity3(Rb);
        float K[9] = {fx, 0, cx, 0, fy, cy, 0, 0, 1}, Ki[9] = {1 / fx, 0, -cx / fx, 0, 1 / fy, -cy / fy, 0, 0, 1};
        float RbT[9], t1[9], Hf[9], Hr[9];
        transpose3(Rb, RbT);
        matMul3(K, RbT, t1); matMul3(t1, Ki, Hf);
        matMul3(K, Rb, t1); matMul3(t1, Ki, Hr);

        float dx = 0, dy = 0;
        if (imageRefine) {
            const int rw = std::min(384, roiW_ - 8), rh = std::min(384, roiH_ - 8), x0 = (roiW_ - rw) / 2, y0 = (roiH_ - rh) / 2;
            for (int iter = 0; iter < 8; ++iter) {
                double sXX = 0, sXY = 0, sYY = 0, sXT = 0, sYT = 0; int used = 0;
                for (int y = y0; y < y0 + rh; ++y)
                    for (int x = x0; x < x0 + rw; ++x) {
                        const size_t i = static_cast<size_t>(y) * roiW_ + x;
                        const float gX = 0.5f * (refLuma_[i + 1] - refLuma_[i - 1]), gY = 0.5f * (refLuma_[i + roiW_] - refLuma_[i - roiW_]);
                        if (gX * gX + gY * gY < 16.0f) continue;
                        const float w = Hf[6] * x + Hf[7] * y + Hf[8];
                        const float u = (Hf[0] * x + Hf[1] * y + Hf[2]) / w + dx, v = (Hf[3] * x + Hf[4] * y + Hf[5]) / w + dy;
                        const int iu = static_cast<int>(std::floor(u)), iv = static_cast<int>(std::floor(v));
                        if (iu < 0 || iu + 1 >= roiW_ || iv < 0 || iv + 1 >= roiH_) continue;
                        const float fu = u - iu, fv = v - iv;
                        const size_t j = static_cast<size_t>(iv) * roiW_ + iu;
                        const float cv = (1 - fu) * (1 - fv) * curB[j] + fu * (1 - fv) * curB[j + 1] + (1 - fu) * fv * curB[j + roiW_] + fu * fv * curB[j + roiW_ + 1];
                        const float r = cv - refLuma_[i];
                        sXX += gX * gX; sXY += gX * gY; sYY += gY * gY; sXT += gX * r; sYT += gY * r; ++used;
                    }
                const double det = sXX * sYY - sXY * sXY;
                if (used < 64 || std::abs(det) < 1e-3) break;
                // residual r ~ g . (d - s):  solve A*delta = b, then d -= delta
                const float ddx = static_cast<float>((sYY * sXT - sXY * sYT) / det), ddy = static_cast<float>((sXX * sYT - sXY * sXT) / det);
                dx = std::max(-6.0f, std::min(6.0f, dx - ddx)); dy = std::max(-6.0f, std::min(6.0f, dy - ddy));
                if (ddx * ddx + ddy * ddy < 1e-6f) break;
            }
        }
        lastDx_ = dx; lastDy_ = dy;
        // x = Hr (u - d)  ==  (Hr * T(-d)) u
        const float Tn[9] = {1, 0, -dx, 0, 1, -dy, 0, 0, 1};
        matMul3(Hr, Tn, Hacc);
    }

    void makeBaselineFromBayer(const std::vector<float>& norm) {
        base_.assign(static_cast<size_t>(roiW_) * roiH_ * 3, 0);
        const std::vector<uint8_t>& lut = srgbLut();
        for (int y = 0; y < roiH_; ++y)
            for (int x = 0; x < roiW_; ++x) {
                float sum[3] = {0, 0, 0}; int cnt[3] = {0, 0, 0};
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int xx = x + dx, yy = y + dy;
                        if (xx < 0 || yy < 0 || xx >= roiW_ || yy >= roiH_) continue;
                        const int c = cfaChannel(xx + roiX_, yy + roiY_);
                        sum[c] += norm[static_cast<size_t>(yy) * roiW_ + xx]; ++cnt[c];
                    }
                uint8_t* o = base_.data() + (static_cast<size_t>(y) * roiW_ + x) * 3;
                for (int c = 0; c < 3; ++c) {
                    const float v = cnt[c] ? sum[c] / cnt[c] * wb_[c] : 0.0f;
                    o[c] = lut[static_cast<size_t>(std::max(0.0f, std::min(255.0f, v)) * 16.0f + 0.5f)];
                }
            }
    }
    void makeBaselineFromYuv(const std::vector<float>& luma, const uint8_t* uP, int uS, int uPS, const uint8_t* vP, int vS, int vPS, bool hasC) {
        base_.assign(static_cast<size_t>(roiW_) * roiH_ * 3, 0);
        for (int y = 0; y < roiH_; ++y)
            for (int x = 0; x < roiW_; ++x) {
                const float Y = luma[static_cast<size_t>(y) * roiW_ + x];
                float r = Y, g = Y, b = Y;
                if (hasC) {
                    const int ax = x + roiX_, ay = y + roiY_;
                    const float U = static_cast<float>(uP[static_cast<size_t>(ay >> 1) * uS + (ax >> 1) * uPS]) - 128.0f;
                    const float V = static_cast<float>(vP[static_cast<size_t>(ay >> 1) * vS + (ax >> 1) * vPS]) - 128.0f;
                    r = Y + 1.402f * V; g = Y - 0.344136f * U - 0.714136f * V; b = Y + 1.772f * U;
                }
                uint8_t* o = base_.data() + (static_cast<size_t>(y) * roiW_ + x) * 3;
                o[0] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, r)) + 0.5f);
                o[1] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, g)) + 0.5f);
                o[2] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, b)) + 0.5f);
            }
    }

    /// Separable [1 2 1]/4 smoothing (registration only): widens the Gauss-Newton basin of convergence.
    std::vector<float> smooth3(const std::vector<float>& in) const {
        const int w = roiW_, h = roiH_;
        std::vector<float> t(in.size()), o(in.size());
        for (int y = 0; y < h; ++y) {
            const float* s = in.data() + static_cast<size_t>(y) * w; float* d = t.data() + static_cast<size_t>(y) * w;
            d[0] = s[0]; d[w - 1] = s[w - 1];
            for (int x = 1; x < w - 1; ++x) d[x] = 0.25f * s[x - 1] + 0.5f * s[x] + 0.25f * s[x + 1];
        }
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                const size_t i = static_cast<size_t>(y) * w + x;
                o[i] = (y > 0 && y < h - 1) ? 0.25f * t[i - w] + 0.5f * t[i] + 0.25f * t[i + w] : t[i];
            }
        return o;
    }

    void splatBayer(const std::vector<float>& norm, const float Hacc[9]) {
        for (int y = 2; y < roiH_ - 2; ++y)
            for (int x = 2; x < roiW_ - 2; ++x) {
                float gx, gy;
                if (!mapToRef(Hacc, x, y, gx, gy)) continue;
                const int ix = static_cast<int>(std::floor(gx)), iy = static_cast<int>(std::floor(gy));
                if (ix < 1 || ix + 2 >= outW_ || iy < 1 || iy + 2 >= outH_) continue;
                const int c = cfaChannel(x + roiX_, y + roiY_);
                const float val = std::min(255.0f, norm[static_cast<size_t>(y) * roiW_ + x] * wb_[c]);
                for (int ky = 0; ky <= 1; ++ky)
                    for (int kx = 0; kx <= 1; ++kx) {
                        const float rx = gx - (ix + kx), ry = gy - (iy + ky), wt = std::exp(-1.1f * (rx * rx + ry * ry));
                        const size_t idx = static_cast<size_t>(iy + ky) * outW_ + (ix + kx);
                        acc_[c][idx] += val * wt; wgt_[c][idx] += wt;
                    }
            }
    }

    void fillHoles(std::vector<float>& plane, std::vector<float>& weights, std::vector<float>& next) const {
        next = plane;
        const int w = outW_, h = outH_;
        parallelFor(1, h - 1, [&](int y) {
            for (int x = 1; x < w - 1; ++x) {
                const size_t idx = static_cast<size_t>(y) * w + x;
                if (weights[idx] > 1e-4f) continue;
                float sum = 0; int cnt = 0;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const size_t nIdx = static_cast<size_t>(y + dy) * w + (x + dx);
                        if (weights[nIdx] > 1e-4f) { sum += plane[nIdx]; ++cnt; }
                    }
                if (cnt > 0) next[idx] = sum / cnt;
            }
        });
        // mark filled pixels as valid for the next pass (serial: cheap compared with the stencil)
        for (size_t i = 0; i < next.size(); ++i) if (weights[i] <= 1e-4f && next[i] != plane[i]) weights[i] = 0.5f;
        plane.swap(next);
    }

    void blurSeparable(const std::vector<float>& src, std::vector<float>& dst, std::vector<float>& tmp) const {
        static constexpr float k[5] = {0.06136f, 0.24477f, 0.38774f, 0.24477f, 0.06136f};  // sigma = 1 px
        const int w = outW_, h = outH_;
        dst = src;  // keeps the 2-px border valid
        parallelFor(0, h, [&](int y) {
            const float* s = src.data() + static_cast<size_t>(y) * w;
            float* t = tmp.data() + static_cast<size_t>(y) * w;
            t[0] = s[0]; t[1] = s[1]; t[w - 2] = s[w - 2]; t[w - 1] = s[w - 1];
            for (int x = 2; x < w - 2; ++x) t[x] = k[0] * s[x - 2] + k[1] * s[x - 1] + k[2] * s[x] + k[3] * s[x + 1] + k[4] * s[x + 2];
        });
        parallelFor(2, h - 2, [&](int y) {
            const size_t r = static_cast<size_t>(y) * w;
            for (int x = 0; x < w; ++x)
                dst[r + x] = k[0] * tmp[r - 2 * w + x] + k[1] * tmp[r - w + x] + k[2] * tmp[r + x] + k[3] * tmp[r + w + x] + k[4] * tmp[r + 2 * w + x];
        });
    }

    ImuFusionBuffer& imu_;
    mutable std::mutex mtx_;
    int fullW_ = 0, fullH_ = 0, roiX_ = 0, roiY_ = 0, roiW_ = 0, roiH_ = 0, outW_ = 0, outH_ = 0, frames_ = 0;
    int64_t refTs_ = 0;
    float fx_ = 0, fy_ = 0, lastDx_ = 0, lastDy_ = 0, wb_[3] = {1, 1, 1};
    int cfa_ = 0, black_ = 0, white_ = 1023;
    bool wbReady_ = false, finalized_ = false, rawLinear_ = false;
    std::vector<float> refLuma_, acc_[3], wgt_[3], luma_;
    std::vector<uint8_t> base_;
};

// ------------------------------------------------------------------------------------------------
// Stereo + TSDF
// ------------------------------------------------------------------------------------------------

struct Intrinsics { float fx = 0, fy = 0, cx = 0, cy = 0; float dist[5] = {0, 0, 0, 0, 0}; };  // dist: OpenCV k1,k2,p1,p2,k3

/// Cam-0 -> cam-1 extrinsics:  X1 = R * X0 + T  (T in millimetres). Camera 1's centre in cam-0 coordinates is -R^T T.
struct StereoConfig {
    Intrinsics left, right;
    float R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float T_mm[3] = {-18.5f, 0, 0};
    bool calibrated = false;  // true: extrinsics come from the device or a user file (no heuristic vertical shift)
};

struct Vertex3D { float p[3]; float n[3]; };
struct Tri { uint32_t a, b, c; };

struct TsdfVolume {
    int dim = 0;
    float voxelMm = 2.0f;
    float center[3] = {0, 0, 250.0f};
    std::vector<float> dist, weight;
    size_t idx(int x, int y, int z) const { return (static_cast<size_t>(z) * dim + y) * dim + x; }
};

/// Kuhn marching tetrahedra on a TSDF (distance > 0 outside / free space, < 0 inside). Triangles are wound
/// so that the geometric normal points toward increasing distance (out of the solid).
inline void extractMesh(const TsdfVolume& V, std::vector<Vertex3D>& outV, std::vector<Tri>& outT) {
    static constexpr int CV[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    static constexpr int KT[6][4] = {{0, 5, 1, 6}, {0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6}, {0, 7, 4, 6}, {0, 4, 5, 6}};
    const int dim = V.dim;
    const float half = 0.5f * dim * V.voxelMm;
    std::unordered_map<uint64_t, uint32_t> cache;
    auto vertexOn = [&](size_t iA, size_t iB, const int pA[3], const int pB[3], float dA, float dB) -> uint32_t {
        const uint64_t key = (static_cast<uint64_t>(std::min(iA, iB)) << 32) | std::max(iA, iB);
        auto it = cache.find(key);
        if (it != cache.end()) return it->second;
        const float t = std::abs(dA - dB) > 1e-9f ? std::max(0.0f, std::min(1.0f, dA / (dA - dB))) : 0.5f;
        const float g[3] = {pA[0] + t * (pB[0] - pA[0]), pA[1] + t * (pB[1] - pA[1]), pA[2] + t * (pB[2] - pA[2])};
        const int ix = std::max(1, std::min(dim - 2, static_cast<int>(std::lround(g[0]))));
        const int iy = std::max(1, std::min(dim - 2, static_cast<int>(std::lround(g[1]))));
        const int iz = std::max(1, std::min(dim - 2, static_cast<int>(std::lround(g[2]))));
        float n[3] = {V.dist[V.idx(ix + 1, iy, iz)] - V.dist[V.idx(ix - 1, iy, iz)],
                      V.dist[V.idx(ix, iy + 1, iz)] - V.dist[V.idx(ix, iy - 1, iz)],
                      V.dist[V.idx(ix, iy, iz + 1)] - V.dist[V.idx(ix, iy, iz - 1)]};
        const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        Vertex3D v{};
        for (int k = 0; k < 3; ++k) {
            v.p[k] = g[k] * V.voxelMm - half + (V.center[k]);
            v.n[k] = len > 1e-9f ? n[k] / len : 0.0f;  // gradient points toward larger distance = outward
        }
        if (len <= 1e-9f) v.n[2] = -1.0f;
        const uint32_t id = static_cast<uint32_t>(outV.size());
        outV.push_back(v); cache.emplace(key, id);
        return id;
    };

    for (int z = 1; z < dim - 2; ++z)
        for (int y = 1; y < dim - 2; ++y)
            for (int x = 1; x < dim - 2; ++x) {
                int cp[8][3]; size_t ci[8]; float cv[8]; int neg = 0; bool ok = true;
                for (int i = 0; i < 8 && ok; ++i) {
                    cp[i][0] = x + CV[i][0]; cp[i][1] = y + CV[i][1]; cp[i][2] = z + CV[i][2];
                    ci[i] = V.idx(cp[i][0], cp[i][1], cp[i][2]);
                    if (V.weight[ci[i]] < 1.0f) { ok = false; break; }
                    cv[i] = V.dist[ci[i]]; if (cv[i] < 0.0f) ++neg;
                }
                if (!ok || neg == 0 || neg == 8) continue;
                for (int t = 0; t < 6; ++t) {
                    const int tv[4] = {KT[t][0], KT[t][1], KT[t][2], KT[t][3]};
                    int in[4], out[4], ni = 0, no = 0;
                    for (int k = 0; k < 4; ++k) { if (cv[tv[k]] < 0.0f) in[ni++] = k; else out[no++] = k; }
                    if (ni == 0 || no == 0) continue;
                    uint32_t q[4]; int qa[4], qb[4]; int nq = 0;
                    auto vert = [&](int a, int b, int slot) { qa[slot] = a; qb[slot] = b; return vertexOn(ci[tv[a]], ci[tv[b]], cp[tv[a]], cp[tv[b]], cv[tv[a]], cv[tv[b]]); };
                    if (ni == 1 || no == 1) {  // one triangle
                        const int s = ni == 1 ? in[0] : out[0]; const int* o = ni == 1 ? out : in;
                        q[0] = vert(s, o[0], 0); q[1] = vert(s, o[1], 1); q[2] = vert(s, o[2], 2); nq = 3;
                    } else {                   // quad -> two triangles
                        q[0] = vert(in[0], out[0], 0); q[1] = vert(in[0], out[1], 1); q[2] = vert(in[1], out[1], 2); q[3] = vert(in[1], out[0], 3); nq = 4;
                    }
                    // Orientation is decided with *exact integer* arithmetic on edge midpoints (2x grid coordinates)
                    // against (mean outside - mean inside), so every tetrahedron winds consistently with its neighbours.
                    long long refv[3];
                    for (int d = 0; d < 3; ++d) {
                        long long sin_ = 0, sout = 0;
                        for (int k = 0; k < ni; ++k) sin_ += cp[tv[in[k]]][d];
                        for (int k = 0; k < no; ++k) sout += cp[tv[out[k]]][d];
                        refv[d] = ni * sout - no * sin_;
                    }
                    auto mid = [&](int slot, long long m[3]) { for (int d = 0; d < 3; ++d) m[d] = cp[tv[qa[slot]]][d] + cp[tv[qb[slot]]][d]; };
                    auto emit = [&](int sa, int sb, int sc) {
                        const uint32_t a = q[sa], b = q[sb], c = q[sc];
                        if (a == b || b == c || c == a) return;
                        long long A[3], B[3], C[3]; mid(sa, A); mid(sb, B); mid(sc, C);
                        const long long e1[3] = {B[0] - A[0], B[1] - A[1], B[2] - A[2]}, e2[3] = {C[0] - A[0], C[1] - A[1], C[2] - A[2]};
                        const long long gn[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
                        if (gn[0] * refv[0] + gn[1] * refv[1] + gn[2] * refv[2] >= 0) outT.push_back({a, b, c}); else outT.push_back({a, c, b});
                    };
                    emit(0, 1, 2);
                    if (nq == 4) emit(0, 2, 3);
                }
            }
}

class StereoTsdfEngine {
public:
    static constexpr int kDown = 4;        // stereo runs at 1/4 resolution
    static constexpr int kMaxDisp = 48;    // disparity search range at 1/4 resolution
    static constexpr int kVoxelDim = 96;

    explicit StereoTsdfEngine(ImuFusionBuffer& imu) : imu_(imu) { reset(); }

    void reset() {
        std::lock_guard<std::mutex> lk(mtx_);
        vol_ = TsdfVolume();
        volReady_ = false; frames_ = 0; verticalAligned_ = false;
    }
    void configure(const StereoConfig& cfg, int streamW, int streamH) {
        std::lock_guard<std::mutex> lk(mtx_);
        cfg_ = cfg; streamW_ = streamW; streamH_ = streamH; lutReady_ = false; verticalAligned_ = cfg.calibrated;
        buildRectification();  // derives baseline, rectified intrinsics, depth range, display rotation
    }
    struct Info { float baselineMm, zMinMm, zMaxMm, fRectPx; int rectRotationDeg, rectW, rectH; };
    Info info() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return {baselineMm_, zMin_, zMax_, rectFx_, rectRotDeg_, rectW_, rectH_};
    }
    int rectWidth() const { return rectW_; }
    int rectHeight() const { return rectH_; }
    int frames() const { return frames_; }

    /// outHeatmapArgb must hold rectWidth()*rectHeight() pixels (may be null).
    int process(const uint8_t* leftY, int leftStride, const uint8_t* rightY, int rightStride, int width, int height,
                int64_t tsNs, uint32_t* outHeatmapArgb, float* debugDepthMm = nullptr) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (width != streamW_ || height != streamH_ || !lutReady_) return frames_;
        const int dw = rectW_, dh = rectH_;
        rectL_.resize(static_cast<size_t>(dw) * dh); rectR_.resize(rectL_.size());
        remap(leftY, leftStride, mapLx_, rectL_);
        remap(rightY, rightStride, mapRx_, rectR_);

        if (!verticalAligned_) {  // heuristic only when the extrinsics are unknown
            const int dy = verticalOffset(rectL_.data(), rectR_.data(), dw, dh);
            if (dy != 0) { cfg_.right.cy += dy * static_cast<float>(kDown); buildRectification(); remap(rightY, rightStride, mapRx_, rectR_); }
            verticalAligned_ = true;
        }
        depth_.assign(rectL_.size(), 0.0f); disp_.assign(rectL_.size(), 0.0f);
        censusSgm(rectL_.data(), rectR_.data(), dw, dh);
        if (outHeatmapArgb) heatmap(outHeatmapArgb);
        if (debugDepthMm) std::memcpy(debugDepthMm, depth_.data(), depth_.size() * sizeof(float));

        float c[3];
        const bool validC = centroid(c);
        if (!volReady_) {
            if (!validC) return frames_;           // nothing to anchor the volume to yet
            initVolume(c);
            refTs_ = tsNs;
        }
        float Rrel[9], R[9];                       // R: d_ref = R d_cur ; TSDF needs X_cur = R^T X_ref
        if (!imu_.relativeRotation(refTs_, tsNs, R) && !imu_.integrateGyro(refTs_, tsNs, R)) identity3(R);
        transpose3(R, Rrel);
        float t[3] = {0, 0, 0};
        const float lim = 0.18f * kVoxelDim * vol_.voxelMm;
        if (validC) for (int k = 0; k < 3; ++k) t[k] = std::max(-lim, std::min(lim, c[k] - vol_.center[k]));
        integrate(Rrel, t);
        return ++frames_;
    }

    int exportMesh(const char* stlPath, const char* plyPath, int* vertsOut = nullptr) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!volReady_) return 0;
        std::vector<Vertex3D> v; std::vector<Tri> t;
        extractMesh(vol_, v, t);
        if (t.empty()) return 0;
        if (stlPath && stlPath[0]) writeStl(stlPath, v, t);
        if (plyPath && plyPath[0]) writePly(plyPath, v, t);
        if (vertsOut) *vertsOut = static_cast<int>(v.size());
        return static_cast<int>(t.size());
    }
    const TsdfVolume& volume() const { return vol_; }

private:
    // -- rectification (Bouguet) ---------------------------------------------------------------
    void buildRectification() {
        const float sx = static_cast<float>(kDown);
        const Intrinsics &L = cfg_.left, &Rr = cfg_.right;
        const float tr = std::max(-1.0f, std::min(1.0f, (cfg_.R[0] + cfg_.R[4] + cfg_.R[8] - 1.0f) * 0.5f));
        const float th = std::acos(tr);
        float om[3] = {0, 0, 0};
        if (th > 1e-5f) {
            const float s = th / (2.0f * std::sin(th));
            om[0] = (cfg_.R[7] - cfg_.R[5]) * s; om[1] = (cfg_.R[2] - cfg_.R[6]) * s; om[2] = (cfg_.R[3] - cfg_.R[1]) * s;
        }
        float rl[9], rr[9];
        rodriguesExp(0.5f * om[0], 0.5f * om[1], 0.5f * om[2], rl);
        rodriguesExp(-0.5f * om[0], -0.5f * om[1], -0.5f * om[2], rr);
        const float t[3] = {rr[0] * cfg_.T_mm[0] + rr[1] * cfg_.T_mm[1] + rr[2] * cfg_.T_mm[2],
                            rr[3] * cfg_.T_mm[0] + rr[4] * cfg_.T_mm[1] + rr[5] * cfg_.T_mm[2],
                            rr[6] * cfg_.T_mm[0] + rr[7] * cfg_.T_mm[1] + rr[8] * cfg_.T_mm[2]};
        const float nT = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
        float e1[3] = {1, 0, 0};
        if (nT > 1e-4f) { baselineMm_ = nT; for (int k = 0; k < 3; ++k) e1[k] = -t[k] / nT; }
        const float xy = std::hypot(e1[0], e1[1]);
        const float e2[3] = {xy > 1e-5f ? -e1[1] / xy : 0.0f, xy > 1e-5f ? e1[0] / xy : 1.0f, 0.0f};
        const float e3[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        const float Rrect[9] = {e1[0], e1[1], e1[2], e2[0], e2[1], e2[2], e3[0], e3[1], e3[2]};
        float R1[9], R2[9], R1i[9], R2i[9];
        matMul3(Rrect, rl, R1); matMul3(Rrect, rr, R2); transpose3(R1, R1i); transpose3(R2, R2i);

        // Baseline along the image's vertical axis => rectified image is rotated by ~90 deg: swap its extent.
        const float phi = std::atan2(e1[1], e1[0]) * 57.2957795f;
        rectRotDeg_ = (((static_cast<int>(std::lround(phi / 90.0f)) * 90) % 360) + 360) % 360;
        const bool swap = (rectRotDeg_ == 90 || rectRotDeg_ == 270);
        rectW_ = (swap ? streamH_ : streamW_) / kDown; rectH_ = (swap ? streamW_ : streamH_) / kDown;
        rectFx_ = L.fx / sx; rectFy_ = L.fy / sx; rectCx_ = 0.5f * rectW_; rectCy_ = 0.5f * rectH_;

        // usable depth range from the disparity search window
        zMin_ = std::max(30.0f, rectFx_ * baselineMm_ / static_cast<float>(kMaxDisp - 2));
        zMax_ = std::min(2500.0f, rectFx_ * baselineMm_ / 2.0f);

        const size_t n = static_cast<size_t>(rectW_) * rectH_;
        mapLx_.resize(2 * n); mapRx_.resize(2 * n);
        auto project = [](float xr, float yr, const float Ri[9], const Intrinsics& I, float& mx, float& my) {
            const float rx = Ri[0] * xr + Ri[1] * yr + Ri[2], ry = Ri[3] * xr + Ri[4] * yr + Ri[5], rz = Ri[6] * xr + Ri[7] * yr + Ri[8];
            const float xn = std::abs(rz) > 1e-6f ? rx / rz : 0.0f, yn = std::abs(rz) > 1e-6f ? ry / rz : 0.0f;
            const float r2 = xn * xn + yn * yn, rad = 1.0f + I.dist[0] * r2 + I.dist[1] * r2 * r2 + I.dist[4] * r2 * r2 * r2;
            mx = (xn * rad + 2 * I.dist[2] * xn * yn + I.dist[3] * (r2 + 2 * xn * xn)) * I.fx + I.cx;
            my = (yn * rad + I.dist[2] * (r2 + 2 * yn * yn) + 2 * I.dist[3] * xn * yn) * I.fy + I.cy;
        };
        for (int v = 0; v < rectH_; ++v)
            for (int u = 0; u < rectW_; ++u) {
                const float xr = (u - rectCx_) / rectFx_, yr = (v - rectCy_) / rectFy_;
                const size_t i = (static_cast<size_t>(v) * rectW_ + u) * 2;
                project(xr, yr, R1i, L, mapLx_[i], mapLx_[i + 1]);
                project(xr, yr, R2i, Rr, mapRx_[i], mapRx_[i + 1]);
            }
        lutReady_ = true;
    }

    /// 4x4 footprint average (4 bilinear taps) so 1/4-scale sampling does not alias.
    void remap(const uint8_t* src, int stride, const std::vector<float>& map, std::vector<uint8_t>& dst) const {
        const int dw = rectW_, dh = rectH_, sw = streamW_, sh = streamH_;
        parallelFor(0, dh, [&](int y) {
            for (int x = 0; x < dw; ++x) {
                const size_t mi = (static_cast<size_t>(y) * dw + x) * 2;
                const float u0 = map[mi], v0 = map[mi + 1];
                float acc = 0; int cnt = 0;
                for (int oy = -1; oy <= 1; oy += 2)
                    for (int ox = -1; ox <= 1; ox += 2) {
                        const float u = u0 + ox, v = v0 + oy;
                        const int iu = static_cast<int>(std::floor(u)), iv = static_cast<int>(std::floor(v));
                        if (iu < 0 || iu + 1 >= sw || iv < 0 || iv + 1 >= sh) continue;
                        const float fu = u - iu, fv = v - iv;
                        const uint8_t *r0 = src + static_cast<size_t>(iv) * stride + iu, *r1 = r0 + stride;
                        acc += (1 - fu) * (1 - fv) * r0[0] + fu * (1 - fv) * r0[1] + (1 - fu) * fv * r1[0] + fu * fv * r1[1];
                        ++cnt;
                    }
                dst[static_cast<size_t>(y) * dw + x] = cnt == 4 ? static_cast<uint8_t>(acc * 0.25f + 0.5f) : 0;
            }
        });
    }

    static int verticalOffset(const uint8_t* L, const uint8_t* R, int w, int h) {
        std::vector<float> pL(h, 0.0f), pR(h, 0.0f);
        for (int y = 1; y < h - 1; ++y) {
            float sL = 0, sR = 0;
            for (int x = w / 4; x < (3 * w) / 4; ++x) {
                sL += std::abs(static_cast<int>(L[(y + 1) * w + x]) - static_cast<int>(L[(y - 1) * w + x]));
                sR += std::abs(static_cast<int>(R[(y + 1) * w + x]) - static_cast<int>(R[(y - 1) * w + x]));
            }
            pL[y] = sL; pR[y] = sR;
        }
        int best = 0; float bestS = -1.0f;
        for (int dy = -6; dy <= 6; ++dy) {
            float s = 0;
            for (int y = 12; y < h - 12; ++y) { const int ty = y + dy; if (ty >= 0 && ty < h) s += pL[y] * pR[ty]; }
            if (s > bestS) { bestS = s; best = dy; }
        }
        return best;
    }

    // -- census + SGM --------------------------------------------------------------------------
    void censusSgm(const uint8_t* left, const uint8_t* right, int w, int h) {
        constexpr int D = kMaxDisp;
        cL_.assign(static_cast<size_t>(w) * h, 0); cR_.assign(cL_.size(), 0);
        parallelFor(2, h - 2, [&](int y) {
            for (int x = 2; x < w - 2; ++x) {
                const uint8_t vL = left[y * w + x], vR = right[y * w + x];
                uint32_t bL = 0, bR = 0; int bit = 0;
                for (int dy = -2; dy <= 2; ++dy)
                    for (int dx = -2; dx <= 2; ++dx) {
                        if (!dx && !dy) continue;
                        const int i = (y + dy) * w + (x + dx);
                        if (left[i] < vL) bL |= 1u << bit;
                        if (right[i] < vR) bR |= 1u << bit;
                        ++bit;
                    }
                cL_[static_cast<size_t>(y) * w + x] = bL; cR_[static_cast<size_t>(y) * w + x] = bR;
            }
        });
        const size_t vol = static_cast<size_t>(w) * h * D;
        base_.assign(vol, 24); agg_.assign(vol, 0);
        parallelFor(2, h - 2, [&](int y) {
            for (int x = 2; x < w - 2; ++x) {
                const uint32_t lb = cL_[static_cast<size_t>(y) * w + x];
                const size_t b = (static_cast<size_t>(y) * w + x) * D;
                for (int d = 0; d < D && x - d >= 2; ++d)
                    base_[b + d] = static_cast<uint8_t>(__builtin_popcount(lb ^ cR_[static_cast<size_t>(y) * w + (x - d)]));
            }
        });
        auto step = [&](uint16_t* path, uint16_t& prevMin, size_t idx, int grad) {
            constexpr int P1 = 6, P2B = 24;
            const int P2 = std::max(P1 + 2, P2B - (grad >> 2));
            uint16_t nxt = 0xffff, np[D];
            for (int d = 0; d < D; ++d) {
                int v = path[d];
                if (d > 0) v = std::min<int>(v, path[d - 1] + P1);
                if (d + 1 < D) v = std::min<int>(v, path[d + 1] + P1);
                v = std::min<int>(v, prevMin + P2);
                np[d] = static_cast<uint16_t>(base_[idx + d] + v - prevMin);
                agg_[idx + d] = static_cast<uint16_t>(agg_[idx + d] + np[d]);
                nxt = std::min(nxt, np[d]);
            }
            std::memcpy(path, np, sizeof(np)); prevMin = nxt;
        };
        parallelFor(2, h - 2, [&](int y) {
            uint16_t a[D]{}, b[D]{}, ma = 0, mb = 0;
            for (int x = 2; x < w - 2; ++x)
                step(a, ma, (static_cast<size_t>(y) * w + x) * D, std::abs(left[y * w + x] - left[y * w + x - 1]));
            for (int x = w - 3; x >= 2; --x)
                step(b, mb, (static_cast<size_t>(y) * w + x) * D, std::abs(left[y * w + x] - left[y * w + x + 1]));
        });
        parallelFor(2, w - 2, [&](int x) {
            uint16_t a[D]{}, b[D]{}, ma = 0, mb = 0;
            for (int y = 2; y < h - 2; ++y)
                step(a, ma, (static_cast<size_t>(y) * w + x) * D, std::abs(left[y * w + x] - left[(y - 1) * w + x]));
            for (int y = h - 3; y >= 2; --y)
                step(b, mb, (static_cast<size_t>(y) * w + x) * D, std::abs(left[y * w + x] - left[(y + 1) * w + x]));
        });
        parallelFor(2, h - 2, [&](int y) {
            for (int x = D; x < w - 2; ++x) {
                const size_t idx = (static_cast<size_t>(y) * w + x) * D;
                int bestD = -1; int bestC = 0x7fffffff;
                for (int d = 2; d < D - 1; ++d) if (agg_[idx + d] < bestC) { bestC = agg_[idx + d]; bestD = d; }
                if (bestD < 2 || bestC > 56) continue;
                int second = 0x7fffffff;  // best cost away from the winner (excludes the +-1 neighbourhood)
                for (int d = 2; d < D - 1; ++d) if (std::abs(d - bestD) > 1 && agg_[idx + d] < second) second = agg_[idx + d];
                if (bestC * 10 > second * 9) continue;  // uniqueness
                const int xr = x - bestD;
                int bestDr = -1; int minCr = 0x7fffffff;  // left-right consistency
                for (int dr = 2; dr < D - 1; ++dr)
                    if (xr + dr < w - 2) { const int cr = agg_[(static_cast<size_t>(y) * w + (xr + dr)) * D + dr]; if (cr < minCr) { minCr = cr; bestDr = dr; } }
                if (std::abs(bestD - bestDr) > 1) continue;
                const float cp = agg_[idx + bestD - 1], c0 = static_cast<float>(bestC), cn = agg_[idx + bestD + 1], den = 2.0f * (cp - 2.0f * c0 + cn);
                const float ds = bestD + (std::abs(den) > 1e-3f ? std::max(-0.5f, std::min(0.5f, (cp - cn) / den)) : 0.0f);
                const float z = rectFx_ * baselineMm_ / ds;
                if (z >= zMin_ * 0.9f && z <= zMax_) { disp_[static_cast<size_t>(y) * w + x] = ds; depth_[static_cast<size_t>(y) * w + x] = z; }
            }
        });
    }

    /// Foreground centroid of the central region: the nearest ~30% of valid depths (ignores a distant background).
    bool centroid(float out[3]) const {
        const int dw = rectW_, dh = rectH_;
        std::vector<float> zs;
        for (int y = dh / 4; y < (3 * dh) / 4; ++y)
            for (int x = dw / 4; x < (3 * dw) / 4; ++x) { const float z = depth_[static_cast<size_t>(y) * dw + x]; if (z > 0.0f) zs.push_back(z); }
        if (zs.size() < 50) return false;
        const size_t k = zs.size() * 3 / 10;
        std::nth_element(zs.begin(), zs.begin() + k, zs.end());
        const float p = zs[k], lo = 0.6f * p, hi = 1.25f * p;
        double sx = 0, sy = 0, sz = 0; int cnt = 0;
        for (int y = dh / 4; y < (3 * dh) / 4; ++y)
            for (int x = dw / 4; x < (3 * dw) / 4; ++x) {
                const float z = depth_[static_cast<size_t>(y) * dw + x];
                if (z >= lo && z <= hi) { sx += (x - rectCx_) * z / rectFx_; sy += (y - rectCy_) * z / rectFy_; sz += z; ++cnt; }
            }
        if (cnt < 30) return false;
        out[0] = static_cast<float>(sx / cnt); out[1] = static_cast<float>(sy / cnt); out[2] = static_cast<float>(sz / cnt);
        return true;
    }

    void heatmap(uint32_t* out) const {
        const int n = rectW_ * rectH_;
        for (int i = 0; i < n; ++i) {
            const float d = disp_[i];
            if (d <= 1.0f) { const uint32_t g = rectL_[i]; out[i] = 0xFF000000u | (g << 16) | (g << 8) | g; continue; }
            const float t = std::max(0.0f, std::min(1.0f, (d - 2.0f) / (kMaxDisp - 4.0f)));
            auto ch = [&](float k) { return static_cast<uint32_t>(255.0f * std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f * t - k)))); };
            out[i] = 0xFF000000u | (ch(3.0f) << 16) | (ch(2.0f) << 8) | ch(1.0f);  // blue (far) -> red (near)
        }
    }

    // -- TSDF ----------------------------------------------------------------------------------
    void initVolume(const float c[3]) {
        const float side = std::max(80.0f, std::min(500.0f, 0.9f * c[2]));  // volume spans ~ the field of view at that depth
        vol_.dim = kVoxelDim; vol_.voxelMm = side / kVoxelDim;
        for (int k = 0; k < 3; ++k) vol_.center[k] = c[k];
        const size_t total = static_cast<size_t>(kVoxelDim) * kVoxelDim * kVoxelDim;
        vol_.dist.assign(total, 1.0f); vol_.weight.assign(total, 0.0f);
        truncMm_ = 4.0f * vol_.voxelMm; volReady_ = true;
    }
    void integrate(const float Rrel[9], const float t[3]) {
        const float half = 0.5f * kVoxelDim * vol_.voxelMm;
        const int dw = rectW_, dh = rectH_;
        parallelFor(0, kVoxelDim, [&](int vz) {
            const float dz = vz * vol_.voxelMm - half;
            for (int vy = 0; vy < kVoxelDim; ++vy) {
                const float dy = vy * vol_.voxelMm - half;
                for (int vx = 0; vx < kVoxelDim; ++vx) {
                    const float dx = vx * vol_.voxelMm - half;
                    const float pcX = Rrel[0] * dx + Rrel[1] * dy + Rrel[2] * dz + vol_.center[0] + t[0];
                    const float pcY = Rrel[3] * dx + Rrel[4] * dy + Rrel[5] * dz + vol_.center[1] + t[1];
                    const float pcZ = Rrel[6] * dx + Rrel[7] * dy + Rrel[8] * dz + vol_.center[2] + t[2];
                    if (pcZ <= 20.0f) continue;
                    const int u = static_cast<int>(std::lround(pcX * rectFx_ / pcZ + rectCx_)), v = static_cast<int>(std::lround(pcY * rectFy_ / pcZ + rectCy_));
                    if (u < 0 || u >= dw || v < 0 || v >= dh) continue;
                    const float mz = depth_[static_cast<size_t>(v) * dw + u];
                    if (mz <= 0.0f) continue;
                    const float sdf = mz - pcZ;
                    if (sdf < -truncMm_) continue;
                    const float tsdf = std::min(1.0f, sdf / truncMm_);
                    const size_t i = vol_.idx(vx, vy, vz);
                    const float wo = vol_.weight[i], wn = std::min(64.0f, wo + 1.0f);
                    vol_.dist[i] = (vol_.dist[i] * wo + tsdf) / wn; vol_.weight[i] = wn;
                }
            }
        });
    }

    static bool writeStl(const char* path, const std::vector<Vertex3D>& v, const std::vector<Tri>& t) {
        FILE* fp = std::fopen(path, "wb"); if (!fp) return false;
        char hdr[80]{}; std::snprintf(hdr, sizeof(hdr), "MultiCam stereo TSDF mesh, units mm (scale follows calibration source)");
        std::fwrite(hdr, 1, 80, fp);
        const uint32_t cnt = static_cast<uint32_t>(t.size()); std::fwrite(&cnt, 4, 1, fp);
        const uint16_t attr = 0;
        for (const auto& tr : t) {
            const Vertex3D &a = v[tr.a], &b = v[tr.b], &c = v[tr.c];
            const float e1[3] = {b.p[0] - a.p[0], b.p[1] - a.p[1], b.p[2] - a.p[2]}, e2[3] = {c.p[0] - a.p[0], c.p[1] - a.p[1], c.p[2] - a.p[2]};
            float n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
            const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (len > 0) for (float& x : n) x /= len;
            std::fwrite(n, 4, 3, fp); std::fwrite(a.p, 4, 3, fp); std::fwrite(b.p, 4, 3, fp); std::fwrite(c.p, 4, 3, fp); std::fwrite(&attr, 2, 1, fp);
        }
        std::fclose(fp); return true;
    }
    static bool writePly(const char* path, const std::vector<Vertex3D>& v, const std::vector<Tri>& t) {
        FILE* fp = std::fopen(path, "w"); if (!fp) return false;
        std::fprintf(fp, "ply\nformat ascii 1.0\ncomment units: mm\nelement vertex %zu\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face %zu\nproperty list uchar int vertex_indices\nend_header\n", v.size(), t.size());
        for (const auto& x : v) std::fprintf(fp, "%.4f %.4f %.4f %.4f %.4f %.4f\n", x.p[0], x.p[1], x.p[2], x.n[0], x.n[1], x.n[2]);
        for (const auto& x : t) std::fprintf(fp, "3 %u %u %u\n", x.a, x.b, x.c);
        std::fclose(fp); return true;
    }

    ImuFusionBuffer& imu_;
    mutable std::mutex mtx_;
    StereoConfig cfg_;
    int streamW_ = 0, streamH_ = 0, rectW_ = 0, rectH_ = 0, rectRotDeg_ = 0, frames_ = 0;
    bool lutReady_ = false, verticalAligned_ = false, volReady_ = false;
    float baselineMm_ = 18.5f, rectFx_ = 1, rectFy_ = 1, rectCx_ = 0, rectCy_ = 0, zMin_ = 100, zMax_ = 1000, truncMm_ = 8;
    int64_t refTs_ = 0;
    TsdfVolume vol_;
    std::vector<float> mapLx_, mapRx_, depth_, disp_;
    std::vector<uint8_t> rectL_, rectR_, base_;
    std::vector<uint16_t> agg_;
    std::vector<uint32_t> cL_, cR_;
};

}  // namespace mc
