// Host-side verification of the computational core (no Android needed):  ./run.sh
// Every check compares the engine against ground truth that is generated independently (physics / analytic shapes).
#include "../app/src/main/cpp/engine.hpp"

#include <cstdio>
#include <cstdlib>
#include <functional>
#include <random>

using namespace mc;

static int gFailed = 0, gPassed = 0;
#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if (cond) { ++gPassed; }                                            \
        else { ++gFailed; std::printf("  FAIL %s:%d  %s  ", __FILE__, __LINE__, #cond); std::printf(__VA_ARGS__); std::printf("\n"); } \
    } while (0)

// ---------------------------------------------------------------------------------------------
static void matToQuat(const float R[9], float q[4]) {
    const float tr = R[0] + R[4] + R[8];
    if (tr > 0) { const float s = std::sqrt(tr + 1.0f) * 2; q[0] = 0.25f * s; q[1] = (R[7] - R[5]) / s; q[2] = (R[2] - R[6]) / s; q[3] = (R[3] - R[1]) / s; }
    else if (R[0] > R[4] && R[0] > R[8]) { const float s = std::sqrt(1.0f + R[0] - R[4] - R[8]) * 2; q[0] = (R[7] - R[5]) / s; q[1] = 0.25f * s; q[2] = (R[1] + R[3]) / s; q[3] = (R[2] + R[6]) / s; }
    else if (R[4] > R[8]) { const float s = std::sqrt(1.0f + R[4] - R[0] - R[8]) * 2; q[0] = (R[2] - R[6]) / s; q[1] = (R[1] + R[3]) / s; q[2] = 0.25f * s; q[3] = (R[5] + R[7]) / s; }
    else { const float s = std::sqrt(1.0f + R[8] - R[0] - R[4]) * 2; q[0] = (R[3] - R[1]) / s; q[1] = (R[2] + R[6]) / s; q[2] = (R[5] + R[7]) / s; q[3] = 0.25f * s; }
}
static float maxAbsDiff(const float* a, const float* b, int n) { float m = 0; for (int i = 0; i < n; ++i) m = std::max(m, std::abs(a[i] - b[i])); return m; }

static void testImu() {
    std::printf("[imu] gyro integration == rotation-vector relative rotation, all sensor orientations\n");
    for (int orient : {0, 90, 180, 270}) {
        ImuFusionBuffer imu;
        imu.setSensorOrientation(orient);
        const float w[3] = {0.31f, -0.22f, 0.17f};   // rad/s, device frame
        float R0[9]; rodriguesExp(0.4f, -0.2f, 0.9f, R0);  // arbitrary initial attitude
        float Rd[9]; std::memcpy(Rd, R0, sizeof(Rd));
        const int64_t dtNs = 1'000'000;
        for (int i = 0; i <= 300; ++i) {
            const int64_t ts = 5'000'000'000LL + i * dtNs;
            imu.pushGyroDevice(ts, w[0], w[1], w[2]);
            float q[4]; matToQuat(Rd, q); imu.pushRotationDevice(ts, q[0], q[1], q[2], q[3]);
            float dR[9], n[9]; rodriguesExp(w[0] * 1e-3f, w[1] * 1e-3f, w[2] * 1e-3f, dR); matMul3(Rd, dR, n); std::memcpy(Rd, n, sizeof(n));  // body-frame increment
        }
        float Rg[9], Rq[9];
        const int64_t a = 5'030'000'000LL, b = 5'250'000'000LL;
        CHECK(imu.integrateGyro(a, b, Rg), "no gyro result");
        CHECK(imu.relativeRotation(a, b, Rq), "no quat result");
        CHECK(maxAbsDiff(Rg, Rq, 9) < 2e-3f, "orientation %d: gyro vs quat differ by %g", orient, maxAbsDiff(Rg, Rq, 9));
        // physical sanity: angle ~ |w| * dt
        const float ang = std::acos(std::max(-1.0f, std::min(1.0f, (Rg[0] + Rg[4] + Rg[8] - 1) * 0.5f)));
        const float expect = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * 0.22f;
        CHECK(std::abs(ang - expect) < 2e-3f, "angle %g vs %g", ang, expect);
    }
    // axis mapping for orientation 90: camera rate = (-wy, -wx, -wz)
    ImuFusionBuffer imu; imu.setSensorOrientation(90);
    for (int i = 0; i < 100; ++i) imu.pushGyroDevice(1'000'000'000LL + i * 1'000'000LL, 0.5f, 0, 0);  // spin about device x
    float R[9]; imu.integrateGyro(1'000'000'000LL, 1'090'000'000LL, R);
    // device-x rotation == camera -y rotation: R[2] = sin(-theta)... check it is a rotation about camera y with negative sign
    CHECK(std::abs(R[5]) < 1e-4f && std::abs(R[3]) < 1e-4f && R[2] < -0.04f, "axis map: R2=%g R5=%g", R[2], R[5]);
}

// ---------------------------------------------------------------------------------------------
struct Scene {  // sum of plane waves, deliberately including frequencies above the 1x Nyquist limit
    std::vector<std::array<float, 4>> w;  // kx, ky, phase, amp
    explicit Scene(unsigned seed, float maxFreq) {
        std::mt19937 rng(seed); std::uniform_real_distribution<float> u(0, 1);
        for (int i = 0; i < 48; ++i) {
            const float f = 0.04f + (maxFreq - 0.04f) * u(rng), a = 6.28318f * u(rng);
            // natural-image-like spectrum: energy falls off with frequency, a little survives above 1x Nyquist
            w.push_back({f * std::cos(a), f * std::sin(a), 6.28318f * u(rng), 14.0f * (0.4f + u(rng)) / (1.0f + (f / 0.18f) * (f / 0.18f))});
        }
    }
    float operator()(float x, float y) const {
        float v = 128;
        for (const auto& k : w) v += k[3] * std::sin(6.28318f * (k[0] * x + k[1] * y) + k[2]);
        return std::max(0.0f, std::min(255.0f, v));
    }
};

static double psnr(const std::vector<float>& a, const std::vector<float>& b) {
    double mse = 0; size_t n = 0;
    for (size_t i = 0; i < a.size(); ++i) { const double d = a[i] - b[i]; mse += d * d; ++n; }
    mse /= std::max<size_t>(1, n);
    return mse < 1e-9 ? 99.0 : 10.0 * std::log10(255.0 * 255.0 / mse);
}

static void testMicroscope(bool useGyro) {
    std::printf("[microscope] %s registration + 2x drizzle vs ground truth\n", useGyro ? "gyro-only" : "image");
    const int W = 192, H = 144, N = 12;
    const float focal = 500.0f;
    ImuFusionBuffer imu; imu.setSensorOrientation(90);
    MicroscopeEngine eng(imu);
    eng.configure(focal, focal, 0, 0, 1023);
    eng.reset(W, H, 1.0f);
    eng.imageRefine = !useGyro;
    Scene scene(7, 0.9f);   // content up to 0.9 cycles/px: aliased in a single frame
    std::mt19937 rng(3); std::uniform_real_distribution<float> u(-1.0f, 1.0f);

    std::vector<std::array<float, 2>> shifts;
    const int64_t t0 = 10'000'000'000LL;
    for (int k = 0; k < N; ++k) {
        float sx, sy;
        const int64_t ts = t0 + k * 33'000'000LL;
        if (useGyro) {
            // camera rotates with rate (wx, wy) in the camera frame; device-frame rate = M^T * w_cam
            const float wxc = 0.0016f * 30 * std::sin(k * 1.3f), wyc = 0.0013f * 30 * std::cos(k * 0.9f);  // rad/s
            (void)wxc; (void)wyc;
            sx = 0; sy = 0;
        }
        if (!useGyro) { sx = (k == 0) ? 0 : 1.6f * u(rng); sy = (k == 0) ? 0 : 1.6f * u(rng); }
        else {
            // Rotation angles accumulated relative to frame 0: theta_x(k), theta_y(k) (camera frame).
            const float thx = 0.0021f * std::sin(k * 0.8f) * (k > 0), thy = 0.0017f * k * 0.4f * (k > 0 ? 1 : 0) * std::cos(k * 0.3f);
            sx = -focal * thy; sy = focal * thx;
            // stream gyro samples so that integrating from frame 0 gives (thx, thy): piecewise-constant rate between frames
            if (k == 0) { shifts.push_back({0, 0}); }
            (void)ts;
        }
        shifts.push_back({sx, sy});
    }
    shifts.erase(shifts.begin(), shifts.begin() + (useGyro ? 1 : 0));
    if (useGyro) {
        // build gyro stream consistent with the angles used above
        float M[9] = {0, -1, 0, -1, 0, 0, 0, 0, -1}, MT[9]; transpose3(M, MT);
        auto theta = [&](int k, float& thx, float& thy) {
            thx = k == 0 ? 0.0f : 0.0021f * std::sin(k * 0.8f);
            thy = k == 0 ? 0.0f : 0.0017f * k * 0.4f * std::cos(k * 0.3f);
        };
        // Camera-frame orientation is rotation about x by thx then y by thy (small angles commute to first order).
        for (int k = 0; k < N - 1; ++k) {
            float ax, ay, bx, by; theta(k, ax, ay); theta(k + 1, bx, by);
            for (int s = 0; s < 33; ++s) {  // 1 kHz samples
                const float wcx = (bx - ax) / 0.033f, wcy = (by - ay) / 0.033f;
                const float wd[3] = {MT[0] * wcx + MT[1] * wcy, MT[3] * wcx + MT[4] * wcy, MT[6] * wcx + MT[7] * wcy};
                imu.pushGyroDevice(t0 + k * 33'000'000LL + s * 1'000'000LL, wd[0], wd[1], wd[2]);
            }
        }
        imu.pushGyroDevice(t0 + (N - 1) * 33'000'000LL, 0, 0, 0);
        imu.pushGyroDevice(t0 + (N - 1) * 33'000'000LL + 1'000'000LL, 0, 0, 0);
    }
    std::vector<uint8_t> frame(static_cast<size_t>(W) * H);
    std::vector<float> first(frame.size());
    for (int k = 0; k < N; ++k) {
        for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
            frame[static_cast<size_t>(y) * W + x] = static_cast<uint8_t>(scene(x - shifts[k][0], y - shifts[k][1]) + 0.5f);  // cur(x) = ref(x - s)
        }
        if (k == 0) for (size_t i = 0; i < frame.size(); ++i) first[i] = frame[i];
        CHECK(eng.addYuv(frame.data(), W, nullptr, 0, 0, nullptr, 0, 0, W, H, t0 + k * 33'000'000LL) == k + 1, "frame rejected");
    }
    CHECK(eng.finalize(0), "finalize");  // no deconvolution: isolates registration + accumulation
    const int ow = eng.outputWidth(), oh = eng.outputHeight();
    std::vector<uint8_t> rgba(static_cast<size_t>(ow) * oh * 4);
    CHECK(eng.exportRgba(rgba.data(), ow * 4), "export");

    std::vector<float> truth, rec, naive;
    const int m = 16;  // ignore borders (accumulator edge / hole fill)
    for (int j = m; j < oh - m; ++j)
        for (int i = m; i < ow - m; ++i) {
            const float xin = (i - 0.5f) / 2.0f, yin = (j - 0.5f) / 2.0f;
            truth.push_back(scene(xin, yin));
            rec.push_back(rgba[(static_cast<size_t>(j) * ow + i) * 4 + 1]);
            // bilinear 2x upsample of the first frame
            const int x0 = std::max(0, std::min(W - 2, static_cast<int>(std::floor(xin)))), y0 = std::max(0, std::min(H - 2, static_cast<int>(std::floor(yin))));
            const float fx = xin - x0, fy = yin - y0;
            naive.push_back((1 - fx) * (1 - fy) * first[y0 * W + x0] + fx * (1 - fy) * first[y0 * W + x0 + 1] + (1 - fx) * fy * first[(y0 + 1) * W + x0] + fx * fy * first[(y0 + 1) * W + x0 + 1]);
        }
    const double pRec = psnr(rec, truth), pNaive = psnr(naive, truth);
    std::printf("  PSNR  reconstruction %.2f dB   single-frame bilinear %.2f dB\n", pRec, pNaive);
    CHECK(pRec > pNaive + 2.0, "burst does not beat single frame: %.2f vs %.2f", pRec, pNaive);
    if (!useGyro) {
        // the last frame's estimated shift must equal the injected one (sign + magnitude)
        const float ex = shifts[N - 1][0], ey = shifts[N - 1][1];
        CHECK(std::abs(eng.lastShiftX() - ex) < 0.15f && std::abs(eng.lastShiftY() - ey) < 0.15f,
              "estimated shift (%.3f, %.3f) vs injected (%.3f, %.3f)", eng.lastShiftX(), eng.lastShiftY(), ex, ey);
    }
}

static void testMicroscopeRawGamma() {
    std::printf("[microscope] RAW16 path: Bayer drizzle + sRGB encode\n");
    const int W = 128, H = 96;
    ImuFusionBuffer imu; MicroscopeEngine eng(imu);
    eng.configure(400, 400, 0 /*RGGB*/, 64, 1023);
    eng.reset(W, H, 1.0f);
    std::vector<uint16_t> raw(static_cast<size_t>(W) * H);
    // flat 18% grey scene (linear) => mid grey ~118 after sRGB
    const float lin = 0.18f;
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) raw[static_cast<size_t>(y) * W + x] = static_cast<uint16_t>(64 + lin * (1023 - 64));
    for (int k = 0; k < 3; ++k) eng.addRaw16(reinterpret_cast<const uint8_t*>(raw.data()), W * 2, W, H, 1000 + k);
    CHECK(eng.finalize(0), "finalize");
    std::vector<uint8_t> out(static_cast<size_t>(eng.outputWidth()) * eng.outputHeight() * 4);
    eng.exportRgba(out.data(), eng.outputWidth() * 4);
    const size_t c = (static_cast<size_t>(eng.outputHeight() / 2) * eng.outputWidth() + eng.outputWidth() / 2) * 4;
    std::printf("  centre pixel RGB = %d %d %d (expect ~118 each)\n", out[c], out[c + 1], out[c + 2]);
    CHECK(std::abs(out[c + 1] - 118) <= 3, "gamma/exposure wrong: %d", out[c + 1]);
    CHECK(std::abs(out[c] - out[c + 1]) <= 3 && std::abs(out[c + 2] - out[c + 1]) <= 3, "white balance of a neutral scene should be neutral");
}

static void testMicroscopeMemoryBound() {
    std::printf("[microscope] memory budget -> centred ROI\n");
    ImuFusionBuffer imu; MicroscopeEngine eng(imu);
    eng.reset(8000, 6000, 12.0f);  // 48 MP sensor, 12 MP output budget
    CHECK(static_cast<double>(eng.outputWidth()) * eng.outputHeight() <= 12.2e6, "output %dx%d exceeds budget", eng.outputWidth(), eng.outputHeight());
    CHECK(eng.outputWidth() * 3 == eng.outputHeight() * 4 || std::abs(eng.outputWidth() * 3 - eng.outputHeight() * 4) < 24, "aspect not preserved");
}

// ---------------------------------------------------------------------------------------------
static void makePair(const Scene& F, int W, int H, float dispPx, bool vertical, std::vector<uint8_t>& L, std::vector<uint8_t>& R) {
    L.resize(static_cast<size_t>(W) * H); R.resize(L.size());
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
        L[static_cast<size_t>(y) * W + x] = static_cast<uint8_t>(F(static_cast<float>(x), static_cast<float>(y)) + 0.5f);
        R[static_cast<size_t>(y) * W + x] = static_cast<uint8_t>(vertical ? F(static_cast<float>(x), y + dispPx) + 0.5f : F(x + dispPx, static_cast<float>(y)) + 0.5f);
    }
}

static void testStereo(float Z, bool vertical) {
    const int W = 1440, H = 1080;
    const float f = 1098.0f, B = 18.5f, d = f * B / Z;
    std::printf("[stereo] plane at Z=%.0f mm  baseline %s  (full-res disparity %.1f px)\n", Z, vertical ? "vertical" : "horizontal", d);
    ImuFusionBuffer imu; StereoTsdfEngine eng(imu);
    StereoConfig cfg;
    cfg.left = {f, f, W / 2.0f, H / 2.0f, {0}}; cfg.right = cfg.left;
    cfg.T_mm[0] = vertical ? 0.0f : -B; cfg.T_mm[1] = vertical ? -B : 0.0f; cfg.T_mm[2] = 0; cfg.calibrated = true;
    eng.configure(cfg, W, H);
    auto info = eng.info();
    CHECK(std::abs(info.baselineMm - B) < 1e-3f, "baseline %g", info.baselineMm);
    CHECK(info.rectRotationDeg == (vertical ? 90 : 0), "rect rotation %d", info.rectRotationDeg);
    CHECK(vertical ? (info.rectW == H / 4 && info.rectH == W / 4) : (info.rectW == W / 4 && info.rectH == H / 4), "rect dims %dx%d", info.rectW, info.rectH);
    Scene F(11, 0.10f);
    std::vector<uint8_t> L, R; makePair(F, W, H, d, vertical, L, R);
    std::vector<uint32_t> hm(static_cast<size_t>(info.rectW) * info.rectH); std::vector<float> depth(hm.size());
    for (int k = 0; k < 4; ++k) eng.process(L.data(), W, R.data(), W, W, H, 1'000'000'000LL + k * 33'000'000LL, hm.data(), depth.data());
    std::vector<float> z;
    for (int y = info.rectH / 4; y < 3 * info.rectH / 4; ++y) for (int x = info.rectW / 4; x < 3 * info.rectW / 4; ++x) { const float v = depth[static_cast<size_t>(y) * info.rectW + x]; if (v > 0) z.push_back(v); }
    const double valid = static_cast<double>(z.size()) / ((info.rectW / 2) * (info.rectH / 2));
    std::sort(z.begin(), z.end());
    const float med = z.empty() ? 0.0f : z[z.size() / 2];
    std::printf("  valid %.0f%%   median depth %.1f mm (truth %.0f)  zRange [%.0f, %.0f]\n", valid * 100, med, Z, info.zMinMm, info.zMaxMm);
    CHECK(valid > 0.30, "too few valid pixels: %.2f", valid);
    CHECK(std::abs(med - Z) / Z < 0.04f, "depth error %.1f%%", std::abs(med - Z) / Z * 100);
    // pipeline end-to-end: fuse, mesh, sanity of geometry
    for (int k = 0; k < 6; ++k) eng.process(L.data(), W, R.data(), W, W, H, 2'000'000'000LL + k * 33'000'000LL, nullptr);
    int nv = 0; const int tris = eng.exportMesh(nullptr, nullptr, &nv);
    CHECK(tris > 200, "mesh has %d triangles", tris);
    if (tris > 0) {
        std::vector<Vertex3D> V; std::vector<Tri> T; extractMesh(eng.volume(), V, T);
        double mz = 0; for (const auto& v : V) mz += v.p[2]; mz /= std::max<size_t>(1, V.size());
        std::printf("  fused mesh: %zu tris, mean z %.1f mm\n", T.size(), mz);
        CHECK(std::abs(mz - Z) / Z < 0.05f, "mesh z %.1f vs %.0f", mz, Z);
    }
}

static void testMesh() {
    std::printf("[mesh] marching tetrahedra on an analytic sphere\n");
    TsdfVolume V; V.dim = 48; V.voxelMm = 2.0f; V.center[0] = V.center[1] = 0; V.center[2] = 250;
    const size_t n = static_cast<size_t>(V.dim) * V.dim * V.dim; V.dist.assign(n, 1.0f); V.weight.assign(n, 1.0f);
    const float half = 0.5f * V.dim * V.voxelMm, Rad = 30.0f;
    for (int z = 0; z < V.dim; ++z) for (int y = 0; y < V.dim; ++y) for (int x = 0; x < V.dim; ++x) {
        const float px = x * V.voxelMm - half, py = y * V.voxelMm - half, pz = z * V.voxelMm - half;
        V.dist[V.idx(x, y, z)] = std::max(-1.0f, std::min(1.0f, (std::sqrt(px * px + py * py + pz * pz) - Rad) / 8.0f));
    }
    std::vector<Vertex3D> vs; std::vector<Tri> ts; extractMesh(V, vs, ts);
    std::unordered_map<uint64_t, int> undirected, directed;
    for (const auto& t : ts) {
        const uint32_t i[3] = {t.a, t.b, t.c};
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = i[k], b = i[(k + 1) % 3];
            undirected[(static_cast<uint64_t>(std::min(a, b)) << 32) | std::max(a, b)]++;
            directed[(static_cast<uint64_t>(a) << 32) | b]++;
        }
    }
    int bad = 0; for (auto& e : undirected) if (e.second != 2) ++bad;
    int dup = 0; for (auto& e : directed) if (e.second != 1) ++dup;
    const long chi = static_cast<long>(vs.size()) - static_cast<long>(undirected.size()) + static_cast<long>(ts.size());
    double vol = 0; for (const auto& t : ts) {
        const auto &a = vs[t.a], &b = vs[t.b], &c = vs[t.c];
        vol += (a.p[0] * (b.p[1] * c.p[2] - b.p[2] * c.p[1]) - a.p[1] * (b.p[0] * c.p[2] - b.p[2] * c.p[0]) + a.p[2] * (b.p[0] * c.p[1] - b.p[1] * c.p[0])) / 6.0;
    }
    // volume relative to origin is translation-invariant for closed meshes
    const double truth = 4.0 / 3.0 * 3.14159265 * Rad * Rad * Rad;
    std::printf("  %zu verts %zu tris | non-manifold edges %d | inconsistent winding %d | Euler %ld | volume %.0f vs %.0f mm^3\n", vs.size(), ts.size(), bad, dup, chi, vol, truth);
    CHECK(bad == 0, "mesh is not closed");
    CHECK(dup == 0, "winding is not consistent");
    CHECK(chi == 2, "Euler characteristic %ld", chi);
    CHECK(vol > 0 && std::abs(vol - truth) / truth < 0.04, "signed volume %.0f vs %.0f (outward winding?)", vol, truth);
    // vertices lie on the sphere
    double maxErr = 0; for (const auto& v : vs) { const double r = std::sqrt(v.p[0] * v.p[0] + v.p[1] * v.p[1] + (v.p[2] - 250) * (v.p[2] - 250)); maxErr = std::max(maxErr, std::abs(r - Rad)); }
    CHECK(maxErr < 1.0, "surface error %.2f mm", maxErr);
}

int main() {
    testImu();
    testMicroscope(false);
    testMicroscope(true);
    testMicroscopeRawGamma();
    testMicroscopeMemoryBound();
    testMesh();
    testStereo(250.0f, false);
    testStereo(150.0f, false);
    testStereo(400.0f, false);
    testStereo(250.0f, true);
    std::printf("\n%d checks passed, %d failed\n", gPassed, gFailed);
    return gFailed ? 1 : 0;
}
