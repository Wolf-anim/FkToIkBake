// FkToIkBake - engine validation suite (no Maya required).
// Build: see tests/build_tests.sh   Run: ./fk2ik_tests
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "../engine/BakeEngine.h"
#include "../engine/KeyReducer.h"
#include "../engine/RigMap.h"
#include "SyntheticRig.h"

using namespace fk2ik;
using namespace fk2ik::test;

static int g_fail = 0;
#define CHECK(cond, ...)                                   \
    do {                                                   \
        if (!(cond)) {                                     \
            ++g_fail;                                      \
            std::printf("  FAIL %s:%d  ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__);                      \
            std::printf("\n");                             \
        }                                                  \
    } while (0)

static double rotAngle(const Mat4& a, const Mat4& b) {
    const Mat4 r = a.rotationOnly() * b.rotationOnly().transposed3x3();
    const double tr = r.m[0][0] + r.m[1][1] + r.m[2][2];
    return std::acos(std::clamp((tr - 1.0) * 0.5, -1.0, 1.0));
}
static double deg(double r) { return r * 180.0 / kPi; }
static double rad(double d) { return d * kPi / 180.0; }

// ---------------------------------------------------------------- unit tests
static void testEuler() {
    std::printf("[unit] Euler <-> matrix, all rotate orders (incl. gimbal)\n");
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(-kPi, kPi);
    double worst = 0;
    for (int ro = 0; ro < 6; ++ro) {
        for (int n = 0; n < 20000; ++n) {
            Euler e{U(rng), U(rng) * 0.5, U(rng)};
            if (n % 50 == 0) e[n % 3] = kPi / 2 * ((n / 50) % 2 ? 1 : -1);  // force gimbal cases
            const Mat4 m = eulerToMatrix(e, static_cast<RotateOrder>(ro));
            const Euler back = matrixToEuler(m, static_cast<RotateOrder>(ro));
            worst = std::max(worst, rotAngle(m, eulerToMatrix(back, static_cast<RotateOrder>(ro))));
        }
    }
    CHECK(worst < 1e-7, "euler roundtrip worst %.3g rad", worst);
    std::printf("  worst roundtrip error %.2e deg\n", deg(worst));

    // closestEuler: same orientation, near previous
    double worstC = 0;
    int badJump = 0;
    for (int ro = 0; ro < 6; ++ro) {
        for (int n = 0; n < 20000; ++n) {
            Euler prev{U(rng) * 4, U(rng) * 4, U(rng) * 4};  // arbitrary unwrapped values
            Euler tgt = prev;
            for (int a = 0; a < 3; ++a) tgt[a] += U(rng) * 0.05;
            const Mat4 m = eulerToMatrix(tgt, static_cast<RotateOrder>(ro));
            const Euler raw = matrixToEuler(m, static_cast<RotateOrder>(ro));
            const Euler c = closestEuler(raw, prev, static_cast<RotateOrder>(ro));
            worstC = std::max(worstC, rotAngle(m, eulerToMatrix(c, static_cast<RotateOrder>(ro))));
            for (int a = 0; a < 3; ++a)
                if (std::fabs(c[a] - prev[a]) > 0.2) { ++badJump; break; }
        }
    }
    CHECK(worstC < 1e-7, "closestEuler changed orientation %.3g", worstC);
    CHECK(badJump == 0, "closestEuler produced %d jumps", badJump);
    std::printf("  closestEuler: orientation kept (%.1e deg), jumps: %d / 120000\n", deg(worstC), badJump);
}

static ControlTransformDesc randomDesc(std::mt19937& rng, bool joint) {
    std::uniform_real_distribution<double> U(-1, 1);
    ControlTransformDesc d;
    d.isJoint = joint;
    d.rotateOrder = static_cast<RotateOrder>(rng() % 6);
    d.scale = {1 + 0.5 * U(rng), 1 + 0.5 * U(rng), 1 + 0.5 * U(rng)};
    d.rotateAxis = {U(rng), U(rng), U(rng)};
    if (joint) {
        d.jointOrient = {U(rng) * 2, U(rng), U(rng) * 2};
        d.inverseParentScale = {1 + 0.3 * U(rng), 1 + 0.3 * U(rng), 1 + 0.3 * U(rng)};
    } else {
        d.shear = {0.2 * U(rng), 0.2 * U(rng), 0.2 * U(rng)};
        d.rotatePivot = {U(rng) * 5, U(rng) * 5, U(rng) * 5};
        d.rotatePivotTranslate = {U(rng), U(rng), U(rng)};
        d.scalePivot = {U(rng) * 3, U(rng) * 3, U(rng) * 3};
        d.scalePivotTranslate = {U(rng), U(rng), U(rng)};
    }
    return d;
}

static void testControlSolver() {
    std::printf("[unit] control channel solve (pivots, rotateAxis, shear, jointOrient, parent space)\n");
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> U(-1, 1);
    double worstT = 0, worstR = 0;
    for (int n = 0; n < 20000; ++n) {
        const ControlTransformDesc d = randomDesc(rng, n % 2 == 1);
        const Vec3 t{U(rng) * 50, U(rng) * 50, U(rng) * 50};
        const Euler r{U(rng) * 3, U(rng) * 1.5, U(rng) * 3};
        Mat4 parent = eulerToMatrix({U(rng) * 3, U(rng), U(rng) * 3}, RotateOrder::XYZ);
        parent = Mat4::scale({1.5, 1.5, 1.5}) * parent;
        parent.setTranslation({U(rng) * 100, U(rng) * 100, U(rng) * 100});
        const Mat4 world = composeLocal(d, t, r) * parent;
        const ControlChannels c = solveChannels(d, world, parent.inverse(), r, true);
        const Mat4 back = composeLocal(d, c.translate, c.rotate) * parent;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 3; ++j) worstT = std::max(worstT, std::fabs(back.m[i][j] - world.m[i][j]));
        for (int a = 0; a < 3; ++a) worstR = std::max(worstR, std::fabs(c.rotate[a] - r[a]));
    }
    CHECK(worstT < 1e-8, "control solve matrix error %.3g", worstT);
    CHECK(worstR < 1e-7, "control solve picked different euler %.3g", worstR);
    std::printf("  worst world-matrix element error %.2e, euler deviation %.2e rad\n", worstT, worstR);
}

static void testReducerAndMap() {
    std::printf("[unit] key reducer + rig map parser\n");
    std::vector<double> t, v;
    for (int i = 0; i <= 1000; ++i) { t.push_back(i); v.push_back(std::sin(i * 0.05) * 10); }
    const double tol = 0.01;
    auto keep = reduceKeys(t, v, tol);
    double worst = 0;
    for (size_t k = 0; k + 1 < keep.size(); ++k) {
        const size_t a = keep[k], b = keep[k + 1];
        for (size_t i = a; i <= b; ++i) {
            const double u = (t[i] - t[a]) / (t[b] - t[a]);
            worst = std::max(worst, std::fabs(v[i] - (v[a] + (v[b] - v[a]) * u)));
        }
    }
    CHECK(worst <= tol + 1e-12, "reducer bound violated %.3g", worst);
    std::vector<double> flat(500, 3.0);
    CHECK(reduceKeys(t, flat, 0).size() == 2, "constant channel not collapsed");
    std::printf("  sine 1001 samples -> %zu keys at tol %.3g (max err %.4f)\n", keep.size(), tol, worst);

    RigMap map;
    std::string err;
    const char* txt =
        "; comment\n[L_arm]\ntype=arm\nmode=rotation\nfkUpper=a\nfkMid=b\nfkEnd=c\n"
        "ikUpper=d\nikMid=e\nikEnd=f\nikControl=g\npoleControl=h\nswitchAttr=sw.blend\nswitchIk=10\n";
    CHECK(parseRigMap(txt, map, err) && map.limbs.size() == 1 &&
              map.limbs[0].mode == MatchMode::Rotation && map.limbs[0].switchIk == 10,
          "map parse: %s", err.c_str());
    CHECK(!parseRigMap("[x]\nfkUpper=a\n", map, err), "incomplete map accepted");
    std::printf("  incomplete map rejected with: %s\n", err.c_str());
}

// ---------------------------------------------------------------- rig tests
struct AnimSet {
    std::string name;
    std::function<Euler(double)> upper, end, midExtra;
    std::function<double(double)> hinge;
    std::function<Mat4(double)> root;
    bool absoluteBend = false;  // hinge value = total bend incl. joint-orient pre-bend
};

static Mat4 movingRoot(double t) {
    Mat4 m = eulerToMatrix({0.1 * std::sin(t * 0.04), t * 0.02, 0.15 * std::sin(t * 0.03)}, RotateOrder::XYZ);
    m.setTranslation({t * 0.5, 3 * std::sin(t * 0.1), 10 * std::cos(t * 0.01)});
    return m;
}

static std::vector<AnimSet> animSets() {
    std::vector<AnimSet> s;
    s.push_back({"normal",
                 [](double t) { return Euler{0.8 * std::sin(0.07 * t), 0.6 * std::sin(0.05 * t + 1), 1.2 * std::sin(0.03 * t)}; },
                 [](double t) { return Euler{0.7 * std::sin(0.11 * t), 0.5 * std::sin(0.13 * t), 0.9 * std::sin(0.08 * t)}; },
                 nullptr, [](double t) { return 0.08 + 1.1 * (0.5 + 0.5 * std::sin(0.09 * t)); }, movingRoot});
    s.push_back({"fast",  // up to ~25-30 deg per frame on several axes
                 [](double t) { return Euler{1.4 * std::sin(0.22 * t), 0.9 * std::sin(0.18 * t + 1), 1.6 * std::sin(0.13 * t)}; },
                 [](double t) { return Euler{1.2 * std::sin(0.33 * t), 0.8 * std::sin(0.28 * t), 1.5 * std::sin(0.24 * t)}; },
                 nullptr, [](double t) { return 0.1 + 1.6 * (0.5 + 0.5 * std::sin(0.2 * t)); }, movingRoot});
    s.push_back({"spin360",  // continuous spins crossing 180/360 many times
                 [](double t) { return Euler{0.3 * std::sin(0.05 * t), 0.2, 0.05 * t}; },
                 [](double t) { return Euler{0.15 * t, 0.3 * std::sin(0.07 * t), -0.11 * t}; },
                 nullptr, [](double t) { return 0.6 + 0.4 * std::sin(0.06 * t); }, movingRoot});
    s.push_back({"straight",  // mocap-like near-straight limb, incl. exactly straight
                 [](double t) { return Euler{0.8 * std::sin(0.07 * t), 0.6 * std::sin(0.05 * t + 1), 1.2 * std::sin(0.03 * t)}; },
                 [](double t) { return Euler{0.4 * std::sin(0.11 * t), 0.3 * std::sin(0.13 * t), 0.5 * std::sin(0.08 * t)}; },
                 nullptr, [](double t) { return std::max(0.0, 0.03 * std::sin(0.05 * t)); }, movingRoot, true});
    s.push_back({"offHinge",  // animator also rotated the elbow off its hinge axis
                 [](double t) { return Euler{0.8 * std::sin(0.07 * t), 0.6 * std::sin(0.05 * t + 1), 1.2 * std::sin(0.03 * t)}; },
                 [](double t) { return Euler{0.7 * std::sin(0.11 * t), 0.5 * std::sin(0.13 * t), 0.9 * std::sin(0.08 * t)}; },
                 [](double t) { return Euler{0.25 * std::sin(0.06 * t), 0.15 * std::sin(0.04 * t), 0}; },
                 [](double t) { return 0.3 + 0.9 * (0.5 + 0.5 * std::sin(0.09 * t)); }, movingRoot});
    return s;
}

static std::vector<RigConfig> rigConfigs() {
    std::vector<RigConfig> v;
    {  // Advanced-Skeleton-like left arm: X down the bone, bends on Z
        RigConfig c;
        c.name = "arm_L  (X-down, ctrl XYZ)";
        c.type = LimbType::Arm;
        c.boneAxis = {1, 0, 0}; c.hingeAxis = 2; c.hingeSign = 1;
        c.joUpper = {0.1, -0.2, -0.6}; c.joMid = {0, 0, 0.1}; c.joEnd = {0.2, 0, -0.1};
        c.ctrlRestWorldRot = {0, 0, 0};
        v.push_back(c);
    }
    {  // Mirrored right arm with an awkward control (rotate order, axis, pivot, scale, twist)
        RigConfig c;
        c.name = "arm_R  (mirrored, ZXY+RA+pivot+scale, twist)";
        c.type = LimbType::Arm;
        c.boneAxis = {-1, 0, 0}; c.hingeAxis = 2; c.hingeSign = 1;
        c.joUpper = {kPi + 0.1, 0.2, 0.6}; c.joMid = {0, 0, 0.1}; c.joEnd = {-0.2, 0, 0.1};
        c.clavicleOffset = {-5, 140, 0};
        c.twist = 0.4;
        c.ctrlDesc.rotateOrder = RotateOrder::ZXY;
        c.ctrlDesc.rotateAxis = {0.2, 0.1, -0.3};
        c.ctrlDesc.rotatePivot = {1, 2, 0.5};
        c.ctrlDesc.rotatePivotTranslate = {0.1, 0, 0.2};
        c.ctrlParentScale = 1.3;
        c.ctrlPivotOffset = {0, -2, 1};
        c.ctrlRestWorldRot = {0.3, 1.0, -0.5};
        c.poleDesc.rotatePivot = {0.5, 0, 0};
        v.push_back(c);
    }
    {  // Leg: Y down the bone, knee bends on X, world-aligned foot control
        RigConfig c;
        c.name = "leg_L  (Y-down, ctrl YZX)";
        c.type = LimbType::Leg;
        c.boneAxis = {0, 1, 0}; c.hingeAxis = 0; c.hingeSign = -1;
        c.joUpper = {kPi, 0, 0.05}; c.joMid = {-0.02, 0, 0}; c.joEnd = {-0.4, 0, 0};
        c.clavicleOffset = {9, 95, 0};
        c.L1 = 45; c.L2 = 42;
        c.ctrlDesc.rotateOrder = RotateOrder::YZX;
        c.ctrlRestWorldRot = {0, 0, 0};
        v.push_back(c);
    }
    {  // Leg whose IK control is a JOINT (jointOrient) + static pole parent
        RigConfig c;
        c.name = "leg_R  (-Y-down, ctrl is joint, XZY)";
        c.type = LimbType::Leg;
        c.boneAxis = {0, -1, 0}; c.hingeAxis = 0; c.hingeSign = 1;
        c.joUpper = {0, 0, -0.05}; c.joMid = {0.02, 0, 0}; c.joEnd = {0.4, 0.1, 0};
        c.clavicleOffset = {-9, 95, 0};
        c.L1 = 45; c.L2 = 42;
        c.ctrlDesc.isJoint = true;
        c.ctrlDesc.rotateOrder = RotateOrder::XZY;
        c.ctrlDesc.jointOrient = {0.3, -0.2, 0.5};
        c.ctrlDesc.rotateAxis = {0.1, 0, 0};
        c.ctrlRestWorldRot = {0.1, 0.2, 0};
        c.animatePoleParent = false;
        v.push_back(c);
    }
    return v;
}

struct RunReport {
    double midPos = 0, endPos = 0, endRot = 0, midRot = 0;  // max errors
    double meanEndPos = 0, meanMidPos = 0;
    int flips = 0, gimbalMoves = 0, fallback = 0, overExt = 0;
    size_t keysDense = 0, keysReduced = 0;
    double reducedEndPos = 0;
};

static RunReport runCase(const RigConfig& baseCfg, const AnimSet& anim, MatchMode mode, int frames,
                         double step, double reduceTol, double* engineSeconds = nullptr) {
    RigConfig cfg = baseCfg;
    cfg.fkUpperAnim = anim.upper;
    cfg.fkEndAnim = anim.end;
    cfg.fkHingeAnim = anim.hinge;
    cfg.fkMidExtra = anim.midExtra;
    cfg.rootAnim = anim.root;
    cfg.absoluteBend = anim.absoluteBend;
    SyntheticLimb limb(cfg);

    // ---- calibration on the IK side at frame 0 (rig in its current IK pose)
    const double t0 = 0.0;
    const Mat4 ctrlW0 = limb.ctrlRestWorld();
    const Mat4 cpInv0 = limb.ctrlParentWorld(t0).inverse();
    const ControlChannels c0 = solveChannels(cfg.ctrlDesc, ctrlW0, cpInv0, {}, false);
    const LimbPose fkRest = limb.fkPose(t0);
    // Rig's pole: somewhere roughly in front of the elbow, deliberately off-plane.
    const Vec3 poleW0 = fkRest.mid.getTranslation() + Vec3{8, -5, 40};
    const LimbPose ik0 = limb.solveIk(t0, ctrlW0, poleW0);

    LimbJob job;
    job.name = cfg.name;
    job.settings = LimbSettings::defaultsFor(cfg.type);
    job.settings.mode = mode;
    job.ctrlDesc = cfg.ctrlDesc;
    job.poleDesc = cfg.poleDesc;
    job.calibration = calibrate(ik0, ctrlW0, poleW0);
    job.ctrlStartRotate = c0.rotate;
    job.hasStartRotate = true;
    prepare(job);

    SyntheticSource src;
    src.limbs = {&limb};
    const std::vector<double> times = buildSampleTimes(1, frames, step);
    std::vector<LimbJob> jobs{job};
    std::vector<LimbBakeResult> res;
    BakeStats stats;
    std::string err;
    BakeEngine eng;
    const auto a = std::chrono::steady_clock::now();
    const bool ok = eng.run(jobs, src, times, res, stats, err);
    const auto b = std::chrono::steady_clock::now();
    CHECK(ok, "bake failed: %s", err.c_str());
    if (engineSeconds) *engineSeconds = std::chrono::duration<double>(b - a).count();

    RunReport rep;
    const LimbBakeResult& r = res[0];
    rep.fallback = r.straightFallbackSamples;
    rep.overExt = r.overExtendedSamples;

    auto evalAt = [&](size_t i, const std::vector<double>* ch, const std::vector<double>* pl, double t) {
        const Vec3 tr{ch[0][i], ch[1][i], ch[2][i]};
        const Euler ro{ch[3][i], ch[4][i], ch[5][i]};
        const Mat4 cw = limb.ctrlWorldFromChannels(t, tr, ro);
        const Vec3 pw = limb.poleWorldFromChannels(t, {pl[0][i], pl[1][i], pl[2][i]});
        return limb.solveIk(t, cw, pw);
    };

    double sumEnd = 0, sumMid = 0;
    for (size_t i = 0; i < times.size(); ++i) {
        const double t = times[i];
        const LimbPose fk = limb.fkPose(t);
        const LimbPose ik = evalAt(i, r.ctrl, r.pole, t);
        const double em = length(ik.mid.getTranslation() - fk.mid.getTranslation());
        const double ee = length(ik.end.getTranslation() - fk.end.getTranslation());
        rep.midPos = std::max(rep.midPos, em);
        rep.endPos = std::max(rep.endPos, ee);
        sumMid += em; sumEnd += ee;
        rep.endRot = std::max(rep.endRot, rotAngle(ik.end, fk.end));
        rep.midRot = std::max(rep.midRot, rotAngle(ik.mid, fk.mid));
        if (i > 0) {
            // Flip = our Euler choice is farther from the previous key than the
            // best equivalent solution found by brute force (2 solutions x wraps).
            const Euler prev{r.ctrl[3][i - 1], r.ctrl[4][i - 1], r.ctrl[5][i - 1]};
            const Euler ours{r.ctrl[3][i], r.ctrl[4][i], r.ctrl[5][i]};
            const RotateOrder ro = cfg.ctrlDesc.rotateOrder;
            int ia, ja, ka;
            rotateOrderAxes(ro, ia, ja, ka);
            Euler alt = ours;
            alt[ia] += kPi; alt[ja] = kPi - alt[ja]; alt[ka] += kPi;
            double best = 1e300;
            for (const Euler& base : {ours, alt})
                for (int kx = -3; kx <= 3; ++kx)
                    for (int ky = -3; ky <= 3; ++ky)
                        for (int kz = -3; kz <= 3; ++kz) {
                            const double dx = base.x + kx * kTwoPi - prev.x, dy = base.y + ky * kTwoPi - prev.y,
                                         dz = base.z + kz * kTwoPi - prev.z;
                            best = std::min(best, dx * dx + dy * dy + dz * dz);
                        }
            const double d = (ours.x - prev.x) * (ours.x - prev.x) + (ours.y - prev.y) * (ours.y - prev.y) +
                             (ours.z - prev.z) * (ours.z - prev.z);
            if (d > best + 1e-9) ++rep.flips;
            for (int c = 3; c < 6; ++c)
                if (std::fabs(r.ctrl[c][i] - r.ctrl[c][i - 1]) > rad(90.0)) { ++rep.gimbalMoves; break; }
        }
    }
    rep.meanEndPos = sumEnd / times.size();
    rep.meanMidPos = sumMid / times.size();
    rep.keysDense = times.size() * 9;

    if (reduceTol > 0) {
        // Reduce every channel, rebuild with linear interpolation, re-measure.
        std::vector<double> rc[6], rp[3];
        size_t kept = 0;
        auto rebuild = [&](const std::vector<double>& src2, std::vector<double>& dst, double tol) {
            auto keep = reduceKeys(times, src2, tol);
            kept += keep.size();
            dst.resize(src2.size());
            for (size_t k = 0; k + 1 < keep.size(); ++k) {
                const size_t a2 = keep[k], b2 = keep[k + 1];
                for (size_t i = a2; i <= b2; ++i) {
                    const double u = (times[i] - times[a2]) / (times[b2] - times[a2]);
                    dst[i] = src2[a2] + (src2[b2] - src2[a2]) * u;
                }
            }
            if (keep.size() == 1) std::fill(dst.begin(), dst.end(), src2[0]);
        };
        for (int c = 0; c < 6; ++c) rebuild(r.ctrl[c], rc[c], c < 3 ? reduceTol : rad(reduceTol));
        for (int c = 0; c < 3; ++c) rebuild(r.pole[c], rp[c], reduceTol * 10);
        rep.keysReduced = kept;
        for (size_t i = 0; i < times.size(); ++i) {
            const LimbPose fk = limb.fkPose(times[i]);
            const LimbPose ik = evalAt(i, rc, rp, times[i]);
            rep.reducedEndPos =
                std::max(rep.reducedEndPos, length(ik.end.getTranslation() - fk.end.getTranslation()));
        }
    }
    return rep;
}

int main() {
    std::printf("FkToIkBake engine validation\n============================\n");
    testEuler();
    testControlSolver();
    testReducerAndMap();

    std::printf("\n[rig] FK -> IK bake, frames 1..2000, step 1.  Errors are max over all frames,\n");
    std::printf("      measured on the IK chain re-solved from the baked keys vs. the FK chain.\n");
    std::printf("      units: scene units (limb ~60-90 units long) and degrees\n\n");
    std::printf("%-44s %-9s %-8s | %-9s %-9s %-9s %-9s | %5s %6s %6s\n", "rig", "anim", "mode", "wristPos",
                "elbowPos", "wristRot", "elbowRot", "flips", ">90/f", "fallbk");
    const auto rigs = rigConfigs();
    const auto anims = animSets();
    for (const auto& rc : rigs) {
        for (const auto& an : anims) {
            for (MatchMode mode : {MatchMode::Position, MatchMode::Rotation}) {
                const RunReport rep = runCase(rc, an, mode, 2000, 1.0, 0.0);
                std::printf("%-44s %-9s %-8s | %9.2e %9.2e %9.2e %9.2e | %5d %6d %6d\n", rc.name.c_str(),
                            an.name.c_str(), mode == MatchMode::Position ? "position" : "rotation",
                            rep.endPos, rep.midPos, deg(rep.endRot), deg(rep.midRot), rep.flips,
                            rep.gimbalMoves, rep.fallback);
                // Hard expectations
                CHECK(rep.endPos < 1e-6, "%s/%s wrist position error %.3g", rc.name.c_str(), an.name.c_str(), rep.endPos);
                CHECK(deg(rep.endRot) < 1e-5, "%s/%s wrist rotation error", rc.name.c_str(), an.name.c_str());
                CHECK(rep.flips == 0, "%s/%s euler flips %d", rc.name.c_str(), an.name.c_str(), rep.flips);
                const bool exactElbowExpected =
                    an.name != "offHinge" && !(mode == MatchMode::Position && an.name == "straight");
                // Near-straight chains lose ~1e-6 to sqrt cancellation in the
                // reference solver's law-of-cosines; allow 1e-4 units there.
                const double elbowTol = an.name == "straight" ? 1e-4 : 1e-6;
                if (exactElbowExpected) {
                    CHECK(rep.midPos < elbowTol, "%s/%s/%d elbow position error %.3g", rc.name.c_str(),
                          an.name.c_str(), (int)mode, rep.midPos);
                    CHECK(deg(rep.midRot) < 1e-5, "%s/%s elbow rotation error %.3g", rc.name.c_str(),
                          an.name.c_str(), deg(rep.midRot));
                }
                if (an.name == "offHinge" && mode == MatchMode::Position)
                    CHECK(rep.midPos < 1e-6, "position mode must keep elbow position, %.3g", rep.midPos);
            }
        }
    }

    std::printf("\n[rig] sampling interval (arm_L, normal): errors at the sampled frames\n");
    for (double step : {0.5, 2.0, 5.0}) {
        const RunReport rep = runCase(rigs[0], anims[0], MatchMode::Position, 2000, step, 0.0);
        std::printf("  step %.1f : wrist %.2e  elbow %.2e\n", step, rep.endPos, rep.midPos);
        CHECK(rep.endPos < 1e-6 && rep.midPos < 1e-6, "step %.1f error", step);
    }

    std::printf("\n[rig] optional key reduction (linear tangents), arm_R / normal, 2000 frames\n");
    for (double tol : {0.001, 0.01, 0.05}) {
        const RunReport rep = runCase(rigs[1], anims[0], MatchMode::Position, 2000, 1.0, tol);
        std::printf("  tol %.3f (units / deg): keys %zu -> %zu (%.0f%%), max wrist error %.4f units\n", tol,
                    rep.keysDense, rep.keysReduced, 100.0 * rep.keysReduced / rep.keysDense, rep.reducedEndPos);
    }

    std::printf("\n[perf] engine cost only (matching + channel solving + synthetic rig evaluation)\n");
    for (int frames : {10000, 100000}) {
        double secs = 0;
        const RunReport rep = runCase(rigs[1], anims[1], MatchMode::Position, frames, 1.0, 0.0, &secs);
        std::printf("  %6d frames, 1 limb : %.3f s  (%.2f us / frame)  max wrist err %.1e, elbow err %.1e, flips %d\n",
                    frames, secs, secs / frames * 1e6, rep.endPos, rep.midPos, rep.flips);
        CHECK(rep.endPos < 1e-6 && rep.midPos < 1e-6 && rep.flips == 0, "long sequence accuracy");
    }
    {   // pure matching + solve throughput, 4 limbs, sample source excluded
        SyntheticLimb limb(rigs[1]);
        const LimbPose ik0 = limb.solveIk(0, limb.ctrlRestWorld(), Vec3{0, 150, 50});
        LimbJob job;
        job.settings = LimbSettings::defaultsFor(LimbType::Arm);
        job.ctrlDesc = rigs[1].ctrlDesc;
        job.calibration = calibrate(ik0, limb.ctrlRestWorld(), Vec3{0, 150, 50});
        prepare(job);
        const LimbPose fk = limb.fkPose(10);
        const Mat4 pinv = limb.ctrlParentWorld(10).inverse();
        const int N = 400000;
        Euler prev{};
        double sink = 0;
        const auto a = std::chrono::steady_clock::now();
        for (int i = 0; i < N; ++i) {
            const MatchResult m = match(fk, job.calibration, job.settings);
            const ControlChannels c = solveChannels(job.ctrlDesc, m.ikCtrlWorld, pinv, prev, true);
            prev = c.rotate;
            sink += c.translate.x + m.poleWorld.y;
        }
        const auto b = std::chrono::steady_clock::now();
        const double s = std::chrono::duration<double>(b - a).count();
        std::printf("  match+solve only: %.3f us per limb-sample  (100k frames x 4 limbs = %.2f s)  [%g]\n",
                    s / N * 1e6, s / N * 400000, sink > 0 ? 1.0 : 0.0);
    }

    std::printf("\n%s (%d failed checks)\n", g_fail ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
