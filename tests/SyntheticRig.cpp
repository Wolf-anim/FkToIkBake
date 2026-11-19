#include "SyntheticRig.h"

#include <algorithm>
#include <cmath>

namespace fk2ik::test {

Mat4 SyntheticLimb::jointLocal(const Euler& anim, const Euler& jo, const Vec3& t) const {
    Mat4 m = eulerToMatrix(anim, RotateOrder::XYZ) * eulerToMatrix(jo, RotateOrder::XYZ);
    m.setTranslation(t);
    return m;
}

Mat4 SyntheticLimb::frame(const Vec3& bone, const Vec3& side) {
    Mat4 f;
    const Vec3 x = normalized(bone);
    const Vec3 y = normalized(perpendicular(side, x));
    f.setRow(0, x);
    f.setRow(1, y);
    f.setRow(2, cross(x, y));
    return f;
}

static Euler hingeEuler(int axis, double a) {
    Euler e;
    e[axis] = a;
    return e;
}

SyntheticLimb::SyntheticLimb(const RigConfig& c) : cfg_(c) {
    // Rest IK chain in upper-parent space (identity parent).
    const Mat4 u = jointLocal({}, c.joUpper, c.clavicleOffset);
    const Mat4 m = jointLocal(hingeEuler(c.hingeAxis, c.hingeSign * c.restBend), c.joMid, c.boneAxis * c.L1) * u;
    const Mat4 e = jointLocal({}, c.joEnd, c.boneAxis * c.L2) * m;
    const Vec3 pu = u.getTranslation(), pm = m.getTranslation(), pe = e.getTranslation();
    const Vec3 side = perpendicular(pm - pu, normalized(pe - pu));  // elbow side
    relUpper_ = u.rotationOnly() * frame(pm - pu, side).transposed3x3();
    relMid_ = m.rotationOnly() * frame(pe - pm, side).transposed3x3();

    // IK control rest: at the wrist, with its own orientation and parent scale.
    const Mat4 restParent = upperParent(0.0);
    const Mat4 eWorld = e * restParent;
    ctrlRest_ = Mat4::scale({c.ctrlParentScale, c.ctrlParentScale, c.ctrlParentScale}) *
                eulerToMatrix(c.ctrlRestWorldRot, RotateOrder::XYZ);
    ctrlRest_.setTranslation(eWorld.getTranslation() + c.ctrlPivotOffset);
    endOffset_ = eWorld * ctrlRest_.inverse();
}

Mat4 SyntheticLimb::upperParent(double t) const {
    return Mat4::identity() * (cfg_.rootAnim ? cfg_.rootAnim(t) : Mat4::identity());
}

LimbPose SyntheticLimb::fkPose(double t) const {
    const Mat4 P = upperParent(t);
    LimbPose p;
    const Mat4 u = jointLocal(cfg_.fkUpperAnim ? cfg_.fkUpperAnim(t) : Euler{}, cfg_.joUpper, cfg_.clavicleOffset);
    double bend = cfg_.hingeSign * (cfg_.fkHingeAnim ? cfg_.fkHingeAnim(t) : cfg_.restBend);
    if (cfg_.absoluteBend) bend -= cfg_.joMid[cfg_.hingeAxis];  // remove joint-orient pre-bend
    Euler mid = hingeEuler(cfg_.hingeAxis, bend);
    if (cfg_.fkMidExtra) {
        const Euler x = cfg_.fkMidExtra(t);
        for (int a = 0; a < 3; ++a) mid[a] += x[a];
    }
    const Mat4 m = jointLocal(mid, cfg_.joMid, cfg_.boneAxis * cfg_.L1);
    const Mat4 e = jointLocal(cfg_.fkEndAnim ? cfg_.fkEndAnim(t) : Euler{}, cfg_.joEnd, cfg_.boneAxis * cfg_.L2);
    p.upper = u * P;
    p.mid = m * p.upper;
    p.end = e * p.mid;
    return p;
}

Mat4 SyntheticLimb::ctrlParentWorld(double t) const {
    // Parent space with a static offset + (optionally) following the root.
    Mat4 off = eulerToMatrix({0.3, -0.2, 0.15}, RotateOrder::XYZ);
    off.setTranslation({2, 1, -3});
    Mat4 s = Mat4::scale({cfg_.ctrlParentScale, cfg_.ctrlParentScale, cfg_.ctrlParentScale});
    Mat4 follow = cfg_.animateCtrlParent && cfg_.rootAnim ? cfg_.rootAnim(t) : Mat4::identity();
    return s * off * follow;
}

Mat4 SyntheticLimb::poleParentWorld(double t) const {
    Mat4 off = eulerToMatrix({-0.4, 0.5, 0.1}, RotateOrder::XYZ);
    off.setTranslation({-1, 3, 2});
    Mat4 follow = Mat4::identity();
    if (cfg_.animatePoleParent) {
        follow = eulerToMatrix({0, 0.4 * std::sin(t * 0.05), 0}, RotateOrder::XYZ);
        follow.setTranslation({std::sin(t * 0.03) * 20, 0, std::cos(t * 0.02) * 10});
        if (cfg_.rootAnim) follow = follow * cfg_.rootAnim(t);
    }
    return off * follow;
}

Mat4 SyntheticLimb::ctrlWorldFromChannels(double t, const Vec3& tr, const Euler& r) const {
    return composeLocal(cfg_.ctrlDesc, tr, r) * ctrlParentWorld(t);
}

Vec3 SyntheticLimb::poleWorldFromChannels(double t, const Vec3& tr) const {
    const Mat4 w = composeLocal(cfg_.poleDesc, tr, Euler{}) * poleParentWorld(t);
    return w.transformPoint(cfg_.poleDesc.rotatePivot);
}

LimbPose SyntheticLimb::solveIk(double t, const Mat4& ctrlWorld, const Vec3& poleWorld) const {
    const Mat4 endW = endOffset_ * ctrlWorld;
    const Vec3 target = endW.getTranslation();
    const Mat4 P = upperParent(t);
    const Vec3 p0 = jointLocal({}, cfg_.joUpper, cfg_.clavicleOffset).getTranslation();
    const Vec3 start = P.transformPoint(p0);

    const double L1 = cfg_.L1, L2 = cfg_.L2;
    Vec3 toT = target - start;
    double d = length(toT);
    const Vec3 a = normalized(toT);
    d = std::clamp(d, std::fabs(L1 - L2) + 1e-9, L1 + L2);
    Vec3 side = normalized(perpendicular(poleWorld - start, a));
    side = rotateAbout(side, a, cfg_.twist);
    const double cosA = std::clamp((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d), -1.0, 1.0);
    const Vec3 elbow = start + a * (L1 * cosA) + side * (L1 * std::sqrt(1 - cosA * cosA));
    const Vec3 wrist = start + a * d;

    LimbPose p;
    p.upper = relUpper_ * frame(elbow - start, side);
    p.upper.setTranslation(start);
    p.mid = relMid_ * frame(wrist - elbow, side);
    p.mid.setTranslation(elbow);
    p.end = endW.rotationOnly();
    p.end.setTranslation(wrist);
    return p;
}

bool SyntheticSource::sample(double t, std::vector<LimbSample>& out) {
    for (size_t i = 0; i < limbs.size(); ++i) {
        out[i].fk = limbs[i]->fkPose(t);
        out[i].ctrlParentInverse = limbs[i]->ctrlParentWorld(t).inverse();
        out[i].poleParentInverse = limbs[i]->poleParentWorld(t).inverse();
    }
    return true;
}

}  // namespace fk2ik::test
