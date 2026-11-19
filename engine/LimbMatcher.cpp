#include "LimbMatcher.h"

#include <algorithm>

namespace fk2ik {

namespace {

inline double smoothstep(double e0, double e1, double x) {
    if (e1 <= e0) return x >= e1 ? 1.0 : 0.0;
    const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

}  // namespace

LimbCalibration calibrate(const LimbPose& ik, const Mat4& ikCtrlWorld, const Vec3& poleWorld) {
    LimbCalibration c;
    c.ctrlOffset = ikCtrlWorld * ik.end.inverse();

    const Vec3 pu = ik.upper.getTranslation();
    const Vec3 pm = ik.mid.getTranslation();
    const Vec3 pe = ik.end.getTranslation();
    c.upperLength = length(pm - pu);
    c.lowerLength = length(pe - pm);
    const double chain = c.upperLength + c.lowerLength;

    const Vec3 axis = normalized(pe - pu);
    const Vec3 elbowPerp = perpendicular(pm - pu, axis);
    const Vec3 polePerp = perpendicular(poleWorld - pu, axis);
    c.poleDistance = length(polePerp);

    Vec3 side;  // unit direction from the upper->end line toward the elbow
    if (length(elbowPerp) > 1e-6 * chain) {
        side = normalized(elbowPerp);
        c.poleAngle = c.poleDistance > kEps ? signedAngle(side, normalized(polePerp), axis) : 0.0;
    } else {
        // Calibrated on a straight chain: the pole defines the plane.
        side = normalized(polePerp);
        c.poleAngle = 0.0;
    }
    const Vec3 normal = cross(axis, side);  // so that side == cross(normal, axis)
    c.normalLocal = ik.mid.rotationOnly().transposed3x3().transformVector(normal);

    if (c.poleDistance < 1e-6) c.poleDistance = chain;
    c.valid = (c.upperLength > kEps && c.lowerLength > kEps && length(side) > 0.5);
    return c;
}

MatchResult match(const LimbPose& fk, const LimbCalibration& cal, const LimbSettings& s) {
    MatchResult r;
    r.ikCtrlWorld = cal.ctrlOffset * fk.end;

    const Vec3 pu = fk.upper.getTranslation();
    const Vec3 pm = fk.mid.getTranslation();
    const Vec3 pe = fk.end.getTranslation();
    const double chain = cal.upperLength + cal.lowerLength;
    const Vec3 axis = normalized(pe - pu);
    r.reachRatio = chain > kEps ? length(pe - pu) / chain : 0.0;

    const Vec3 elbowPerp = perpendicular(pm - pu, axis);
    const double elbowOff = length(elbowPerp);
    r.straightness = chain > kEps ? elbowOff / chain : 0.0;
    const double dist = s.poleDistance > 0 ? s.poleDistance : cal.poleDistance;
    const Vec3 foot = pu + axis * dot(pm - pu, axis);

    // Elbow side from the FK elbow/knee ROTATION.
    const Vec3 nWorld = fk.mid.rotationOnly().transformVector(cal.normalLocal);
    Vec3 sideRot = normalized(perpendicular(cross(nWorld, axis), axis));
    // Elbow side from the FK elbow/knee POSITION.
    Vec3 sidePos = elbowOff > kEps ? elbowPerp / elbowOff : Vec3{0, 0, 0};

    Vec3 side;
    if (s.mode == MatchMode::Rotation) {
        side = length(sideRot) > 0.5 ? sideRot : sidePos;
        r.fallbackWeight = 1.0;
    } else {
        double w = 1.0 - smoothstep(s.straightLow, s.straightHigh, r.straightness);
        if (length(sidePos) < 0.5) w = 1.0;
        if (length(sideRot) < 0.5) w = 0.0;
        r.fallbackWeight = w;
        if (w <= 0.0) side = sidePos;
        else if (w >= 1.0) side = sideRot;
        else side = rotateAbout(sidePos, axis, signedAngle(sidePos, sideRot, axis) * w);
    }
    if (length(side) < 0.5) side = normalized(perpendicular(fk.mid.rotationOnly().row(1), axis));
    r.poleWorld = foot + rotateAbout(side, axis, cal.poleAngle) * dist;
    return r;
}

}  // namespace fk2ik
