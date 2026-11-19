#include "ControlSolver.h"

namespace fk2ik {

namespace {

Mat4 linearPart(const ControlTransformDesc& d, const Mat4& R) {
    const Mat4 RA = eulerToMatrix(d.rotateAxis, RotateOrder::XYZ);
    if (d.isJoint) {
        const Mat4 JO = eulerToMatrix(d.jointOrient, RotateOrder::XYZ);
        return Mat4::scale(d.scale) * RA * R * JO * Mat4::scale(d.inverseParentScale);
    }
    return Mat4::scale(d.scale) * Mat4::shear(d.shear) * RA * R;
}

}  // namespace

Mat4 composeLocal(const ControlTransformDesc& d, const Vec3& t, const Euler& r) {
    const Mat4 R = eulerToMatrix(r, d.rotateOrder);
    if (d.isJoint) {
        Mat4 m = linearPart(d, R);
        m.setTranslation(t);
        return m;
    }
    const Mat4 RA = eulerToMatrix(d.rotateAxis, RotateOrder::XYZ);
    Mat4 m = Mat4::translation(-d.scalePivot) * Mat4::scale(d.scale) * Mat4::shear(d.shear) *
             Mat4::translation(d.scalePivot) * Mat4::translation(d.scalePivotTranslate) *
             Mat4::translation(-d.rotatePivot) * RA * R * Mat4::translation(d.rotatePivot) *
             Mat4::translation(d.rotatePivotTranslate) * Mat4::translation(t);
    return m;
}

ControlChannels solveChannels(const ControlTransformDesc& d, const Mat4& world,
                              const Mat4& parentInverse, const Euler& prev, bool hasPrev) {
    const Mat4 local = world * parentInverse;

    // Linear 3x3 of `local` equals  pre * R * post  where pre/post are static.
    const Mat4 RA = eulerToMatrix(d.rotateAxis, RotateOrder::XYZ);
    Mat4 pre, post;
    if (d.isJoint) {
        pre = Mat4::scale(d.scale) * RA;
        post = eulerToMatrix(d.jointOrient, RotateOrder::XYZ) * Mat4::scale(d.inverseParentScale);
    } else {
        pre = Mat4::scale(d.scale) * Mat4::shear(d.shear) * RA;
    }
    Mat4 L = local;
    L.setTranslation({0, 0, 0});
    Mat4 R = pre.inverse() * L * post.inverse();
    R = R.rotationOnly();  // remove numerical drift / residual parent scale

    ControlChannels out;
    out.rotate = matrixToEuler(R, d.rotateOrder);
    if (hasPrev) out.rotate = closestEuler(out.rotate, prev, d.rotateOrder);

    // Translation: everything except T is now known; T is a pure offset.
    const Mat4 m0 = composeLocal(d, {0, 0, 0}, out.rotate);
    out.translate = local.getTranslation() - m0.getTranslation();
    return out;
}

}  // namespace fk2ik
