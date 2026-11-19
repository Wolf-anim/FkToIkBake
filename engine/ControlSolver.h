// FkToIkBake - Matching and Baking Engine
// ControlSolver.h : converts a desired WORLD matrix for a control into the
// control's own keyable channels (translate / rotate), honoring every static
// part of Maya's transform stack.  This is what makes the tool rig-agnostic:
// controls with offset parents, rotate axis, pivots, joint orient, non-unit
// scale or a different rotate order all solve exactly.
//
// Maya transform:  M = [-SP][S][SH][SP][SPT][-RP][RA][R][RP][RPT][T]
// Maya joint:      M = [S][RA][R][JO][IS][T]   (IS = inverse parent scale)
#pragma once

#include "Math.h"

namespace fk2ik {

struct ControlTransformDesc {
    bool isJoint = false;
    RotateOrder rotateOrder = RotateOrder::XYZ;
    Vec3 scale{1, 1, 1};
    Vec3 shear{0, 0, 0};
    Euler rotateAxis{};          // rotateAxis (transform.rotateAxis, joint.rotateAxis)
    Euler jointOrient{};         // joints only (always XYZ order in Maya)
    Vec3 inverseParentScale{1, 1, 1};  // joints with segmentScaleCompensate
    Vec3 rotatePivot{0, 0, 0};
    Vec3 rotatePivotTranslate{0, 0, 0};
    Vec3 scalePivot{0, 0, 0};
    Vec3 scalePivotTranslate{0, 0, 0};
};

struct ControlChannels {
    Vec3 translate;
    Euler rotate;  // radians
};

// Build the local matrix of a control from channels (forward model; used by
// the solver for the translation step and by the validation tests).
Mat4 composeLocal(const ControlTransformDesc& d, const Vec3& t, const Euler& r);

// Solve channels so that  composeLocal(d, t, r) * parentWorld == world.
// `parentInverse` is the control's parentInverseMatrix at the same time.
// `prev` is the previous sample's rotation used for Euler continuity; pass
// `hasPrev=false` for the first sample.
ControlChannels solveChannels(const ControlTransformDesc& d, const Mat4& world,
                              const Mat4& parentInverse, const Euler& prev, bool hasPrev);

}  // namespace fk2ik
