// FkToIkBake - test-suite
// SyntheticRig.h : a self-contained two-bone rig used to validate the engine
// without Maya.  It deliberately does NOT reuse any engine matching code:
//   * FK chain  : joints with jointOrient + animated rotation under an
//                 animated root (Maya joint matrix [R][JO][T]).
//   * IK chain  : same rest structure, solved by an independent analytic
//                 Rotate-Plane solver (pole vector + twist, like ikRPsolver),
//                 IK end joint orient-constrained (maintain offset) to the IK
//                 control, ikHandle parented under the IK control.
//   * Controls  : IK control and pole control under animated parent spaces,
//                 with rotate order / rotate axis / pivots / parent scale.
#pragma once

#include <functional>
#include <string>

#include "../engine/BakeEngine.h"

namespace fk2ik::test {

struct RigConfig {
    std::string name;
    LimbType type = LimbType::Arm;
    Vec3 boneAxis{1, 0, 0};    // joint-local axis pointing down the bone
    int hingeAxis = 2;         // joint-local axis the elbow/knee bends around
    double hingeSign = 1.0;    // positive bend direction
    Euler joUpper, joMid, joEnd;  // joint orients (radians)
    double L1 = 30, L2 = 28;
    double restBend = 0.35;    // preferred angle (radians)
    double twist = 0.0;        // ikHandle twist (radians)
    Vec3 clavicleOffset{5, 140, 0};
    // Control setup
    ControlTransformDesc ctrlDesc, poleDesc;
    Euler ctrlRestWorldRot;    // IK control world orientation at rest
    Vec3 ctrlPivotOffset{0, 0, 0};
    double ctrlParentScale = 1.0;
    bool animateCtrlParent = true;
    bool animatePoleParent = true;
    // FK animation (time in frames -> local Euler, radians)
    std::function<Euler(double)> fkUpperAnim, fkEndAnim;
    std::function<double(double)> fkHingeAnim;  // bend angle about hinge axis
    std::function<Euler(double)> fkMidExtra;    // optional off-hinge elbow rotation
    std::function<Mat4(double)> rootAnim;
    bool absoluteBend = false;
};

class SyntheticLimb {
public:
    explicit SyntheticLimb(const RigConfig& c);

    // FK side
    Mat4 upperParent(double t) const;
    LimbPose fkPose(double t) const;

    // Control spaces
    Mat4 ctrlParentWorld(double t) const;
    Mat4 poleParentWorld(double t) const;

    // IK side: given control world matrix + pole world pivot -> IK joints.
    LimbPose solveIk(double t, const Mat4& ctrlWorld, const Vec3& poleWorld) const;

    // Helpers to evaluate the controls from channel values.
    Mat4 ctrlWorldFromChannels(double t, const Vec3& tr, const Euler& r) const;
    Vec3 poleWorldFromChannels(double t, const Vec3& tr) const;

    const RigConfig& config() const { return cfg_; }
    Mat4 ctrlRestWorld() const { return ctrlRest_; }

private:
    Mat4 jointLocal(const Euler& anim, const Euler& jo, const Vec3& t) const;
    static Mat4 frame(const Vec3& bone, const Vec3& side);

    RigConfig cfg_;
    Mat4 relUpper_, relMid_;  // rest rotation relative to (bone, side) frame
    Mat4 endOffset_;          // ikEndRest * inverse(ctrlRest)
    Mat4 ctrlRest_;
};

// SampleSource backed by synthetic limbs.
class SyntheticSource : public SampleSource {
public:
    std::vector<const SyntheticLimb*> limbs;
    bool sample(double time, std::vector<LimbSample>& out) override;
};

}  // namespace fk2ik::test
