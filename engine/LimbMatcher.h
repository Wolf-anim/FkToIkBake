// FkToIkBake - Matching and Baking Engine
// LimbMatcher.h : the FK -> IK matching math for one two-bone limb.
//
// Inputs are world matrices only, so the matcher has no knowledge of Maya,
// node names or rig conventions.
//
// Calibration (once per bake, measured on the IK side of the rig):
//   ctrlOffset   = ikCtrlWorld * inverse(ikEndJointWorld)
//                  -> how the IK control sits relative to the IK wrist/ankle.
//   normalLocal  = chain-plane normal expressed in the IK elbow/knee frame
//                  -> drives the ROTATION matching mode.
//   poleAngle    = signed angle (about the upper->end axis) between the chain
//                  plane and the pole plane; absorbs ikHandle twist and
//                  pole offsets so the POSITION mode is exact for them too.
//   poleDistance = distance of the pole from the upper->end axis.
//
// Per sample (FK side):
//   ikCtrlTarget = ctrlOffset * fkEndWorld           (wrist/ankle pos + orient)
//   pole         = POSITION mode: in the plane of fk upper/mid/end, on the
//                  elbow/knee side, rotated by poleAngle; smoothly falls back
//                  to the rotation-derived direction as the limb straightens.
//                  ROTATION mode: the bend plane is taken from the FK
//                  elbow/knee ROTATION (normalLocal * fkMid) instead of the
//                  elbow position, then the same poleAngle is applied.  Stable
//                  on straight / nearly straight limbs and preserves elbow
//                  twist, like Advanced Skeleton's "MotionCapture" option.
#pragma once

#include "Math.h"

namespace fk2ik {

enum class LimbType { Arm, Leg };
enum class MatchMode { Position, Rotation };

struct LimbSettings {
    LimbType type = LimbType::Arm;
    MatchMode mode = MatchMode::Position;
    // Straight-limb fallback band, as (elbow distance from the upper->end
    // line) / (chain length).  Below `straightLow` the rotation-derived pole
    // direction is used; above `straightHigh` the positional one; smooth
    // blend in between.  Only used in Position mode.
    // For two equal bones, straightness ~= bendAngle / 4, so the defaults
    // engage only within ~1.5 degrees (arm) / ~2 degrees (leg) of straight.
    double straightLow = 0.001;
    double straightHigh = 0.006;
    // <= 0 : use the calibrated rig pole distance.
    double poleDistance = -1.0;

    static LimbSettings defaultsFor(LimbType t) {
        LimbSettings s;
        s.type = t;
        if (t == LimbType::Leg) {
            // Legs spend much more time nearly straight (standing, walking
            // contact poses), so start the fallback blend a bit earlier.
            s.straightLow = 0.0015;
            s.straightHigh = 0.008;
        }
        return s;
    }
};

struct LimbPose {  // world matrices for one sample
    Mat4 upper, mid, end;
};

struct LimbCalibration {
    Mat4 ctrlOffset;
    Vec3 normalLocal;  // unit, in IK mid joint rotation frame
    double poleAngle = 0.0;
    double poleDistance = 1.0;
    double upperLength = 0.0, lowerLength = 0.0;
    bool valid = false;
};

struct MatchResult {
    Mat4 ikCtrlWorld;
    Vec3 poleWorld;
    double straightness = 0.0;  // elbow offset / chain length (0 = straight)
    double reachRatio = 0.0;    // |end-upper| / (upper+lower IK length); >1 = over-extended
    double fallbackWeight = 0.0;  // 1 = rotation-derived pole used entirely
};

// Measure calibration from the IK chain + IK controls at one moment.
LimbCalibration calibrate(const LimbPose& ikPose, const Mat4& ikCtrlWorld, const Vec3& poleWorld);

// Compute IK control + pole targets for one FK sample.
MatchResult match(const LimbPose& fkPose, const LimbCalibration& cal, const LimbSettings& s);

}  // namespace fk2ik
