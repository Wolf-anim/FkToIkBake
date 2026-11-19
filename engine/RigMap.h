// FkToIkBake - Matching and Baking Engine
// RigMap.h : rig-agnostic limb description, loaded from a small INI file.
//
// Universal automatic rig detection is not reliable (naming, hierarchy and
// FK/IK chain layouts differ between Advanced Skeleton, mGear, custom rigs
// ...), so each rig is described once by a map file.  Example:
//
//   [L_arm]
//   type        = arm            ; arm | leg
//   mode        = position       ; position | rotation
//   fkUpper     = FKXShoulder_L  ; nodes that carry the FK pose (joints)
//   fkMid       = FKXElbow_L
//   fkEnd       = FKXWrist_L
//   ikUpper     = IKXShoulder_L  ; IK chain joints (driven by the IK handle)
//   ikMid       = IKXElbow_L
//   ikEnd       = IKXWrist_L
//   ikControl   = IKArm_L
//   poleControl = PoleArm_L
//   switchAttr  = FKIKArm_L.FKIKBlend   ; optional
//   switchFk    = 0
//   switchIk    = 10
//
// Lines starting with ';' or '#' are comments.  Names may be prefixed at
// run time with a namespace (fkToIkBake -namespace "char01:").
#pragma once

#include <string>
#include <vector>

#include "LimbMatcher.h"

namespace fk2ik {

struct LimbDefinition {
    std::string name;
    LimbType type = LimbType::Arm;
    MatchMode mode = MatchMode::Position;
    std::string fkUpper, fkMid, fkEnd;
    std::string ikUpper, ikMid, ikEnd;
    std::string ikControl, poleControl;
    std::string switchAttr;  // "node.attr" or empty
    double switchFk = 0.0, switchIk = 1.0;
    bool hasSwitchValues = false;
    double poleDistance = -1.0;
    double straightLow = -1.0, straightHigh = -1.0;  // <0 : type default

    LimbSettings settings() const;
    // Returns empty string when valid, otherwise a description of the problem.
    std::string validate() const;
};

struct RigMap {
    std::vector<LimbDefinition> limbs;
    const LimbDefinition* find(const std::string& name) const;
};

// Parse map text.  Returns false and fills `error` (with line number) on failure.
bool parseRigMap(const std::string& text, RigMap& out, std::string& error);
bool loadRigMap(const std::string& path, RigMap& out, std::string& error);

}  // namespace fk2ik
