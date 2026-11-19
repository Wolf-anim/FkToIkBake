// FkToIkBake - Maya integration layer
// MayaRig.h : resolves a LimbDefinition to Maya plugs and implements the
// engine's SampleSource by evaluating world matrices with MDGContext (no
// timeline change, no viewport refresh) or, as a fallback for rigs that
// only evaluate correctly at the current time, by moving the timeline.
#pragma once

#include <maya/MComputation.h>
#include <maya/MDagPath.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MStatus.h>
#include <maya/MString.h>

#include <string>
#include <vector>

#include "../engine/BakeEngine.h"
#include "../engine/RigMap.h"

namespace fk2ik::maya {

enum class EvalMode { Context, Timeline };

struct ResolvedLimb {
    LimbDefinition def;
    MDagPath fk[3], ik[3], ctrl, pole;
    MPlug fkWorld[3], ikWorld[3];
    MPlug ctrlWorld, ctrlParentInv, poleWorld, poleParentInv;
    MPlug switchPlug;  // null if none
    bool switchSettable = false;  // not connected / locked: can be forced while sampling
    double switchOriginal = 0.0;
};

// Resolve all node names (with optional namespace prefix).  Fills `error`.
MStatus resolveLimb(const LimbDefinition& def, const std::string& ns, ResolvedLimb& out, MString& error);

// Read the static transform description of a control (rotate order, axis,
// pivots, scale, joint orient ...).
ControlTransformDesc readControlDesc(const MDagPath& path);

// Read a matrix plug at `frame` (ui time units) in the given mode.
Mat4 readMatrix(const MPlug& plug);
Mat4 toMat4(const MMatrix& m);

// Current Euler rotation of a control (radians).
Euler readRotate(const MDagPath& path);

class TimeScope {  // sets the evaluation time for reads inside its lifetime
public:
    TimeScope(double frame, EvalMode mode);
    ~TimeScope();
private:
    struct Impl;
    Impl* impl_;
};

// Set every settable switch to the FK or IK value (used while sampling).
void forceSwitches(std::vector<ResolvedLimb>& limbs, bool ik);
void restoreSwitches(std::vector<ResolvedLimb>& limbs);

class MayaSampleSource : public SampleSource {
public:
    MayaSampleSource(std::vector<ResolvedLimb>& limbs, EvalMode mode) : limbs_(limbs), mode_(mode) {}
    bool sample(double frame, std::vector<LimbSample>& out) override;

    // FK joint world positions recorded during sampling (for validation).
    std::vector<std::vector<Vec3>> fkPositions;  // [limb][sample*3 + joint]
    std::vector<std::vector<Mat4>> fkEndRot;     // [limb][sample]
    MComputation* computation = nullptr;   // optional: Esc to cancel
private:
    std::vector<ResolvedLimb>& limbs_;
    EvalMode mode_;
};

}  // namespace fk2ik::maya
