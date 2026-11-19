// FkToIkBake - Matching and Baking Engine
// BakeEngine.h : runs calibration + matching + channel solving over a list of
// sample times for any number of limbs in ONE pass over time (each time is
// evaluated once for all limbs - the expensive part in Maya is evaluation).
//
// The engine never talks to Maya.  A SampleSource supplies world matrices;
// the Maya plugin implements it with MDGContext evaluation, the test-suite
// implements it with a synthetic rig.  Results are plain arrays that the
// caller writes to animation curves.
#pragma once

#include <string>
#include <vector>

#include "ControlSolver.h"
#include "LimbMatcher.h"

namespace fk2ik {

struct LimbJob {
    std::string name;
    LimbSettings settings;
    ControlTransformDesc ctrlDesc;
    ControlTransformDesc poleDesc;
    bool hasPole = true;
    Euler poleRotate;          // pole's current (unkeyed) rotation
    Vec3 polePivotAtZero;      // rotatePivot * composeLocal(poleDesc, 0, poleRotate); set by prepare()
    LimbCalibration calibration;  // filled by the caller via calibrate()
    Euler ctrlStartRotate;        // current control rotation: Euler continuity seed
    bool hasStartRotate = false;
};

struct LimbSample {
    LimbPose fk;
    Mat4 ctrlParentInverse;
    Mat4 poleParentInverse;
};

class SampleSource {
public:
    virtual ~SampleSource() = default;
    // Fill `out` (size == number of limbs) with the FK pose and the control
    // parent-inverse matrices at `time`.  Return false to abort.
    virtual bool sample(double time, std::vector<LimbSample>& out) = 0;
};

struct LimbBakeResult {
    // Channel arrays, one value per sample time.  Rotations in radians.
    std::vector<double> ctrl[6];  // tx ty tz rx ry rz
    std::vector<double> pole[3];  // tx ty tz
    std::vector<Vec3> poleTarget; // world pole position per sample (for a re-solve
                                  // when the pole's parent follows the IK control)
    double maxReachRatio = 0.0;
    int overExtendedSamples = 0;  // reach > 1.0005 : IK cannot reach FK wrist/ankle
    int straightFallbackSamples = 0;
};

struct BakeStats {
    double sampleSeconds = 0.0;  // time spent inside SampleSource::sample
    double solveSeconds = 0.0;   // matching + channel solving
    size_t samples = 0;
};

class BakeEngine {
public:
    // Runs the bake.  `times` must be ascending.  Returns false if the source
    // aborted; `error` receives a message.
    bool run(std::vector<LimbJob>& jobs, SampleSource& source, const std::vector<double>& times,
             std::vector<LimbBakeResult>& results, BakeStats& stats, std::string& error);
};

// Pole translate that puts the pole's rotate pivot at `worldTarget`.
Vec3 poleTranslate(const LimbJob& job, const Mat4& poleParentInverse, const Vec3& worldTarget);

// Fills derived per-job constants (call after descriptors are set).
void prepare(LimbJob& job);

// Build the sample time list for [start, end] with `step`, always including
// `end` exactly (so the last frame is never dropped by float stepping).
std::vector<double> buildSampleTimes(double start, double end, double step);

}  // namespace fk2ik
