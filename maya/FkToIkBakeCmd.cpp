#include "FkToIkBakeCmd.h"

#include <maya/MAnimControl.h>
#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MComputation.h>
#include <maya/MDoubleArray.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MPlugArray.h>
#include <maya/MStringArray.h>
#include <maya/MTime.h>
#include <maya/MTimeArray.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../engine/BakeEngine.h"
#include "../engine/KeyReducer.h"
#include "../engine/RigMap.h"
#include "MayaRig.h"

using namespace fk2ik;
using namespace fk2ik::maya;

const char* FkToIkBakeCmd::kName = "fkToIkBake";

#define F_MAP "-mp", "-rigMap"
#define F_LIMB "-l", "-limb"
#define F_NS "-ns", "-namespace"
#define F_START "-sf", "-startFrame"
#define F_END "-ef", "-endFrame"
#define F_ANIMRANGE "-ar", "-animationRange"
#define F_STEP "-st", "-step"
#define F_MODE "-mo", "-mode"
#define F_TOL "-tol", "-tolerance"
#define F_SWITCH "-sik", "-switchToIk"
#define F_CALFRAME "-cf", "-calibrationFrame"
#define F_EVAL "-ev", "-evalMode"
#define F_VALIDATE "-val", "-validate"
#define F_POLEDIST "-pd", "-poleDistance"
#define F_TYPE "-lt", "-type"
#define F_FK "-fk", "-fkJoints"
#define F_IK "-ik", "-ikJoints"
#define F_IKCTRL "-ic", "-ikControl"
#define F_POLE "-pc", "-poleControl"
#define F_SWATTR "-sw", "-switchAttr"
#define F_SWVAL "-sv", "-switchValues"

MSyntax FkToIkBakeCmd::newSyntax() {
    MSyntax s;
    s.addFlag(F_MAP, MSyntax::kString);
    s.addFlag(F_LIMB, MSyntax::kString);
    s.makeFlagMultiUse("-l");
    s.addFlag(F_NS, MSyntax::kString);
    s.addFlag(F_START, MSyntax::kDouble);
    s.addFlag(F_END, MSyntax::kDouble);
    s.addFlag(F_ANIMRANGE);
    s.addFlag(F_STEP, MSyntax::kDouble);
    s.addFlag(F_MODE, MSyntax::kString);
    s.addFlag(F_TOL, MSyntax::kDouble);
    s.addFlag(F_SWITCH, MSyntax::kBoolean);
    s.addFlag(F_CALFRAME, MSyntax::kDouble);
    s.addFlag(F_EVAL, MSyntax::kString);
    s.addFlag(F_VALIDATE);
    s.addFlag(F_POLEDIST, MSyntax::kDouble);
    s.addFlag(F_TYPE, MSyntax::kString);
    s.addFlag(F_FK, MSyntax::kString, MSyntax::kString, MSyntax::kString);
    s.addFlag(F_IK, MSyntax::kString, MSyntax::kString, MSyntax::kString);
    s.addFlag(F_IKCTRL, MSyntax::kString);
    s.addFlag(F_POLE, MSyntax::kString);
    s.addFlag(F_SWATTR, MSyntax::kString);
    s.addFlag(F_SWVAL, MSyntax::kDouble, MSyntax::kDouble);
    return s;
}

namespace {

std::string str(const MString& m) { return std::string(m.asChar()); }

MStatus fail(const MString& msg) {
    MGlobal::displayError(MString("fkToIkBake: ") + msg);
    return MS::kFailure;
}

double nowSec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Find (or schedule creation of) the time-based anim curve driving `plug`.
MStatus curveFor(const MPlug& plug, MDGModifier& mod, MObject& curve, MString& err) {
    if (plug.isLocked()) {
        err = plug.name() + " is locked";
        return MS::kFailure;
    }
    MPlugArray src;
    plug.connectedTo(src, true, false);
    if (src.length() > 0) {
        MObject n = src[0].node();
        if (n.hasFn(MFn::kAnimCurve)) {
            MFnAnimCurve fn(n);
            const MFnAnimCurve::AnimCurveType t = fn.animCurveType();
            if (t == MFnAnimCurve::kAnimCurveTA || t == MFnAnimCurve::kAnimCurveTL ||
                t == MFnAnimCurve::kAnimCurveTU) {
                curve = n;
                return MS::kSuccess;
            }
            err = plug.name() + " is driven by a driven-key curve";
            return MS::kFailure;
        }
        err = plug.name() + " is driven by " + MFnDependencyNode(n).name() +
              " (constraint / anim layer / expression). Bake onto a free channel or the base layer.";
        return MS::kFailure;
    }
    MStatus st;
    MFnAnimCurve fn;
    curve = fn.create(plug, &mod, &st);
    if (!st) {
        err = MString("cannot create anim curve for ") + plug.name();
        return MS::kFailure;
    }
    return MS::kSuccess;
}

void removeKeysInRange(MFnAnimCurve& fn, double start, double end, MAnimCurveChange* change) {
    for (int k = static_cast<int>(fn.numKeys()) - 1; k >= 0; --k) {
        const double f = fn.time(static_cast<unsigned>(k)).as(MTime::uiUnit());
        if (f >= start - 1e-6 && f <= end + 1e-6) fn.remove(static_cast<unsigned>(k), change);
    }
}

struct Options {
    std::string mapPath, ns, modeOverride, typeInline;
    std::vector<std::string> limbs;
    double start = 0, end = 0, step = 1, tol = 0, calFrame = 0, poleDist = -1;
    bool hasCalFrame = false, switchToIk = true, validate = false;
    EvalMode eval = EvalMode::Context;
};

}  // namespace

MStatus FkToIkBakeCmd::doIt(const MArgList& args) {
    MStatus st;
    MArgDatabase db(syntax(), args, &st);
    if (!st) return st;
    const double t0 = nowSec();

    // ------------------------------------------------------------ options
    Options o;
    MString ms;
    if (db.isFlagSet("-mp")) { db.getFlagArgument("-mp", 0, ms); o.mapPath = str(ms); }
    if (db.isFlagSet("-ns")) { db.getFlagArgument("-ns", 0, ms); o.ns = str(ms); }
    if (!o.ns.empty() && o.ns.back() != ':') o.ns += ':';
    for (unsigned i = 0; i < db.numberOfFlagUses("-l"); ++i) {
        MArgList a;
        db.getFlagArgumentList("-l", i, a);
        o.limbs.push_back(str(a.asString(0)));
    }
    if (db.isFlagSet("-ar")) {
        o.start = MAnimControl::animationStartTime().as(MTime::uiUnit());
        o.end = MAnimControl::animationEndTime().as(MTime::uiUnit());
    } else {
        o.start = MAnimControl::minTime().as(MTime::uiUnit());
        o.end = MAnimControl::maxTime().as(MTime::uiUnit());
    }
    if (db.isFlagSet("-sf")) db.getFlagArgument("-sf", 0, o.start);
    if (db.isFlagSet("-ef")) db.getFlagArgument("-ef", 0, o.end);
    if (db.isFlagSet("-st")) db.getFlagArgument("-st", 0, o.step);
    if (db.isFlagSet("-tol")) db.getFlagArgument("-tol", 0, o.tol);
    if (db.isFlagSet("-sik")) db.getFlagArgument("-sik", 0, o.switchToIk);
    if (db.isFlagSet("-cf")) { db.getFlagArgument("-cf", 0, o.calFrame); o.hasCalFrame = true; }
    if (db.isFlagSet("-pd")) db.getFlagArgument("-pd", 0, o.poleDist);
    o.validate = db.isFlagSet("-val");
    if (db.isFlagSet("-mo")) { db.getFlagArgument("-mo", 0, ms); o.modeOverride = str(ms.toLowerCase()); }
    if (db.isFlagSet("-ev")) {
        db.getFlagArgument("-ev", 0, ms);
        const std::string e = str(ms.toLowerCase());
        if (e == "timeline") o.eval = EvalMode::Timeline;
        else if (e != "context") return fail("-evalMode must be context or timeline");
    }
    if (o.end < o.start) return fail("end frame is before start frame");
    if (o.step <= 0) return fail("-step must be > 0");
    if (!o.hasCalFrame) o.calFrame = o.start;
    if (!o.modeOverride.empty() && o.modeOverride != "position" && o.modeOverride != "rotation" &&
        o.modeOverride != "mocap")
        return fail("-mode must be position or rotation");

    // ------------------------------------------------------------ limbs
    std::vector<LimbDefinition> defs;
    std::string err;
    if (!o.mapPath.empty()) {
        RigMap map;
        if (!loadRigMap(o.mapPath, map, err)) return fail(err.c_str());
        if (o.limbs.empty()) defs = map.limbs;
        for (auto& n : o.limbs) {
            const LimbDefinition* d = map.find(n);
            if (!d) return fail(MString("limb not in map: ") + n.c_str());
            defs.push_back(*d);
        }
    } else {
        if (!db.isFlagSet("-fk") || !db.isFlagSet("-ik") || !db.isFlagSet("-ic") || !db.isFlagSet("-pc"))
            return fail("give -rigMap, or -fkJoints/-ikJoints/-ikControl/-poleControl");
        LimbDefinition d;
        d.name = "limb";
        MString a, b, c;
        db.getFlagArgument("-fk", 0, a); db.getFlagArgument("-fk", 1, b); db.getFlagArgument("-fk", 2, c);
        d.fkUpper = str(a); d.fkMid = str(b); d.fkEnd = str(c);
        db.getFlagArgument("-ik", 0, a); db.getFlagArgument("-ik", 1, b); db.getFlagArgument("-ik", 2, c);
        d.ikUpper = str(a); d.ikMid = str(b); d.ikEnd = str(c);
        db.getFlagArgument("-ic", 0, a); d.ikControl = str(a);
        db.getFlagArgument("-pc", 0, a); d.poleControl = str(a);
        if (db.isFlagSet("-lt")) {
            db.getFlagArgument("-lt", 0, a);
            d.type = str(a.toLowerCase()) == "leg" ? LimbType::Leg : LimbType::Arm;
        }
        if (db.isFlagSet("-sw")) { db.getFlagArgument("-sw", 0, a); d.switchAttr = str(a); }
        if (db.isFlagSet("-sv")) {
            db.getFlagArgument("-sv", 0, d.switchFk);
            db.getFlagArgument("-sv", 1, d.switchIk);
            d.hasSwitchValues = true;
        }
        d.name = d.type == LimbType::Leg ? "leg" : "arm";
        const std::string v = d.validate();
        if (!v.empty()) return fail(v.c_str());
        defs.push_back(d);
    }
    for (auto& d : defs) {
        if (o.modeOverride == "position") d.mode = MatchMode::Position;
        else if (!o.modeOverride.empty()) d.mode = MatchMode::Rotation;
        if (o.poleDist > 0) d.poleDistance = o.poleDist;
    }

    std::vector<ResolvedLimb> limbs(defs.size());
    for (size_t i = 0; i < defs.size(); ++i) {
        MString e;
        if (!resolveLimb(defs[i], o.ns, limbs[i], e)) return fail(MString(defs[i].name.c_str()) + ": " + e);
    }

    // ------------------------------------------------------------ curves (fail early)
    // 9 channels per limb: ctrl t/r, pole t.
    static const char* kCtrlAttrs[6] = {"translateX", "translateY", "translateZ", "rotateX", "rotateY", "rotateZ"};
    static const char* kPoleAttrs[3] = {"translateX", "translateY", "translateZ"};
    std::vector<std::vector<MObject>> curves(limbs.size(), std::vector<MObject>(9));
    for (size_t l = 0; l < limbs.size(); ++l) {
        MFnDependencyNode cfn(limbs[l].ctrl.node()), pfn(limbs[l].pole.node());
        for (int c = 0; c < 9; ++c) {
            MPlug p = c < 6 ? cfn.findPlug(kCtrlAttrs[c], true) : pfn.findPlug(kPoleAttrs[c - 6], true);
            MString e;
            if (!curveFor(p, curveMod_, curves[l][c], e)) return fail(MString(defs[l].name.c_str()) + ": " + e);
        }
    }

    const double tPrep = nowSec();
    const double originalTime = MAnimControl::currentTime().as(MTime::uiUnit());

    // ------------------------------------------------------------ calibration (IK side)
    std::vector<LimbJob> jobs(limbs.size());
    MStringArray warnings;
    forceSwitches(limbs, true);
    {
        TimeScope scope(o.calFrame, o.eval);
        for (size_t l = 0; l < limbs.size(); ++l) {
            ResolvedLimb& r = limbs[l];
            LimbJob& j = jobs[l];
            j.name = r.def.name;
            j.settings = r.def.settings();
            j.ctrlDesc = readControlDesc(r.ctrl);
            j.poleDesc = readControlDesc(r.pole);
            j.poleRotate = readRotate(r.pole);
            j.ctrlStartRotate = readRotate(r.ctrl);
            j.hasStartRotate = true;
            LimbPose ik;
            ik.upper = readMatrix(r.ikWorld[0]);
            ik.mid = readMatrix(r.ikWorld[1]);
            ik.end = readMatrix(r.ikWorld[2]);
            const Mat4 ctrlW = readMatrix(r.ctrlWorld);
            const Mat4 poleW = readMatrix(r.poleWorld);
            const Vec3 polePivot = poleW.transformPoint(j.poleDesc.rotatePivot);
            j.calibration = calibrate(ik, ctrlW, polePivot);
            prepare(j);
            const double reach = length(ik.end.getTranslation() - ik.upper.getTranslation()) /
                                 (j.calibration.upperLength + j.calibration.lowerLength);
            if (reach > 0.999)
                warnings.append(MString(j.name.c_str()) +
                                ": IK chain is (almost) fully extended at the calibration frame; the IK control "
                                "offset may be wrong. Use -calibrationFrame on a frame where the IK limb is bent.");
        }
    }
    if (o.eval == EvalMode::Timeline) MAnimControl::setCurrentTime(MTime(originalTime, MTime::uiUnit()));

    // ------------------------------------------------------------ bake (FK side)
    forceSwitches(limbs, false);
    const std::vector<double> times = buildSampleTimes(o.start, o.end, o.step);
    MayaSampleSource source(limbs, o.eval);
    MComputation comp;
    comp.beginComputation();
    source.computation = &comp;
    BakeEngine engine;
    std::vector<LimbBakeResult> results;
    BakeStats stats;
    const bool ok = engine.run(jobs, source, times, results, stats, err);
    comp.endComputation();
    restoreSwitches(limbs);
    if (o.eval == EvalMode::Timeline) MAnimControl::setCurrentTime(MTime(originalTime, MTime::uiUnit()));
    if (!ok) return fail(err.c_str());
    const double tBake = nowSec();

    // ------------------------------------------------------------ write keys
    curveMod_.doIt();  // create + connect any new curves
    const bool reduce = o.tol > 0;
    const MFnAnimCurve::TangentType tan = reduce ? MFnAnimCurve::kTangentLinear : MFnAnimCurve::kTangentAuto;
    size_t keysWritten = 0, keysDense = 0;
    const double degToRad = 3.14159265358979323846 / 180.0;
    auto writeChannel = [&](MObject& curve, const std::vector<double>& v, double tol) {
        const std::vector<size_t> keep = reduceKeys(times, v, reduce ? tol : 0.0);
        MTimeArray ta;
        MDoubleArray va;
        for (size_t k : keep) {
            ta.append(MTime(times[k], MTime::uiUnit()));
            va.append(v[k]);
        }
        MFnAnimCurve fn(curve);
        removeKeysInRange(fn, o.start, o.end, &animChange_);
        fn.addKeys(&ta, &va, tan, tan, true, &animChange_);
        keysWritten += keep.size();
        keysDense += v.size();
    };
    auto dirtyControls = [&]() {
        MString cmd = "dgdirty";
        for (auto& r : limbs) cmd += MString(" \"") + r.ctrl.fullPathName() + "\" \"" + r.pole.fullPathName() + "\"";
        MGlobal::executeCommand(cmd, false, false);
    };
    // 1) IK controls (translate: units, rotate: -tolerance is in degrees)
    for (size_t l = 0; l < limbs.size(); ++l)
        for (int c = 0; c < 6; ++c) writeChannel(curves[l][c], results[l].ctrl[c], c >= 3 ? o.tol * degToRad : o.tol);

    dirtyControls();
    // 2) Pole controls.  Their parent space may follow the IK control (e.g.
    //    an AS pole "follow" attribute), which has just changed, so re-read
    //    the pole parent matrices with the new IK animation and re-solve the
    //    pole translate for the same world targets.
    for (size_t i = 0; i < times.size(); ++i) {
        TimeScope scope(times[i], o.eval);
        for (size_t l = 0; l < limbs.size(); ++l) {
            const Vec3 t = poleTranslate(jobs[l], readMatrix(limbs[l].poleParentInv), results[l].poleTarget[i]);
            results[l].pole[0][i] = t.x;
            results[l].pole[1][i] = t.y;
            results[l].pole[2][i] = t.z;
        }
    }
    if (o.eval == EvalMode::Timeline) MAnimControl::setCurrentTime(MTime(originalTime, MTime::uiUnit()));
    for (size_t l = 0; l < limbs.size(); ++l)
        for (int c = 0; c < 3; ++c) writeChannel(curves[l][6 + c], results[l].pole[c], o.tol);

    // ------------------------------------------------------------ switch to IK
    if (o.switchToIk) {
        for (auto& r : limbs) {
            if (r.switchPlug.isNull() || !r.def.hasSwitchValues) continue;
            MPlugArray src;
            r.switchPlug.connectedTo(src, true, false);
            if (src.length() > 0 && src[0].node().hasFn(MFn::kAnimCurve)) {
                MFnAnimCurve fn(src[0].node());
                removeKeysInRange(fn, o.start, o.end, &animChange_);
                fn.addKey(MTime(o.start, MTime::uiUnit()), r.def.switchIk, MFnAnimCurve::kTangentStep,
                          MFnAnimCurve::kTangentStep, &animChange_);
                fn.addKey(MTime(o.end, MTime::uiUnit()), r.def.switchIk, MFnAnimCurve::kTangentStep,
                          MFnAnimCurve::kTangentStep, &animChange_);
            } else if (r.switchSettable) {
                if (!valueMod_.newPlugValueDouble(r.switchPlug, r.def.switchIk))
                    valueMod_.newPlugValueInt(r.switchPlug, static_cast<int>(std::lround(r.def.switchIk)));
            } else {
                warnings.append(MString(r.def.name.c_str()) + ": switch is driven by a non-curve connection; not changed");
            }
        }
        valueMod_.doIt();
    }
    dirtyControls();
    undoable_ = true;
    const double tWrite = nowSec();

    // ------------------------------------------------------------ validate
    std::vector<double> maxErr(limbs.size() * 4, 0.0), meanErr(limbs.size() * 3, 0.0);
    if (o.validate) {
        for (size_t i = 0; i < times.size(); ++i) {
            TimeScope scope(times[i], o.eval);
            for (size_t l = 0; l < limbs.size(); ++l) {
                for (int j = 0; j < 3; ++j) {
                    const Mat4 m = readMatrix(limbs[l].ikWorld[j]);
                    const double e = length(m.getTranslation() - source.fkPositions[l][i * 3 + j]);
                    maxErr[l * 4 + j] = std::max(maxErr[l * 4 + j], e);
                    meanErr[l * 3 + j] += e / static_cast<double>(times.size());
                    if (j == 2) {
                        const Mat4 rel = m.rotationOnly() * source.fkEndRot[l][i].transposed3x3();
                        const double tr = rel.m[0][0] + rel.m[1][1] + rel.m[2][2];
                        const double ang = std::acos(std::clamp((tr - 1.0) * 0.5, -1.0, 1.0)) / degToRad;
                        maxErr[l * 4 + 3] = std::max(maxErr[l * 4 + 3], ang);
                    }
                }
            }
        }
        if (o.eval == EvalMode::Timeline) MAnimControl::setCurrentTime(MTime(originalTime, MTime::uiUnit()));
    }
    const double tEnd = nowSec();

    // ------------------------------------------------------------ report
    MStringArray result;
    char buf[512];
    for (size_t l = 0; l < limbs.size(); ++l) {
        const LimbBakeResult& r = results[l];
        std::snprintf(buf, sizeof(buf),
                      "%s [%s, %s]: %zu samples, maxReach %.4f, overExtended %d, straightFallback %d",
                      jobs[l].name.c_str(), jobs[l].settings.type == LimbType::Leg ? "leg" : "arm",
                      jobs[l].settings.mode == MatchMode::Rotation ? "rotation" : "position", times.size(),
                      r.maxReachRatio, r.overExtendedSamples, r.straightFallbackSamples);
        MString line(buf);
        if (o.validate) {
            std::snprintf(buf, sizeof(buf),
                          " | error max/mean: upper %.2e/%.2e  mid %.2e/%.2e  end %.2e/%.2e (units), end rot max %.3f deg",
                          maxErr[l * 4 + 0], meanErr[l * 3 + 0], maxErr[l * 4 + 1], meanErr[l * 3 + 1],
                          maxErr[l * 4 + 2], meanErr[l * 3 + 2], maxErr[l * 4 + 3]);
            line += buf;
        }
        result.append(line);
        MGlobal::displayInfo(MString("fkToIkBake: ") + line);
        if (r.overExtendedSamples > 0)
            warnings.append(MString(jobs[l].name.c_str()) +
                            ": FK limb is longer than the IK chain on some frames (FK stretch?) - IK cannot reach there.");
    }
    std::snprintf(buf, sizeof(buf),
                  "timing: total %.3fs | prepare %.3fs | sample %.3fs | solve %.3fs | write %.3fs | validate %.3fs | keys %zu/%zu",
                  tEnd - t0, tPrep - t0, stats.sampleSeconds, stats.solveSeconds, tWrite - tBake, tEnd - tWrite,
                  keysWritten, keysDense);
    MGlobal::displayInfo(MString("fkToIkBake: ") + buf);
    result.append(buf);
    for (unsigned i = 0; i < warnings.length(); ++i) MGlobal::displayWarning(MString("fkToIkBake: ") + warnings[i]);
    setResult(result);
    return MS::kSuccess;
}

MStatus FkToIkBakeCmd::redoIt() {
    curveMod_.doIt();
    animChange_.redoIt();
    valueMod_.doIt();
    return MS::kSuccess;
}

MStatus FkToIkBakeCmd::undoIt() {
    valueMod_.undoIt();
    animChange_.undoIt();
    curveMod_.undoIt();
    return MS::kSuccess;
}
