#include "MayaRig.h"

#include <maya/MAnimControl.h>
#include <cmath>
#include <maya/MComputation.h>
#include <maya/MDGContext.h>
#include <maya/MDGContextGuard.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnMatrixData.h>
#include <maya/MGlobal.h>
#include <maya/MPlugArray.h>
#include <maya/MSelectionList.h>
#include <maya/MTime.h>

namespace fk2ik::maya {

namespace {

std::string withNamespace(const std::string& name, const std::string& ns) {
    if (ns.empty() || name.empty() || name.find(':') != std::string::npos || name.find('|') != std::string::npos)
        return name;
    return ns + name;
}

MStatus getDag(const std::string& name, MDagPath& path, MString& error) {
    MSelectionList sl;
    if (sl.add(MString(name.c_str())) != MS::kSuccess) {
        error = MString("node not found: ") + name.c_str();
        return MS::kFailure;
    }
    if (sl.length() != 1) {
        error = MString("name is not unique: ") + name.c_str();
        return MS::kFailure;
    }
    if (sl.getDagPath(0, path) != MS::kSuccess) {
        error = MString("not a DAG node: ") + name.c_str();
        return MS::kFailure;
    }
    return MS::kSuccess;
}

MPlug arrayElement(const MDagPath& p, const char* attr) {
    MFnDagNode fn(p);
    MPlug arr = fn.findPlug(attr, true);
    return arr.elementByLogicalIndex(p.instanceNumber());
}

double plugD(const MFnDependencyNode& fn, const char* attr, double def = 0.0) {
    MStatus st;
    MPlug p = fn.findPlug(attr, true, &st);
    if (!st) return def;
    return p.asDouble();
}

Vec3 plug3(const MFnDependencyNode& fn, const char* a, const char* b, const char* c, double def = 0.0) {
    return {plugD(fn, a, def), plugD(fn, b, def), plugD(fn, c, def)};
}

}  // namespace

Mat4 toMat4(const MMatrix& m) {
    Mat4 o;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) o.m[r][c] = m(r, c);
    return o;
}

Mat4 readMatrix(const MPlug& plug) {
    MObject o = plug.asMObject();  // evaluated in the current context (MDGContextGuard)
    MFnMatrixData fn(o);
    return toMat4(fn.matrix());
}

Euler readRotate(const MDagPath& path) {
    MFnDependencyNode fn(path.node());
    const Vec3 r = plug3(fn, "rotateX", "rotateY", "rotateZ");
    return {r.x, r.y, r.z};
}

ControlTransformDesc readControlDesc(const MDagPath& path) {
    MFnDependencyNode fn(path.node());
    ControlTransformDesc d;
    d.isJoint = path.hasFn(MFn::kJoint);
    d.rotateOrder = static_cast<RotateOrder>(static_cast<int>(plugD(fn, "rotateOrder")));
    d.scale = plug3(fn, "scaleX", "scaleY", "scaleZ", 1.0);
    const Vec3 ra = plug3(fn, "rotateAxisX", "rotateAxisY", "rotateAxisZ");
    d.rotateAxis = {ra.x, ra.y, ra.z};
    if (d.isJoint) {
        const Vec3 jo = plug3(fn, "jointOrientX", "jointOrientY", "jointOrientZ");
        d.jointOrient = {jo.x, jo.y, jo.z};
        if (plugD(fn, "segmentScaleCompensate", 1.0) > 0.5) {
            const Vec3 is = plug3(fn, "inverseScaleX", "inverseScaleY", "inverseScaleZ", 1.0);
            d.inverseParentScale = {is.x != 0 ? 1.0 / is.x : 1.0, is.y != 0 ? 1.0 / is.y : 1.0,
                                    is.z != 0 ? 1.0 / is.z : 1.0};
        }
    } else {
        d.shear = plug3(fn, "shearXY", "shearXZ", "shearYZ");
        d.rotatePivot = plug3(fn, "rotatePivotX", "rotatePivotY", "rotatePivotZ");
        d.rotatePivotTranslate = plug3(fn, "rotatePivotTranslateX", "rotatePivotTranslateY", "rotatePivotTranslateZ");
        d.scalePivot = plug3(fn, "scalePivotX", "scalePivotY", "scalePivotZ");
        d.scalePivotTranslate = plug3(fn, "scalePivotTranslateX", "scalePivotTranslateY", "scalePivotTranslateZ");
    }
    return d;
}

MStatus resolveLimb(const LimbDefinition& def, const std::string& ns, ResolvedLimb& out, MString& error) {
    out = ResolvedLimb{};
    out.def = def;
    const std::string fkN[3] = {def.fkUpper, def.fkMid, def.fkEnd};
    const std::string ikN[3] = {def.ikUpper, def.ikMid, def.ikEnd};
    for (int i = 0; i < 3; ++i) {
        if (!getDag(withNamespace(fkN[i], ns), out.fk[i], error)) return MS::kFailure;
        if (!getDag(withNamespace(ikN[i], ns), out.ik[i], error)) return MS::kFailure;
        out.fkWorld[i] = arrayElement(out.fk[i], "worldMatrix");
        out.ikWorld[i] = arrayElement(out.ik[i], "worldMatrix");
    }
    if (!getDag(withNamespace(def.ikControl, ns), out.ctrl, error)) return MS::kFailure;
    if (!getDag(withNamespace(def.poleControl, ns), out.pole, error)) return MS::kFailure;
    out.ctrlWorld = arrayElement(out.ctrl, "worldMatrix");
    out.ctrlParentInv = arrayElement(out.ctrl, "parentInverseMatrix");
    out.poleWorld = arrayElement(out.pole, "worldMatrix");
    out.poleParentInv = arrayElement(out.pole, "parentInverseMatrix");

    if (!def.switchAttr.empty()) {
        const size_t dot = def.switchAttr.find('.');
        const std::string node = withNamespace(def.switchAttr.substr(0, dot), ns);
        const std::string full = node + def.switchAttr.substr(dot);
        MSelectionList sl;
        if (sl.add(MString(full.c_str())) != MS::kSuccess || sl.getPlug(0, out.switchPlug) != MS::kSuccess) {
            error = MString("switch attribute not found: ") + full.c_str();
            return MS::kFailure;
        }
        out.switchOriginal = out.switchPlug.asDouble();
        MPlugArray src;
        out.switchPlug.connectedTo(src, true, false);
        out.switchSettable = !out.switchPlug.isLocked() && src.length() == 0;
    }
    return MS::kSuccess;
}

struct TimeScope::Impl {
    EvalMode mode;
    MDGContext ctx;
    MDGContextGuard* guard = nullptr;
    explicit Impl(double frame, EvalMode m) : mode(m), ctx(MTime(frame, MTime::uiUnit())) {}
};

TimeScope::TimeScope(double frame, EvalMode mode) : impl_(new Impl(frame, mode)) {
    if (mode == EvalMode::Context) {
        // At the current time, read through the normal context: a timed
        // context equal to the current time can return values cached before
        // curves created by this command were connected.
        const double now = MAnimControl::currentTime().as(MTime::uiUnit());
        if (std::fabs(now - frame) > 1e-6) impl_->guard = new MDGContextGuard(impl_->ctx);
    } else {
        MAnimControl::setCurrentTime(MTime(frame, MTime::uiUnit()));
    }
}

TimeScope::~TimeScope() {
    delete impl_->guard;
    delete impl_;
}

void forceSwitches(std::vector<ResolvedLimb>& limbs, bool ik) {
    for (auto& l : limbs)
        if (!l.switchPlug.isNull() && l.switchSettable && l.def.hasSwitchValues)
            l.switchPlug.setValue(ik ? l.def.switchIk : l.def.switchFk);
}

void restoreSwitches(std::vector<ResolvedLimb>& limbs) {
    for (auto& l : limbs)
        if (!l.switchPlug.isNull() && l.switchSettable) l.switchPlug.setValue(l.switchOriginal);
}

bool MayaSampleSource::sample(double frame, std::vector<LimbSample>& out) {
    if (fkPositions.size() != limbs_.size()) {
        fkPositions.assign(limbs_.size(), {});
        fkEndRot.assign(limbs_.size(), {});
    }
    TimeScope scope(frame, mode_);
    for (size_t i = 0; i < limbs_.size(); ++i) {
        ResolvedLimb& l = limbs_[i];
        LimbSample& s = out[i];
        s.fk.upper = readMatrix(l.fkWorld[0]);
        s.fk.mid = readMatrix(l.fkWorld[1]);
        s.fk.end = readMatrix(l.fkWorld[2]);
        s.ctrlParentInverse = readMatrix(l.ctrlParentInv);
        s.poleParentInverse = readMatrix(l.poleParentInv);
        fkPositions[i].push_back(s.fk.upper.getTranslation());
        fkPositions[i].push_back(s.fk.mid.getTranslation());
        fkPositions[i].push_back(s.fk.end.getTranslation());
        fkEndRot[i].push_back(s.fk.end.rotationOnly());
    }
    // Cooperative cancel (Esc) for very long bakes.
    return !(computation && computation->isInterruptRequested());
}

}  // namespace fk2ik::maya
