// FkToIkBake - Maya integration layer
// FkToIkBakeCmd.h : the `fkToIkBake` MEL/Python command.  Thin layer: parses
// flags, resolves the rig, hands world matrices to the engine and writes the
// engine's channel arrays to animation curves (undoable).
#pragma once

#include <maya/MAnimCurveChange.h>
#include <maya/MDGModifier.h>
#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>

class FkToIkBakeCmd : public MPxCommand {
public:
    static const char* kName;
    static void* creator() { return new FkToIkBakeCmd; }
    static MSyntax newSyntax();

    MStatus doIt(const MArgList& args) override;
    MStatus redoIt() override;
    MStatus undoIt() override;
    bool isUndoable() const override { return undoable_; }

private:
    MDGModifier curveMod_;   // creates / connects new anim curves
    MDGModifier valueMod_;   // sets un-animated switch attributes
    MAnimCurveChange animChange_;
    bool undoable_ = false;
};
