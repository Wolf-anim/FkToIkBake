// FkToIkBake - plugin entry point.
#include <maya/MFnPlugin.h>

#include "FkToIkBakeCmd.h"

#define FK2IK_VERSION "1.0.0"

MStatus initializePlugin(MObject obj) {
    MFnPlugin plugin(obj, "FkToIkBake", FK2IK_VERSION, "Any");
    return plugin.registerCommand(FkToIkBakeCmd::kName, FkToIkBakeCmd::creator, FkToIkBakeCmd::newSyntax);
}

MStatus uninitializePlugin(MObject obj) {
    MFnPlugin plugin(obj);
    return plugin.deregisterCommand(FkToIkBakeCmd::kName);
}
