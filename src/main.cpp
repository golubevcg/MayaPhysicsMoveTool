#include <CustomMoveManip.h>
#include <CustomMoveManipContext.h>
#include <maya/MFnPlugin.h>
#include <MayaIncludes.h>
//
// moveManipContext
//
// This is the command that will be used to create instances
// of our context.
//
class CustoMoveManipContext : public MPxContextCommand {
public:
    CustoMoveManipContext() {};
    MPoint getCurrentPosition();
    MPxContext* makeObj() override;

public:
    static void* creator();
};

MPxContext* CustoMoveManipContext::makeObj() {
    return new CustomMoveManipContext();
}

void* CustoMoveManipContext::creator() {
    return new CustoMoveManipContext;
}

//
// The following routines are used to register/unregister
// the context and manipulator
//
MStatus initializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj, PLUGIN_COMPANY, "1.0", "Andrew Golubev");
    status = plugin.registerContextCommand("customMoveManipContext",
        &CustoMoveManipContext::creator);
    if (!status) {
        MGlobal::displayError("Error registering customMoveManipContext command");
        return status;
    }
    status = plugin.registerNode("customMoveManip", CustomMoveManip::id,
        &CustomMoveManip::creator, &CustomMoveManip::initialize,
        MPxNode::kManipContainer);
    if (!status) {
        MGlobal::displayError("Error registering customMoveManip node");
        return status;
    }
    return status;
}

MStatus uninitializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj);
    status = plugin.deregisterContextCommand("customMoveManipContext");
    if (!status) {
        MGlobal::displayError("Error deregistering customMoveManipContext command");
        return status;
    }
    status = plugin.deregisterNode(CustomMoveManip::id);
    if (!status) {
        MGlobal::displayError("Error deregistering customMoveManip node");
        return status;
    }
    return status;
}
