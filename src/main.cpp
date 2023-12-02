#include <maya/MFnPlugin.h>
#include <MayaIncludes.h>
#include <CustomMoveManipContext.h>

//
// moveManipContextCommand
// This is the command that will be used to create instances
// of our context.
//
class CustomMoveManipContextCommand : public MPxContextCommand {
public:
    CustomMoveManipContextCommand() {};
    MPxContext* makeObj() override;
public:
    static void* creator();
};


MPxContext* CustomMoveManipContextCommand::makeObj() {
    return new CustomMoveManipContext();
}

void* CustomMoveManipContextCommand::creator() {
    return new CustomMoveManipContext();
}

//
// register/unregister the context and manipulator
//
MStatus initializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj, PLUGIN_COMPANY, "1.0", "Andrew Golubev");
    status = plugin.registerContextCommand("customMoveManipContext",
        &CustomMoveManipContextCommand::creator);
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
