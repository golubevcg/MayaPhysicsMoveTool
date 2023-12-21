#include <CustomMoveManip.h>
#include <CustomMoveManipContext.h>
#include <maya/MFnPlugin.h>
#include <MayaIncludes.h>

/**
 * @class CustomMoveManipContextCommand
 * @brief Command class for creating instances of CustomMoveManipContext.
 *
 * This class is responsible for creating and managing instances of the
 * CustomMoveManipContext. It extends MPxContextCommand to integrate with Maya's command system.
 */
class CustomMoveManipContextCommand : public MPxContextCommand {
    public:
        CustomMoveManipContextCommand() {};
        MPxContext* makeObj() override;
        static void* creator();
};

MPxContext* CustomMoveManipContextCommand::makeObj() {
    return new CustomMoveManipContext();
}

void* CustomMoveManipContextCommand::creator() {
    return new CustomMoveManipContextCommand;
}

/**
 * @brief Initializes the plugin when loaded into Maya.
 * @param obj The plugin object.
 * @return Status of the initialization.
 */
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

/**
 * @brief Uninitializes the plugin when unloaded from Maya.
 * @param obj The plugin object.
 * @return Status of the uninitialization.
 */
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
