#include "PhysicsManips/PhysicsMoveManip.h"
#include "PhysicsContext/PhysicsManipContext.h"
#include <maya/MFnPlugin.h>
#include "MayaIncludes.h"

/**
 * @class PhysicsManipContextCommand
 * @brief Command class for creating instances of PhysicsMoveManipContext.
 *
 * This class is responsible for creating and managing instances of the
 * PhysicsMoveManipContext. It extends MPxContextCommand to integrate with Maya's command system.
 */
class PhysicsManipContextCommand : public MPxContextCommand {
    public:
        PhysicsManipContextCommand() {};
        MPxContext* makeObj() override;
        static void* creator();
};

MPxContext* PhysicsManipContextCommand::makeObj() {
    return new PhysicsManipContext();
}

void* PhysicsManipContextCommand::creator() {
    return new PhysicsManipContextCommand;
}

/**
 * @brief Initializes the plugin when loaded into Maya.
 * @param obj The plugin object.
 * @return Status of the initialization.
 */
MStatus initializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj, PLUGIN_COMPANY, "1.0", "Andrew Golubev");
    status = plugin.registerContextCommand("physicsManipContext",
        &PhysicsManipContextCommand::creator);
    if (!status) {
        MGlobal::displayError("Error registering physicsManipContext command");
        return status;
    }
    status = plugin.registerNode("physicsMoveManip", PhysicsMoveManip::id,
        &PhysicsMoveManip::creator, &PhysicsMoveManip::initialize,
        MPxNode::kManipContainer);
    if (!status) {
        MGlobal::displayError("Error registering physicsMoveManip node");
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
    status = plugin.deregisterContextCommand("physicsManipContext");
    if (!status) {
        MGlobal::displayError("Error deregistering physicsManipContext command");
        return status;
    }
    status = plugin.deregisterNode(PhysicsMoveManip::id);
    if (!status) {
        MGlobal::displayError("Error deregistering physicsMoveManip node");
        return status;
    }
    return status;
}
