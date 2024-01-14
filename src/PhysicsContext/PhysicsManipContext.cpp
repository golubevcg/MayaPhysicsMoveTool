#include "PhysicsManipContext.h"
#include "../PhysicsManips/PhysicsMoveManip.h"
#include "../MayaIncludes.h"

PhysicsManipContext::PhysicsManipContext() {
    MString str("Plugin move Manipulator");
    setTitleString(str);
}

/**
 * Called when the tool is activated. Sets up the help string and selection changed callback.
 */
void PhysicsManipContext::toolOnSetup(MEvent&) {
    MString str("Move the object using the manipulator");
    setHelpString(str);
    MStatus status;
    this->id = MModelMessage::addCallback(MModelMessage::kActiveListModified, selectionChanged, this, &status);
    if (!status) {
        MGlobal::displayError("Model addCallback failed");
    }
}

/**
 * Called when the tool is deactivated. Cleans up by removing the callback.
 */
void PhysicsManipContext::toolOffCleanup() {
    MStatus status;
    status = MModelMessage::removeCallback(this->id);
    if (!status) {
        MGlobal::displayError("Model remove callback failed");
    }
    MPxContext::toolOffCleanup();
}

/**
 * Static callback function for selection changes. Updates manipulators based on the new selection.
 * Also updates all colliders based on all MFnMeshes inside scene, setups up activeRigidBodies based on selected MFnMesh
 * and initializes bullet3 dynamics world.
 * @param data Custom data for the callback.
 */
void PhysicsManipContext::selectionChanged(void* data) {
    PhysicsManipContext* ctxPtr = (PhysicsManipContext*)data;
    ctxPtr->deleteManipulators();

    PhysicsManipContext::setupPhysicsWorldSingletons();

    // Create a single manipulator at the average position
    MString manipName("physicsMoveManip");
    MObject manipObject;
    PhysicsMoveManip* manipulator = (PhysicsMoveManip*)PhysicsMoveManip::newManipulator(manipName, manipObject);

    if (manipulator != NULL) {
        MVector avgPosition;
        avgPosition = PhysicsManipContext::getAveragePositionFromSelection();

        ctxPtr->addManipulator(manipObject);
        manipulator->updateManipLocation(avgPosition);
    }
}

void PhysicsManipContext::setupPhysicsWorldSingletons() {
    CollisionCandidatesFinder& collisionCandidatesFinder = CollisionCandidatesFinder::getInstance();
    collisionCandidatesFinder.addActiveObjects();
    if (collisionCandidatesFinder.allSceneMFnMeshes.empty()) {
        collisionCandidatesFinder.getSceneMFnMeshes();
    }

    BulletCollisionHandler& bulletCollisionHandler = BulletCollisionHandler::getInstance();
    bulletCollisionHandler.createDynamicsWorld();
    bulletCollisionHandler.updateActiveObjects(collisionCandidatesFinder.activeMFnMeshes);
    bulletCollisionHandler.updateColliders(collisionCandidatesFinder.allSceneMFnMeshes, collisionCandidatesFinder.activeMFnMeshes);
}

MVector PhysicsManipContext::getAveragePositionFromSelection() {
    MVector avgPosition(0.0, 0.0, 0.0);

    MStatus stat = MStatus::kSuccess;
    MSelectionList list;
    stat = MGlobal::getActiveSelectionList(list);
    MItSelectionList iter(list, MFn::kInvalid, &stat);
    if (MS::kSuccess != stat) {
        return avgPosition;
    }

    unsigned int count = 0;

    // Iterate over all selected objects and accumulate positions
    for (; !iter.isDone(); iter.next()) {
        MObject dependNode;
        iter.getDependNode(dependNode);
        if (dependNode.isNull() || !dependNode.hasFn(MFn::kTransform)) {
            continue;
        }
        MFnTransform transFn(dependNode);
        MMatrix worldMatrix = transFn.transformationMatrix();
        MVector trans(worldMatrix[3][0], worldMatrix[3][1], worldMatrix[3][2]);
        avgPosition += trans;
        count++;
    }
    iter.reset();

    // Calculate the average position for manip placement
    if (count > 0) {
        avgPosition /= count;
    }

    return avgPosition;
}