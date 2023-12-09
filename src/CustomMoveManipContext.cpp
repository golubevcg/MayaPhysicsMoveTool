#include <CustomMoveManipContext.h>
#include <CustomMoveManip.h>
#include <MayaIncludes.h>

//
// MoveManipContext
// This class is a simple context to support our custom a move manipulator.
//
CustomMoveManipContext::CustomMoveManipContext() {
    MString str("Plugin move Manipulator");
    setTitleString(str);
}

void CustomMoveManipContext::toolOnSetup(MEvent&) {
    MString str("Move the object using the manipulator");
    setHelpString(str);
    //selectionChanged(this);
    MStatus status;
    this->id = MModelMessage::addCallback(MModelMessage::kActiveListModified, selectionChanged, this, &status);
    if (!status) {
        MGlobal::displayError("Model addCallback failed");
    }
}

void CustomMoveManipContext::toolOffCleanup() {
    MStatus status;
    status = MModelMessage::removeCallback(this->id);
    if (!status) {
        MGlobal::displayError("Model remove callback failed");
    }

    MPxContext::toolOffCleanup();
}

void CustomMoveManipContext::selectionChanged(void* data) {
    MStatus stat = MStatus::kSuccess;

    CustomMoveManipContext* ctxPtr = (CustomMoveManipContext*)data;
    ctxPtr->deleteManipulators();
    MSelectionList list;
    stat = MGlobal::getActiveSelectionList(list);
    MItSelectionList iter(list, MFn::kInvalid, &stat);
    if (MS::kSuccess != stat) {
        return;
    }
    MGlobal::displayWarning("CONTEXT::SELECTION CHANGED!!!!!");

    CollisionCandidatesFinder& collisionCandidatesFinder = CollisionCandidatesFinder::getInstance();
    collisionCandidatesFinder.addActiveObjects();
    // update this
    if (collisionCandidatesFinder.allSceneMFnMeshes.empty()) {
        collisionCandidatesFinder.getSceneMFnMeshes();
    }

    BulletCollisionHandler& bulletCollisionHandler = BulletCollisionHandler::getInstance();
    bulletCollisionHandler.createDynamicsWorld();
    bulletCollisionHandler.updateActiveObject(collisionCandidatesFinder.activeMFnMesh);

    // update this
    bulletCollisionHandler.updateColliders(collisionCandidatesFinder.allSceneMFnMeshes, collisionCandidatesFinder.activeMFnMesh);


    //-------------------------------
    MVector avgPosition(0.0, 0.0, 0.0);
    unsigned int count = 0;

    // Iterate over all selected objects and accumulate positions
    for (; !iter.isDone(); iter.next()) {
        MObject dependNode;
        iter.getDependNode(dependNode);
        if (dependNode.isNull() || !dependNode.hasFn(MFn::kTransform)) {
            continue;
        }



        MFnTransform transFn(dependNode);
        // Get world space transformation matrix
        MMatrix worldMatrix = transFn.transformationMatrix();
        MVector trans(worldMatrix[3][0], worldMatrix[3][1], worldMatrix[3][2]);

        MGlobal::displayInfo("transFn.translation: " + MString() + trans.x + " " + MString() + trans.y + " " + MString() + trans.z);

        avgPosition += trans;

        count++;
    }
    iter.reset();

    // Calculate the average position
    if (count > 0) {
        avgPosition /= count;
    }

    MGlobal::displayInfo("avgPosition: " + MString() + avgPosition.x + " " + MString() + avgPosition.y + " " + MString() + avgPosition.z);

    // Create a single manipulator at the average position
    MString manipName("customMoveManip");
    MObject manipObject;
    CustomMoveManip* manipulator = (CustomMoveManip*)CustomMoveManip::newManipulator(manipName, manipObject);

    if (manipulator != NULL) {
        ctxPtr->addManipulator(manipObject);
        manipulator->updateManipLocation(avgPosition);
    }
}
