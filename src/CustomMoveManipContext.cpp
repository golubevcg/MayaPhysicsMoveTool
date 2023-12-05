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
    collisionCandidatesFinder.addActiveObject();
    // update this
    if (collisionCandidatesFinder.allSceneMFnMeshes.empty()) {
        collisionCandidatesFinder.getSceneMFnMeshes();
    }

    BulletCollisionHandler& bulletCollisionHandler = BulletCollisionHandler::getInstance();
    bulletCollisionHandler.createDynamicsWorld();
    bulletCollisionHandler.updateActiveObject(collisionCandidatesFinder.activeMFnMesh);

    // update this
    bulletCollisionHandler.updateColliders(collisionCandidatesFinder.allSceneMFnMeshes, collisionCandidatesFinder.activeMFnMesh);

    for (; !iter.isDone(); iter.next()) {
        MObject dependNode;
        iter.getDependNode(dependNode);
        if (dependNode.isNull() || !dependNode.hasFn(MFn::kDependencyNode))
        {
            MGlobal::displayWarning("Object in selection list is not "
                "a depend node.");
            continue;
        }
        MFnDependencyNode dependNodeFn(dependNode);

        MPlug tPlug = dependNodeFn.findPlug("translate", true, &stat);
        if (tPlug.isNull()) {
            MGlobal::displayWarning("Object cannot be manipulated: " +
                dependNodeFn.name());
            continue;
        }

        MString manipName("customMoveManip");
        MObject manipObject;
        CustomMoveManip* manipulator = (CustomMoveManip*)CustomMoveManip::newManipulator(manipName, manipObject);
        if (NULL != manipulator) {
            ctxPtr->addManipulator(manipObject);
            // Connect the manipulator to the object in the selection list.
            if (!manipulator->connectToDependNode(dependNode))
            {
                MGlobal::displayWarning("Error connecting manipulator to"
                    " object: " + dependNodeFn.name());
            }
        }
    }
}
