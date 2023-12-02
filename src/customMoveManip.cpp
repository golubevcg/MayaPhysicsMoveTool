#include <CustomMoveManip.h>

MTypeId CustomMoveManip::id(0x8001d);

CustomMoveManip::CustomMoveManip()       
    :bulletCollisionHandler(BulletCollisionHandler::getInstance()),
    collisionCandidatesFinder(CollisionCandidatesFinder::getInstance()) {
    // Constructor implementation
}

void CustomMoveManip::setupCollisions() {
    this->collisionCandidatesFinder.addActiveObject();
    this->collisionCandidatesFinder.getSceneMFnMeshes();

    this->bulletCollisionHandler.createDynamicsWorld();

    this->bulletCollisionHandler.updateActiveObject(this->collisionCandidatesFinder.activeMFnMesh);
    this->bulletCollisionHandler.updateColliders(this->collisionCandidatesFinder.allSceneMFnMeshes);
}

void* CustomMoveManip::creator() {
    return new CustomMoveManip();
}

MStatus CustomMoveManip::initialize() {
    MStatus stat;
    stat = MPxManipContainer::initialize();

    MString message_text = "CustomMoveManip initialized";
    MGlobal::displayInfo(message_text);
    return stat;
}

MStatus CustomMoveManip::createChildren() {
    MStatus stat = MStatus::kSuccess;
    MPoint startPoint(0.0, 0.0, 0.0);
    MVector direction(0.0, 1.0, 0.0);
    this->fFreePointManipDagPath = addFreePointTriadManip("pointManip",
        "freePoint");

    return stat;
}

MStatus CustomMoveManip::connectToDependNode(const MObject& node) {
    MStatus stat;
    //
    // This routine connects the translate plug to the position plug on the freePoint
    // manipulator.
    //
    MFnDependencyNode nodeFn(node);
    MPlug tPlug = nodeFn.findPlug("translate", true, &stat);
    MFnFreePointTriadManip freePointManipFn(this->fFreePointManipDagPath);
    //freePointManipFn.connectToPointPlug(tPlug);
    this->updateManipLocations(node);
    this->finishAddingManips();
    MPxManipContainer::connectToDependNode(node);
    return stat;
}
// Viewport 2.0 manipulator draw overrides

void CustomMoveManip::updateManipLocations(const MObject& node) {
//
// Description
//        setTranslation and setRotation to the parent's transformation.
//

    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);

    MDagPath dagPath;
    MFnDagNode(node).getPath(dagPath);

    MObject transformNode = dagPath.transform();
    MFnTransform fnTransform(transformNode);
    MTransformationMatrix originalTM = fnTransform.transformation();

    double rot[3];
    MTransformationMatrix::RotationOrder rOrder;
    originalTM.getRotation(rot, rOrder);

    manipFn.setRotation(rot, rOrder);
    manipFn.setTranslation(originalTM.getTranslation(MSpace::kTransform), MSpace::kTransform);

    MStatus status;
}

MStatus CustomMoveManip::doPress() {
    MStatus status = MPxManipContainer::doPress();

    return status;
}

MStatus CustomMoveManip::doDrag() {
    // Update the world.
    this->bulletCollisionHandler.updateWorld(5);

    // Read translation from manip.
    MFnManip3D manipFn(this->fFreePointManipDagPath);
    MPoint currentPosition;
    this->getConverterManipValue(0, currentPosition);
    MPoint currentTranslation = manipFn.translation(MSpace::kWorld);

    btVector3 currentPos = this->bulletCollisionHandler.activeRigidBody->getWorldTransform().getOrigin();
    btVector3 targetPos(
        currentPosition.x + currentTranslation.x,
        currentPosition.z + currentTranslation.z,
        -(currentPosition.y + currentTranslation.y)
    );

    float timeStep = 1.0f / 60.0f;
    btVector3 requiredVelocity = (targetPos - currentPos) / timeStep;
    this->bulletCollisionHandler.activeRigidBody->setLinearVelocity(requiredVelocity * 0.01 * 0.5);

    // Update world again for accuracy.
    this->bulletCollisionHandler.updateWorld(50);

    // Read transform from active object.
    MMatrix activeObjectUpdatedMatrix = this->bulletCollisionHandler.getActiveObjectTransformMMatrix();
    this->applyTransformToActiveObjectTransform(activeObjectUpdatedMatrix);

    btVector3 transformV = this->bulletCollisionHandler.activeRigidBody->getWorldTransform().getOrigin();
    return MS::kUnknownParameter;
}


void CustomMoveManip::applyTransformToActiveObjectTransform(MMatrix matrix) {
    // Get the MFnDagNode of the active object
    MFnDagNode& activeDagNode = this->collisionCandidatesFinder.activeTransformMFnDagNode;

    MDagPath dagPath;
    activeDagNode.getPath(dagPath);

    MFnTransform activeTransform(dagPath);
    MStatus status = activeTransform.set(MTransformationMatrix(matrix));
    if (!status) {
        // Handle the error if the transformation could not be set
        MGlobal::displayError("Error setting transformation: " + status.errorString());
    }
}