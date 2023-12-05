#include <CustomMoveManip.h>
#include <MayaIncludes.h>

MTypeId CustomMoveManip::id(0x8001d);

CustomMoveManip::CustomMoveManip():
        bulletCollisionHandler(BulletCollisionHandler::getInstance()),
        collisionCandidatesFinder(CollisionCandidatesFinder::getInstance()) {
}

CustomMoveManip::~CustomMoveManip() {
    MGlobal::displayInfo("out of destructor!!!!");
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

    this->updateManipLocations(node);
    this->finishAddingManips();
    MPxManipContainer::connectToDependNode(node);
    return stat;
}

void CustomMoveManip::updateManipLocations(const MObject& node)
//
// Description
//        setTranslation and setRotation to the parent's transformation.
//
{
    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);

    MDagPath dagPath;
    MFnDagNode(node).getPath(dagPath);

    MObject transformNode = dagPath.transform();
    MFnTransform fnTransform(transformNode);
    MTransformationMatrix originalTM = fnTransform.transformation();

    double rot[3];
    MTransformationMatrix::RotationOrder rOrder;
    originalTM.getRotation(rot, rOrder);

    //manipFn.setRotation(rot, rOrder);
    manipFn.setTranslation(originalTM.getTranslation(MSpace::kWorld), MSpace::kWorld);

    MStatus status;
}

MStatus CustomMoveManip::doPress() {
    MStatus status = MPxManipContainer::doPress();
    return status;
}

MStatus CustomMoveManip::doDrag() {
    // Update the world.

    this->bulletCollisionHandler.updateWorld(5.0);

    // Read translation from manip.
    MFnManip3D manipFn(this->fFreePointManipDagPath);
    MPoint currentPosition;

    this->getConverterManipValue(0, currentPosition);
    MPoint currentTranslation = manipFn.translation(MSpace::kWorld);

    // Get the current position of the rigid body
    btVector3 currentPos = this->bulletCollisionHandler.activeRigidBody->getWorldTransform().getOrigin();
    // Calculate target position (convert to Bullet's coordinate system)
    btVector3 targetPos(
        currentPosition.x + currentTranslation.x,
        currentPosition.z + currentTranslation.z,
        -(currentPosition.y + currentTranslation.y)
    );
    float timeStep = 1.0f / 60.0f;

    // Calculate the required velocity to reach the target position in one time step
    btVector3 requiredVelocity = (targetPos - currentPos) / timeStep;

    // Apply this velocity to the rigid body
    this->bulletCollisionHandler.activeRigidBody->setLinearVelocity(requiredVelocity*0.01);
    this->bulletCollisionHandler.updateWorld(50);

    // Read transform from active object.
    MMatrix activeObjectUpdatedMatrix = this->bulletCollisionHandler.getActiveObjectTransformMMatrix();
    this->applyTransformAndRotateToActiveObjectTransform(activeObjectUpdatedMatrix);
    
    return MS::kUnknownParameter;
}


void CustomMoveManip::applyTransformAndRotateToActiveObjectTransform(MMatrix matrix) {
    MFnDagNode& activeDagNode = this->collisionCandidatesFinder.activeTransformMFnDagNode;

    MDagPath dagPath;
    activeDagNode.getPath(dagPath);

    MFnTransform activeTransform(dagPath);

    // Create an MTransformationMatrix from the input MMatrix
    MTransformationMatrix transMatrix(matrix);

    // Extract the translation from the MTransformationMatrix
    MVector translation = transMatrix.getTranslation(MSpace::kTransform);

    // Extract the rotation as a quaternion
    MQuaternion rotation;
    transMatrix.getRotationQuaternion(rotation.x, rotation.y, rotation.z, rotation.w);

    // Apply only the translation and rotation to the active transform
    MStatus status = activeTransform.setTranslation(translation, MSpace::kTransform);
    if (!status) {
        MGlobal::displayError("Error setting translation: " + status.errorString());
        return;
    }

    status = activeTransform.setRotation(rotation);
    if (!status) {
        MGlobal::displayError("Error setting rotation: " + status.errorString());
        return;
    }
}

