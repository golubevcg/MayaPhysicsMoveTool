#include "PhysicsMoveManip.h"
#include "../MayaIncludes.h"
#include <LinearMath/btScalar.h>

MTypeId PhysicsMoveManip::id(0x8001d);

PhysicsMoveManip::PhysicsMoveManip():
        bulletCollisionHandler(BulletCollisionHandler::getInstance()),
        collisionCandidatesFinder(CollisionCandidatesFinder::getInstance()) {
}

PhysicsMoveManip::~PhysicsMoveManip() {

}

void* PhysicsMoveManip::creator() {
    return new PhysicsMoveManip();
}

MStatus PhysicsMoveManip::initialize() {
    MStatus stat;
    stat = MPxManipContainer::initialize();
    MGlobal::displayInfo("PhysicsMoveManip initialized");
    return stat;
}

MStatus PhysicsMoveManip::createChildren() {
    MStatus stat = MStatus::kSuccess;
    this->fFreePointManipDagPath = addFreePointTriadManip("pointManip","freePoint");
    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);
    double transformManipHandleScale[3] = { 1.25, 1.25, 1.25 };
    manipFn.scaleBy(transformManipHandleScale);
    return stat;
}

MStatus PhysicsMoveManip::connectToDependNode(const MObject& node) {
    MStatus stat;
    this->finishAddingManips();
    MPxManipContainer::connectToDependNode(node);
    return stat;
}

void PhysicsMoveManip::updateManipLocation(const MVector vector) {
    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);
    manipFn.setTranslation(vector, MSpace::kWorld);
    this->currentManipPosition = vector;
    MStatus status;
}

MStatus PhysicsMoveManip::doPress() {
    MStatus status = MPxManipContainer::doPress();
    return status;
}

MStatus PhysicsMoveManip::doDrag() {
    // object pos of locator
    MPoint currentPosition;
    this->getConverterManipValue(0, currentPosition);

    btVector3 currentPos(
        this->currentManipPosition.x,
        this->currentManipPosition.z,
        -(this->currentManipPosition.y)
    );

    btVector3 targetPos(
        currentPosition.x,
        currentPosition.z,
        -currentPosition.y
    );

    // Here we calculate velocity vector by clamping in specified range
    // it is nessesary to do in order to avoid too huge or too small velocity values
    float timeStep = 1.0f / 60.0f;
    btVector3 requiredVelocity = (targetPos - currentPos) / timeStep;
    requiredVelocity *= 0.04*2;
    float threshold = 0.05f;

    if (std::abs(requiredVelocity.x()) < threshold) requiredVelocity.setX(0);
    if (std::abs(requiredVelocity.y()) < threshold) requiredVelocity.setY(0);
    if (std::abs(requiredVelocity.z()) < threshold) requiredVelocity.setZ(0);

    // Define range for clamping
    float minValue = -0.4f*4;
    float maxValue = 0.4f*4;
    // Clamp values within the range
    requiredVelocity.setX(std::min(std::max(requiredVelocity.x(), minValue), maxValue));
    requiredVelocity.setY(std::min(std::max(requiredVelocity.y(), minValue), maxValue));
    requiredVelocity.setZ(std::min(std::max(requiredVelocity.z(), minValue), maxValue));

    MVector avgPosition(0.000, 0.000, 0.000);
    unsigned int count = 0;

    for (auto& pair : this->bulletCollisionHandler.activeRigidBodies) {
        std::string name = pair.first;
        btRigidBody* body = pair.second;
        body->setLinearVelocity(requiredVelocity);
    }

    this->bulletCollisionHandler.updateWorld(10);
    // Remove all additional velocities in order to avoid unnesesary movement after world update
    this->bulletCollisionHandler.stopVelocitiesInWorld();

    for (auto& pair : this->bulletCollisionHandler.activeRigidBodies) {
        std::string name = pair.first;

        // Read transform from active object.
        MMatrix activeObjectUpdatedMatrix = this->bulletCollisionHandler.getActiveObjectTransformMMatrix(name);
        this->applyTransformAndRotateToActiveObjectTransform(activeObjectUpdatedMatrix, name);

        MTransformationMatrix transMatrix(activeObjectUpdatedMatrix);
        avgPosition += transMatrix.getTranslation(MSpace::kWorld);
        count++;
    }

    if (count > 0) {
        avgPosition /= count;
    }

    this->currentManipPosition = currentPosition;

    // because our manip is not directly connected to the node
    // we update manip position so it will snap back to the selected objects avgPosition
    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);
    manipFn.setTranslation(avgPosition, MSpace::kWorld);

    return MS::kUnknownParameter;
}

void PhysicsMoveManip::applyTransformAndRotateToActiveObjectTransform(MMatrix matrix, std::string name) {
    if (this->collisionCandidatesFinder.activeTransformMFnDagNodes.find(name) == this->collisionCandidatesFinder.activeTransformMFnDagNodes.end()) {
        return;
    }

    MObject mobj = this->collisionCandidatesFinder.activeTransformMFnDagNodes[name];
    MFnDagNode activeDagNode;
    activeDagNode.setObject(mobj);

    MDagPath dagPath;
    activeDagNode.getPath(dagPath);
    
    // Extract trasnform as vector
    MFnTransform activeTransform(dagPath);
    MTransformationMatrix transMatrix(matrix);
    MVector translation = transMatrix.getTranslation(MSpace::kWorld);

    // Extract the rotation as a quaternion
    MQuaternion rotation;
    transMatrix.getRotationQuaternion(rotation.x, rotation.y, rotation.z, rotation.w);

    // Apply only the translation and rotation to the active transform
    MStatus status = activeTransform.setTranslation(translation, MSpace::kWorld);
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
