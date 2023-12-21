#include <CustomMoveManip.h>
#include <MayaIncludes.h>
#include <LinearMath/btScalar.h>

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
    MGlobal::displayInfo("CustomMoveManip initialized");
    return stat;
}

MStatus CustomMoveManip::createChildren() {
    MStatus stat = MStatus::kSuccess;
    
    this->fFreePointManipDagPath = addFreePointTriadManip(
        "pointManip",
        "freePoint"
    );
    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);
    double transformManipHandleScale[3] = { 1.25, 1.25, 1.25 };
    manipFn.scaleBy(transformManipHandleScale);

    return stat;
}

MStatus CustomMoveManip::connectToDependNode(const MObject& node) {
    //
    // This routine connects the translate plug to the position plug on the freePoint
    // manipulator.
    //
    MStatus stat;
    this->finishAddingManips();
    MPxManipContainer::connectToDependNode(node);
    return stat;
}

void CustomMoveManip::updateManipLocation(const MVector vector) {
    //
    // Description
    //        setTranslation and setRotation to the parent's transformation.
    //
    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);
    manipFn.setTranslation(vector, MSpace::kWorld);
    this->currentManipPosition = vector;

    MStatus status;
}

MStatus CustomMoveManip::doPress() {
    MStatus status = MPxManipContainer::doPress();
    return status;
}

MStatus CustomMoveManip::doDrag() {
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
        btRigidBody* body = pair.second;
        body->setLinearVelocity(requiredVelocity);
        
        std::string name = pair.first;
        MMatrix transf = this->bulletCollisionHandler.getActiveObjectTransformMMatrix(name);

        MMatrix matrix;
        MTransformationMatrix mtm(matrix);
        MVector translation = mtm.getTranslation(MSpace::kWorld);
        MGlobal::displayInfo("bullet Translation: " + MString() + translation.x + ", " + translation.y + ", " + translation.z);


        // get body position directly and print it
        // convert maya position, convert it to bullet and print it
    }

    this->bulletCollisionHandler.updateWorld(10);
    this->bulletCollisionHandler.stopVelocitiesInWorld();
    this->bulletCollisionHandler.updateWorld(10);

    for (auto& pair : this->bulletCollisionHandler.activeRigidBodies) {
        std::string name = pair.first;
        btRigidBody* body = pair.second;
        body->setLinearVelocity(requiredVelocity);

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

    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);
    manipFn.setTranslation(avgPosition, MSpace::kWorld);

    return MS::kUnknownParameter;
}

MStatus CustomMoveManip::doRelease() {
    MStatus status = MPxManipContainer::doRelease();
}

void CustomMoveManip::applyTransformAndRotateToActiveObjectTransform(MMatrix matrix, std::string name) {
    if (this->collisionCandidatesFinder.activeTransformMFnDagNodes.find(name) == this->collisionCandidatesFinder.activeTransformMFnDagNodes.end()) {
        return;
    }

    MObject mobj = this->collisionCandidatesFinder.activeTransformMFnDagNodes[name];
    MFnDagNode activeDagNode;
    activeDagNode.setObject(mobj);

    MDagPath dagPath;
    activeDagNode.getPath(dagPath);
    MFnTransform activeTransform(dagPath);

    // Create an MTransformationMatrix from the input MMatrix
    MTransformationMatrix transMatrix(matrix);

    // Extract the translation from the MTransformationMatrix
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
