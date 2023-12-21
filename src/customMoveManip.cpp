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

    /*
    this->fRotateManipDagPath = addRotateManip(
        "rotateManip", 
        "rotation"
    );

    MFnRotateManip rotateManip(fRotateManipDagPath);
    double rotateManipHandleScale[3] = { 0.5, 0.5, 0.5 };
    rotateManip.scaleBy(rotateManipHandleScale);

    if (this->collisionCandidatesFinder.activeTransformMFnDagNodes.size() > 1) {
        double scale_manip_scale[3] = { 1.5, 1.5, 1.5 };
        this->xScaleManipDagPath = addDistanceManip(
            "scaleXManip",
            "scaleX"
        );
        MFnDistanceManip xDistanceFn(this->xScaleManipDagPath);
        MVector xDirection(-3.0, 0.0, 0.0);
        xDistanceFn.setDirection(xDirection);
        xDistanceFn.scaleBy(scale_manip_scale);

        this->yScaleManipDagPath = addDistanceManip(
            "scaleYManip",
            "scaleY"
        );
        MFnDistanceManip yDistanceFn(this->yScaleManipDagPath);
        MVector yDirection(0, -3.0, 0.0);
        yDistanceFn.setDirection(yDirection);
        yDistanceFn.scaleBy(scale_manip_scale);


        this->zScaleManipDagPath = addDistanceManip(
            "scaleZManip",
            "scaleZ"
        );
        MFnDistanceManip zDistanceFn(this->zScaleManipDagPath);
        MVector zDirection(0, 0.0, -3.0);
        zDistanceFn.setDirection(zDirection);
        zDistanceFn.scaleBy(scale_manip_scale);
    }
    */
    return stat;
}

MStatus CustomMoveManip::connectToDependNode(const MObject& node) {
    //
    // This routine connects the translate plug to the position plug on the freePoint
    // manipulator.
    //
    MStatus stat;
    /*
    MFnDependencyNode nodeFn(node);
    MPlug tPlug = nodeFn.findPlug("translate", true, &stat);
    MFnFreePointTriadManip freePointManipFn(this->fFreePointManipDagPath);
    freePointManipFn.connectToPointPlug(tPlug);
    */
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

    /*

    // Connect the Rotate Manipulator
    MFnRotateManip rotateManip(this->fRotateManipDagPath);
    rotateManip.setRotateMode(MFnRotateManip::kObjectSpace);
    rotateManip.setTranslation(vector, MSpace::kWorld);

    if (this->collisionCandidatesFinder.activeTransformMFnDagNodes.size() > 1) {
        MFnDistanceManip xScaleManip(this->xScaleManipDagPath);
        xScaleManip.setTranslation(vector, MSpace::kWorld);

        MFnDistanceManip yScaleManip(this->yScaleManipDagPath);
        yScaleManip.setTranslation(vector, MSpace::kWorld);

        MFnDistanceManip zScaleManip(this->zScaleManipDagPath);
        zScaleManip.setTranslation(vector, MSpace::kWorld);
    }*/


    MStatus status;
}

MStatus CustomMoveManip::doPress() {
    MStatus status = MPxManipContainer::doPress();
    return status;
}

MStatus CustomMoveManip::doDrag() {
    // Update the world.
    //this->bulletCollisionHandler.updateWorld(10);

    // object pos of locator
    MPoint currentPosition;
    this->getConverterManipValue(0, currentPosition);

    // object pos of locator
    MEulerRotation rotPos;
    this->getConverterManipValue(0, rotPos);

    // object pos of locator
    unsigned scXPos;
    this->getConverterManipValue(0, scXPos);

    // object pos of locator
    double scYPos;
    this->getConverterManipValue(0, scYPos);

    // object pos of locator
    MPoint scZPos;
    this->getConverterManipValue(0, scZPos);

    MFnFreePointTriadManip testFreePointManip(this->fFreePointManipDagPath);
    MPoint testManipPosition = testFreePointManip.getTranslation(MSpace::kWorld);

    /*
    MGlobal::displayInfo("******");
    MGlobal::displayInfo("testManipPosition:" + MString() + testManipPosition.x + MString(",") + testManipPosition.y + MString(",") + testManipPosition.z);
    MGlobal::displayInfo("rotPos euler:" + MString() + rotPos.x + MString(",") + rotPos.y + MString(",") + rotPos.z);
    MGlobal::displayInfo("scXPos unsigned int:" + MString() + scXPos);
    MGlobal::displayInfo("scYPos double:" + MString() + scYPos);
    MGlobal::displayInfo("scZPos:" + MString() + scZPos.x + MString(",") + scZPos.y + MString(",") + scZPos.z);
    */
    MGlobal::displayInfo("******");

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
    this->avgPosition = avgPosition;

    this->currentManipPosition = currentPosition;

    //MGlobal::displayInfo("avgPosition:" + MString() + avgPosition.x + MString(",") + avgPosition.y + MString(",") + avgPosition.z);
    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);
    manipFn.setTranslation(avgPosition, MSpace::kWorld);

    //MPoint manipPoint(avgPosition.x, avgPosition.y, avgPosition.z);
    //manipFn.setPoint(manipPoint);
    //manipFn.translateBy(avgPosition, MSpace::kWorld);
    /*
    MFnRotateManip rotateManip(this->fRotateManipDagPath);
    rotateManip.setTranslation(avgPosition, MSpace::kWorld);
    */

    //MVector test_pos = manipFn.getTranslation(MSpace::kWorld);
    //MGlobal::displayInfo("test_pos :" + MString() + test_pos.x + MString(",") + test_pos.y + MString(",") + test_pos.z);

    return MS::kUnknownParameter;
}

MStatus CustomMoveManip::doRelease() {
    MStatus status = MPxManipContainer::doRelease();
    MGlobal::displayInfo("doPress event!!!!");
    MFnFreePointTriadManip manipFn(this->fFreePointManipDagPath);
    manipFn.setTranslation(this->avgPosition, MSpace::kWorld);

    MPoint manipPoint(avgPosition.x, avgPosition.y, avgPosition.z);
    manipFn.setPoint(manipPoint);
    manipFn.translateBy(avgPosition, MSpace::kWorld);
    MGlobal::executeCommand("refresh");
    return status;
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
