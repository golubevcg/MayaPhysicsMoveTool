#include <CustomMoveManip.h>
#include <MayaIncludes.h>


MTypeId CustomMoveManip::id(0x8001d);

CustomMoveManip::CustomMoveManip() {
    if (this->created != true) {
        // The constructor must not call createChildren for user-defined manipulators.
        this->collisionCandidatesFinder.addActiveObject();
        this->collisionCandidatesFinder.getSceneMFnMeshes();
        this->bulletCollisionHandler.createDynamicsWorld();

        MGlobal::displayInfo("Amount of scene MFnMeshes:" + MString() + this->collisionCandidatesFinder.allSceneMFnMeshes.size());
        this->bulletCollisionHandler.updateActiveObject(this->collisionCandidatesFinder.activeMFnMesh);
        this->bulletCollisionHandler.updateColliders(this->collisionCandidatesFinder.allSceneMFnMeshes);

        this->created = true;
    }

}

CustomMoveManip::~CustomMoveManip() {

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
    /*
    // Check for collision candidates and update colliders.
    std::vector<MFnMesh*> collisionCandidates = this->collisionCandidatesFinder.checkNearbyObjects();
    if (!collisionCandidates.empty()) {
        this->bulletCollisionHandler.updateColliders(collisionCandidates);
    }*/

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
    MGlobal::displayInfo("CALCULATED VELOCITY VALUE:" + MString() + requiredVelocity.getX() + requiredVelocity.getY() + requiredVelocity.getZ());
    this->bulletCollisionHandler.activeRigidBody->setLinearVelocity(requiredVelocity*0.01);

    // Update world again for accuracy.
    this->bulletCollisionHandler.updateWorld(50);

    // Read transform from active object.
    MMatrix activeObjectUpdatedMatrix = this->bulletCollisionHandler.getActiveObjectTransformMMatrix();
    this->applyTransformToActiveObjectTransform(activeObjectUpdatedMatrix);

    // Debugging information.
    //int numObjects = this->bulletCollisionHandler.dynamicsWorld->getNumCollisionObjects();
    //MGlobal::displayInfo(MString("---Number of Collision Objects: ") + numObjects);

    btVector3 transformV = this->bulletCollisionHandler.activeRigidBody->getWorldTransform().getOrigin();
    MGlobal::displayInfo("======ACTIVE BODY POS:" + MString() + transformV.getX() + transformV.getY() + transformV.getZ());
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

void CustomMoveManip::drawUI(MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext) const {
    drawManager.beginDrawable();
    drawManager.setColor(MColor(0.0f, 1.0f, 0.1f));
    drawManager.text(MPoint(0, 0), "Stretch Me!", MHWRender::MUIDrawManager::kLeft);
    drawManager.endDrawable();
}
