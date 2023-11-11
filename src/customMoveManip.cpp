#include <CollisionCandidatesFinder.h>
#include <BulletCollisionHandler.h>
#include <BulletOpenGLWidget.h>

#include <maya/MFnPlugin.h>
#include <maya/MStreamUtils.h>

#include <QtWidgets/qtextedit.h>





class MyTickCallback {
public:
    static void myTickCallback(btDynamicsWorld* world, btScalar timeStep) {
        MString msg = "myTickCallback";
        int numManifolds = world->getDispatcher()->getNumManifolds();
        for (int i = 0; i < numManifolds; ++i) {
            btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
            const btCollisionObject* obA = contactManifold->getBody0();
            const btCollisionObject* obB = contactManifold->getBody1();

            int numContacts = contactManifold->getNumContacts();
            for (int j = 0; j < numContacts; ++j) {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                if (pt.getDistance() < 0.f) {
                    const btVector3& ptA = pt.getPositionWorldOnA();
                    const btVector3& ptB = pt.getPositionWorldOnB();
                    const btVector3& normalOnB = pt.m_normalWorldOnB;

                    // Handle the collision point, print it to Maya's console
                    unsigned long long userPointer0 = reinterpret_cast<unsigned long long>(obA->getUserPointer());
                    unsigned long long userPointer1 = reinterpret_cast<unsigned long long>(obB->getUserPointer());

                    // Construct the message
                    MString msg = "Collision detected between: ";
                    msg += MString() + userPointer0 + " and " + MString() + userPointer1;
                    MGlobal::displayInfo(msg);
                }
            }
        }
    }
};





class CustomMoveManip : public MPxManipContainer
{
public:
    CustomMoveManip();
    ~CustomMoveManip() override;


    static void* creator();
    static MStatus initialize();
    MStatus createChildren() override;
    MStatus connectToDependNode(const MObject&) override;
    // Viewport 2.0 rendering
    void drawUI(MHWRender::MUIDrawManager&, const MHWRender::MFrameContext&) const override;
    MStatus doDrag() override;
    MStatus doPress() override;

    void applyProxyTransformToActiveObject(MMatrix matrix);

private:
    void updateManipLocations(const MObject& node);
public:
    MDagPath fFreePointManipDagPath;
    static MTypeId id;

    CollisionCandidatesFinder collisionCandidatesFinder;
    BulletCollisionHandler bulletCollisionHandler;
};

MTypeId CustomMoveManip::id(0x8001d);

CustomMoveManip::CustomMoveManip()
{
    // The constructor must not call createChildren for user-defined manipulators.
    this->collisionCandidatesFinder.getSceneMFnMeshes();
    this->collisionCandidatesFinder.initializeRTree();
    this->bulletCollisionHandler.createDynamicsWorld();

    //TEMP
    this->bulletCollisionHandler.updateColliders(this->collisionCandidatesFinder.allSceneMFnMeshes);
    this->bulletCollisionHandler.dynamicsWorld->setInternalTickCallback(MyTickCallback::myTickCallback);
    showBulletDialog();
}

CustomMoveManip::~CustomMoveManip()
{

}

void* CustomMoveManip::creator()
{
    return new CustomMoveManip();
}

MStatus CustomMoveManip::initialize()
{
    MStatus stat;
    stat = MPxManipContainer::initialize();

    MString message_text = "CustomMoveManip initialized";
    MGlobal::displayInfo(message_text);
    return stat;
}

MStatus CustomMoveManip::createChildren()
{
    MStatus stat = MStatus::kSuccess;
    MPoint startPoint(0.0, 0.0, 0.0);
    MVector direction(0.0, 1.0, 0.0);
    this->fFreePointManipDagPath = addFreePointTriadManip("pointManip",
        "freePoint");

    return stat;
}

MStatus CustomMoveManip::connectToDependNode(const MObject& node)
{
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

MStatus CustomMoveManip::doPress()
{
    MStatus status = MPxManipContainer::doPress();

    return status;
}

MStatus CustomMoveManip::doDrag() {
    // Update the world.
    this->bulletCollisionHandler.updateWorld(5);

    // Check for collision candidates and update colliders.
    /*
    std::vector<MFnMesh*> collisionCandidates = this->collisionCandidatesFinder.checkNearbyObjects();
    if (!collisionCandidates.empty()) {
        this->bulletCollisionHandler.updateColliders(collisionCandidates);
    }*/

    // Read translation from manip.
    MFnManip3D manipFn(this->fFreePointManipDagPath);
    MPoint currentPosition;
    this->getConverterManipValue(0, currentPosition);
    MPoint currentTranslation = manipFn.translation(MSpace::kWorld);

    // Set transform to proxy object in Bullet's coordinate system.
    this->bulletCollisionHandler.setProxyObjectPosition(
        currentPosition.x + currentTranslation.x, 
        currentPosition.y + currentTranslation.y,  // Don't invert these here
        currentPosition.z + currentTranslation.z   // Use Y and Z directly
    );

    // Update world again for accuracy.
    this->bulletCollisionHandler.updateWorld(10);

    // Read transform from active object.
    MMatrix proxyObjectUpdatedMatrix = this->bulletCollisionHandler.getProxyObjectTransformMMatrix();

    // Apply the transform to Maya's transform, converting it inside the function.
    this->applyProxyTransformToActiveObject(proxyObjectUpdatedMatrix);

    // Debugging information.
    int numObjects = this->bulletCollisionHandler.dynamicsWorld->getNumCollisionObjects();
    //MGlobal::displayInfo(MString("---Number of Collision Objects: ") + numObjects);

    return MS::kUnknownParameter;
}


void CustomMoveManip::applyProxyTransformToActiveObject(MMatrix matrix) {
    //MGlobal::displayInfo("applyProxyTransformToActiveObject");
    if (this->bulletCollisionHandler.proxyRigidBody) {

        // Get the MFnDagNode of the active object
        MFnDagNode& activeDagNode = this->collisionCandidatesFinder.activeTransformMFnDagNode;

        // Get the MDagPath of the active object to ensure we're modifying the correct instance
        MDagPath dagPath;
        activeDagNode.getPath(dagPath);

        // Get the MFnTransform of the active object using the dagPath
        MFnTransform activeTransform(dagPath);

        // Before setting the transformation, convert Bullet's matrix back to Maya's coordinate system
        // Swap Y (second column) and Z (third column)
        double temp;
        for (int i = 0; i < 4; i++) {
            temp = matrix(i, 1);
            matrix(i, 1) = matrix(i, 2);
            matrix(i, 2) = temp;
        }

        // Invert the Z values (third column) to switch from left-handed to right-handed
        for (int i = 0; i < 4; i++) {
            matrix(i, 2) = -matrix(i, 2);
        }

        // Now, matrix is in Maya's coordinate system

        // Set the transformation matrix of the active object to match the proxy object
        MStatus status = activeTransform.set(MTransformationMatrix(matrix));
        if (!status) {
            // Handle the error if the transformation could not be set
            MGlobal::displayError("Error setting transformation: " + status.errorString());
        }
    }
}

void CustomMoveManip::drawUI(MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext) const
{
    drawManager.beginDrawable();
    drawManager.setColor(MColor(0.0f, 1.0f, 0.1f));
    drawManager.text(MPoint(0, 0), "Stretch Me!", MHWRender::MUIDrawManager::kLeft);
    drawManager.endDrawable();
}

//
// MoveManipContext
//
// This class is a simple context for supporting a move manipulator.
//
class CustomMoveManipContext : public MPxSelectionContext
{
public:
    CustomMoveManipContext();
    void    toolOnSetup(MEvent& event) override;
    void    toolOffCleanup() override;
    // Callback issued when selection list changes
    static void selectionChanged(void* data);
private:
    MCallbackId id1;
};

CustomMoveManipContext::CustomMoveManipContext()
{
    MString str("Plugin move Manipulator");
    setTitleString(str);
}

void CustomMoveManipContext::toolOnSetup(MEvent&)
{
    MString str("Move the object using the manipulator");
    setHelpString(str);
    selectionChanged(this);
    MStatus status;
    id1 = MModelMessage::addCallback(MModelMessage::kActiveListModified,
        selectionChanged,
        this, &status);
    if (!status) {
        MGlobal::displayError("Model addCallback failed");
    }
}

void CustomMoveManipContext::toolOffCleanup()
{
    MStatus status;
    status = MModelMessage::removeCallback(id1);
    if (!status) {
        MGlobal::displayError("Model remove callback failed");
    }

    MPxContext::toolOffCleanup();
}

void CustomMoveManipContext::selectionChanged(void* data)
{
    MStatus stat = MStatus::kSuccess;

    CustomMoveManipContext* ctxPtr = (CustomMoveManipContext*)data;
    ctxPtr->deleteManipulators();
    MSelectionList list;
    stat = MGlobal::getActiveSelectionList(list);
    MItSelectionList iter(list, MFn::kInvalid, &stat);
    if (MS::kSuccess != stat) {
        return;
    }

    MString testText = "Selection changed";
    MGlobal::displayInfo(testText);

    for (; !iter.isDone(); iter.next()) {
        // Make sure the selection list item is a depend node and has the
        // required plugs before manipulating it.
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

            manipulator->collisionCandidatesFinder.addActiveObject();
            manipulator->bulletCollisionHandler.updateActiveObject(manipulator->collisionCandidatesFinder.activeMFnMesh);
         }
    }
}

//
// moveManipContext
//
// This is the command that will be used to create instances
// of our context.
//
class CustoMoveManipContext : public MPxContextCommand
{
public:
    CustoMoveManipContext() {};
    MPoint getCurrentPosition();
    MPxContext* makeObj() override;

public:
    static void* creator();
};

MPxContext* CustoMoveManipContext::makeObj()
{
    return new CustomMoveManipContext();
}

void* CustoMoveManipContext::creator()
{
    return new CustoMoveManipContext;
}

//
// The following routines are used to register/unregister
// the context and manipulator
//
MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, PLUGIN_COMPANY, "1.0", "Andrew Golubev");
    status = plugin.registerContextCommand("customMoveManipContext",
        &CustoMoveManipContext::creator);
    if (!status) {
        MGlobal::displayError("Error registering customMoveManipContext command");
        return status;
    }
    status = plugin.registerNode("customMoveManip", CustomMoveManip::id,
        &CustomMoveManip::creator, &CustomMoveManip::initialize,
        MPxNode::kManipContainer);
    if (!status) {
        MGlobal::displayError("Error registering customMoveManip node");
        return status;
    }
    return status;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj);
    status = plugin.deregisterContextCommand("customMoveManipContext");
    if (!status) {
        MGlobal::displayError("Error deregistering customMoveManipContext command");
        return status;
    }
    status = plugin.deregisterNode(CustomMoveManip::id);
    if (!status) {
        MGlobal::displayError("Error deregistering customMoveManip node");
        return status;
    }
    return status;
}
