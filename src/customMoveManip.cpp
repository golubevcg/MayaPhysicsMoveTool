#include <maya/MIOStream.h>
#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <maya/MFn.h>
#include <maya/MPxNode.h>
#include <maya/MPxManipContainer.h>
#include <maya/MPxSelectionContext.h>
#include <maya/MPxContextCommand.h>
#include <maya/MModelMessage.h>
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MItSelectionList.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MDagPath.h>
#include <maya/MManipData.h>
#include <maya/MMatrix.h>
#include <maya/MItDag.h>
#include <maya/MFnMesh.h>
#include <maya/MBoundingBox.h>
#include <maya/MPointArray.h>


// Manipulators
#include <maya/MFnFreePointTriadManip.h>
#include <maya/MFnDistanceManip.h>
// Boost geometry
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>
// Reactphysics3d
#include <reactphysics3d/reactphysics3d.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value;


class CustomMoveManip : public MPxManipContainer
{

public:
    CustomMoveManip();
    ~CustomMoveManip() override;

    static void* creator();
    static MStatus initialize();
    MStatus createChildren() override;
    MStatus connectToDependNode(const MObject& node) override;
    // Viewport 2.0 rendering
    void drawUI(MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext) const override;
    MStatus doDrag() override;
    MStatus doPress() override;
    MStatus addSelectedMObjects();
    MStatus getSceneMFnMeshes();
    MStatus initializeRTree();
    void checkNearbyObjects();
    void handleCollisions(const std::vector<MObject>& collisionCandidates);
    reactphysics3d::CollisionShape* createCollisionShapeFromMFnMesh(MFnMesh& fnMesh, const MDagPath& dagPath);
private:
    void updateManipLocations(const MObject& node);
public:
    MDagPath fFreePointManip;
    MSelectionList selList;
    std::vector<MObject> selectedObjects;
    std::vector<MFnMesh*> mFnMeshes;
    bgi::rtree<value, bgi::quadratic<16>> rtree;

    reactphysics3d::PhysicsCommon physicsCommon;
    reactphysics3d::PhysicsWorld* physicsWorld;

    static MTypeId id;
};
MTypeId CustomMoveManip::id(0x8001d);

CustomMoveManip::CustomMoveManip()
{
    // The constructor must not call createChildren for user-defined
    // manipulators.

    MString message_text = "getSceneMFnMeshes";
    MGlobal::displayWarning(message_text);
    getSceneMFnMeshes();

    message_text = "MFnMeshes:";
    MGlobal::displayInfo(message_text);
    for (const auto& mesh : mFnMeshes) {
        MString meshInfo = "Mesh: ";
        meshInfo += mesh->name();  // Assuming MFnMesh has a name() method to get the mesh name
        MGlobal::displayInfo(meshInfo);
    }

    message_text = "initializeRTree";
    MGlobal::displayWarning(message_text);
    initializeRTree();

    // Create a PhysicsWorld
    reactphysics3d::PhysicsWorld::WorldSettings settings;
    settings.isSleepingEnabled = True;
    settings.gravity = reactphysics3d::Vector3(0, 0, 0);
    // Create the physics world with your settings
    reactphysics3d::PhysicsWorld* world = physicsCommon.createPhysicsWorld(
        settings);



    reactphysics3d::PhysicsWorld* physicsWorld = physicsCommon.createPhysicsWorld();
}

CustomMoveManip::~CustomMoveManip()
{
    for (MFnMesh* mesh : mFnMeshes) {
        delete mesh;
    }
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
    fFreePointManip = addFreePointTriadManip("pointManip",
        "freePoint");

    return stat;
}

void CustomMoveManip::updateManipLocations(const MObject& node)
//
// Description
//        setTranslation and setRotation to the parent's transformation.
//
{
    MFnDagNode dagNodeFn(node);
    MDagPath nodePath;
    dagNodeFn.getPath(nodePath);
    MFnFreePointTriadManip manipFn(fFreePointManip);
    MTransformationMatrix m(nodePath.exclusiveMatrix());
    double rot[3];
    MTransformationMatrix::RotationOrder rOrder;
    m.getRotation(rot, rOrder);
    manipFn.setRotation(rot, rOrder);
    MVector trans = m.getTranslation(MSpace::kWorld);
    manipFn.setTranslation(trans, MSpace::kWorld);
    MStatus status;
} 

MStatus CustomMoveManip::addSelectedMObjects() {

    MGlobal::getActiveSelectionList(selList);
    MItSelectionList iter(selList);

    selectedObjects.clear();

    if (iter.isDone()) {
        MString warningMessage = "No objects selected";
        MGlobal::displayWarning(warningMessage);
        return MS::kFailure;
    }
    else {
        for (; !iter.isDone(); iter.next()) {
            MObject node;
            iter.getDependNode(node);
            selectedObjects.push_back(node);
        }
    }
    return MS::kSuccess;
}

MStatus CustomMoveManip::getSceneMFnMeshes() {
    
    MStatus status;
    MItDag dagIterator(MItDag::kDepthFirst, MFn::kMesh, &status);

    if (status != MS::kSuccess) {
        MGlobal::displayError("MItDag initialization failed");
        return status;
    }

    for (; !dagIterator.isDone(); dagIterator.next()) {
        MDagPath dagPath;
        status = dagIterator.getPath(dagPath);

        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get MDagPath");
            continue;
        }

        MFnMesh* fnMesh = new MFnMesh(dagPath, &status);
        if (status != MS::kSuccess) {
            MGlobal::displayError("MFnMesh initialization failed");
            continue;
        }

        MBoundingBox boundingBox = fnMesh->boundingBox(&status);
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get bounding box");
            continue;
        }

        mFnMeshes.push_back(fnMesh);
    }

    return MS::kSuccess;
}

MStatus CustomMoveManip::initializeRTree() {

    if (mFnMeshes.empty()) {
        MGlobal::displayError("MFnMeshes vector is empty");
        return MS::kFailure;
    }

    MStatus status;

    for (size_t i = 0; i < mFnMeshes.size(); ++i) {
        MDagPath dagPath;
        status = mFnMeshes[i]->getPath(dagPath);
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get MDagPath from MFnMesh");
            return status;
        }

        MBoundingBox mbbox = mFnMeshes[i]->boundingBox();
        MMatrix worldMatrix = dagPath.inclusiveMatrix();

        MPoint minPoint = mbbox.min() * worldMatrix;
        MPoint maxPoint = mbbox.max() * worldMatrix;

        box bbox(point(minPoint.x, minPoint.y, minPoint.z), point(maxPoint.x, maxPoint.y, maxPoint.z));
        rtree.insert(std::make_pair(bbox, i));  // i is the identifier
    }

    return MS::kSuccess;
}


MStatus CustomMoveManip::connectToDependNode(const MObject& node)
{
    MStatus stat;
    //
    // This routine connects the distance manip to the scaleY plug on the node
    // and connects the translate plug to the position plug on the freePoint
    // manipulator.
    //
    MFnDependencyNode nodeFn(node);
    MPlug tPlug = nodeFn.findPlug("translate", true, &stat);
    MFnFreePointTriadManip freePointManipFn(fFreePointManip);
    freePointManipFn.connectToPointPlug(tPlug);
    updateManipLocations(node);
    finishAddingManips();
    MPxManipContainer::connectToDependNode(node);
    return stat;
}
// Viewport 2.0 manipulator draw overrides

MStatus CustomMoveManip::doPress()
{
    MStatus status = MPxManipContainer::doPress();

    return status;
}

MStatus CustomMoveManip::doDrag()
{
    // Call the parent class's doDrag function
    // Add any additional functionality here if needed

    MStatus status;

    MString txt = "Moving Objects:";
    MGlobal::displayInfo(txt);

    checkNearbyObjects();

    return MS::kUnknownParameter;
}

void CustomMoveManip::checkNearbyObjects() {

    if (selectedObjects.size() == 0) {
        return;
    }

    std::vector<MObject> collisionCandidates;

    for (size_t i = 0; i < selectedObjects.size(); ++i) {
        MStatus status;

        MFnDagNode dagNode(selectedObjects[i]);
        MBoundingBox boundingBox = dagNode.boundingBox();

        // Expand the bounding box by a certain distance to find nearby objects
        double distance = 0.1;
        MPoint minPoint = boundingBox.min() - MVector(distance, distance, distance);
        MPoint maxPoint = boundingBox.max() + MVector(distance, distance, distance);
        
        box queryBox(point(minPoint.x, minPoint.y, minPoint.z), point(maxPoint.x, maxPoint.y, maxPoint.z));

        std::vector<value> result;
        rtree.query(bgi::intersects(queryBox), std::back_inserter(result));

        // collect all collided objects
        for (const auto& item : result) {
            MString txt = "iter";
            MGlobal::displayInfo(txt);
            if (mFnMeshes[item.second]->object() != dagNode.object()) {  // Exclude the selected mesh itself
                MString txt = "The selected object is near another object.";
                MGlobal::displayInfo(txt);
                collisionCandidates.push_back(mFnMeshes[item.second]->object());
            }
        }
    }

    if (collisionCandidates.size() != 0) {
        handleCollisions(collisionCandidates);
    }

}

void CustomMoveManip::handleCollisions(const std::vector<MObject>& collisionCandidates) {
    // calculate collision response for each selected object with each collided one
    MStatus status;
}

reactphysics3d::CollisionShape* CustomMoveManip::createCollisionShapeFromMFnMesh(MFnMesh& fnMesh, const MDagPath& dagPath) {
    MStatus status;

    // Get vertices
    MPointArray vertices;
    status = fnMesh.getPoints(vertices, MSpace::kObject);
    if (status != MStatus::kSuccess) {
        MGlobal::displayError("Failed to get vertices");
        return nullptr;
    }

    // Get face counts and face vertices
    MIntArray faceCounts, faceVertices;
    status = fnMesh.getVertices(faceCounts, faceVertices);
    if (status != MStatus::kSuccess) {
        MGlobal::displayError("Failed to get face vertices");
        return nullptr;
    }

    // Apply transformation to vertices
    MMatrix transformMatrix = dagPath.inclusiveMatrix();
    for (unsigned int i = 0; i < vertices.length(); ++i) {
        vertices[i] *= transformMatrix;
    }

    // TODO: Create a reactphysics3d collision shape using the transformed vertices and faces
    // This might involve creating a ConvexMeshShape or ConcaveMeshShape depending on your needs
    // For simplicity, let’s assume you are creating a ConvexMeshShape

    // Convert MPointArray to std::vector<reactphysics3d::Vector3>
    std::vector<reactphysics3d::Vector3> rp3dVertices;
    for (unsigned int i = 0; i < vertices.length(); ++i) {
        rp3dVertices.push_back(reactphysics3d::Vector3(vertices[i].x, vertices[i].y, vertices[i].z));
    }

    // Create ConvexMeshShape
    // Note: This is a simplified example. You might need to consider the faces and indices for more complex shapes
    reactphysics3d::ConvexMeshShape* convexMeshShape = new reactphysics3d::ConvexMeshShape(&rp3dVertices[0], vertices.length(), sizeof(reactphysics3d::Vector3));

    return convexMeshShape;
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
    static void updateManipulators(void* data);
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
    updateManipulators(this);
    MStatus status;
    id1 = MModelMessage::addCallback(MModelMessage::kActiveListModified,
        updateManipulators,
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

void CustomMoveManipContext::updateManipulators(void* data)
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

    for (; !iter.isDone(); iter.next()) {
        // Make sure the selection list item is a depend node and has the
        // required plugs before manipulating it.
        //
        MObject dependNode;
        iter.getDependNode(dependNode);
        if (dependNode.isNull() || !dependNode.hasFn(MFn::kDependencyNode))
        {
            MGlobal::displayWarning("Object in selection list is not "
                "a depend node.");
            continue;
        }
        MFnDependencyNode dependNodeFn(dependNode);
        MPlug rPlug = dependNodeFn.findPlug("translate", true, &stat);
        MPlug sPlug = dependNodeFn.findPlug("scaleY", true, &stat);
        if (rPlug.isNull() || sPlug.isNull()) {
            MGlobal::displayWarning("Object cannot be manipulated: " +
                dependNodeFn.name());
            continue;
        }
        // Add manipulator to the selected object
        //
        MString manipName("customMoveManip");
        MObject manipObject;
        CustomMoveManip* manipulator =
            (CustomMoveManip*)CustomMoveManip::newManipulator(manipName, manipObject);
        if (NULL != manipulator) {
            // Add the manipulator
            //
            ctxPtr->addManipulator(manipObject);
            // Connect the manipulator to the object in the selection list.
            //
            if (!manipulator->connectToDependNode(dependNode))
            {
                MGlobal::displayWarning("Error connecting manipulator to"
                    " object: " + dependNodeFn.name());
            }
            manipulator->addSelectedMObjects();
            //update active objects in physics world
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