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
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>


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
    MStatus connectToDependNode(const MObject&) override;
    // Viewport 2.0 rendering
    void drawUI(MHWRender::MUIDrawManager&, const MHWRender::MFrameContext&) const override;
    MStatus doDrag() override;
    MStatus doPress() override;
    MStatus addSelectedMObjects();
    MStatus getSceneMFnMeshes();
    MStatus initializeRTree();
    std::vector<MObject> checkNearbyObjects();
    void handleCollisions(std::vector<reactphysics3d::ConcaveMeshShape*>);
    // for rigid bodies
    reactphysics3d::ConvexMeshShape* CustomMoveManip::createConvexCollisionShapeFromMObject(const MObject& object);
    reactphysics3d::RigidBody* CustomMoveManip::createRigidBody(const reactphysics3d::Transform& transform, reactphysics3d::CollisionShape* collisionShape);
    void clearPhysicsWorld();
    void addRigidBodyFromSelectedObject();
    // for static colliders
    reactphysics3d::ConcaveMeshShape* createConcaveCollisionFromMObject(const MObject&);
    std::vector<reactphysics3d::ConcaveMeshShape*> convertMFnMeshesToConcaveCollisionShapes(std::vector<MObject>);
private:
    void updateManipLocations(const MObject& node);
public:
    MDagPath fFreePointManip;
    MSelectionList selList;
    std::set<MObject> selectedObjects;
    std::vector<MFnMesh*> mFnMeshes;
    bgi::rtree<value, bgi::quadratic<16>> rtree;

    reactphysics3d::PhysicsCommon physicsCommon;
    reactphysics3d::PhysicsWorld::WorldSettings physicsWorldSettings;
    reactphysics3d::PhysicsWorld* physicsWorld;
    std::set<reactphysics3d::RigidBody*> activeRigidBodies;

    static MTypeId id;
};
MTypeId CustomMoveManip::id(0x8001d);

CustomMoveManip::CustomMoveManip()
{
    // The constructor must not call createChildren for user-defined
    // manipulators.

    getSceneMFnMeshes();

    initializeRTree();

    // Create a PhysicsWorld
    //settings.isSleepingEnabled = True;
    physicsWorldSettings.gravity = reactphysics3d::Vector3(0, 0, 0);

    // Create the physics world with your settings
    reactphysics3d::PhysicsWorld* world = physicsCommon.createPhysicsWorld(
        physicsWorldSettings);

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

void CustomMoveManip::clearPhysicsWorld() {
    if (physicsWorld != nullptr) {
        // Delete the current physics world
        physicsCommon.destroyPhysicsWorld(physicsWorld);

        // Create a new physics world
        physicsWorld = physicsCommon.createPhysicsWorld(physicsWorldSettings);
    }
    else {
        MGlobal::displayError("Physics world is null");
    }
}

void CustomMoveManip::addRigidBodyFromSelectedObject() {

    if (selectedObjects.empty()) {
        return;
    }

    if (physicsWorld == nullptr) {
        MGlobal::displayError("Physics world is null");
        return;
    }

    for (const auto& object : selectedObjects) {
        // Create a collision shape from the MObject
        reactphysics3d::ConvexMeshShape* collisionShape = createConvexCollisionShapeFromMObject(object);
        if (collisionShape == nullptr) {
            MGlobal::displayError("Failed to create collision shape");
            continue;
        }

        // Get the initial transform of the object
        MFnDagNode dagNode(object);
        MDagPath nodePath;
        dagNode.getPath(nodePath);
        MMatrix m = nodePath.inclusiveMatrix();

        // Extract translation from the matrix
        MVector translation(m[3][0], m[3][1], m[3][2]);

        // Extract rotation matrix and convert it to quaternion
        MMatrix rotationMatrix;
        rotationMatrix[0][0] = m[0][0]; rotationMatrix[0][1] = m[0][1]; rotationMatrix[0][2] = m[0][2];
        rotationMatrix[1][0] = m[1][0]; rotationMatrix[1][1] = m[1][1]; rotationMatrix[1][2] = m[1][2];
        rotationMatrix[2][0] = m[2][0]; rotationMatrix[2][1] = m[2][1]; rotationMatrix[2][2] = m[2][2];
        MTransformationMatrix tm(rotationMatrix);
        MEulerRotation eulerRotation = tm.eulerRotation();  // Corrected this line
        MQuaternion rotation = eulerRotation.asQuaternion();

        // Convert Maya transform to ReactPhysics3D transform
        reactphysics3d::Vector3 position(translation.x, translation.y, translation.z);
        reactphysics3d::Quaternion orientation(rotation.x, rotation.y, rotation.z, rotation.w);
        reactphysics3d::Transform transform(position, orientation);

        // Create a rigid body with the transform and collision shape
        reactphysics3d::RigidBody* rigidBody = createRigidBody(transform, collisionShape);
        if (rigidBody == nullptr) {
            MGlobal::displayError("Failed to create rigid body");
            continue;
        }

        activeRigidBodies.insert(rigidBody);
    }
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
            if (selectedObjects.find(node) != selectedObjects.end()) {
                selectedObjects.insert(node);
            }
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
    // This routine connects the translate plug to the position plug on the freePoint
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

    std::vector<MObject> collisionCandidates = checkNearbyObjects();
    if (collisionCandidates.size() != 0) {
        std::vector<reactphysics3d::ConcaveMeshShape*> collisionsShapes = convertMFnMeshesToConcaveCollisionShapes(collisionCandidates);
        handleCollisions(collisionsShapes);
    }

    return MS::kUnknownParameter;
}

std::vector<MObject> CustomMoveManip::checkNearbyObjects() {

    std::vector<MObject> collisionCandidates;
    if (selectedObjects.size() == 0) {
        return collisionCandidates;
    }


    for (const auto& currentMObject : selectedObjects) {
        MStatus status;

        MFnDagNode dagNode(currentMObject);
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
            // TODO:CHANGE THIS TO CONDITION CHECK THAT DAG MOBJECT ALREADY IN THE LIST
            if (mFnMeshes[item.second]->object() != dagNode.object()) {  // Exclude the selected mesh itself
                MString txt = "The selected object is near another object.";
                MGlobal::displayInfo(txt);
                collisionCandidates.push_back(mFnMeshes[item.second]->object());
            }
        }
    }

    return collisionCandidates;
}

void CustomMoveManip::handleCollisions(std::vector<reactphysics3d::ConcaveMeshShape*> collisionCandidates) {
    // Example implementation
    for (auto* shape : collisionCandidates) {
        // Handle collision with shape
        // ...
    }
}

std::vector<reactphysics3d::ConcaveMeshShape*> CustomMoveManip::convertMFnMeshesToConcaveCollisionShapes(std::vector<MObject> meshes) {
    std::vector<reactphysics3d::ConcaveMeshShape*> collisionShapes;

    return collisionShapes;
}

reactphysics3d::ConcaveMeshShape* CustomMoveManip::createConcaveCollisionFromMObject(const MObject& object) {
    MStatus status;
    MDagPath dagPath;
    MFnDagNode(object).getPath(dagPath);
    MFnMesh fnMesh(object, &status);

    // Get vertices
    MPointArray vertices;
    status = fnMesh.getPoints(vertices, MSpace::kObject);
    if (status != MStatus::kSuccess) {
        MGlobal::displayError("Failed to get vertices");
        throw std::invalid_argument("Failed to get vertices from the object");
    }

    // Apply transformation to vertices
    MMatrix transformMatrix = dagPath.inclusiveMatrix();
    std::vector<float> transformedVertices;
    for (unsigned int i = 0; i < vertices.length(); ++i) {
        MPoint transformedVertex = vertices[i] * transformMatrix;
        transformedVertices.push_back(static_cast<float>(transformedVertex.x));
        transformedVertices.push_back(static_cast<float>(transformedVertex.y));
        transformedVertices.push_back(static_cast<float>(transformedVertex.z));
    }

    // Get polygons
    MIntArray triangleCounts, triangleVertices;
    status = fnMesh.getTriangles(triangleCounts, triangleVertices);
    if (status != MStatus::kSuccess) {
        MGlobal::displayError("Failed to get triangles");
        throw std::invalid_argument("Failed to get triangles from the object");
    }

    std::vector<int> indices;
    for (unsigned int i = 0; i < triangleVertices.length(); ++i) {
        indices.push_back(triangleVertices[i]);
    }

    // Create TriangleVertexArray
    auto triangleArray = new reactphysics3d::TriangleVertexArray(
        vertices.length(),
        transformedVertices.data(),
        3 * sizeof(float),
        triangleVertices.length() / 3,  // Corrected this line to get the number of triangles
        indices.data(),
        3 * sizeof(int),
        reactphysics3d::TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
        reactphysics3d::TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE);


    // Create TriangleMesh and add the TriangleVertexArray as a subpart
    auto triangleMesh = physicsCommon.createTriangleMesh();
    triangleMesh->addSubpart(triangleArray);

    // Create the ConcaveMeshShape
    auto concaveMeshShape = physicsCommon.createConcaveMeshShape(triangleMesh);

    return concaveMeshShape;
}

reactphysics3d::ConvexMeshShape* CustomMoveManip::createConvexCollisionShapeFromMObject(const MObject& object) {
    MStatus status;
    MDagPath dagPath;
    MFnDagNode(object).getPath(dagPath);
    MFnMesh fnMesh(object, &status);

    // Get vertices
    MPointArray vertices;
    status = fnMesh.getPoints(vertices, MSpace::kObject);
    if (status != MStatus::kSuccess) {
        MGlobal::displayError("Failed to get vertices");
        return nullptr;
    }

    // Apply transformation to vertices
    MMatrix transformMatrix = dagPath.inclusiveMatrix();
    std::vector<float> transformedVertices;
    for (unsigned int i = 0; i < vertices.length(); ++i) {
        MPoint transformedVertex = vertices[i] * transformMatrix;
        transformedVertices.push_back(static_cast<float>(transformedVertex.x));
        transformedVertices.push_back(static_cast<float>(transformedVertex.y));
        transformedVertices.push_back(static_cast<float>(transformedVertex.z));
    }

    // Get polygons
    MIntArray triangleCounts, triangleVertices;
    status = fnMesh.getTriangles(triangleCounts, triangleVertices);
    if (status != MStatus::kSuccess) {
        MGlobal::displayError("Failed to get triangles");
        return nullptr;
    }

    std::vector<int> indices;
    for (unsigned int i = 0; i < triangleVertices.length(); ++i) {
        indices.push_back(triangleVertices[i]);
    }

    // Create PolygonFaces
    std::vector<reactphysics3d::PolygonVertexArray::PolygonFace> polygonFaces;
    int indexBase = 0;
    for (unsigned int i = 0; i < triangleCounts.length(); ++i) {
        reactphysics3d::PolygonVertexArray::PolygonFace face;
        face.indexBase = indexBase;
        face.nbVertices = triangleCounts[i];  // Assuming triangleCounts contains the number of vertices per face
        polygonFaces.push_back(face);
        indexBase += triangleCounts[i];
    }

    // Create PolygonVertexArray
    auto polygonVertexArray = new reactphysics3d::PolygonVertexArray(
        vertices.length(),
        transformedVertices.data(),
        3 * sizeof(float),
        indices.data(),
        sizeof(int),
        polygonFaces.size(),
        polygonFaces.data(),
        reactphysics3d::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
        reactphysics3d::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);

    // Create PolyhedronMesh
    auto polyhedronMesh = physicsCommon.createPolyhedronMesh(polygonVertexArray);

    // Create ConvexMeshShape
    auto convexMeshShape = physicsCommon.createConvexMeshShape(polyhedronMesh);

    return convexMeshShape;
}

reactphysics3d::RigidBody* CustomMoveManip::createRigidBody(const reactphysics3d::Transform& transform, reactphysics3d::CollisionShape* collisionShape) {
    if (collisionShape == nullptr) {
        MGlobal::displayError("Collision shape is null");
        return nullptr;
    }

    // Create a rigid body in the physics world
    reactphysics3d::RigidBody* rigidBody = nullptr;
    if (physicsWorld != nullptr) {
        // Create a rigid body with the given transform
        rigidBody = physicsWorld->createRigidBody(transform);

        if (rigidBody == nullptr) {
            MGlobal::displayError("Failed to create a rigid body");
            return nullptr;
        }

        // Create a collider for the rigid body
        reactphysics3d::Collider* collider = rigidBody->addCollider(collisionShape, reactphysics3d::Transform::identity());

        if (collider == nullptr) {
            MGlobal::displayError("Failed to add collider to the rigid body");
            return nullptr;
        }
    }
    else {
        MGlobal::displayError("Physics world is null");
    }

    return rigidBody;
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
        MPlug tPlug = dependNodeFn.findPlug("translate", true, &stat);
        if (tPlug.isNull()) {
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
            //update active objects and physics world
            manipulator->addSelectedMObjects();
            manipulator->clearPhysicsWorld();
            manipulator->addRigidBodyFromSelectedObject();
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