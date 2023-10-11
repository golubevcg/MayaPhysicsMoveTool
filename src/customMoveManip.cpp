#include <stdio.h>
#include <vector>
#include <stdlib.h>

// Maya
#include <maya/MIOStream.h>
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
    MStatus addSelectedMFnMesh();
    MStatus getSceneMFnMeshes();
    MStatus initializeRTree();
    std::vector<MObject> checkNearbyObjects();
    void handleCollisions(std::vector<reactphysics3d::ConcaveMeshShape*>);
    // for rigid bodies
    reactphysics3d::ConvexMeshShape* CustomMoveManip::createConvexCollisionShapeFromMObject(const MObject& object);
    reactphysics3d::ConcaveMeshShape* CustomMoveManip::createConcaveCollisionShapeFromMObject(const MObject& object);
    reactphysics3d::RigidBody* CustomMoveManip::createRigidBody(const reactphysics3d::Transform& transform, reactphysics3d::CollisionShape* collisionShape);
    void clearPhysicsWorld();
    void addRigidBodyFromSelectedObject();
    // for static colliders
    void CustomMoveManip::createColliderFromMObject(MObject object);
    void createCollidersFromMObjects(std::vector<MObject>);
private:
    void updateManipLocations(const MObject& node);
public:
    MDagPath fFreePointManip;
    MSelectionList selList;
    std::vector<MObject> selectedMFnMeshes;
    std::vector<MObject> selectedTransforms;
    std::vector<MFnMesh*> mFnMeshes;
    bgi::rtree<value, bgi::quadratic<16>> rtree;

    reactphysics3d::PhysicsCommon physicsCommon;
    reactphysics3d::PhysicsWorld::WorldSettings physicsWorldSettings;
    reactphysics3d::PhysicsWorld* physicsWorld;
    std::vector<reactphysics3d::RigidBody*> activeRigidBodies;
    reactphysics3d::RigidBody* proxyRigidBody;
    std::vector<reactphysics3d::CollisionBody*> colliders;

    static MTypeId id;
};
MTypeId CustomMoveManip::id(0x8001d);

CustomMoveManip::CustomMoveManip()
{
    // The constructor must not call createChildren for user-defined
    // manipulators.

    // Create a PhysicsWorld
    //settings.isSleepingEnabled = True;
    this->physicsWorldSettings.gravity = reactphysics3d::Vector3(0, 0, 0);
    this->physicsWorld = this->physicsCommon.createPhysicsWorld(this->physicsWorldSettings);

    this->getSceneMFnMeshes();
    this->initializeRTree();
}

CustomMoveManip::~CustomMoveManip()
{
    for (MFnMesh* mesh : this->mFnMeshes) {
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
    this->fFreePointManip = addFreePointTriadManip("pointManip",
        "freePoint");

    return stat;
}

void CustomMoveManip::clearPhysicsWorld() {
    MString warningMessage4 = "clearPhysicsWorld";
    MGlobal::displayWarning(warningMessage4);

    if (this->physicsWorld != nullptr) {
        // Delete the current physics world
        this->physicsCommon.destroyPhysicsWorld(this->physicsWorld);
    }

    this->physicsWorld = this->physicsCommon.createPhysicsWorld(this->physicsWorldSettings);
} 

void CustomMoveManip::addRigidBodyFromSelectedObject() {

    if (this->selectedMFnMeshes.empty()) {
        return;
    }

    if (this->physicsWorld == nullptr) {
        MGlobal::displayError("Physics world is null");
        return;
    }

    this->activeRigidBodies.clear();

    MObject object = this->selectedMFnMeshes[0];

    //for (const auto& object : this->selectedObjects) {
    // Create a collision shape from the MObject
    reactphysics3d::ConvexMeshShape* collisionShape = createConvexCollisionShapeFromMObject(object);
    if (collisionShape == nullptr) {
        MGlobal::displayError("Failed to create collision shape");
        return;
    }

    MString warningMessage8 = "addRigidBodyFromSelectedObject4";
    MGlobal::displayWarning(warningMessage8);

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
    MEulerRotation eulerRotation = tm.eulerRotation();
    MQuaternion rotation = eulerRotation.asQuaternion();

    // Convert Maya transform to ReactPhysics3D transform
    reactphysics3d::Vector3 position(translation.x, translation.y, translation.z);
    reactphysics3d::Quaternion orientation(rotation.x, rotation.y, rotation.z, rotation.w);
    reactphysics3d::Transform transform(position, orientation);

    // Create a rigid body with the transform and collision shape
    reactphysics3d::RigidBody* rigidBody = createRigidBody(transform, collisionShape);
    if (rigidBody == nullptr) {
        MGlobal::displayError("Failed to create rigid body");
        return;
    }

    MString warningMessage9 = "addRigidBodyFromSelectedObject5";
    MGlobal::displayWarning(warningMessage9);

    // Remove the old rigid body
    //physicsWorld->destroyRigidBody(this->proxyRigidBody);
    //proxyRigidBody = nullptr;

    this->proxyRigidBody = this->physicsWorld->createRigidBody(transform);
    this->proxyRigidBody->setType(reactphysics3d::BodyType::KINEMATIC);

    // Create a sphere shape using PhysicsCommon
    float radius = 0.5;  // Set an appropriate radius value
    reactphysics3d::SphereShape* sphereShape = this->physicsCommon.createSphereShape(radius);

    // Add the sphere shape to the proxy rigid body
    this->proxyRigidBody->addCollider(sphereShape, reactphysics3d::Transform::identity());

    // Create a fixed joint between the proxy and the original rigid body
    reactphysics3d::Vector3 anchorPointWorldSpace(translation.x, translation.y, translation.z);  // You can set this to the appropriate position
    reactphysics3d::FixedJointInfo jointInfo(this->proxyRigidBody, rigidBody, anchorPointWorldSpace);
    reactphysics3d::Joint* joint = this->physicsWorld->createJoint(jointInfo);

    const reactphysics3d::uint32 kinematicCategory = 0x0001;
    const reactphysics3d::uint32 defaultCategory = 0x0002;

    //rigidBody->getCollider(0)->setCollideWithMaskBits(defaultCategory); // Kinematic body collides with others but not with kinematic
    //this->proxyRigidBody->getCollider(0)->setCollideWithMaskBits(0xFFFF ^ kinematicCategory); // Other bodies do not collide with kinematic


    this->activeRigidBodies.push_back(rigidBody);
    MString txt = "----------addRigidBodyFromSelectedObject6";
    MGlobal::displayInfo(txt);
}

MStatus CustomMoveManip::addSelectedMFnMesh() {

    MString warningMessage3 = "addSelectedMFnMesh";
    MGlobal::displayWarning(warningMessage3);

    MSelectionList selList;
    MGlobal::getActiveSelectionList(selList);

    this->selectedMFnMeshes.clear();
    this->selectedTransforms.clear();

    if (selList.isEmpty()) {
        MString warningMessage = "No objects selected";
        MGlobal::displayWarning(warningMessage);
        return MS::kFailure;
    }

    MObject node;
    selList.getDependNode(0, node);  // Get the first selected node

    if (node.hasFn(MFn::kTransform)) {
        MFnDagNode dagNode(node);
        bool meshFound = false;

        this->selectedTransforms.push_back(node);

        for (unsigned int i = 0; i < dagNode.childCount(); ++i) {
            MObject child = dagNode.child(i);
            if (child.hasFn(MFn::kMesh)) {
                MFnMesh mesh(child);

                MString infoMessage = "Mesh object found: " + mesh.name();
                MGlobal::displayInfo(infoMessage);

                this->selectedMFnMeshes.push_back(child);
                meshFound = true;
                break;
            }
        }

        if (!meshFound) {
            MString warningMessage = "No mesh found for the selected transform";
            MGlobal::displayWarning(warningMessage);
            return MS::kFailure;
        }
    }
    else {
        MString warningMessage = "Selected object is not a transform";
        MGlobal::displayWarning(warningMessage);
        return MS::kFailure;
    }

    MString warningMessage4 = "Mesh object added!";
    MGlobal::displayWarning(warningMessage4);

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

        this->mFnMeshes.push_back(fnMesh);
    }

    return MS::kSuccess;
}

MStatus CustomMoveManip::initializeRTree() {

    if (this->mFnMeshes.empty()) {
        MGlobal::displayError("MFnMeshes vector is empty");
        return MS::kFailure;
    }

    MStatus status;

    for (size_t i = 0; i < this->mFnMeshes.size(); ++i) {
        MDagPath dagPath;
        status = this->mFnMeshes[i]->getPath(dagPath);
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get MDagPath from MFnMesh");
            return status;
        }

        //exclude selected object

        MBoundingBox mbbox = this->mFnMeshes[i]->boundingBox();
        MMatrix worldMatrix = dagPath.inclusiveMatrix();

        MPoint minPoint = mbbox.min() * worldMatrix;
        MPoint maxPoint = mbbox.max() * worldMatrix;

        box bbox(point(minPoint.x, minPoint.y, minPoint.z), point(maxPoint.x, maxPoint.y, maxPoint.z));
        this->rtree.insert(std::make_pair(bbox, i));  // i is the identifier
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
    MFnFreePointTriadManip freePointManipFn(this->fFreePointManip);
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
    MFnFreePointTriadManip manipFn(this->fFreePointManip);

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

MStatus CustomMoveManip::doDrag()
{
    //MStatus status = MPxManipContainer::doDrag();

    MString txt = "doDrag";
    MGlobal::displayInfo(txt);


    std::vector<MObject> collisionCandidates = this->checkNearbyObjects();
    MString txt2 = "collisionCandidates";
    MGlobal::displayInfo(txt2);

    if (!collisionCandidates.empty()) {
        this->createCollidersFromMObjects(collisionCandidates);
    }

    float timeStep = 0.01f;  // Adjust this value as needed
    int numberOfUpdates = 10;  // Update the simulation 10 times
    // 1. Update the physics world
    this->physicsWorld->update(timeStep);

    // TRY TO GET TRANSLATION DIRECTLY FROM fFeePointManip because it is MDagPath
    // 2. Retrieve the current position of the manipulator
    MFnManip3D manipFn(this->fFreePointManip);

    MPoint currentPosition;
    this->getConverterManipValue(0, currentPosition);

    MPoint currentTranslation = manipFn.translation(MSpace::kWorld);

    // 3. Set the position of the proxy rigid body to the manipulator's position
    reactphysics3d::Vector3 newPosition(currentPosition.x + currentTranslation.x, currentPosition.y + currentTranslation.y, currentPosition.z + currentTranslation.z);

    // Log the current state for debugging
    MString logMessage = "Setting transform for proxyRigidBody. Current manip position: ";
    logMessage += MString() + currentPosition.x + currentTranslation.x + ", " + currentPosition.y + currentTranslation.y + ", " + currentPosition.z + currentTranslation.z;
    MGlobal::displayInfo(logMessage);

    MString logMessage2 = "is proxyRigidBody null ptr: ";
    if (this->proxyRigidBody == nullptr) {
        logMessage2 += "true";
    }
    else {
        logMessage2 += "false";
    }
    MGlobal::displayInfo(logMessage2);

    const reactphysics3d::Transform& transform = this->activeRigidBodies[0]->getTransform();
    // Get the position from the transform
    const reactphysics3d::Vector3& position = transform.getPosition();
    /*
    // Print the position to MGlobal
    MString positionInfo = "Position of proxyRigidBody: ";
    positionInfo += "(" + MString() + position.x + ", " + position.y + ", " + position.z + ")";
    MGlobal::displayInfo(positionInfo);
    */
    this->proxyRigidBody->setTransform(reactphysics3d::Transform(newPosition, this->proxyRigidBody->getTransform().getOrientation()));
    
    /*
    MString txt3 = "BEFORE UPDATE";
    MGlobal::displayInfo(txt3);
    */
    
    // 4. Update the physics world multiple times and print the position of the proxy object after each update
    for (int i = 0; i < numberOfUpdates; ++i) {
        this->physicsWorld->update(timeStep);
        MString txt55 = "UPDATE";
        MGlobal::displayInfo(txt55);
        // Print the position of the proxy object for debugging
        if (this->proxyRigidBody != nullptr) {
            reactphysics3d::Vector3 position = this->proxyRigidBody->getTransform().getPosition();
            //MString positionStr = MString("Position: ") + position.x + ", " + position.y + ", " + position.z;
            //MGlobal::displayInfo(positionStr);
        }
    }

    // 5. Get the final position and orientation of the proxy rigid body
    MPoint finalPosition;
    MQuaternion finalOrientation;
    if (this->proxyRigidBody != nullptr) {
        reactphysics3d::Vector3 position = this->proxyRigidBody->getTransform().getPosition();
        reactphysics3d::Quaternion orientation = this->proxyRigidBody->getTransform().getOrientation();
        finalPosition = MPoint(position.x, position.y, position.z);
        finalOrientation = MQuaternion(orientation.x, orientation.y, orientation.z, orientation.w); // Convert to Maya's MQuaternion
        /*
        MString logMessage2 = "reactphysics3d::Vector3: ";
        logMessage2 += MString() + position.x + ", " + position.y + ", " + position.z;
        MGlobal::displayInfo(logMessage2);
        */
    }

    // ... (rest of your code remains the same)

    // 6. Set this final position and orientation to the selected object
    MDagPath dagPath;
    MFnDagNode(selectedMFnMeshes[0]).getPath(dagPath);
    MObject transformNode = dagPath.transform();
    MFnTransform fnTransform(transformNode);
    MTransformationMatrix originalTM = fnTransform.transformation();

    // Extract the original scale
    double scale[3];
    originalTM.getScale(scale, MSpace::kWorld);

    // Set translation and rotation
    MTransformationMatrix newTM;
    MVector translation(finalPosition.x, finalPosition.y, finalPosition.z);
    newTM.setTranslation(translation, MSpace::kWorld);
    newTM.setRotationQuaternion(finalOrientation.x, finalOrientation.y, finalOrientation.z, finalOrientation.w);
    // Apply the original scale
    newTM.setScale(scale, MSpace::kWorld);

    // Apply the new transformation matrix to the transform node
    fnTransform.set(newTM.asMatrix());

    return MS::kUnknownParameter;
}

std::vector<MObject> CustomMoveManip::checkNearbyObjects() {

    std::vector<MObject> collisionCandidates;
    if (this->selectedMFnMeshes.size() == 0) {
        return collisionCandidates;
    }

    for (const auto& currentMObject : this->selectedMFnMeshes) {
        MStatus status;

        MFnDagNode dagNode(currentMObject);
        MDagPath dagPath;
        MDagPath::getAPathTo(currentMObject, dagPath);
        MFnTransform transform(dagPath.transform(), &status);
        if (status != MStatus::kSuccess) {
            // Handle error
            return collisionCandidates;
        }
        MMatrix worldMatrix = transform.transformationMatrix();

        MBoundingBox boundingBox = dagNode.boundingBox();
        MPoint worldMinPoint = boundingBox.min() * worldMatrix;
        MPoint worldMaxPoint = boundingBox.max() * worldMatrix;

        // Expand the bounding box by a certain distance to find nearby objects
        double distance = 0.1;
        worldMinPoint -= MVector(distance, distance, distance);
        worldMaxPoint += MVector(distance, distance, distance);

        box queryBox(point(worldMinPoint.x, worldMinPoint.y, worldMinPoint.z),
            point(worldMaxPoint.x, worldMaxPoint.y, worldMaxPoint.z));

        std::vector<value> result;
        this->rtree.query(bgi::intersects(queryBox), std::back_inserter(result));

        // collect all collided objects
        for (const auto& item : result) {
            // TODO:CHANGE THIS TO CONDITION CHECK THAT DAG MOBJECT ALREADY IN THE LIST
            if (this->mFnMeshes[item.second]->object() != currentMObject) {  // Exclude the selected mesh itself
                MFnDagNode dagNode1(this->mFnMeshes[item.second]->object());
                MString rtree = "added mobject from RTREE" + MString() + dagNode1.fullPathName() + MString() + worldMinPoint.x + worldMinPoint.y + worldMinPoint.z;
                MGlobal::displayInfo(rtree);
                                                                             
                //MString txt = "The selected object is near another object.";
                //MGlobal::displayInfo(txt);
                collisionCandidates.push_back(this->mFnMeshes[item.second]->object());
                MString txt11 = "Added object to collision candidates";
                MGlobal::displayInfo(txt11);
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

void CustomMoveManip::createCollidersFromMObjects(std::vector<MObject> meshes) {

    if (meshes.empty()) {
        MString txt = "Provided vector of collision candidates is empty";
        MGlobal::displayWarning(txt);
        return;
    }

    //TODO DESTROY EXISTING COLLIDERS
    // CLEANUP MESHES

    //PROBABLY U NEED TO CHECK THAT SELECTED OBJECT IS NOT IN THE LIST OF COLLISION CANDIDATES

    for (size_t i = 0; i < meshes.size(); ++i) {
        if (meshes[i] == this->selectedMFnMeshes[0]) {
            continue;
        }
        this->createConcaveCollisionShapeFromMObject(meshes[i]);
        //reactphysics3d::CollisionBody* collider = this->createColliderFromMObject(meshes[i]);
        //this->colliders.push_back(collider);
        MString txt2 = "created ConcaveCollider for mObject";
        MGlobal::displayInfo(txt2);
    }
}

//reactphysics3d::CollisionBody* CustomMoveManip::createColliderFromMObject(MObject object) {
void CustomMoveManip::createColliderFromMObject(MObject object) {
    MString txt = "createColliderFromMObject1";
    MGlobal::displayWarning(txt);
    if (object.isNull() || this->physicsWorld == nullptr) {
        MGlobal::displayError("Either selected objects are empty or physics world is null");
        //return nullptr;
    }

    MString txt2 = "createColliderFromMObject2";
    MGlobal::displayWarning(txt2);
    reactphysics3d::ConvexMeshShape* collisionShape = createConvexCollisionShapeFromMObject(object);

    if (collisionShape == nullptr) {
        MGlobal::displayError("Failed to create collision shape");
        //return nullptr;
    }
    MString txt3 = "createColliderFromMObject3";
    MGlobal::displayWarning(txt3);

    MFnDagNode dagNode(object);
    MDagPath nodePath;
    dagNode.getPath(nodePath);
    MMatrix m = nodePath.inclusiveMatrix();

    MVector translation(m[3][0], m[3][1], m[3][2]);

    MMatrix rotationMatrix;
    rotationMatrix[0][0] = m[0][0]; rotationMatrix[0][1] = m[0][1]; rotationMatrix[0][2] = m[0][2];
    rotationMatrix[1][0] = m[1][0]; rotationMatrix[1][1] = m[1][1]; rotationMatrix[1][2] = m[1][2];
    rotationMatrix[2][0] = m[2][0]; rotationMatrix[2][1] = m[2][1]; rotationMatrix[2][2] = m[2][2];
    MTransformationMatrix tm(rotationMatrix);
    MEulerRotation eulerRotation = tm.eulerRotation();
    MQuaternion rotation = eulerRotation.asQuaternion();

    reactphysics3d::Vector3 position(translation.x, translation.y, translation.z);
    reactphysics3d::Quaternion orientation(rotation.x, rotation.y, rotation.z, rotation.w);
    reactphysics3d::Transform transform(position, orientation);

    MString txt4 = "createColliderFromMObject4";
    MGlobal::displayWarning(txt4);


    // Create a collision body in the world
    reactphysics3d::CollisionBody* collisionBody = this->physicsWorld->createCollisionBody(transform);
    if (collisionBody == nullptr) {
        MGlobal::displayError("Failed to create collision body");
        //return nullptr;
    }

    // Add the collision shape as a collider to the collision body
    collisionBody->addCollider(collisionShape, reactphysics3d::Transform::identity());
    if (collisionBody->getCollider(0) == nullptr) {
        MGlobal::displayError("Failed to add collider to collision body");
        //return nullptr;
    }
    //return collisionBody;
}

reactphysics3d::ConcaveMeshShape* CustomMoveManip::createConcaveCollisionShapeFromMObject(const MObject& object) {
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
        //MPoint transformedVertex = vertices[i] * transformMatrix;
        MPoint transformedVertex = vertices[i];
        transformedVertices.push_back(static_cast<float>(transformedVertex.x));
        transformedVertices.push_back(static_cast<float>(transformedVertex.y));
        transformedVertices.push_back(static_cast<float>(transformedVertex.z));
    }

    // Get triangles
    MIntArray triangleCounts, triangleVertices;
    status = fnMesh.getTriangles(triangleCounts, triangleVertices);
    if (status != MStatus::kSuccess) {
        MGlobal::displayError("Failed to get triangles");
        return nullptr;
    }

    // Validate that the triangleVertices is a multiple of 3
    if (triangleVertices.length() % 3 != 0) {
        MGlobal::displayError("triangleVertices is not a multiple of 3");
        return nullptr;
    }

    // Convert MIntArray to std::vector<int>
    std::vector<int> triangleVerticesVector(triangleVertices.begin(), triangleVertices.end());

    // Create TriangleVertexArray
    auto triangleVertexArray = new reactphysics3d::TriangleVertexArray(
        vertices.length(),
        transformedVertices.data(),
        3 * sizeof(float),
        triangleVerticesVector.size() / 3,
        triangleVerticesVector.data(),
        3 * sizeof(int),
        reactphysics3d::TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
        reactphysics3d::TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE
    );

    // Create TriangleMesh and add the TriangleVertexArray as a subpart
    auto triangleMesh = this->physicsCommon.createTriangleMesh();
    triangleMesh->addSubpart(triangleVertexArray);

    // Create ConcaveMeshShape
    auto concaveMeshShape = this->physicsCommon.createConcaveMeshShape(triangleMesh);
    MString testText4 = "------concaveMeshShape";
    if (concaveMeshShape == nullptr) {
        testText4 += " NullPOINTER!!!!";
    }
    MGlobal::displayWarning(testText4);

    if (concaveMeshShape != nullptr) {
        // Create a static rigid body (mass = 0)
        MMatrix m = dagPath.inclusiveMatrix();

        // Extract translation from the matrix
        MVector translation(m[3][0], m[3][1], m[3][2]);

        // Extract rotation matrix and convert it to quaternion
        MMatrix rotationMatrix;
        rotationMatrix[0][0] = m[0][0]; rotationMatrix[0][1] = m[0][1]; rotationMatrix[0][2] = m[0][2];
        rotationMatrix[1][0] = m[1][0]; rotationMatrix[1][1] = m[1][1]; rotationMatrix[1][2] = m[1][2];
        rotationMatrix[2][0] = m[2][0]; rotationMatrix[2][1] = m[2][1]; rotationMatrix[2][2] = m[2][2];
        MTransformationMatrix tm(rotationMatrix);
        MEulerRotation eulerRotation = tm.eulerRotation();
        MQuaternion rotation = eulerRotation.asQuaternion();

        // Convert Maya transform to ReactPhysics3D transform
        reactphysics3d::Vector3 position(translation.x, translation.y, translation.z);
        reactphysics3d::Quaternion orientation(rotation.x, rotation.y, rotation.z, rotation.w);
        reactphysics3d::Transform transform(position, orientation);

        reactphysics3d::RigidBody* colliderRigidBody = physicsWorld->createRigidBody(transform);
        colliderRigidBody->setType(reactphysics3d::BodyType::STATIC);

        // Attach the concave mesh shape to the rigid body
        colliderRigidBody->addCollider(concaveMeshShape, reactphysics3d::Transform::identity());

        // Now the static rigid body with the attached collision shape is part of the physics world
    }

    return concaveMeshShape;
}

reactphysics3d::ConvexMeshShape* CustomMoveManip::createConvexCollisionShapeFromMObject(const MObject& object) {
    MStatus status;
    MDagPath dagPath;
    MFnDagNode(object).getPath(dagPath);
    MFnMesh fnMesh(object, &status);

    MString txt = "entered createConvexCollisionShapeFromMObject";
    MGlobal::displayWarning(txt);

    // Get vertices
    MPointArray vertices;
    status = fnMesh.getPoints(vertices, MSpace::kObject);
    if (status != MStatus::kSuccess) {
        MGlobal::displayError("Failed to get vertices");
        return nullptr;
    }

    MString txt1 = "createConvexCollisionShapeFromMObject2";
    MGlobal::displayWarning(txt1);

    // Apply transformation to vertices
    MMatrix transformMatrix = dagPath.inclusiveMatrix();
    std::vector<float> transformedVertices;
    for (unsigned int i = 0; i < vertices.length(); ++i) {
        //MPoint transformedVertex = vertices[i] * transformMatrix;
        MPoint transformedVertex = vertices[i];
        transformedVertices.push_back(static_cast<float>(transformedVertex.x));
        transformedVertices.push_back(static_cast<float>(transformedVertex.y));
        transformedVertices.push_back(static_cast<float>(transformedVertex.z));
    }

    MString txt3 = "createConvexCollisionShapeFromMObject3";
    MGlobal::displayWarning(txt3);

    // Get triangles
    MIntArray triangleCounts, triangleVertices;
    status = fnMesh.getTriangles(triangleCounts, triangleVertices);
    if (status != MStatus::kSuccess) {
        MGlobal::displayError("Failed to get triangles");
        return nullptr;
    }

    MString txt4 = "createConvexCollisionShapeFromMObject4";
    MGlobal::displayWarning(txt4);

    // Validate that the triangleVertices is a multiple of 3
    if (triangleVertices.length() % 3 != 0) {
        MGlobal::displayError("triangleVertices is not a multiple of 3");
        return nullptr;
    }

    MString txt5 = "createConvexCollisionShapeFromMObject5";
    MGlobal::displayWarning(txt5);

    std::vector<int> indices(triangleVertices.begin(), triangleVertices.end());

    // Create PolygonFaces
    std::vector<reactphysics3d::PolygonVertexArray::PolygonFace> polygonFaces;
    for (unsigned int i = 0; i < triangleVertices.length(); i += 3) {
        reactphysics3d::PolygonVertexArray::PolygonFace face;
        face.indexBase = i;
        face.nbVertices = 3;  // Since we're dealing with triangles
        polygonFaces.push_back(face);
    }

    //-----
    MString testText = "------createConvexCollisionShapeFromMObject";
    MGlobal::displayInfo(testText);

    //------------------------------------------------------
    // Print vertices length
    MString info = "--------Vertices Length: ";
    info += vertices.length();
    MGlobal::displayInfo(info);

    // Print transformed vertices data
    info = "Transformed Vertices: ";
    for (unsigned int i = 0; i < transformedVertices.size(); i += 3) {
        info += MString(" (") + transformedVertices[i] + ", " + transformedVertices[i + 1] + ", " + transformedVertices[i + 2] + ")";
    }
    MGlobal::displayInfo(info);

    // Print size of float
    MString info1 = "Vertices Length: ";
    info1 += MString("") + 3 * sizeof(float);
    MGlobal::displayInfo(info1);

    // Print indices data
    MString info2 = "Indices: ";
    for (unsigned int i = 0; i < indices.size(); ++i) {
        info2 += indices[i];
        if (i < indices.size() - 1) {
            info2 += ", ";
        }
    }
    MGlobal::displayInfo(info2);

    // Print size of int
    MString info3 = "Size of Int: ";
    info3 += MString("") + sizeof(int);
    MGlobal::displayInfo(info3);

    // Print polygon faces size
    MString info4 = "Polygon Faces Size: ";
    info4 += MString("") + polygonFaces.size();
    MGlobal::displayInfo(info4);

    // Print polygon faces data
    MString info5 = "Polygon Faces Data: ";
    for (const auto& face : polygonFaces) {
        info5 += MString(" (indexBase=") + face.indexBase + ", nbVertices=" + face.nbVertices + ")";
    }
    MGlobal::displayInfo(info5);
    //------------------------------------------------------


    // Create PolygonVertexArray
    auto polygonVertexArray = new reactphysics3d::PolygonVertexArray(
        vertices.length(), //nbVertices Number of vertices in the array
        transformedVertices.data(), //verticesStart Pointer to the start of the vertices data
        3 * sizeof(float), //verticesStride The number of bytes between two consecutive vertices in the array
        indices.data(), //indexesStart    Pointer to the start of the face indices data    
        sizeof(int), //indexesStride   The number of bytes between two consecutive face
        polygonFaces.size(), //nbFaces The number of faces in the array
        polygonFaces.data(), //nbFaces Pointer to the start of the faces data
        reactphysics3d::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
        reactphysics3d::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE
    );

    MString testText3 = "------polygonVertexArray";
    if (polygonVertexArray == nullptr) {
        testText3 += " NullPOINTER!!!!";
    }
    MGlobal::displayWarning(testText3);

    // Create PolyhedronMesh
    auto polyhedronMesh = this->physicsCommon.createPolyhedronMesh(polygonVertexArray);
    MString testText2 = "------polyhedronMesh";
    if (polyhedronMesh == nullptr) {
        testText2 += " NullPOINTER!!!!";
    }
    MGlobal::displayWarning(testText2);


    // Create ConvexMeshShape
    auto convexMeshShape = this->physicsCommon.createConvexMeshShape(polyhedronMesh);
    MString testText4 = "------convexMeshShape";
    if (convexMeshShape == nullptr) {
        testText4 += " NullPOINTER!!!!";
    }
    MGlobal::displayWarning(testText4);

    return convexMeshShape;
}

reactphysics3d::RigidBody* CustomMoveManip::createRigidBody(const reactphysics3d::Transform& transform, reactphysics3d::CollisionShape* collisionShape) {
    if (collisionShape == nullptr) {
        MGlobal::displayError("Collision shape is null");
        return nullptr;
    }

    // Create a rigid body in the physics world
    reactphysics3d::RigidBody* rigidBody = nullptr;
    if (this->physicsWorld != nullptr) {
        // Create a rigid body with the given transform
        rigidBody = this->physicsWorld->createRigidBody(transform);

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
        CustomMoveManip* manipulator = (CustomMoveManip*)CustomMoveManip::newManipulator(manipName, manipObject);
        MString testText1 = "NULL != manipulator";
        MGlobal::displayInfo(testText1);
        if (NULL != manipulator) {

            MString testText2 = "--INSIDE";
            MGlobal::displayInfo(testText2);

            // Add the manipulator
            //
            ctxPtr->addManipulator(manipObject);
            // Connect the manipulator to the object in the selection list.
            //

            /**/
            if (!manipulator->connectToDependNode(dependNode))
            {
                MGlobal::displayWarning("Error connecting manipulator to"
                    " object: " + dependNodeFn.name());
            }

            //update active objects and physics world
            manipulator->addSelectedMFnMesh();
            manipulator->clearPhysicsWorld();
            manipulator->addRigidBodyFromSelectedObject();

            MString testText3 = "------selectionChanged";
            MGlobal::displayInfo(testText3);
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