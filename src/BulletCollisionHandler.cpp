// MayaToBulletUtils.cpp
#include <MayaIncludes.h>
#include "BulletCollisionHandler.h"

#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <LinearMath/btAlignedObjectArray.h>

BulletCollisionHandler::BulletCollisionHandler()
    : broadphase(nullptr),
    collisionConfiguration(nullptr),
    dispatcher(nullptr),
    solver(nullptr),
    dynamicsWorld(nullptr),
    activeRigidBody(nullptr)
{
}

BulletCollisionHandler::~BulletCollisionHandler() {
    deleteDynamicsWorld();
}

void BulletCollisionHandler::createDynamicsWorld() {
    // Create a broadphase interface
    this->broadphase = new btDbvtBroadphase();

    // Create a collision configuration
    this->collisionConfiguration = new btDefaultCollisionConfiguration();

    // Create a collision dispatcher
    this->dispatcher = new btCollisionDispatcher(collisionConfiguration);

    // Create a constraint solver
    this->solver = new btSequentialImpulseConstraintSolver;

    // Create the dynamics world
    this->dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    this->dynamicsWorld->setGravity(btVector3(0, 0, 0));
}

void BulletCollisionHandler::deleteDynamicsWorld() {
    if (this->dynamicsWorld) {
        // Remove and delete all rigid bodies from the dynamics world
        for (int i = this->dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
            btCollisionObject* obj = this->dynamicsWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState()) {
                delete body->getMotionState();
            }
            this->dynamicsWorld->removeCollisionObject(obj);
            delete obj;
        }

        // Delete the dynamics world
        delete this->dynamicsWorld;
    }

    if (solver) {
        delete solver;
        solver = nullptr;
    }

    if (dispatcher) {
        delete dispatcher;
        dispatcher = nullptr;
    }

    if (collisionConfiguration) {
        delete collisionConfiguration;
        collisionConfiguration = nullptr;
    }

    if (broadphase) {
        delete broadphase;
        broadphase = nullptr;
    }
}

void BulletCollisionHandler::updateActiveObject(MFnMesh* mesh)
{

    // Check if there's an existing activeRigidBody, and remove it from the world
    if (this->activeRigidBody != nullptr) {
        this->dynamicsWorld->removeRigidBody(this->activeRigidBody);
        delete this->activeRigidBody;
    }

    this->activeRigidBody = this->convertMFnMeshToRigidBody(mesh);

    if (this->activeRigidBody == nullptr) {
        MString test = "activeRigidBody is nullptr.";
        MGlobal::displayInfo(test);
    }
    this->dynamicsWorld->addRigidBody(this->activeRigidBody);
    MString info_msg = "BulletCollisionHandler:active object was updated and added to the dynamicsWorld.";
    MGlobal::displayInfo(info_msg);
}

void BulletCollisionHandler::updateColliders(std::vector<MFnMesh*> collidersMFnMeshes)
{

}

btRigidBody* BulletCollisionHandler::convertMFnMeshToRigidBody(MFnMesh* mfnMesh) {

    float mass = 10;

    // Extract vertices and indices from the MFnMesh
    MPointArray vertexArray;
    MIntArray triangleCounts, triangleVertices;
    mfnMesh->getTriangles(triangleCounts, triangleVertices);
    mfnMesh->getPoints(vertexArray, MSpace::kWorld);

    // Create Bullet's triangle mesh
    btTriangleMesh* triMesh = new btTriangleMesh();
    for (unsigned int i = 0; i < triangleVertices.length(); i += 3) {
        MPoint& v0 = vertexArray[triangleVertices[i]];
        MPoint& v1 = vertexArray[triangleVertices[i + 1]];
        MPoint& v2 = vertexArray[triangleVertices[i + 2]];

        btVector3 btV0(v0.x, v0.y, v0.z);
        btVector3 btV1(v1.x, v1.y, v1.z);
        btVector3 btV2(v2.x, v2.y, v2.z);

        triMesh->addTriangle(btV0, btV1, btV2);
    }

    // Get the transformation of the MFnMesh
    MDagPath dagPath;
    mfnMesh->getPath(dagPath);
    MFnTransform mfnTransform(dagPath);
    MTransformationMatrix mTransMatrix = mfnTransform.transformation();

    // Extract translation
    MVector translation = mTransMatrix.getTranslation(MSpace::kWorld);
    btVector3 btTranslation(translation.x, translation.y, translation.z);

    // Extract rotation
    MQuaternion rotation = mTransMatrix.rotation();
    btQuaternion btRotation(rotation.x, rotation.y, rotation.z, rotation.w);

    // Initialize the motion state with the extracted transformation
    btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btRotation, btTranslation));


    // Use the GImpact mesh shape for the rigid body
    btGImpactMeshShape* gImpactShape = new btGImpactMeshShape(triMesh);
    gImpactShape->updateBound();

    // Calculate the local inertia
    btVector3 localInertia(0, 0, 0);
    gImpactShape->calculateLocalInertia(mass, localInertia);

    // Create the rigid body
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, gImpactShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setActivationState(DISABLE_DEACTIVATION); // Keep the body always active
    

    return body;
}

/*
btCollisionShape* BulletCollisionHandler::convertMFnMeshToCollider(MFnMesh* mfnMesh) {

}
*/