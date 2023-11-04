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
    activeRigidBody(nullptr),
    proxyRigidBody(nullptr)
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
    this->cleanRigidBody(this->activeRigidBody);

    this->activeRigidBody = this->convertMFnMeshToRigidBody(mesh);

    this->dynamicsWorld->addRigidBody(this->activeRigidBody);
    MString info_msg = "BulletCollisionHandler:active object was updated and added to the dynamicsWorld.";
    MGlobal::displayInfo(info_msg);

    // setup proxy object
    this->updateActiveObjectProxy(this->activeRigidBody->getWorldTransform());
    this->constrainBodies(this->activeRigidBody, this->proxyRigidBody);
    //constrain them
}

void BulletCollisionHandler::updateActiveObjectProxy(const btTransform& startTransform) {
    this->cleanRigidBody(this->proxyRigidBody);

    // Assume a box shape for the proxy object for simplicity
    btCollisionShape* shape = new btBoxShape(btVector3(1, 1, 1)); // A 1m cube

    // Kinematic objects have zero mass
    btScalar mass(0.f);

    // Motion state with the starting transform
    btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);

    // Rigidbody construction info
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape);

    // Create the rigid body
    this->proxyRigidBody = new btRigidBody(rbInfo);

    // Set the body to kinematic
    this->proxyRigidBody->setCollisionFlags(this->proxyRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);

    // Disable deactivation
    this->proxyRigidBody->setActivationState(DISABLE_DEACTIVATION);

    // Add the body to the world
    this->dynamicsWorld->addRigidBody(this->proxyRigidBody);
}

void BulletCollisionHandler::cleanRigidBody(btRigidBody* body) {
    if (body) {
        // Remove it from the world
        this->dynamicsWorld->removeRigidBody(body);

        // Delete the motion state and the rigid body to avoid memory leaks
        delete body->getMotionState();
        delete body;
        body = nullptr; // Ensure the pointer is reset to nullptr after deletion
    }
}

void BulletCollisionHandler::constrainBodies(btRigidBody* mainBody, btRigidBody* proxyBody) {
    // Create a point-to-point constraint between the two bodies
    // This will keep them at the same location, but allow for rotations.
    btTypedConstraint* constraint = new btPoint2PointConstraint(*mainBody, *proxyBody, btVector3(0, 0, 0), btVector3(0, 0, 0));

    // Optionally set some constraint parameters
    // static_cast<btPoint2PointConstraint*>(constraint)->m_setting.m_impulseClamp = 10; // example parameter

    // Add the constraint to the world
    this->dynamicsWorld->addConstraint(constraint, true);
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