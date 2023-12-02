// MayaToBulletUtils.cpp

#include <string>
#include <MayaIncludes.h>
#include "BulletCollisionHandler.h"
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <btBulletDynamicsCommon.h>

BulletCollisionHandler::BulletCollisionHandler(): 
    broadphase(nullptr),
    collisionConfiguration(nullptr),
    dispatcher(nullptr),
    solver(nullptr),
    dynamicsWorld(nullptr),
    activeRigidBody(nullptr),
    proxyRigidBody(nullptr){
    MGlobal::displayWarning("---BulletCollisionHandler CONSTRUCTOR");
}

BulletCollisionHandler::~BulletCollisionHandler() {
    deleteDynamicsWorld();
}

void BulletCollisionHandler::createDynamicsWorld() {
    this->broadphase = new btDbvtBroadphase();
    this->collisionConfiguration = new btDefaultCollisionConfiguration();
    this->dispatcher = new btCollisionDispatcher(collisionConfiguration);
    this->solver = new btSequentialImpulseConstraintSolver;

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

void BulletCollisionHandler::updateWorld(float framesToUpdate) {
    float fps = 60.0f; // This is your simulation's fps

    // Calculate deltaTime assuming each frame is 1/fps seconds long
    float deltaTime = framesToUpdate / fps;

    int maxSubSteps = 200;
    float fixedTimeStep = 1.f / fps; // This is the time each physics "step" represents at 24fps
    this->dynamicsWorld->getSolverInfo().m_numIterations = 30; // Example value

    // Update the dynamics world by the deltaTime
    this->dynamicsWorld->stepSimulation(deltaTime, maxSubSteps, fixedTimeStep);
}

void BulletCollisionHandler::updateActiveObject(MFnMesh* mesh)
{
    this->cleanRigidBody(this->activeRigidBody);
    this->activeRigidBody = this->createFullActiveRigidBodyFromMFnMesh(mesh);
}

MMatrix BulletCollisionHandler::getActiveObjectTransformMMatrix() {
    if (this->activeRigidBody) {
        btTransform btTrans;
        btMotionState* motionState = this->activeRigidBody->getMotionState();
        if (motionState) {
            motionState->getWorldTransform(btTrans);
        }
        else {
            btTrans = this->activeRigidBody->getWorldTransform();
        }

        return this->convertBulletToMayaMatrix(btTrans);
    }

    // If the proxy object does not exist, return an identity matrix
    return MMatrix::identity;
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

void BulletCollisionHandler::updateColliders(std::vector<MFnMesh*> collidersMFnMeshes)
{
    // Remove all existing colliders from the world and delete them
    for (auto& collider : this->colliders) {
        if (collider) {
            this->dynamicsWorld->removeCollisionObject(collider);
            delete collider->getCollisionShape();
            btRigidBody* body = btRigidBody::upcast(collider);
            if (body && body->getMotionState()) {
                delete body->getMotionState();
            }
            delete collider;
        }
    }
    this->colliders.clear();

    // Add new colliders based on the provided MFnMeshes
    for (auto& mfnMesh : collidersMFnMeshes) {

        btRigidBody* rigidBody = this->createFullColliderFromMFnMesh(mfnMesh);

        // Add the new rigid body to the dynamics world
        //this->dynamicsWorld->addRigidBody(rigidBody, 1, 1);
        this->dynamicsWorld->addRigidBody(rigidBody, btBroadphaseProxy::StaticFilter, btBroadphaseProxy::AllFilter);

        // Keep a reference to the collider for later removal or other operations
        this->colliders.push_back(rigidBody);
        MGlobal::displayInfo("Collider was added..." + MString() + mfnMesh->fullPathName());
    }
}

btRigidBody* BulletCollisionHandler::createFullColliderFromMFnMesh(MFnMesh* mfnMesh) {
    btCollisionShape* newShape = this->convertMFnMeshToStaticCollisionShape(mfnMesh);
    newShape->setMargin(0.05);

    btTransform bulletTransform = btTransform::getIdentity();

    // Create a rigid body with a mass of 0 for a static object
    btDefaultMotionState* motionState = new btDefaultMotionState(bulletTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0, motionState, newShape, btVector3(0, 0, 0));
    btRigidBody* rigidBody = new btRigidBody(rbInfo);

    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
    rigidBody->setActivationState(DISABLE_DEACTIVATION);

    rigidBody->setRestitution(0);
    rigidBody->setFriction(0.1);
    rigidBody->setRollingFriction(0.1);
    rigidBody->setSpinningFriction(0.1);
    rigidBody->setDamping(0.1, 0.1);

    //ADD MASS AND INERTIA FOR STATIC COLLIDERS
    return rigidBody;
}

btRigidBody* BulletCollisionHandler::createFullActiveRigidBodyFromMFnMesh(MFnMesh* mfnMesh) {
    //btCollisionShape* collisionShape = this->convertMFnMeshToStaticCollisionShape(mfnMesh);
    
    btCollisionShape* collisionShape = this->convertMFnMeshToActiveCollisionShape(mfnMesh);
    collisionShape->setMargin(0.05);

    // Convert Maya's transformation matrix to Bullet's btTransform
    btTransform bulletTransform = this->getBulletTransformFromMFnMeshTransform(mfnMesh);

    // Define the mass of the rigid body
    float mass = 1.5;
    btVector3 localInertia(0, 0, 0);
    collisionShape->calculateLocalInertia(mass, localInertia);

    // Create the rigid body with the Bullet transform and collision shape
    btDefaultMotionState* motionState = new btDefaultMotionState(bulletTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, collisionShape, localInertia);
    btRigidBody* rigidBody = new btRigidBody(rbInfo);

    rigidBody->setRestitution(0);
    rigidBody->setFriction(0.15);
    rigidBody->setRollingFriction(0.15);
    rigidBody->setSpinningFriction(0.15);
    rigidBody->setDamping(0.15, 0.15);
    //rigidBody->setContactStiffnessAndDamping(0, 0.1);

    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_DYNAMIC_OBJECT);
    rigidBody->setActivationState(DISABLE_DEACTIVATION);

    // Add rigid body to the dynamics world and set up proxy object
    this->dynamicsWorld->addRigidBody(rigidBody, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
    MString info_msg = "BulletCollisionHandler: Active object updated and added to the dynamicsWorld.";
    MGlobal::displayInfo(info_msg);

    MGlobal::displayInfo("CREATING MESHES FOR ACTIVE OBJECTS");

    return rigidBody;
}

btTransform BulletCollisionHandler::getBulletTransformFromMFnMeshTransform(MFnMesh* mfnMesh) {
    if (!mfnMesh) {
        // Handle the case where mfnMesh is null
        // For example, return an identity transform or throw an exception
        return btTransform::getIdentity();
    }

    // Extract the transformation from MFnMesh
    MDagPath dagPath;
    mfnMesh->getPath(dagPath);

    MGlobal::displayInfo("DAGPATH OBJECT FULL PATH" + dagPath.fullPathName());
    MMatrix worldMatrix = dagPath.inclusiveMatrix();

    // Convert Maya's transformation matrix to Bullet's btTransform
    btTransform bulletTransform = convertMayaToBulletMatrix(worldMatrix);

    return bulletTransform;
}

btCollisionShape* BulletCollisionHandler::convertMFnMeshToStaticCollisionShape(MFnMesh* mfnMesh) {
    // Get the points in local space
    MPointArray mayaVertices;
    mfnMesh->getPoints(mayaVertices, MSpace::kWorld);

    // Create the Bullet triangle mesh
    btTriangleMesh* triMesh = new btTriangleMesh();

    // Get triangles from the mesh
    MIntArray triangleCounts, triangleVertices;
    mfnMesh->getTriangles(triangleCounts, triangleVertices);

    // Index variables for triangleVertices
    int triangleIndex = 0;
    MGlobal::displayInfo(MString("-----GENERATING COLLIDER FOR MESH: ") + MString() + mfnMesh->fullPathName());

    // Loop through the number of triangles
    for (unsigned int i = 0; i < triangleCounts.length(); ++i) {
        for (int j = 0; j < triangleCounts[i]; ++j) {
            btVector3 vertices[3];

            for (int k = 0; k < 3; ++k) {
                // Get the vertex index
                int vertexIndex = triangleVertices[triangleIndex + k];

                // Transform the vertex position to world space
                MPoint worldSpaceVertex = mayaVertices[vertexIndex];

                // Add vertex to Bullet triangle mesh
                // Convert from Maya's right-handed Y-up to Bullet's left-handed Z-up system
                vertices[k] = btVector3(
                    static_cast<btScalar>(worldSpaceVertex.x),
                    static_cast<btScalar>(worldSpaceVertex.z), // Swap Y and Z
                    static_cast<btScalar>(-worldSpaceVertex.y)
                ); // Invert Z for left-handed system
            }

            // Add the triangle to the mesh
            //triMesh->addTriangle(vertices[0], vertices[2], vertices[1]);
            triMesh->addTriangle(vertices[0], vertices[1], vertices[2]);

            MGlobal::displayInfo(MString("vertex pos of collider ") + vertices->getX() + ", " + vertices->getY() + ", " + vertices->getZ());

            // Move to the next set of vertices
            triangleIndex += 3;
        }
    }

    // Create the mesh shape
    bool useQuantizedAABBCompression = true;
    btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape(triMesh, useQuantizedAABBCompression);

    return meshShape;
}

btCollisionShape* BulletCollisionHandler::convertMFnMeshToActiveCollisionShape(MFnMesh* mfnMesh) {
    // Get the points in local space
    MPointArray mayaVertices;
    mfnMesh->getPoints(mayaVertices, MSpace::kObject);

    // Create the Bullet convex hull shape
    btConvexHullShape* convexHull = new btConvexHullShape();

    // Loop through the vertices and add them to the convex hull
    for (unsigned int i = 0; i < mayaVertices.length(); ++i) {
        MPoint vertex = mayaVertices[i];

        // Convert from Maya's right-handed Y-up to Bullet's left-handed Z-up system
        btVector3 bulletVertex(
            static_cast<btScalar>(vertex.x),
            static_cast<btScalar>(vertex.z), // Swap Y and Z
            static_cast<btScalar>(-vertex.y) // Invert Z for left-handed system
        );

        // Add the vertex to the convex hull shape
        convexHull->addPoint(bulletVertex);
    }

    // Optionally: Optimize the shape
    convexHull->optimizeConvexHull();
    convexHull->initializePolyhedralFeatures();

    return convexHull;
}

btTransform BulletCollisionHandler::convertMayaToBulletMatrix(const MMatrix& mayaMatrix) {
    MTransformationMatrix mayaTransMatrix(mayaMatrix);
    MQuaternion mayaQuat = mayaTransMatrix.rotation();

    // Convert Maya quaternion to Bullet quaternion, adjusting for coordinate system differences
    btQuaternion bulletQuat(mayaQuat.x, -mayaQuat.z, mayaQuat.y, mayaQuat.w);

    MVector mayaTranslation = mayaTransMatrix.getTranslation(MSpace::kTransform);

    // Adjust the translation for Bullet's coordinate system (Z-up)
    btVector3 bulletTranslation(mayaTranslation.x, mayaTranslation.z, -mayaTranslation.y);

    btTransform bulletTransform;
    bulletTransform.setRotation(bulletQuat);
    bulletTransform.setOrigin(bulletTranslation);

    return bulletTransform;
}

MMatrix BulletCollisionHandler::convertBulletToMayaMatrix(const btTransform& bulletTransform) {
    btQuaternion bulletQuat = bulletTransform.getRotation();

    // Convert Bullet quaternion to Maya quaternion
    MQuaternion mayaQuat(bulletQuat.getX(), -bulletQuat.getZ(), bulletQuat.getY(), bulletQuat.getW());

    MMatrix mayaMatrix = mayaQuat.asMatrix();

    btVector3 bulletTranslation = bulletTransform.getOrigin();

    // Adjust the translation for Maya's coordinate system (Y-up)
    mayaMatrix[3][0] = bulletTranslation.getX();
    mayaMatrix[3][1] = -bulletTranslation.getZ();  // Inverting Z axis
    mayaMatrix[3][2] = bulletTranslation.getY();  // Swapping Y and Z

    return mayaMatrix;
}