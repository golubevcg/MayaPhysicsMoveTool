// MayaToBulletUtils.cpp

#include <string>
#include <MayaIncludes.h>
#include "BulletCollisionHandler.h"

#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <btBulletDynamicsCommon.h>

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

void BulletCollisionHandler::updateWorld(float framesToUpdate) {
    float fps = 24.0f; // This is your simulation's fps

    // Calculate deltaTime assuming each frame is 1/fps seconds long
    float deltaTime = framesToUpdate / fps;

    int maxSubSteps = 10;
    float fixedTimeStep = 1.f / fps; // This is the time each physics "step" represents at 24fps

    // Update the dynamics world by the deltaTime
    this->dynamicsWorld->stepSimulation(deltaTime, maxSubSteps, fixedTimeStep);
}

void BulletCollisionHandler::updateActiveObject(MFnMesh* mesh)
{
    this->cleanRigidBody(this->activeRigidBody);
    this->activeRigidBody = this->createFullActiveRigidBodyFromMFnMesh(mesh);
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
    this->proxyRigidBody->setCollisionFlags(this->proxyRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    this->proxyRigidBody->setActivationState(DISABLE_DEACTIVATION);

    // Add the body to the world
    this->dynamicsWorld->addRigidBody(this->proxyRigidBody, 1, 1);
}

void BulletCollisionHandler::setProxyObjectPosition(float x, float y, float z) {
    // Check if the proxy body exists
    if (this->proxyRigidBody) {
        // Create a new position vector from the input floats
        btVector3 newPosition(x, y, z);

        // Get the current transform
        btTransform currentTransform = this->proxyRigidBody->getWorldTransform();
        currentTransform.setOrigin(newPosition);

        // Update the transform of the proxy body
        this->proxyRigidBody->setWorldTransform(currentTransform);

        // For kinematic objects, you also need to update the motion state
        btMotionState* motionState = this->proxyRigidBody->getMotionState();
        if (motionState) {
            motionState->setWorldTransform(currentTransform);
        }
    }
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
        this->dynamicsWorld->addRigidBody(rigidBody, 1, 1);
        // Keep a reference to the collider for later removal or other operations
        this->colliders.push_back(rigidBody);
    }
}

btRigidBody* BulletCollisionHandler::createFullColliderFromMFnMesh(MFnMesh* mfnMesh) {
    // Convert MFnMesh to Bullet Collision Shape
    btCollisionShape* newShape = this->convertMFnMeshToCollisionShape(mfnMesh);

    // Convert Maya's MMatrix to Bullet's btTransform
    btTransform bulletTransform = this->getBulletTransformFromMFnMeshTransform(mfnMesh);

    // Create a rigid body with a mass of 0 for a static object
    btDefaultMotionState* motionState = new btDefaultMotionState(bulletTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0, motionState, newShape, btVector3(0, 0, 0));
    btRigidBody* rigidBody = new btRigidBody(rbInfo);
    
    //DEBUG FUNCTION CALL REMOVE LATER
    //this->createMayaMeshFromBulletRigidBody(rigidBody);

    return rigidBody;
}

btRigidBody* BulletCollisionHandler::createFullActiveRigidBodyFromMFnMesh(MFnMesh* mfnMesh) {
    btCollisionShape* collisionShape = this->convertMFnMeshToCollisionShape(mfnMesh);

    // Convert Maya's transformation matrix to Bullet's btTransform
    btTransform bulletTransform = this->getBulletTransformFromMFnMeshTransform(mfnMesh);

    // Define the mass of the rigid body
    float mass = 10;
    btVector3 localInertia(0, 0, 0);
    collisionShape->calculateLocalInertia(mass, localInertia);

    // Create the rigid body with the Bullet transform and collision shape
    btDefaultMotionState* motionState = new btDefaultMotionState(bulletTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, collisionShape, localInertia);
    btRigidBody* rigidBody = new btRigidBody(rbInfo);
    rigidBody->setActivationState(DISABLE_DEACTIVATION); // Keep the body always active

    // Add rigid body to the dynamics world and set up proxy object
    this->dynamicsWorld->addRigidBody(rigidBody, 1, 1);
    MString info_msg = "BulletCollisionHandler: Active object updated and added to the dynamicsWorld.";
    MGlobal::displayInfo(info_msg);

    this->updateActiveObjectProxy(rigidBody->getWorldTransform());
    this->constrainBodies(rigidBody, this->proxyRigidBody);

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
    MFnTransform mfnTransform(dagPath);
    MTransformationMatrix mTransMatrix = mfnTransform.transformation();

    // Convert Maya's transformation matrix to Bullet's btTransform
    btTransform bulletTransform = convertMayaToBulletMatrix(mTransMatrix.asMatrix());

    return bulletTransform;
}

btCollisionShape* BulletCollisionHandler::convertMFnMeshToCollisionShape(MFnMesh* mfnMesh) {
    // Get the points in local space
    MPointArray mayaVertices;
    mfnMesh->getPoints(mayaVertices, MSpace::kObject);

    // Create the Bullet triangle mesh
    btTriangleMesh* triMesh = new btTriangleMesh();

    // Get triangles from the mesh
    MIntArray triangleCounts, triangleVertices;
    mfnMesh->getTriangles(triangleCounts, triangleVertices);

    // Index variables for triangleVertices
    int triangleIndex = 0;

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

            //MGlobal::displayInfo(MString("vertex pos of collider ") + vertices->getX() + ", " + vertices->getY() + ", " + vertices->getZ());

            // Move to the next set of vertices
            triangleIndex += 3;
        }
    }

    // Create the mesh shape
    bool useQuantizedAABBCompression = true;
    btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape(triMesh, useQuantizedAABBCompression);

    return meshShape;
}

MObject BulletCollisionHandler::createMayaMeshFromBulletRigidBody(btRigidBody* rigidBody) {
    if (!rigidBody) {
        // Handle the case where rigidBody is null
        return MObject::kNullObj;
    }

    btCollisionShape* collisionShape = rigidBody->getCollisionShape();
    btBvhTriangleMeshShape* meshShape = dynamic_cast<btBvhTriangleMeshShape*>(collisionShape);

    if (!meshShape) {
        // Handle the case where the cast fails
        return MObject::kNullObj;
    }

    const btStridingMeshInterface* meshInterface = meshShape->getMeshInterface();

    MFloatPointArray points;
    MIntArray polygonCounts;
    MIntArray polygonConnects;

    const btVector3* bulletVertices;
    const unsigned char* vertexbase;
    int numverts;
    PHY_ScalarType type;
    int stride;
    const unsigned char* indexbase;
    int indexstride;
    int numfaces;
    PHY_ScalarType indicestype;

    // Accessing vertex and index data from the Bullet mesh
    for (int partId = 0; partId < meshInterface->getNumSubParts(); partId++) {
        meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numfaces, indicestype, partId);
        bulletVertices = reinterpret_cast<const btVector3*>(vertexbase);

        for (int i = 0; i < numfaces; i++) {
            // Assuming the mesh is made of triangles
            int* indexptr = reinterpret_cast<int*>(const_cast<unsigned char*>(indexbase) + i * indexstride);
            for (int j = 0; j < 3; j++) {
                const btVector3& vertex = bulletVertices[*indexptr];
                // Conversion: Bullet (X, Y, Z) -> Maya (X, Z, -Y)
                points.append(MFloatPoint(vertex.x(), vertex.z(), -vertex.y()));
                polygonConnects.append(points.length() - 1);
                indexptr++;
            }
            polygonCounts.append(3); // As it's a triangle
        }
    }

    // Create Maya mesh
    MFnMesh meshFn;
    MObject newMesh = meshFn.create(points.length(), polygonCounts.length(), points, polygonCounts, polygonConnects, MObject::kNullObj);

    // Extract Bullet's world transform and convert it to Maya's transform
    btTransform bulletTransform = rigidBody->getWorldTransform();
    MMatrix mayaMatrix = this->convertBulletToMayaMatrix(bulletTransform);

    // Apply the Maya transform to the created mesh
    MDagPath dagPath;
    MFnDagNode fnDagNode(newMesh);
    fnDagNode.getPath(dagPath);
    MFnTransform fnTransform(dagPath.transform());
    fnTransform.set(MTransformationMatrix(mayaMatrix));

    return newMesh;
}

btTransform BulletCollisionHandler::convertMayaToBulletMatrix(const MMatrix& mayaMatrix) {
    // Create a conversion matrix that flips the Z-axis
    btMatrix3x3 conversionMatrix(
        1.0, 0.0, 0.0,  // First row
        0.0, 1.0, 0.0,  // Second row
        0.0, 0.0, -1.0  // Third row - flipping Z-axis
    );

    // Convert Maya matrix to Bullet format
    btMatrix3x3 bulletRotation(
        mayaMatrix[0][0], mayaMatrix[0][1], mayaMatrix[0][2],
        mayaMatrix[1][0], mayaMatrix[1][1], mayaMatrix[1][2],
        mayaMatrix[2][0], mayaMatrix[2][1], mayaMatrix[2][2]
    );

    // Apply the conversion matrix to the rotation
    bulletRotation = conversionMatrix * bulletRotation;

    // Apply the conversion to the translation, flipping the Z-axis
    btVector3 bulletTranslation(mayaMatrix[3][0], mayaMatrix[3][1], -mayaMatrix[3][2]);

    // Create a Bullet transform
    btTransform bulletTransform;
    bulletTransform.setBasis(bulletRotation);
    bulletTransform.setOrigin(bulletTranslation);

    return bulletTransform;
}

MMatrix BulletCollisionHandler::convertBulletToMayaMatrix(const btTransform& bulletTransform) {
    // Create a reverse conversion matrix (e.g., flipping the Z-axis back)
    btMatrix3x3 reverseConversionMatrix(
        1.0, 0.0, 0.0,  // First row
        0.0, 1.0, 0.0,  // Second row
        0.0, 0.0, -1.0  // Third row - flipping Z-axis back
    );

    // Extract rotation and translation from Bullet transform
    btMatrix3x3 bulletRotation = bulletTransform.getBasis();
    btVector3 bulletTranslation = bulletTransform.getOrigin();

    // Apply the reverse conversion matrix to the rotation
    bulletRotation = reverseConversionMatrix * bulletRotation;

    // Adjust the translation (e.g., flipping the Z-axis back)
    MVector mayaTranslation(bulletTranslation.getX(), -bulletTranslation.getZ(), bulletTranslation.getY());

    // Convert to MMatrix and set translation
    MMatrix mayaMatrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mayaMatrix[i][j] = bulletRotation[i][j];
        }
    }
    mayaMatrix[3][0] = mayaTranslation.x;
    mayaMatrix[3][1] = mayaTranslation.y;
    mayaMatrix[3][2] = mayaTranslation.z;
    mayaMatrix[3][3] = 1.0;

    return mayaMatrix;
}
