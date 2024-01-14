#pragma once

// BulletCollisionHandler.h
// Manages Bullet physics dynamics world and object collisions.

#ifndef BULLET_COLLISION_HANDLER_H
#define BULLET_COLLISION_HANDLER_H

#include <vector>
#include <unordered_map>
#include <btBulletDynamicsCommon.h>

/**
 * @class BulletCollisionHandler
 * @brief Manages the Bullet physics engine's dynamics world and collision handling.
 *
 * This class is responsible for initializing and managing the Bullet physics dynamics world,
 * creating and deleting rigid bodies, and handling collisions. It's implemented as a singleton
 * to ensure there is only one instance of the dynamics world.
 */
class BulletCollisionHandler {
    public:
        // Singleton access
        static BulletCollisionHandler& getInstance();

        // Prevent copying and assignment
        BulletCollisionHandler(const BulletCollisionHandler&) = delete;
        BulletCollisionHandler& operator=(const BulletCollisionHandler&) = delete;

        // Dynamics world management
        void createDynamicsWorld();
        void deleteDynamicsWorld();

        // Rigid body management
        void cleanRigidBodies(std::unordered_map<std::string, btRigidBody*> rigidBodies);
        void updateActiveObjects(std::unordered_map<std::string, MFnMesh*> MFnMeshes);
        void updateColliders(std::vector<MFnMesh*> collidersMFnMeshes, std::unordered_map<std::string, MFnMesh*> excludeMeshes);

        // Collider management
        void clearColliders();
        void deleteCollider(btRigidBody* collider);

        // Utility functions
        btRigidBody* createFullColliderFromMFnMesh(MFnMesh* mfnMesh);
        btRigidBody* createFullActiveRigidBodyFromMFnMesh(MFnMesh* mfnMesh);
        MMatrix getActiveObjectTransformMMatrix(std::string meshName);
        bool isRigidBodyInWorld(btRigidBody* body);
        btCollisionShape* convertMFnMeshToActiveCollisionShape(MFnMesh* mfnMesh);
        btCollisionShape* convertMFnMeshToStaticCollisionShape(MFnMesh* mfnMesh);
        btTransform convertMayaToBulletMatrix(const MMatrix& mayaMatrix);
        MMatrix convertBulletToMayaMatrix(const btTransform& bulletTransform);
        btTransform getBulletTransformFromMFnMeshTransform(MFnMesh* mfnMesh);

        // Simulation management
        void updateWorld(float framesToUpdate);
        void stopVelocitiesInWorld();

        // Public members
        std::unordered_map<std::string, btRigidBody*> activeRigidBodies;
        std::unordered_map<std::string, btRigidBody*> colliders;
        btDiscreteDynamicsWorld* dynamicsWorld;

    private:
        // Constructor and destructor
        BulletCollisionHandler();
        ~BulletCollisionHandler();

        // Singleton initialization
        static void initSingleton();
        static BulletCollisionHandler* instance;
        static std::once_flag initInstanceFlag;

        // Bullet physics components
        btBroadphaseInterface* broadphase;
        btDefaultCollisionConfiguration* collisionConfiguration;
        btCollisionDispatcher* dispatcher;
        btSequentialImpulseConstraintSolver* solver;
};

#endif // BULLET_COLLISION_HANDLER_H
