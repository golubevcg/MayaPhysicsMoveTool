#pragma once

// BulletCollisionHandler.h
#ifndef BULLET_COLLISION_HANDLER_H
#define BULLET_COLLISION_HANDLER_H

#include <vector>
#include <unordered_map>
#include "btBulletDynamicsCommon.h"

class BulletCollisionHandler 
{
    public:
        static BulletCollisionHandler& getInstance();

        // Delete copy constructor and copy assignment operator to prevent copies of the singleton
        BulletCollisionHandler(const BulletCollisionHandler&) = delete;
        BulletCollisionHandler& operator=(const BulletCollisionHandler&) = delete;

        void createDynamicsWorld();
        void deleteDynamicsWorld();

        void cleanRigidBodies(std::unordered_map<std::string, btRigidBody*> rigidBodies);
        void updateActiveObjects(std::unordered_map<std::string, MFnMesh*> MFnMeshes);
        void updateColliders(std::vector<MFnMesh*> collidersMFnMeshes, std::unordered_map<std::string, MFnMesh*> excludeMeshes);

        void clearColliders();
        void deleteCollider(btRigidBody* collider);

        btRigidBody* createFullColliderFromMFnMesh(MFnMesh* mfnMesh);
        btRigidBody* createFullActiveRigidBodyFromMFnMesh(MFnMesh* mfnMesh);

        MMatrix getActiveObjectTransformMMatrix(std::string meshName);
        bool isRigidBodyInWorld(btRigidBody* body);

        btCollisionShape* convertMFnMeshToActiveCollisionShape(MFnMesh* mfnMesh);
        btCollisionShape* convertMFnMeshToStaticCollisionShape(MFnMesh * mfnMesh);

        btTransform convertMayaToBulletMatrix(const MMatrix& mayaMatrix);
        MMatrix convertBulletToMayaMatrix(const btTransform& bulletTransform);

        btTransform getBulletTransformFromMFnMeshTransform(MFnMesh* mfnMesh);

        void updateWorld(float framesToUpdate);

        std::unordered_map<std::string, btRigidBody*> activeRigidBodies;
        std::unordered_map<std::string, btRigidBody*> colliders;
        btDiscreteDynamicsWorld* dynamicsWorld;

    private:
        BulletCollisionHandler();  // Constructor is private
        ~BulletCollisionHandler();  // Constructor is private
        static void initSingleton();
        static BulletCollisionHandler* instance;
        static std::once_flag initInstanceFlag;

        btBroadphaseInterface* broadphase;
        btDefaultCollisionConfiguration* collisionConfiguration;
        btCollisionDispatcher* dispatcher;
        btSequentialImpulseConstraintSolver* solver;
};

#endif // BULLET_COLLISION_HANDLER_H
