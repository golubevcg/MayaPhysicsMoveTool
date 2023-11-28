#pragma once

// BulletCollisionHandler.h
#ifndef BULLET_COLLISION_HANDLER_H
#define BULLET_COLLISION_HANDLER_H

#include <vector>
#include "btBulletDynamicsCommon.h"

class BulletCollisionHandler 
{
    public:
        static BulletCollisionHandler& getInstance() {
            static BulletCollisionHandler instance;
            return instance;
        }

        // Delete copy constructor and copy assignment operator to prevent copies of the singleton
        BulletCollisionHandler(const BulletCollisionHandler&) = delete;
        BulletCollisionHandler& operator=(const BulletCollisionHandler&) = delete;

        void createDynamicsWorld();
        void deleteDynamicsWorld();

        void cleanRigidBody(btRigidBody* body);
        void updateActiveObject(MFnMesh* mesh);
        void updateColliders(std::vector<MFnMesh*> collidersMFnMeshes);
        btRigidBody* createFullColliderFromMFnMesh(MFnMesh* mfnMesh);
        btRigidBody* createFullActiveRigidBodyFromMFnMesh(MFnMesh* mfnMesh);

        MMatrix getActiveObjectTransformMMatrix();

        btCollisionShape* convertMFnMeshToActiveCollisionShape(MFnMesh* mfnMesh);
        btCollisionShape* convertMFnMeshToStaticCollisionShape(MFnMesh * mfnMesh);

        btTransform convertMayaToBulletMatrix(const MMatrix& mayaMatrix);
        MMatrix convertBulletToMayaMatrix(const btTransform& bulletTransform);

        btTransform getBulletTransformFromMFnMeshTransform(MFnMesh* mfnMesh);

        void updateWorld(float framesToUpdate);

        btRigidBody* activeRigidBody;
        btRigidBody* proxyRigidBody;
        std::vector<btRigidBody*> colliders;

        btDiscreteDynamicsWorld* dynamicsWorld;
    private:
        BulletCollisionHandler();
        ~BulletCollisionHandler();

        btBroadphaseInterface* broadphase;
        btDefaultCollisionConfiguration* collisionConfiguration;
        btCollisionDispatcher* dispatcher;
        btSequentialImpulseConstraintSolver* solver;
};

#endif // BULLET_COLLISION_HANDLER_H
