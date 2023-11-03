#pragma once

// BulletCollisionHandler.h
#ifndef BULLET_COLLISION_HANDLER_H
#define BULLET_COLLISION_HANDLER_H

#include <vector>

#include "btBulletDynamicsCommon.h"

class BulletCollisionHandler 
{
    public:
        BulletCollisionHandler();
        ~BulletCollisionHandler();

        void createDynamicsWorld();
        void deleteDynamicsWorld();
        void updateActiveObject(MFnMesh* mesh);
        void updateColliders(std::vector<MFnMesh*> collidersMFnMeshes);
        btRigidBody* convertMFnMeshToRigidBody(MFnMesh* mfnMesh);
        btCollisionShape* convertMFnMeshToCollider(MFnMesh * mfnMesh);

        btRigidBody* activeRigidBody;
        std::vector<btCollisionShape*> colliders;
    private:
        btBroadphaseInterface* broadphase;
        btDefaultCollisionConfiguration* collisionConfiguration;
        btCollisionDispatcher* dispatcher;
        btSequentialImpulseConstraintSolver* solver;
        btDiscreteDynamicsWorld* dynamicsWorld;
};

#endif // BULLET_COLLISION_HANDLER_H
