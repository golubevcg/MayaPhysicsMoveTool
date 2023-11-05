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

        void cleanRigidBody(btRigidBody* body);
        void updateActiveObject(MFnMesh* mesh);
        void updateActiveObjectProxy(const btTransform& startTransform);
        void constrainBodies(btRigidBody* mainBody, btRigidBody* proxyBody);
        void updateColliders(std::vector<MFnMesh*> collidersMFnMeshes);

        void setProxyObjectPosition(float x, float y, float z);
        MMatrix getProxyObjectTransformMMatrix();

        btRigidBody* convertMFnMeshToRigidBody(MFnMesh* mfnMesh);
        btCollisionShape* convertMFnMeshToCollider(MFnMesh * mfnMesh);

        void updateWorld(float framesToUpdate);

        btRigidBody* activeRigidBody;
        btRigidBody* proxyRigidBody;
        std::vector<btRigidBody*> colliders;
    private:
        btBroadphaseInterface* broadphase;
        btDefaultCollisionConfiguration* collisionConfiguration;
        btCollisionDispatcher* dispatcher;
        btSequentialImpulseConstraintSolver* solver;
        btDiscreteDynamicsWorld* dynamicsWorld;
};

#endif // BULLET_COLLISION_HANDLER_H
