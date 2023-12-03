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
        void updateColliders(std::vector<MFnMesh*> collidersMFnMeshes, MFnMesh* exclude = nullptr);
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
        btBroadphaseInterface* broadphase;
        btDefaultCollisionConfiguration* collisionConfiguration;
        btCollisionDispatcher* dispatcher;
        btSequentialImpulseConstraintSolver* solver;
};

#endif // BULLET_COLLISION_HANDLER_H
