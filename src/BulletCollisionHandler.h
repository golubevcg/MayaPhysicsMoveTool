#pragma once

// BulletCollisionHandler.h
#ifndef BULLET_COLLISION_HANDLER_H
#define BULLET_COLLISION_HANDLER_H

#include <vector>

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

        void cleanRigidBody(btRigidBody* body);
        void updateActiveObject(MFnMesh* mesh);
        void updateColliders(std::vector<MFnMesh*> collidersMFnMeshes, MFnMesh* exclude = nullptr);
        btRigidBody* createFullColliderFromMFnMesh(MFnMesh* mfnMesh);
        btRigidBody* createFullActiveRigidBodyFromMFnMesh(MFnMesh* mfnMesh);


        MMatrix getActiveObjectTransformMMatrix();
        bool isRigidBodyInWorld(btRigidBody* body);

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
