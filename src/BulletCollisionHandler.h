// BulletCollisionHandler.h
#ifndef BULLET_COLLISION_HANDLER_H
#define BULLET_COLLISION_HANDLER_H

#include "btBulletDynamicsCommon.h"

class BulletCollisionHandler 
{
    public:
        BulletCollisionHandler();
        ~BulletCollisionHandler();

        void createDynamicsWorld();
        void deleteDynamicsWorld();

    private:
        btBroadphaseInterface* broadphase;
        btDefaultCollisionConfiguration* collisionConfiguration;
        btCollisionDispatcher* dispatcher;
        btSequentialImpulseConstraintSolver* solver;
        btDiscreteDynamicsWorld* dynamicsWorld;
};

#endif // BULLET_COLLISION_HANDLER_H
