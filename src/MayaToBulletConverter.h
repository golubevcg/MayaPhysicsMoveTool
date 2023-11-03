#pragma once

#ifndef MAYA_TO_BULLET_CONVERTER_H
#define MAYA_TO_BULLET_CONVERTER_H

#include <stdio.h>
#include <vector>
#include <stdlib.h>

#include <MayaIncludes.h>
// Include Bullet Physics headers
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>


class MayaToBulletConverter
{
public:
    MayaToBulletConverter();
    ~MayaToBulletConverter();

    btRigidBody* convertMFnMeshToRigidBody(MFnMesh* mfnMesh);
    btCollisionShape* convertMFnMeshToCollider(MFnMesh* mfnMesh);
};

#endif  //MAYA_TO_BULLET_CONVERTER_H