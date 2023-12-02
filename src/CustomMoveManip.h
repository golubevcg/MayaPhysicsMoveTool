#ifndef CUSTOM_MOVE_MANIP_H
#define CUSTOM_MOVE_MANIP_H

#include <MayaIncludes.h>
#include <CollisionCandidatesFinder.h>
#include <BulletCollisionHandler.h>

class CustomMoveManip : public MPxManipContainer {
public:
    CustomMoveManip();

    static void* creator();
    static MStatus initialize();
    MStatus createChildren() override;
    MStatus connectToDependNode(const MObject& node) override;
    MStatus doDrag() override;
    MStatus doPress() override;
    void setupCollisions();
    void applyTransformToActiveObjectTransform(const MMatrix matrix);

private:
    void updateManipLocations(const MObject& node);
    CollisionCandidatesFinder& collisionCandidatesFinder;
    BulletCollisionHandler& bulletCollisionHandler;

public:
    static MTypeId id;
    MDagPath fFreePointManipDagPath;
};

#endif // CUSTOM_MOVE_MANIP_H
