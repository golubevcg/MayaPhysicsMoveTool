#ifndef CUSTOM_MOVE_MANIP_H
#define CUSTOM_MOVE_MANIP_H

#include <MayaIncludes.h>
#include <CollisionCandidatesFinder.h>
#include <BulletCollisionHandler.h>

//
// CustomMoveManip
// This class is a simple context to register our custom command and manipulator.
//
class CustomMoveManip : public MPxManipContainer {
    public:
        CustomMoveManip();
        ~CustomMoveManip() override;

        static void* creator();
        static MStatus initialize();
        MStatus createChildren() override;
        MStatus connectToDependNode(const MObject&) override;
        MStatus doDrag() override;
        MStatus doRelease() override;
        MStatus doPress() override;

        void applyTransformAndRotateToActiveObjectTransform(MMatrix matrix, std::string name);
        void updateManipLocation(const MVector vector);
    public:
        MDagPath fFreePointManipDagPath;
        MDagPath fRotateManipDagPath;
        
        MDagPath xScaleManipDagPath;
        MDagPath yScaleManipDagPath;
        MDagPath zScaleManipDagPath;

        static MTypeId id;
        CollisionCandidatesFinder& collisionCandidatesFinder;
        BulletCollisionHandler& bulletCollisionHandler;
        MPoint currentManipPosition;
};

#endif // CUSTOM_MOVE_MANIP_H