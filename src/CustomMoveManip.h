#ifndef CUSTOM_MOVE_MANIP_H
#define CUSTOM_MOVE_MANIP_H

#include <MayaIncludes.h>
#include <CollisionCandidatesFinder.h>
#include <BulletCollisionHandler.h>

/**
 * @class CustomMoveManip
 * @brief Custom manipulator container class for move operations.
 *
 * This class extends MPxManipContainer to create a custom manipulator for moving objects.
 * It integrates with Maya's manipulation framework and provides functionalities for
 * drag, press, and release events. It also interfaces with collision detection and physics handling.
 */
class CustomMoveManip : public MPxManipContainer {
    public:
        CustomMoveManip();
        ~CustomMoveManip() override;

        // Factory method and initializer
        static void* creator();
        static MStatus initialize();

        // Overridden manipulator methods
        MStatus createChildren() override;
        MStatus connectToDependNode(const MObject&) override;

        /**
         * @brief Do drag method called when manip is moved around the scene.
         * This method updates coresponding bullet3 active rigid body, updates world and reapplies resulted
         * transform to currently selected object in Maya scene.
         * @param event The event that triggered the tool setup.
         */
        MStatus doDrag() override;
        MStatus doPress() override;

        // Custom methods for manipulating object transforms
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
