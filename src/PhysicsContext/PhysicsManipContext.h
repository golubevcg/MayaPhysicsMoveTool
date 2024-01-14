#ifndef PHYSICS_MANIP_CONTEXT_H
#define PHYSICS_MANIP_CONTEXT_H

#include "../MayaIncludes.h"
#include "../PhysicsManips/PhysicsMoveManip.h"
#include "../CollisionHandlers/CollisionCandidatesFinder.h"
#include "../CollisionHandlers/BulletCollisionHandler.h"

/**
 * @class PhysicsManipContext
 * @brief Custom context for handling move manipulations in Maya.
 *
 * This class extends MPxSelectionContext to create a custom manipulation
 * context. It is designed to integrate a custom move manipulator with Maya's
 * selection and interaction framework.
 */
class PhysicsManipContext : public MPxSelectionContext {
    public:
        /**
         * @brief Constructor for PhysicsManipContext.
         */
        PhysicsManipContext();

        /**
         * @brief Sets up the tool on activation.
         * @param event The event that triggered the tool setup.
         */
        void toolOnSetup(MEvent& event) override;

        /**
         * @brief Cleans up the tool on deactivation.
         */
        void toolOffCleanup() override;

        /**
         * @brief Callback for selection changes.
         * @param data Custom data passed to the callback.
         */
        static void selectionChanged(void* data);

        /**
         * @brief Initialize Physics world singletons
         */
        static void setupPhysicsWorldSingletons();

        /**
         * @brief Get average position from selected object
         */
        static MVector getAveragePositionFromSelection();
    private:
        std::vector<MObject> selectedObjects; ///< Currently selected objects.
        MCallbackId id; ///< Callback ID for selection change.
};

#endif // PHYSICS_MANIP_CONTEXT_H