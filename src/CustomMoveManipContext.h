#ifndef CUSTOM_MOVE_MANIP_CONTEXT_H
#define CUSTOM_MOVE_MANIP_CONTEXT_H

#include <MayaIncludes.h>
#include <CustomMoveManip.h>
#include <CollisionCandidatesFinder.h>
#include <BulletCollisionHandler.h>

/**
 * @class CustomMoveManipContext
 * @brief Custom context for handling move manipulations in Maya.
 *
 * This class extends MPxSelectionContext to create a custom manipulation
 * context. It is designed to integrate a custom move manipulator with Maya's
 * selection and interaction framework.
 */
class CustomMoveManipContext : public MPxSelectionContext {
    public:
        /**
         * @brief Constructor for CustomMoveManipContext.
         */
        CustomMoveManipContext();

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
         * @brief Initialize Dynamic world singletons
         */
        static void setupDynamicWorldSingletons();

        /**
         * @brief Get average position from selected object
         */
        static MVector getAveragePositionFromSelection();
    private:
        std::vector<MObject> selectedObjects; ///< Currently selected objects.
        MCallbackId id; ///< Callback ID for selection change.
};

#endif // CUSTOM_MOVE_MANIP_CONTEXT_H