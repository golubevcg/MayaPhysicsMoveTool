#ifndef CUSTOM_MOVE_MANIP_CONTEXT_H
#define CUSTOM_MOVE_MANIP_CONTEXT_H

#include <MayaIncludes.h>
#include <CustomMoveManip.h>
#include <CollisionCandidatesFinder.h>
#include <BulletCollisionHandler.h>


//
// MoveManipContext
//
// This class is a simple context for supporting a move manipulator.
//

class CustomMoveManipContext : public MPxSelectionContext {
    public:
        CustomMoveManipContext();
        void toolOnSetup(MEvent& event) override;
        void toolOffCleanup() override;
        static void selectionChanged(void* data);

    private:
        std::vector<MObject> selectedObjects;
        MCallbackId id;
};

#endif // CUSTOM_MOVE_MANIP_CONTEXT_H
