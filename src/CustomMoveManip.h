#ifndef CUSTOM_MOVE_MANIP_H
#define CUSTOM_MOVE_MANIP_H

#include <MayaIncludes.h>
#include <CollisionCandidatesFinder.h>
#include <BulletCollisionHandler.h>

class CustomMoveManip : public MPxManipContainer {
public:
    CustomMoveManip();
    ~CustomMoveManip() override;


    static void* creator();
    static MStatus initialize();
    MStatus createChildren() override;
    MStatus connectToDependNode(const MObject&) override;
    // Viewport 2.0 rendering
    void drawUI(MHWRender::MUIDrawManager&, const MHWRender::MFrameContext&) const override;
    MStatus doDrag() override;
    MStatus doPress() override;
    bool created = false;

    void applyTransformToActiveObjectTransform(MMatrix matrix);

private:
    void updateManipLocations(const MObject& node);
public:
    MDagPath fFreePointManipDagPath;
    static MTypeId id;

    CollisionCandidatesFinder collisionCandidatesFinder;
    BulletCollisionHandler bulletCollisionHandler;
};


#endif // CUSTOM_MOVE_MANIP_H
