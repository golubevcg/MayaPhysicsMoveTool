#include <maya/MIOStream.h>
#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <maya/MFn.h>
#include <maya/MPxNode.h>
#include <maya/MPxManipContainer.h>
#include <maya/MPxSelectionContext.h>
#include <maya/MPxContextCommand.h>
#include <maya/MModelMessage.h>
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MItSelectionList.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MDagPath.h>
#include <maya/MManipData.h>
#include <maya/MMatrix.h>
// Manipulators
#include <maya/MFnFreePointTriadManip.h>
#include <maya/MFnDistanceManip.h>
// Boost geometry
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


class CustomMoveManip : public MPxManipContainer
{

public:
    CustomMoveManip();
    ~CustomMoveManip() override;

    static void* creator();
    static MStatus initialize();
    MStatus createChildren() override;
    MStatus connectToDependNode(const MObject& node) override;
    // Viewport 2.0 rendering
    void drawUI(MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext) const override;
    MStatus doDrag() override;
    MStatus doPress() override;
    void addSelectedMObjects(std::vector<MObject> mObjectsVector);
private:
    void updateManipLocations(const MObject& node);
public:
    MDagPath fFreePointManip;
    MSelectionList selList;
    std::vector<MObject> selectedObjects;
    static MTypeId id;
};
MTypeId CustomMoveManip::id(0x8001d);

CustomMoveManip::CustomMoveManip()
{
    // The constructor must not call createChildren for user-defined
    // manipulators.
}

CustomMoveManip::~CustomMoveManip()
{
}

void* CustomMoveManip::creator()
{
    return new CustomMoveManip();
}
MStatus CustomMoveManip::initialize()
{
    MStatus stat;
    stat = MPxManipContainer::initialize();
    return stat;
}

MStatus CustomMoveManip::createChildren()
{
    MStatus stat = MStatus::kSuccess;
    MPoint startPoint(0.0, 0.0, 0.0);
    MVector direction(0.0, 1.0, 0.0);
    fFreePointManip = addFreePointTriadManip("pointManip",
        "freePoint");
    return stat;
}

void CustomMoveManip::updateManipLocations(const MObject& node)
//
// Description
//        setTranslation and setRotation to the parent's transformation.
//
{
    MFnDagNode dagNodeFn(node);
    MDagPath nodePath;
    dagNodeFn.getPath(nodePath);
    MFnFreePointTriadManip manipFn(fFreePointManip);
    MTransformationMatrix m(nodePath.exclusiveMatrix());
    double rot[3];
    MTransformationMatrix::RotationOrder rOrder;
    m.getRotation(rot, rOrder);
    manipFn.setRotation(rot, rOrder);
    MVector trans = m.getTranslation(MSpace::kWorld);
    manipFn.setTranslation(trans, MSpace::kWorld);
    MStatus status;

    CustomMoveManip::addSelectedMObjects(selectedObjects);

    

}

void CustomMoveManip::addSelectedMObjects(std::vector<MObject> mObjectsVector) {

    MGlobal::getActiveSelectionList(selList);
    MItSelectionList iter(selList);
    if (iter.isDone()) {
        MString warningMessage = "No objects selected";
        MGlobal::displayWarning(warningMessage);
    }
    else {
        for (; !iter.isDone(); iter.next()) {
            MObject node;
            iter.getDependNode(node);
            mObjectsVector.push_back(node);
        }
    }
}

MStatus CustomMoveManip::connectToDependNode(const MObject& node)
{
    MStatus stat;
    //
    // This routine connects the distance manip to the scaleY plug on the node
    // and connects the translate plug to the position plug on the freePoint
    // manipulator.
    //
    MFnDependencyNode nodeFn(node);
    MPlug tPlug = nodeFn.findPlug("translate", true, &stat);
    MFnFreePointTriadManip freePointManipFn(fFreePointManip);
    freePointManipFn.connectToPointPlug(tPlug);
    updateManipLocations(node);
    finishAddingManips();
    MPxManipContainer::connectToDependNode(node);
    return stat;
}
// Viewport 2.0 manipulator draw overrides

MStatus CustomMoveManip::doPress()
{
    MStatus status = MPxManipContainer::doPress();

    // Your custom code here
    MGlobal::displayInfo("Mouse pressed, object selected");

    return status;
}

MStatus CustomMoveManip::doDrag()
{
    // Call the parent class's doDrag function
    // Add any additional functionality here if needed

    MStatus status;
    MString txt = "Moving Objects:";
    
    MGlobal::displayInfo(txt);

    return MS::kUnknownParameter;
}

void CustomMoveManip::drawUI(MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext) const
{
    drawManager.beginDrawable();
    drawManager.setColor(MColor(0.0f, 1.0f, 0.1f));
    drawManager.text(MPoint(0, 0), "Stretch Me!", MHWRender::MUIDrawManager::kLeft);

    drawManager.endDrawable();
}

//
// MoveManipContext
//
// This class is a simple context for supporting a move manipulator.
//
class CustomMoveManipContext : public MPxSelectionContext
{
public:
    CustomMoveManipContext();
    void    toolOnSetup(MEvent& event) override;
    void    toolOffCleanup() override;
    // Callback issued when selection list changes
    static void updateManipulators(void* data);
private:
    MCallbackId id1;
};

CustomMoveManipContext::CustomMoveManipContext()
{
    MString str("Plugin move Manipulator");
    setTitleString(str);
}

void CustomMoveManipContext::toolOnSetup(MEvent&)
{
    MString str("Move the object using the manipulator");
    setHelpString(str);
    updateManipulators(this);
    MStatus status;
    id1 = MModelMessage::addCallback(MModelMessage::kActiveListModified,
        updateManipulators,
        this, &status);
    if (!status) {
        MGlobal::displayError("Model addCallback failed");
    }
}

void CustomMoveManipContext::toolOffCleanup()
{
    MStatus status;
    status = MModelMessage::removeCallback(id1);
    if (!status) {
        MGlobal::displayError("Model remove callback failed");
    }
    MPxContext::toolOffCleanup();
}

void CustomMoveManipContext::updateManipulators(void* data)
{
    MStatus stat = MStatus::kSuccess;

    CustomMoveManipContext* ctxPtr = (CustomMoveManipContext*)data;
    ctxPtr->deleteManipulators();
    MSelectionList list;
    stat = MGlobal::getActiveSelectionList(list);
    MItSelectionList iter(list, MFn::kInvalid, &stat);
    if (MS::kSuccess == stat) {
        for (; !iter.isDone(); iter.next()) {
            // Make sure the selection list item is a depend node and has the
            // required plugs before manipulating it.
            //
            MObject dependNode;
            iter.getDependNode(dependNode);
            if (dependNode.isNull() || !dependNode.hasFn(MFn::kDependencyNode))
            {
                MGlobal::displayWarning("Object in selection list is not "
                    "a depend node.");
                continue;
            }
            MFnDependencyNode dependNodeFn(dependNode);
            MPlug rPlug = dependNodeFn.findPlug("translate", true, &stat);
            MPlug sPlug = dependNodeFn.findPlug("scaleY", true, &stat);
            if (rPlug.isNull() || sPlug.isNull()) {
                MGlobal::displayWarning("Object cannot be manipulated: " +
                    dependNodeFn.name());
                continue;
            }
            // Add manipulator to the selected object
            //
            MString manipName("customMoveManip");
            MObject manipObject;
            CustomMoveManip* manipulator =
                (CustomMoveManip*)CustomMoveManip::newManipulator(manipName, manipObject);
            if (NULL != manipulator) {
                // Add the manipulator
                //
                ctxPtr->addManipulator(manipObject);
                // Connect the manipulator to the object in the selection list.
                //
                if (!manipulator->connectToDependNode(dependNode))
                {
                    MGlobal::displayWarning("Error connecting manipulator to"
                        " object: " + dependNodeFn.name());
                }
            }
        }
    }
}

//
// moveManipContext
//
// This is the command that will be used to create instances
// of our context.
//
class CustoMoveManipContext : public MPxContextCommand
{
public:
    CustoMoveManipContext() {};
    MPxContext* makeObj() override;
public:
    static void* creator();
};

MPxContext* CustoMoveManipContext::makeObj()
{
    return new CustomMoveManipContext();
}

void* CustoMoveManipContext::creator()
{
    return new CustoMoveManipContext;
}

//
// The following routines are used to register/unregister
// the context and manipulator
//
MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, PLUGIN_COMPANY, "1.0", "Andrew Golubev");
    status = plugin.registerContextCommand("customMoveManipContext",
        &CustoMoveManipContext::creator);
    if (!status) {
        MGlobal::displayError("Error registering customMoveManipContext command");
        return status;
    }
    status = plugin.registerNode("customMoveManip", CustomMoveManip::id,
        &CustomMoveManip::creator, &CustomMoveManip::initialize,
        MPxNode::kManipContainer);
    if (!status) {
        MGlobal::displayError("Error registering customMoveManip node");
        return status;
    }
    return status;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj);
    status = plugin.deregisterContextCommand("customMoveManipContext");
    if (!status) {
        MGlobal::displayError("Error deregistering customMoveManipContext command");
        return status;
    }
    status = plugin.deregisterNode(CustomMoveManip::id);
    if (!status) {
        MGlobal::displayError("Error deregistering customMoveManip node");
        return status;
    }
    return status;
}