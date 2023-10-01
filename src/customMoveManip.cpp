//-
// ==========================================================================
// Copyright 1995,2006,2008 Autodesk, Inc. All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+
/*
    This example demonstrates the FreePointTriad and Distance manipulators in
    the API.  This example uses three classes to accomplish this task: First,
    a context command (moveManipContext) is provided to create instances of
    the context.  Next, a custom selection context (MoveManipContext) is
    created to manage the rotation manipulator.   Finally, the rotation
    manipulator is provided as a custom node class.
    Loading and unloading:
    ----------------------
    The move manipulator context and tool button can be created with the
    following mel commands:
        customMoveManipContext;
        setParent Shelf1;
        toolButton  -cl toolCluster
                    -t customMoveManipContext1
                    -i1 "moveToolManip.xpm"
                    customMoveManip;
    If the preceding commands were used to create the manipulator context,
    the following commands can destroy it:
        deleteUI moveManipContext1;
        deleteUI moveManip;
    If the plugin is loaded and unloaded frequently (eg. during testing),
    it is useful to make these command sequences into shelf buttons.
    How to use:
    -----------
    Once the tool button has been created using the script above, select the
    tool button then click on an object.  The move manipulator should appear
    at the center of the selected object and a distance manipulator should
    appear at the origin.  Use the move manipulator to move the object, and
    the distance manipulator to control the scaling in Y direction.

*/
#include <maya/MIOStream.h>
#include <stdio.h>
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
private:
    void updateManipLocations(const MObject& node);
public:
    MDagPath fDistanceManip;
    MDagPath fFreePointManip;
    MSelectionList selList;
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
    fDistanceManip = addDistanceManip("distanceManip",
        "distance");
    // The distance manip will extend in the y-direction.
    //
    MFnDistanceManip distanceManipFn(fDistanceManip);
    MPoint startPoint(0.0, 0.0, 0.0);
    MVector direction(0.0, 1.0, 0.0);
    distanceManipFn.setStartPoint(startPoint);
    distanceManipFn.setDirection(direction);
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
    MPlug syPlug = nodeFn.findPlug("scaleY", true, &stat);
    MPlug tPlug = nodeFn.findPlug("translate", true, &stat);
    MFnDistanceManip distanceManipFn(fDistanceManip);
    distanceManipFn.connectToDistancePlug(syPlug);
    MFnFreePointTriadManip freePointManipFn(fFreePointManip);
    freePointManipFn.connectToPointPlug(tPlug);
    updateManipLocations(node);
    finishAddingManips();
    MPxManipContainer::connectToDependNode(node);
    return stat;
}
// Viewport 2.0 manipulator draw overrides

MStatus CustomMoveManip::doDrag()
{
    // Call the parent class's doDrag function
    // Add any additional functionality here if needed

    MStatus status;
    MString txt = "Moving Objects:";
    
    MGlobal::getActiveSelectionList(selList);
    MItSelectionList iter(selList);

    if (iter.isDone()) {
        txt += "No objects selected";
    }
    else {
        for (; !iter.isDone(); iter.next()) {
            txt += 1;
            MGlobal::displayInfo(txt);

            MObject node;

            txt += 1;
            MGlobal::displayInfo(txt);
            iter.getDependNode(node);
            
            txt += 1;
            MGlobal::displayInfo(txt);
            MFnDependencyNode depNodeFn;
            
            txt += 1;
            MGlobal::displayInfo(txt);
            depNodeFn.setObject(node);

            txt += depNodeFn.name() + " ";
            MGlobal::displayInfo(txt);
        }
    }

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