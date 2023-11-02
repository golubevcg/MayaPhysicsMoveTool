#include <stdio.h>
#include <vector>
#include <stdlib.h>

// Maya
#include <maya/MIOStream.h>
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
#include <maya/MItDag.h>
#include <maya/MFnMesh.h>
#include <maya/MBoundingBox.h>
#include <maya/MPointArray.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnFreePointTriadManip.h>
#include <maya/MFnDistanceManip.h>
#include <maya/MFnPlugin.h>

#include <cstdint>

// Boost geometry
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value;


class CustomMoveManip : public MPxManipContainer
{

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
    MStatus addSelectedMFnMesh();
    MStatus getSceneMFnMeshes();
    MStatus initializeRTree();
    std::vector<MObject> checkNearbyObjects();
private:
    void updateManipLocations(const MObject& node);
public:
    MDagPath fFreePointManip;
    MSelectionList selList;
    std::vector<MObject> selectedMFnMeshes;
    std::vector<MObject> selectedTransforms;
    std::vector<MFnMesh*> mFnMeshes;
    bgi::rtree<value, bgi::quadratic<16>> rtree;

    static MTypeId id;
};
MTypeId CustomMoveManip::id(0x8001d);

CustomMoveManip::CustomMoveManip()
{
    // The constructor must not call createChildren for user-defined
    // manipulators.

    this->getSceneMFnMeshes();
    this->initializeRTree();
}

CustomMoveManip::~CustomMoveManip()
{
    for (MFnMesh* mesh : this->mFnMeshes) {
        delete mesh;
    }
}

void* CustomMoveManip::creator()
{
    return new CustomMoveManip();
}

MStatus CustomMoveManip::initialize()
{
    MStatus stat;
    stat = MPxManipContainer::initialize();

    MString message_text = "CustomMoveManip initialized";
    MGlobal::displayInfo(message_text);
    return stat;
}

MStatus CustomMoveManip::createChildren()
{
    MStatus stat = MStatus::kSuccess;
    MPoint startPoint(0.0, 0.0, 0.0);
    MVector direction(0.0, 1.0, 0.0);
    this->fFreePointManip = addFreePointTriadManip("pointManip",
        "freePoint");

    return stat;
}

MStatus CustomMoveManip::addSelectedMFnMesh() {

    MString warningMessage3 = "addSelectedMFnMesh";
    MGlobal::displayWarning(warningMessage3);

    MSelectionList selList;
    MGlobal::getActiveSelectionList(selList);

    this->selectedMFnMeshes.clear();
    this->selectedTransforms.clear();

    if (selList.isEmpty()) {
        MString warningMessage = "No objects selected";
        MGlobal::displayWarning(warningMessage);
        return MS::kFailure;
    }

    MObject node;
    selList.getDependNode(0, node);  // Get the first selected node

    if (node.hasFn(MFn::kTransform)) {
        MFnDagNode dagNode(node);
        bool meshFound = false;

        this->selectedTransforms.push_back(node);

        for (unsigned int i = 0; i < dagNode.childCount(); ++i) {
            MObject child = dagNode.child(i);
            if (child.hasFn(MFn::kMesh)) {
                MFnMesh mesh(child);

                MString infoMessage = "Mesh object found: " + mesh.name();
                MGlobal::displayInfo(infoMessage);

                this->selectedMFnMeshes.push_back(child);
                meshFound = true;
                break;
            }
        }

        if (!meshFound) {
            MString warningMessage = "No mesh found for the selected transform";
            MGlobal::displayWarning(warningMessage);
            return MS::kFailure;
        }
    }
    else {
        MString warningMessage = "Selected object is not a transform";
        MGlobal::displayWarning(warningMessage);
        return MS::kFailure;
    }

    MString warningMessage4 = "Mesh object added!";
    MGlobal::displayWarning(warningMessage4);

    return MS::kSuccess;
}

MStatus CustomMoveManip::getSceneMFnMeshes() {
    
    MStatus status;
    MItDag dagIterator(MItDag::kDepthFirst, MFn::kMesh, &status);

    if (status != MS::kSuccess) {
        MGlobal::displayError("MItDag initialization failed");
        return status;
    }

    for (; !dagIterator.isDone(); dagIterator.next()) {
        MDagPath dagPath;
        status = dagIterator.getPath(dagPath);

        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get MDagPath");
            continue;
        }

        MFnMesh* fnMesh = new MFnMesh(dagPath, &status);
        if (status != MS::kSuccess) {
            MGlobal::displayError("MFnMesh initialization failed");
            continue;
        }

        MBoundingBox boundingBox = fnMesh->boundingBox(&status);
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get bounding box");
            continue;
        }

        this->mFnMeshes.push_back(fnMesh);
    }

    return MS::kSuccess;
}

MStatus CustomMoveManip::initializeRTree() {

    if (this->mFnMeshes.empty()) {
        MGlobal::displayError("MFnMeshes vector is empty");
        return MS::kFailure;
    }

    MStatus status;

    for (size_t i = 0; i < this->mFnMeshes.size(); ++i) {
        MDagPath dagPath;
        status = this->mFnMeshes[i]->getPath(dagPath);
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get MDagPath from MFnMesh");
            return status;
        }

        //exclude selected object

        MBoundingBox mbbox = this->mFnMeshes[i]->boundingBox();
        MMatrix worldMatrix = dagPath.inclusiveMatrix();

        MPoint minPoint = mbbox.min() * worldMatrix;
        MPoint maxPoint = mbbox.max() * worldMatrix;

        box bbox(point(minPoint.x, minPoint.y, minPoint.z), point(maxPoint.x, maxPoint.y, maxPoint.z));
        this->rtree.insert(std::make_pair(bbox, i));  // i is the identifier
    }

    return MS::kSuccess;
}

MStatus CustomMoveManip::connectToDependNode(const MObject& node)
{
    MStatus stat;
    //
    // This routine connects the translate plug to the position plug on the freePoint
    // manipulator.
    //
    MFnDependencyNode nodeFn(node);
    MPlug tPlug = nodeFn.findPlug("translate", true, &stat);
    MFnFreePointTriadManip freePointManipFn(this->fFreePointManip);
    //freePointManipFn.connectToPointPlug(tPlug);
    this->updateManipLocations(node);
    this->finishAddingManips();
    MPxManipContainer::connectToDependNode(node);
    return stat;
}
// Viewport 2.0 manipulator draw overrides

void CustomMoveManip::updateManipLocations(const MObject& node)
//
// Description
//        setTranslation and setRotation to the parent's transformation.
//
{
    MFnFreePointTriadManip manipFn(this->fFreePointManip);

    MDagPath dagPath;
    MFnDagNode(node).getPath(dagPath);

    MObject transformNode = dagPath.transform();
    MFnTransform fnTransform(transformNode);
    MTransformationMatrix originalTM = fnTransform.transformation();

    double rot[3];
    MTransformationMatrix::RotationOrder rOrder;
    originalTM.getRotation(rot, rOrder);

    manipFn.setRotation(rot, rOrder);
    manipFn.setTranslation(originalTM.getTranslation(MSpace::kTransform), MSpace::kTransform);

    MStatus status;
}

MStatus CustomMoveManip::doPress()
{
    MStatus status = MPxManipContainer::doPress();

    return status;
}

MStatus CustomMoveManip::doDrag()
{
    std::vector<MObject> collisionCandidates = this->checkNearbyObjects();

    if (!collisionCandidates.empty()) {
    }

    return MS::kUnknownParameter;
}

std::vector<MObject> CustomMoveManip::checkNearbyObjects() {

    std::vector<MObject> collisionCandidates;
    if (this->selectedMFnMeshes.size() == 0) {
        return collisionCandidates;
    }

    for (const auto& currentMObject : this->selectedMFnMeshes) {
        MStatus status;

        MFnDagNode dagNode(currentMObject);
        MDagPath dagPath;
        MDagPath::getAPathTo(currentMObject, dagPath);
        MFnTransform transform(dagPath.transform(), &status);
        if (status != MStatus::kSuccess) {
            // Handle error
            return collisionCandidates;
        }
        MMatrix worldMatrix = transform.transformationMatrix();

        MBoundingBox boundingBox = dagNode.boundingBox();
        MPoint worldMinPoint = boundingBox.min() * worldMatrix;
        MPoint worldMaxPoint = boundingBox.max() * worldMatrix;

        // Expand the bounding box by a certain distance to find nearby objects
        double distance = 0.1;
        worldMinPoint -= MVector(distance, distance, distance);
        worldMaxPoint += MVector(distance, distance, distance);

        box queryBox(point(worldMinPoint.x, worldMinPoint.y, worldMinPoint.z),
            point(worldMaxPoint.x, worldMaxPoint.y, worldMaxPoint.z));

        std::vector<value> result;
        this->rtree.query(bgi::intersects(queryBox), std::back_inserter(result));

        // collect all collided objects
        for (const auto& item : result) {
            // TODO:CHANGE THIS TO CONDITION CHECK THAT DAG MOBJECT ALREADY IN THE LIST
            if (this->mFnMeshes[item.second]->object() != currentMObject) {  // Exclude the selected mesh itself
                MFnDagNode dagNode1(this->mFnMeshes[item.second]->object());
                MString rtree = "added mobject from RTREE" + MString() + dagNode1.fullPathName() + MString() + worldMinPoint.x + worldMinPoint.y + worldMinPoint.z;
                MGlobal::displayInfo(rtree);
                                                                             
                //MString txt = "The selected object is near another object.";
                //MGlobal::displayInfo(txt);
                collisionCandidates.push_back(this->mFnMeshes[item.second]->object());
                MString txt11 = "Added object to collision candidates";
                MGlobal::displayInfo(txt11);
            }
        }
    }

    return collisionCandidates;
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
    static void selectionChanged(void* data);
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
    selectionChanged(this);
    MStatus status;
    id1 = MModelMessage::addCallback(MModelMessage::kActiveListModified,
        selectionChanged,
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

void CustomMoveManipContext::selectionChanged(void* data)
{
    MStatus stat = MStatus::kSuccess;

    CustomMoveManipContext* ctxPtr = (CustomMoveManipContext*)data;
    ctxPtr->deleteManipulators();
    MSelectionList list;
    stat = MGlobal::getActiveSelectionList(list);
    MItSelectionList iter(list, MFn::kInvalid, &stat);
    if (MS::kSuccess != stat) {
        return;
    }

    MString testText = "Selection changed";
    MGlobal::displayInfo(testText);

    for (; !iter.isDone(); iter.next()) {
        // Make sure the selection list item is a depend node and has the
        // required plugs before manipulating it.
        MObject dependNode;
        iter.getDependNode(dependNode);
        if (dependNode.isNull() || !dependNode.hasFn(MFn::kDependencyNode))
        {
            MGlobal::displayWarning("Object in selection list is not "
                "a depend node.");
            continue;
        }
        MFnDependencyNode dependNodeFn(dependNode);
        MPlug tPlug = dependNodeFn.findPlug("translate", true, &stat);
        if (tPlug.isNull()) {
            MGlobal::displayWarning("Object cannot be manipulated: " +
                dependNodeFn.name());
            continue;
        }
        CustomMoveManip* manipulator = (CustomMoveManip*)CustomMoveManip::newManipulator(manipName, manipObject);
        if (NULL != manipulator) {
            ctxPtr->addManipulator(manipObject);
            // Connect the manipulator to the object in the selection list.
            if (!manipulator->connectToDependNode(dependNode))
            {
                MGlobal::displayWarning("Error connecting manipulator to"
                    " object: " + dependNodeFn.name());
            }

            manipulator->addSelectedMFnMesh();
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