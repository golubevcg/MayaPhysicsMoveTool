#include <CustomMoveManipContext.h>


CustomMoveManipContext::CustomMoveManipContext()
     :id1(0), 
      bulletCollisionHandler(BulletCollisionHandler::getInstance()),
      collisionCandidatesFinder(CollisionCandidatesFinder::getInstance()) {
    MString str("Plugin move Manipulator");
    setTitleString(str);
    MGlobal::displayWarning("---CustomMoveManipContext CREATOR");
}

void CustomMoveManipContext::toolOnSetup(MEvent&) {
    MString str("Move the object using the manipulator");
    setHelpString(str);
    //selectionChanged(this);
    MGlobal::displayWarning("---CustomMoveManipContext toolOnSetup");
    MStatus status;
    id1 = MModelMessage::addCallback(MModelMessage::kActiveListModified,
        selectionChanged,
        this, &status);
    if (!status) {
        MGlobal::displayError("Model addCallback failed");
    }
}

void CustomMoveManipContext::toolOffCleanup() {
    MStatus status;
    status = MModelMessage::removeCallback(id1);
    if (!status) {
        MGlobal::displayError("Model remove callback failed");
    }

    MPxContext::toolOffCleanup();
}

void CustomMoveManipContext::selectionChanged(void* data) {
    MStatus stat = MStatus::kSuccess;
    MGlobal::displayWarning("---CustomMoveManipContext selectionChanged");

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

        MString manipName("customMoveManip");
        MObject manipObject;
        CustomMoveManip* manipulator = (CustomMoveManip*)CustomMoveManip::newManipulator(manipName, manipObject);
        if (NULL != manipulator) {
            ctxPtr->addManipulator(manipObject);
            // Connect the manipulator to the object in the selection list.
            if (!manipulator->connectToDependNode(dependNode))
            {
                MGlobal::displayWarning("Error connecting manipulator to"
                    " object: " + dependNodeFn.name());
            }
            manipulator->initialize();
        }
    }
}
