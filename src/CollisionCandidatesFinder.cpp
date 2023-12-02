#include "CollisionCandidatesFinder.h"


CollisionCandidatesFinder::CollisionCandidatesFinder()
    : activeMFnMesh(nullptr) {
}

CollisionCandidatesFinder::~CollisionCandidatesFinder() {
    for (MFnMesh* mesh : this->allSceneMFnMeshes) {
        delete mesh;
    }
}

MStatus CollisionCandidatesFinder::addActiveObject() {
    MGlobal::getActiveSelectionList(this->selList);
    if (this->selList.isEmpty()) {
        MString warningMessage = "No objects selected";
        MGlobal::displayWarning(warningMessage);
        return MS::kFailure;
    }

    MObject node;
    this->selList.getDependNode(0, node);  // Get the first selected node
    if (node.hasFn(MFn::kTransform)) {
        bool meshFound = false;
        this->activeTransformMFnDagNode.setObject(node);

        for (unsigned int i = 0; i < this->activeTransformMFnDagNode.childCount(); ++i) {
            MObject child = this->activeTransformMFnDagNode.child(i);
            if (!child.hasFn(MFn::kMesh)) {
                continue;
            }

            MFnMesh mesh(child);
            MString infoMessage = "Active mesh object was found: " + mesh.name();
            MGlobal::displayInfo(infoMessage);

            this->activeMFnMesh = new MFnMesh(child);
            meshFound = true;
            break;
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

    return MS::kSuccess;
}

MStatus CollisionCandidatesFinder::getSceneMFnMeshes() {
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

        this->allSceneMFnMeshes.push_back(fnMesh);
    }

    return MS::kSuccess;
}
