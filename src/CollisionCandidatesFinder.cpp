#include <mutex>
#include "CollisionCandidatesFinder.h"


CollisionCandidatesFinder* CollisionCandidatesFinder::instance = nullptr;
std::once_flag CollisionCandidatesFinder::initInstanceFlag;

CollisionCandidatesFinder& CollisionCandidatesFinder::getInstance() {
    std::call_once(initInstanceFlag, &CollisionCandidatesFinder::initSingleton);
    return *instance;
}

void CollisionCandidatesFinder::initSingleton() {
    instance = new CollisionCandidatesFinder();
}

// Iterate over selected object children and find all MFnMeshes
MStatus CollisionCandidatesFinder::addActiveObjects() {
    MGlobal::getActiveSelectionList(this->selList);
    if (this->selList.isEmpty()) {
        MString warningMessage = "No objects selected";
        MGlobal::displayWarning(warningMessage);
        return MS::kFailure;
    }

    this->activeMFnMeshes.clear();
    this->activeTransformMFnDagNodes.clear();

    MStatus stat = MStatus::kSuccess;
    MSelectionList list;
    stat = MGlobal::getActiveSelectionList(list);
    MItSelectionList iter(list, MFn::kInvalid, &stat);
    for (; !iter.isDone(); iter.next()) {
        MObject dependNode;
        iter.getDependNode(dependNode);
        if (dependNode.hasFn(MFn::kTransform)) {
            bool meshFound = false;
            MFnDagNode dagNode;
            dagNode.setObject(dependNode);

            for (unsigned int i = 0; i < dagNode.childCount(); ++i) {
                MObject child = dagNode.child(i);
                if (!child.hasFn(MFn::kMesh)) {
                    continue;
                }

                MFnMesh mesh(child);
                std::string fullName = mesh.fullPathName().asChar();
                this->activeMFnMeshes[fullName] = new MFnMesh(child);
                meshFound = true;
                this->activeTransformMFnDagNodes[fullName] = dependNode;
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
    }
    iter.reset();


    return MS::kSuccess;
}

// Get map of all scene MFnMeshes with name : MFnMesh signature
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

        MBoundingBox boundingBox = fnMesh->boundingBox(&status);
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get bounding box");
            continue;
        }

        this->allSceneMFnMeshes.push_back(fnMesh);
    }

    return MS::kSuccess;
}
