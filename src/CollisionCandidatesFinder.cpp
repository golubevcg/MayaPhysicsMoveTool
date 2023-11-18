#include "CollisionCandidatesFinder.h"


CollisionCandidatesFinder::CollisionCandidatesFinder()
{

}

CollisionCandidatesFinder::~CollisionCandidatesFinder() 
{
    for (MFnMesh* mesh : this->allSceneMFnMeshes) {
        delete mesh;
    }
}

MStatus CollisionCandidatesFinder::addActiveObject()
{
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

MStatus CollisionCandidatesFinder::getSceneMFnMeshes()
{
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


        //TEMP REMOVE ACTIVE OBJECT
        if (fnMesh->fullPathName() == this->activeMFnMesh->fullPathName()) {
            MGlobal::displayInfo("*****Active mesh skipped" + MString() + this->activeMFnMesh->fullPathName());
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

MStatus CollisionCandidatesFinder::initializeRTree()
{
    if (this->allSceneMFnMeshes.empty()) {
        MGlobal::displayError("MFnMeshes vector is empty");
        return MS::kFailure;
    }

    MStatus status;
    for (size_t i = 0; i < this->allSceneMFnMeshes.size(); ++i) {
        MDagPath dagPath;
        status = this->allSceneMFnMeshes[i]->getPath(dagPath);
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get MDagPath from MFnMesh");
            return status;
        }

        MBoundingBox mbbox = this->allSceneMFnMeshes[i]->boundingBox();
        MMatrix worldMatrix = dagPath.inclusiveMatrix();

        MPoint minPoint = mbbox.min() * worldMatrix;
        MPoint maxPoint = mbbox.max() * worldMatrix;

        box bbox(point(minPoint.x, minPoint.y, minPoint.z), point(maxPoint.x, maxPoint.y, maxPoint.z));
        this->rTree.insert(std::make_pair(bbox, i));
    }

    return MS::kSuccess;
}

std::vector<MFnMesh*> CollisionCandidatesFinder::checkNearbyObjects()
{
    MObject selectedMObject = this->activeMFnMesh->object();
    std::vector<MFnMesh*> collisionCandidates;

    if (selectedMObject.isNull()) {
        return collisionCandidates;
    }

    MStatus status;
    MFnDagNode dagNode(selectedMObject);
    MDagPath dagPath;
    MDagPath::getAPathTo(selectedMObject, dagPath);
    MFnTransform transform(dagPath.transform(), &status);
    if (status != MStatus::kSuccess) {
        MString error_message = "Error during retriving transform object";
        MGlobal::displayError(error_message);
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
    this->rTree.query(bgi::intersects(queryBox), std::back_inserter(result));

    // collect all collided objects
    for (const auto& item : result) {
        // Exclude the selected mesh itself
        if (this->allSceneMFnMeshes[item.second]->fullPathName() == this->activeMFnMesh->fullPathName()) {  
            MGlobal::displayInfo("Active object skipped, YEEEAH!");
            continue;
        }

        collisionCandidates.push_back(this->allSceneMFnMeshes[item.second]);

        // debug print
        MFnDagNode colliderCandidateDagNode(this->allSceneMFnMeshes[item.second]->object());
        MString debug_message = "Added object to collision candidates:" + MString() + colliderCandidateDagNode.fullPathName();
        MGlobal::displayInfo(debug_message);
    }
    
    return collisionCandidates;
}