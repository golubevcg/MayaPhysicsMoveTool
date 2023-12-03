#pragma once

#ifndef COLLISION_CANDIDATES_FINDER_H
#define COLLISION_CANDIDATES_FINDER_H

#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <MayaIncludes.h>


class CollisionCandidatesFinder {
    public:
        CollisionCandidatesFinder();
        ~CollisionCandidatesFinder();

        MStatus addActiveObject();
        MStatus getSceneMFnMeshes();

    public:
        MSelectionList selList;
        MFnMesh* activeMFnMesh;
        MFnDagNode activeTransformMFnDagNode;
        std::vector<MFnMesh*> allSceneMFnMeshes;
};


#endif //COLLISION_CANDIDATES_FINDER_H