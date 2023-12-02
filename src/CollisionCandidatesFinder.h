#pragma once

#ifndef COLLISION_CANDIDATES_FINDER_H
#define COLLISION_CANDIDATES_FINDER_H

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <MayaIncludes.h>

class CollisionCandidatesFinder{
    public:
        static CollisionCandidatesFinder& getInstance() {
            static CollisionCandidatesFinder instance;
            return instance;
        }

        CollisionCandidatesFinder(const CollisionCandidatesFinder&) = delete;
        CollisionCandidatesFinder& operator=(const CollisionCandidatesFinder&) = delete;

        MStatus addActiveObject();
        MStatus getSceneMFnMeshes();

        MSelectionList selList;
        MFnMesh* activeMFnMesh;
        MFnDagNode activeTransformMFnDagNode;
        std::vector<MFnMesh*> allSceneMFnMeshes;

    private:
        CollisionCandidatesFinder();
        ~CollisionCandidatesFinder();
};


#endif //COLLISION_CANDIDATES_FINDER_H