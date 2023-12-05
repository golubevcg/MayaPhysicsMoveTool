#pragma once

#ifndef COLLISION_CANDIDATES_FINDER_H
#define COLLISION_CANDIDATES_FINDER_H

#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <MayaIncludes.h>


class CollisionCandidatesFinder {
    public:
        static CollisionCandidatesFinder& getInstance();

        // Delete copy constructor and copy assignment operator to prevent copies of the singleton
        CollisionCandidatesFinder(const CollisionCandidatesFinder&) = delete;
        CollisionCandidatesFinder& operator=(const CollisionCandidatesFinder&) = delete;

        MStatus addActiveObject();
        MStatus getSceneMFnMeshes();

    public:
        MSelectionList selList;
        MFnMesh* activeMFnMesh;
        MFnDagNode activeTransformMFnDagNode;
        std::vector<MFnMesh*> allSceneMFnMeshes;

    private:
        CollisionCandidatesFinder();  // Constructor is private
        ~CollisionCandidatesFinder();  // Constructor is private
        static void initSingleton();
        static CollisionCandidatesFinder* instance;
        static std::once_flag initInstanceFlag;
};


#endif //COLLISION_CANDIDATES_FINDER_H