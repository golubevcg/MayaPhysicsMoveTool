#pragma once

#ifndef COLLISION_CANDIDATES_FINDER_H
#define COLLISION_CANDIDATES_FINDER_H

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <MayaIncludes.h>
#include <unordered_map>

/**
 * @class CollisionCandidatesFinder
 * @brief Singleton class to manage and find collision candidates in a Maya scene.
 *
 * This class handles the identification and management of mesh objects that
 * are potential candidates for collision detection. It maintains a list of
 * active meshes and provides functionalities to update and retrieve them.
 */
class CollisionCandidatesFinder {
    public:
        /**
         * @brief Retrieves the singleton instance of the class.
         * @return Reference to the singleton instance.
         */
        static CollisionCandidatesFinder& getInstance();

        // Prevent copying and assignment.
        CollisionCandidatesFinder(const CollisionCandidatesFinder&) = delete;
        CollisionCandidatesFinder& operator=(const CollisionCandidatesFinder&) = delete;

        /**
         * @brief Adds only selected MFnMeshes in two maps activeMFnMeshes and activeTransfomMFnDagNodes
         */
        MStatus addActiveObjects();

        /**
         * @brief Retrieves all MFnMeshes in a scene into allSceneMFnMeshes map
         */
        MStatus getSceneMFnMeshes();

    public:
        MSelectionList selList;
        std::unordered_map<std::string, MFnMesh*> activeMFnMeshes;
        std::unordered_map<std::string, MObject> activeTransformMFnDagNodes;
        std::vector<MFnMesh*> allSceneMFnMeshes;

    private:
        CollisionCandidatesFinder();  // Constructor is private for singleton
        ~CollisionCandidatesFinder();  // Destructor
        static void initSingleton();
        static CollisionCandidatesFinder* instance;
        static std::once_flag initInstanceFlag;
};

#endif // COLLISION_CANDIDATES_FINDER_H
