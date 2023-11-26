#pragma once

#ifndef COLLISION_CANDIDATES_FINDER_H
#define COLLISION_CANDIDATES_FINDER_H

#include <stdio.h>
#include <vector>
#include <stdlib.h>

#include <MayaIncludes.h>

// Boost geometry
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value;

class CollisionCandidatesFinder 
{
    public:
        CollisionCandidatesFinder();
        ~CollisionCandidatesFinder();

        MStatus addActiveObject();
        MStatus getSceneMFnMeshes();
        MStatus initializeRTree();
        std::vector<MFnMesh*> checkNearbyObjects();
    public:
        MSelectionList selList;

        MFnMesh* activeMFnMesh;
        MFnDagNode activeTransformMFnDagNode;
        std::vector<MFnMesh*> allSceneMFnMeshes;
        bgi::rtree<value, bgi::quadratic<16>> rTree;
};


#endif //COLLISION_CANDIDATES_FINDER_H