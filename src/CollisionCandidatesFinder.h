// CollisionCandidatesFinder.h
#ifndef COLLISION_CANDIDATES_FINDER_H
#define COLLISION_CANDIDATES_FINDER_H

#include <stdio.h>
#include <vector>
#include <stdlib.h>

#include <maya/MGlobal.h>
#include <maya/MFnDagNode.h>

// Boost geometry
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>

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
        std::vector<MObject> checkNearbyObjects();
    public:
        MFnMesh* activeMFnMesh;
        MFnDagNode activeTransformMFnDagNode;
        std::vector<MFnMesh*> allSceneMFnMeshes;
        bgi::rtree<value, bgi::quadratic<16>> rTree;
};


#endif //COLLISION_CANDIDATES_FINDER_H