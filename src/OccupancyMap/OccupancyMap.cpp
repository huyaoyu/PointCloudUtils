//
// Created by yaoyu on 5/6/20.
//

#include <cmath>

#include <thrust/device_vector.h>

#include "OccupancyMap/OccupancyMap.hpp"

using namespace pcu;

OccupancyMap::OccupancyMap()
: resolution(0.05),
  probHit(0.7), probMiss(0.4),
  clampingThresholdMin(0.12), clampingThresholdMax(0.97)
{

}

OccupancyMap::~OccupancyMap() {
}

void OccupancyMap::initialize( float res ) {
    assert(res > 0);
    resolution = res;

    pOcTree.reset( new octomap::OcTree(resolution) );
    refresh_basic_parameters();
}

octomap::OcTree& OccupancyMap::get_octree() {
    return *pOcTree;
}

void OccupancyMap::set_basic_parameters( float res, float probH, float probM, float clampMin, float clampMax ) {
    if ( pOcTree.get() ) {
        BOOST_THROW_EXCEPTION( CoordinateCheckFailed() << ExceptionInfoString("Already initialized") );
    }

    assert( res > 0 );
    assert( probH > 0 );
    assert( probM >= 0 );
    assert( clampMin >= 0);
    assert( clampMax > 0 );

    resolution           = res;
    probHit              = probH;
    probMiss             = probM;
    clampingThresholdMin = clampMin;
    clampingThresholdMax = clampMax;
}

void OccupancyMap::refresh_basic_parameters() {
    if ( pOcTree.get() != nullptr ) {
        pOcTree->setProbHit(probHit);
        pOcTree->setProbMiss(probMiss);
        pOcTree->setClampingThresMin(clampingThresholdMin);
        pOcTree->setClampingThresMax(clampingThresholdMax);
    }
}

void OccupancyMap::read(const std::string &fn) {
    pOcTree.reset(dynamic_cast<octomap::OcTree*>(
            octomap::AbstractOcTree::read(fn) ) );

    resolution = pOcTree->getResolution();
    probHit    = pOcTree->getProbHit();
    probMiss   = pOcTree->getProbMiss();
    clampingThresholdMin = pOcTree->getClampingThresMin();
    clampingThresholdMax = pOcTree->getClampingThresMax();
}

void OccupancyMap::write(const std::string &fn) {
    pOcTree->write(fn);
}

//static void get_octomap_size_parameters(
//        const octomap::OcTree &ocTree,
//        octomap::point3d &minPoint,
//        octomap::point3d &size3D ) {
//    minPoint = ocTree.getBBXMin();
//    size3D   = ocTree.getBBXMax() - minPoint;
//}

void OccupancyMap::get_dense_grid_index( const double* minPoint,
                           const octomap::point3d &point,
                           std::size_t *index ) {
    const double dx = point.x() - minPoint[0];
    const double dy = point.y() - minPoint[1];
    const double dz = point.z() - minPoint[2];

    assert( dx >= 0 );
    assert( dy >= 0 );
    assert( dz >= 0 );

    index[0] = std::floor( dx / resolution );
    index[1] = std::floor( dy / resolution );
    index[2] = std::floor( dz / resolution );
}

void OccupancyMap::traverse_octree_and_fill_dense_grid(CMask *denseGrid ) {
    double m[3];
    pOcTree->getMetricMin(m[0], m[1], m[2]);

    std::size_t index3[3];
    std::size_t countFree = 0, countOccupied = 0;
    for ( octomap::OcTree::leaf_iterator it = pOcTree->begin_leafs(15),
            end = pOcTree->end_leafs(); it != end; ++it ) {
        octomap::point3d corr = it.getCoordinate();
        auto v = it->getValue();

        // Figure out the index into the denseGrid.
        get_dense_grid_index(m, corr, index3);
        std::size_t index = index3[2]*vx*vy + index3[1]*vx +index3[0];

        // Fill.
        if ( v < -0.05 ) {
            // Free.
            denseGrid[index] = OCP_MAP_OCC_FREE;
            countFree++;
        } else {
            // Occupied.
            denseGrid[index] = OCP_MAP_OCC_OCCUPIED;
            countOccupied++;
        }
    }

    std::cout << "countFree = " << countFree << ", "
              << "countOccupied = " << countOccupied << ". " << std::endl;
}

void OccupancyMap::find_frontiers() {
    // Get the metric dimensions of the OcTree object.
    double m0[3];
    double m1[3];
    pOcTree->getMetricMin( m0[0], m0[1], m0[2] );
    pOcTree->getMetricMax( m1[0], m1[1], m1[2] );

    // The dense grid size.
    vx = ( m1[0] - m0[0] ) / resolution;
    vy = ( m1[1] - m0[1] ) / resolution;
    vz = ( m1[2] - m0[2] ) / resolution;

    // Allocate memory.
    CR_DenseGrid denseGrid;
    denseGrid.resize( vx, vy, vz );

    // Fill in free and occupied voxels.
    traverse_octree_and_fill_dense_grid( denseGrid.get_dense_grid() );
}
