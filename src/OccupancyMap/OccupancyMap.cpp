//
// Created by yaoyu on 5/6/20.
//

#include <cmath>

#include <thrust/device_vector.h>

#include "Profiling/SimpleTime.hpp"

#include "OccupancyMap/OccupancyMap.hpp"

using namespace pcu;

OccupancyMap::OccupancyMap()
: resolution(0.05), occupancyThreshold(0.5),
  probHit(0.7), probMiss(0.3),
  clampingThresholdMin(0.1), clampingThresholdMax(0.95),
  vx(0), vy(0), vz(0)
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

void OccupancyMap::set_basic_parameters( float res,
        float ocpThres, float probH, float probM, float clampMin, float clampMax ) {
    if ( pOcTree.get() ) {
        BOOST_THROW_EXCEPTION( CoordinateCheckFailed() << ExceptionInfoString("Already initialized") );
    }

    assert( res > 0 );
    assert( ocpThres > 0 );
    assert( probH > 0 );
    assert( probM >= 0 );
    assert( clampMin >= 0);
    assert( clampMax > 0 );

    resolution           = res;
    occupancyThreshold   = ocpThres;
    probHit              = probH;
    probMiss             = probM;
    clampingThresholdMin = clampMin;
    clampingThresholdMax = clampMax;
}

void OccupancyMap::refresh_basic_parameters() {
    if ( pOcTree.get() != nullptr ) {
        pOcTree->setOccupancyThres(occupancyThreshold);
        pOcTree->setProbHit(probHit);
        pOcTree->setProbMiss(probMiss);
        pOcTree->setClampingThresMin(clampingThresholdMin);
        pOcTree->setClampingThresMax(clampingThresholdMax);
    }
}

void OccupancyMap::read(const std::string &fn) {
    pOcTree.reset(dynamic_cast<octomap::OcTree*>(
            octomap::AbstractOcTree::read(fn) ) );

    resolution           = pOcTree->getResolution();
    occupancyThreshold   = pOcTree->getOccupancyThres();
    probHit              = pOcTree->getProbHit();
    probMiss             = pOcTree->getProbMiss();
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

    index[0] = std::floor( dx / resolution + 0.5 );
    index[1] = std::floor( dy / resolution + 0.5 );
    index[2] = std::floor( dz / resolution + 0.5 );
}

void OccupancyMap::get_dense_grid_index( const octomap::OcTreeKey &key,
                           std::size_t *index ) {
    index[0] = key[0] - refKey[0];
    index[1] = key[1] - refKey[1];
    index[2] = key[2] - refKey[2];
}

void OccupancyMap::traverse_octree_and_fill_dense_grid(CMask *denseGrid ) {
    std::size_t index3[3];
    std::size_t countFree = 0, countOccupied = 0;
    octomap::OcTreeKey key;
    for ( octomap::OcTree::leaf_iterator it = pOcTree->begin_leafs(16),
            end = pOcTree->end(); it != end; ++it ) {
        std::size_t depth = it.getDepth();

//        if ( depth != pOcTree->getTreeDepth() ) {
//            continue;
//        }

        key = it.getKey();

//        // Test use.
//        if ( key[0] == 31852 && key[1] == 32749 && key[2] == 32742 ) {
//            std::cout << "Here it is. " << std::endl;
//        }

        // Figure out the index into the denseGrid.
        get_dense_grid_index(key, index3);
        std::size_t index = index3[2]*vx*vy + index3[1]*vx +index3[0];

        assert( index3[0] < vx );
        assert( index3[1] < vy );
        assert( index3[2] < vz );

        // Fill.
        if ( pOcTree->isNodeOccupied(*it) ) {
            // Occupied.
            denseGrid[index] = OCP_MAP_OCC_OCCUPIED;
            countOccupied++;
        } else {
            // Free.
            denseGrid[index] = OCP_MAP_OCC_FREE;
            countFree++;
        }
    }

    std::cout << "countFree = " << countFree << ", "
              << "countOccupied = " << countOccupied << ". " << std::endl;
}

void OccupancyMap::find_frontiers() {
    QUICK_TIME_START(te)

    // Expand the OcTree object.
    pOcTree->expand();

    // Get the metric dimensions of the OcTree object.
    double m0[3];
    double m1[3];
    pOcTree->getMetricMin( m0[0], m0[1], m0[2] );
    pOcTree->getMetricMax( m1[0], m1[1], m1[2] );

    // Test use.
    octomap::OcTreeKey minKey, maxKey;
    if ( !pOcTree->coordToKeyChecked( octomap::point3d( m0[0], m0[1], m0[2] ), minKey ) ) {
        std::cout << "m0 is not in the OcTree object." << std::endl;
    } else {
        std::cout << "minKey = [ "
                  << minKey[0] << ", "
                  << minKey[1] << ", "
                  << minKey[2] << " ]" << std::endl;
    }

    if ( !pOcTree->coordToKeyChecked( octomap::point3d( m1[0], m1[1], m1[2] ), maxKey ) ) {
        std::cout << "m1 is not in the OcTree object." << std::endl;
    } else {
        std::cout << "maxKey = [ "
                  << maxKey[0] << ", "
                  << maxKey[1] << ", "
                  << maxKey[2] << " ]" << std::endl;
    }

    // The dense grid size.
    refKey = minKey;
    vx = maxKey[0] - minKey[0] + 1;
    vy = maxKey[1] - minKey[1] + 1;
    vz = maxKey[2] - minKey[2] + 1;

    // Test use.
    std::cout << "vx, vy, vz = "
              << vx << ", " << vy << ", " << vz << ". " << std::endl;

//    // Test use.
//    octomap::point3d testPoint( -45.7752, -0.949873, -1.278810 );
//    octomap:: OcTreeKey testKey;
//    bool testCheck;
//    testCheck = pOcTree->coordToKeyChecked(testPoint, testKey);
//    std::cout << "testCheck = " << testCheck << std::endl;
//    std::cout << "testKey = [ " << testKey[0] << ", "
//              << testKey[1] << ", "
//              << testKey[2] << " ] " << std::endl;
//    std::cout << "pOcTree->getTreeDepth() = " << pOcTree->getTreeDepth() << std::endl;
//
//    octomap::OcTreeNode *pNode = pOcTree->search(testPoint, 16);
//    if ( pNode ) {
//        std::cout << "pNode->getOccupancy() = " << pNode->getOccupancy() << std::endl;
//    } else {
//        std::cout << "Cannot find node. " << std::endl;
//    }

    // Allocate memory.
//    CR_DenseGrid denseGrid;
    denseGrid.resize( vx, vy, vz );

    // Fill in free and occupied voxels.
    traverse_octree_and_fill_dense_grid( denseGrid.get_dense_grid() );

    // Find all frontiers.
    denseGrid.find_frontiers();

    QUICK_TIME_SHOW(te, "Find frontier")
}

template < typename rT >
void OccupancyMap::get_coordinates_by_index(
        std::size_t ix, std::size_t iy, std::size_t iz,
        rT &x, rT &y, rT &z) {
    octomap::OcTreeKey key;
    key[0] = refKey[0] + ix;
    key[1] = refKey[1] + iy;
    key[2] = refKey[2] + iz;

    octomap::point3d c = pOcTree->keyToCoord(key, 16);

    x = c.x();
    y = c.y();
    z = c.z();
}

void OccupancyMap::write_voxels_as_list(const std::string &fn) {
    const CMask *pDenseGrid = denseGrid.get_dense_grid();
    if ( nullptr == pDenseGrid ) {
        std::stringstream ss;
        ss << "pDenseGrid is nullptr. ";
        throw( std::runtime_error( ss.str() ) );
    }

    std::ofstream ofs(fn);
    if ( !ofs.good() ) {
        std::stringstream ss;
        ss << "File " << fn << " not good. ";
        throw( std::runtime_error( ss.str() ) );
    }

    // Only write voxels that are free, occupied, and frontier.
    const auto xy = vx * vy;
    double coor[3];
    int m;
    for ( std::size_t z = 0; z < vz; ++z ) {
        for ( std::size_t y = 0; y < vy; ++y ) {
            for ( std::size_t x = 0; x < vx; ++x ) {
                auto idx = z * xy + y * vx + x;
                m = static_cast<int>(pDenseGrid[idx]);

                if ( OCP_MAP_OCC_UNKNOWN != m ) {
                    get_coordinates_by_index( x, y, z,
                            coor[0], coor[1], coor[2] );
                    ofs << coor[0] << ", "
                        << coor[1] << ", "
                        << coor[2] << ", "
                        << m << std::endl;
                }
            }
        }
    }

    ofs.close();
}
