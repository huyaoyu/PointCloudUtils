//
// Created by yaoyu on 5/6/20.
//

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
    pOcTree->readBinary(fn);

    resolution = pOcTree->getResolution();
    probHit    = pOcTree->getProbHit();
    probMiss   = pOcTree->getProbMiss();
    clampingThresholdMin = pOcTree->getClampingThresMin();
    clampingThresholdMax = pOcTree->getClampingThresMax();
}

void OccupancyMap::write(const std::string &fn) {
    pOcTree->writeBinary(fn);
}

