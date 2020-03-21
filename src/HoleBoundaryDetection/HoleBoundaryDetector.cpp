//
// Created by yaoyu on 3/20/20.
//

#include "HoleBoundaryDetection/HoleBoundaryDetector.hpp"

using namespace pcu;

HBDetector::HBDetector()
{

}

HBDetector::~HBDetector()
{

}

void HBDetector::set_point_cloud(PC_t::Ptr& pInput) {
    mpInput = pInput;
}