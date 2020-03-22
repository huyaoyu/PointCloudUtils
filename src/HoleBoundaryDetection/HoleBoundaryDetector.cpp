//
// Created by yaoyu on 3/20/20.
//

#include "HoleBoundaryDetection/HoleBoundaryDetector.hpp"

using namespace pcu;

HBDetector::HBDetector()
: pgK(10), pgR(0.02), pgSDB(100000)
{

}

HBDetector::~HBDetector()
{

}

void HBDetector::set_point_cloud(PC_t::Ptr& p) {
    pInput = p;
}

void HBDetector::set_proximity_graph_params(int k, double radius, int showDetailBase) {
    pgK   = k;
    pgR   = radius;
    pgSDB = showDetailBase;
}

ProximityGraph<HBDetector::P_t>& HBDetector::get_proximity_graph() {
    return proximityGraph;
}

void HBDetector::build_proximity_graph() {
    proximityGraph.set_point_cloud(pInput);

    proximityGraph.process(pgK, pgR, pgSDB);
}

void HBDetector::compute_criteria() {
    BoundaryCriterion<P_t> bc;
    bc.set_point_cloud(pInput);
    bc.set_proximity_graph(&proximityGraph);

    bc.compute<float>(criteria);
}

void HBDetector::process(){
    // Build the proximity graph.
    build_proximity_graph();

    // Compute the criteria.
    compute_criteria();
}