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

void HBDetector::create_rgb_representation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput) {
    QUICK_TIME_START(te)

    assert(pInput.get() != nullptr);
    assert( pInput->size() == criteria.rows() );

    // Copy the input to the output.
    pcl::copyPointCloud( *pInput, *pOutput );

    // Loop over all the points to create the color.
    for ( int i = 0; i < pInput->size(); ++i ) {
        if ( criteria(i, 0) >= 0.3 ) {
            pOutput->at(i).rgba = 0xFFFF0000; // Red.
        } else {
            pOutput->at(i).rgba = 0xFFFFFFFF; // Wight.
        }
    }

    QUICK_TIME_END(te)

    std::cout << "Create RGB representation in " << te << "ms. " << std::endl;
}