//
// Created by yaoyu on 3/20/20.
//

#include "HoleBoundaryDetection/HoleBoundaryDetector.hpp"

using namespace pcu;

HBDetector::HBDetector()
: pgK(10), pgR(0.02), pgSDB(100000),
  criteriaComputationStartIdx(0),
  factorAngleCriterion(0.4), factorHalfDiscCriterion(0.4), factorShapeCriterion(0.2),
  criterionThreshold(0.5)
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

void HBDetector::set_criterion_computation_start_index(int idx) {
    assert( idx >= 0 );
    criteriaComputationStartIdx = idx;
}

void HBDetector::set_criterion_params(float fA, float fH, float fS, float t) {
    factorAngleCriterion    = fA;
    factorHalfDiscCriterion = fH;
    factorShapeCriterion    = fS;
    criterionThreshold      = t;
}

void HBDetector::compute_criteria() {
    BoundaryCriterion<P_t, float> bc;
    bc.set_point_cloud(pInput);
    bc.set_proximity_graph(&proximityGraph);

    bc.compute(criteria, maxAngleNeighbors, criteriaComputationStartIdx);
}

bool HBDetector::criterion_over_threshold( float ac, float hc, float sc ) {
    return ( factorAngleCriterion * ac +
             factorHalfDiscCriterion * hc +
             factorShapeCriterion * sc >=
             criterionThreshold );
}

void HBDetector::find_candidates_by_criteria( std::vector<bool>& vbFlag, std::vector<int>& candidates ) {
    // Clear the candidates.
    candidates.clear();

    bool flag = false;

    for ( int i = 0; i < pInput->size(); ++i ) {

        flag = criterion_over_threshold( criteria(i, 0), criteria(i, 1), criteria(i, 2) );

        if ( flag ) {
            vbFlag[i] = true;
            candidates.push_back(i);
        } else {
            vbFlag[i] = false;
        }
    }
}

void HBDetector::coherence_filter( std::vector<bool>& vbFlag, std::vector<int>& candidates ) {
    // Temporary vector storing the valid boundary candidates.
    std::vector<int> tempCandidates;

    // Temporary vector storing the invalid boundary candidates.
    std::vector<int> tempInvalid;

    int n0, n1;
    int c; // The current candidate.
    int changeCount = -1, loop = 0;

    while ( changeCount != 0 ) {
        std::cout << "loop: " << loop
                  << ", candidates number: " << candidates.size()
                  << std::endl;

        // Loop over all current candidates.
        tempCandidates.clear();
        tempInvalid.clear();
        changeCount = 0;

        for ( int i = 0; i < candidates.size(); ++i ) {
            c = candidates[i];

            // Get the two neighbors.
            n0 = maxAngleNeighbors(c, 0);
            n1 = maxAngleNeighbors(c, 1);

            // Test use.
            if ( c == 35595 ) {
                std::cout << "c: " << c << ", "
                          << "n0: " << n0 << ", "
                          << "n1: " << n1 << std::endl;
            }

            if ( vbFlag[n0] == true && vbFlag[n1] == true ) {
                tempCandidates.push_back(c);
            } else {
                tempInvalid.push_back(c);
                changeCount++;
            }
        }

        // Update candidates.
        candidates.resize( tempCandidates.size() );
        std::copy( tempCandidates.begin(), tempCandidates.end(), candidates.begin() );

        // Update the vbFlag.
        for ( const int& i : tempInvalid ) {
            vbFlag[i] = false;
        }

        // Test use.
        break;
    }
}

void HBDetector::coherence_filter() {
    QUICK_TIME_START(te)

    // Create the temporary flag vector.
    std::vector<bool> vbFlag( pInput->size() );

    // Create the initial boundary candidates.
    find_candidates_by_criteria( vbFlag, boundaryIndices );

    coherence_filter( vbFlag, boundaryIndices );

    QUICK_TIME_END(te)

    std::cout << "Coherence filter in " << te << "ms. " << std::endl;
}

void HBDetector::process(){
    // Build the proximity graph.
    build_proximity_graph();

    // Compute the criteria.
    compute_criteria();

    // Coherence filter.
    coherence_filter();
}

void HBDetector::create_rgb_representation_by_criteria(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput) {
    QUICK_TIME_START(te)

    assert(pInput.get() != nullptr);
    assert( pInput->size() == criteria.rows() );

    // Copy the input to the output.
    pcl::copyPointCloud( *pInput, *pOutput );

    // Loop over all the points to create the color.
//    for ( int i = 0; i < pInput->size(); ++i ) {
//        if ( criteria(i, 0) >= 0.3 ) {
//            pOutput->at(i).rgba = 0xFFFF0000; // Red.
//        } else {
//            pOutput->at(i).rgba = 0xFFFFFFFF; // Wight.
//        }
//    }

//    for ( int i = 0; i < pInput->size(); ++i ) {
//        if ( criteria(i, 1) >= 0.75 ) {
//            pOutput->at(i).rgba = 0xFFFF0000; // Red.
//        } else {
//            pOutput->at(i).rgba = 0xFFFFFFFF; // Wight.
//        }
//    }

//    for ( int i = 0; i < pInput->size(); ++i ) {
//        if ( criteria(i, 2) >= 0.6 ) {
//            pOutput->at(i).rgba = 0xFFFF0000; // Red.
//        } else {
//            pOutput->at(i).rgba = 0xFFFFFFFF; // Wight.
//        }
//    }

    bool flag = false;

    for ( int i = 0; i < pInput->size(); ++i ) {

        flag = criterion_over_threshold( criteria(i, 0), criteria(i, 1), criteria(i, 2) );

        if ( flag ) {
            pOutput->at(i).rgba = 0xFFFF0000; // Red.
        } else {
            pOutput->at(i).rgba = 0xFFFFFFFF; // Wight.
        }
    }

    QUICK_TIME_END(te)

    std::cout << "Create RGB representation by criteria in " << te << "ms. " << std::endl;
}

void HBDetector::create_rgb_representation_by_boundary_candidates( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput ) {
    QUICK_TIME_START(te)

    assert( pInput.get() != nullptr );
    assert( pInput->size() == criteria.rows() );

    // Copy the input to the output.
    pcl::copyPointCloud( *pInput, *pOutput );

    for ( int i = 0; i < pInput->size(); ++i ) {
        pOutput->at(i).rgba = 0xFFFFFFFF; // Wight.
    }

    for ( const int& i : boundaryIndices ) {
        pOutput->at(i).rgba = 0xFFFF0000; // Red.
    }

    QUICK_TIME_END(te)

    std::cout << "Create RGB representation by boundary candidates in " << te << "ms. " << std::endl;
}