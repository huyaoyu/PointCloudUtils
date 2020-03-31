//
// Created by yaoyu on 3/20/20.
//

#include <fstream>

#include <boost/pending/disjoint_sets.hpp>

#include <pcl/common/centroid.h> // computeMeanAndCovarianceMatrix().
#include <pcl/features/feature.h> // solvePlaneParameters().

#include "Graph/Edge.hpp"
#include "HoleBoundaryDetection/HoleBoundaryDetector.hpp"
#include "PCCommon/common.hpp"
#include "PCCommon/extraction.hpp"
#include "Visualization/Color.hpp"

using namespace pcu;

HBDetector::HBDetector()
: pgK(10), pgR(0.02), pgSDB(100000),
  criteriaComputationStartIdx(0),
  factorAngleCriterion(0.4), factorHalfDiscCriterion(0.4), factorShapeCriterion(0.2),
  criterionThreshold(0.5),
  equivalentNormalAveragingLimit(50),
  pEquivalentNormal(new pcl::PointCloud<pcl::PointNormal>)
{
    normalViewPoint << 0.0f, 0.0f, 0.0f, 1.0f;
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
    proximityGraph.set_k_r(pgK, pgR);

    proximityGraph.process(pgSDB);
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

void HBDetector::set_normal_view_point(float x, float y, float z) {
    normalViewPoint << x, y, z, 1.0f;
}

void HBDetector::set_equivalent_normal_averaging_limit(int limit) {
    assert( limit >= 3 );
    equivalentNormalAveragingLimit = limit;
}

void HBDetector::compute_criteria() {
    BoundaryCriterion<P_t, float> bc;
    bc.set_point_cloud(pInput);
    bc.set_proximity_graph(&proximityGraph);

    bc.compute(criteria, maxAngleNeighbors, rp, criteriaComputationStartIdx);
}

float HBDetector::criterion_value( float ac, float hc, float sc ) {
    return ( factorAngleCriterion * ac +
             factorHalfDiscCriterion * hc +
             factorShapeCriterion * sc );
}

bool HBDetector::criterion_over_threshold( float ac, float hc, float sc ) {
    return ( criterion_value(ac, hc, sc) >= criterionThreshold );
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

//            // Test use.
//            if ( c == 35595 ) {
//                std::cout << "c: " << c << ", "
//                          << "n0: " << n0 << ", "
//                          << "n1: " << n1 << std::endl;
//            }

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

        // For now only perform once since it tends to
        // remove all the points along along chain of candidates.
        break;
    }
}

void HBDetector::coherence_filter() {
    QUICK_TIME_START(te)

    // Create the temporary flag vector.
    std::vector<bool> vbFlag( pInput->size() );

    // Create the initial boundary candidates.
    find_candidates_by_criteria(vbFlag, boundaryCandidates );

    coherence_filter(vbFlag, boundaryCandidates );

    QUICK_TIME_END(te)

    std::cout << "Coherence filter in " << te << "ms. " << std::endl;
}

void HBDetector::map_indices_in_proximity_graph(
        ProximityGraph<P_t>& pg, const std::vector<int>& reference ) {
    const auto n = pg.size();

    assert( n > 0 );
    assert( n == reference.size() );

    std::vector<ProximityGraph<P_t>::Index_t> tempNeighbors;

    for ( std::size_t i = 0; i < n; ++i) {
        // Get the neighbors stored in the proximity graph.
        ProximityGraph<P_t>::Neighbors_t& neighbors = pg.get_neighbors(i);

        tempNeighbors.resize( neighbors.size() );

        std::size_t j = 0;

        for ( auto nb : neighbors ) {
            tempNeighbors[j] = reference[ nb ];
            j++;
        }

        // Update the stored neighbors in the proximity graph.
        std::copy( tempNeighbors.begin(), tempNeighbors.end(), std::inserter( neighbors, neighbors.begin() ) );
    }
}

void HBDetector::create_edges_from_points(
        const std::vector<int>& referenceIndices,
        const ProximityGraph<P_t>& pg,
        std::vector<Edge<int, float>>& edges ) {

    QUICK_TIME_START(te)

    float weight = 0;

    // Loop over all neighbors specified in the proximity graph.
    const std::size_t N = pg.size();

    // The temporary cross check map.
    std::vector< std::set<int> > crossCheck( N );

    for ( std::size_t i = 0; i < N; ++i ) {
        const int centerIndex   = referenceIndices[i];
        const auto& centerPoint = pInput->at( centerIndex );
        const auto c0 = criterion_value(
                criteria(centerIndex, 0),
                criteria(centerIndex, 1),
                criteria(centerIndex, 2) );
        const auto rCenter = rp(centerIndex, 0);

        const auto& neighbors = pg.get_neighbors(i);

        auto& cci = crossCheck[i];

        for ( const auto& n : neighbors ) {
            bool flagContinue = false;

            // Cross check.
            if ( !cci.empty() ) {
                if ( cci.end() != cci.find(n) ) {
                    flagContinue = true;
                }
            }

            if ( !flagContinue ) {
                cci.insert(n);
            }

            auto& ccn = crossCheck[n];
            if ( !ccn.empty() ) {
                if ( ccn.end() != ccn.find(i) ) {
                    continue;
                }
            }
            ccn.insert(i);

            if ( flagContinue ) {
                continue;
            }

            // Cross-check done.

            const auto nRef = referenceIndices[n];

            const auto& neighborPoint = pInput->at( nRef );
            const auto distance = distance_two_points( centerPoint, neighborPoint );
            const auto c1 = criterion_value(
                    criteria(nRef, 0),
                    criteria(nRef, 1),
                    criteria(nRef, 2) );

            weight = 2.0f - c0 - c1 + ( 2.0f * distance ) / ( rCenter + rp(nRef, 0) );

            // Create the edge.
            edges.emplace_back( Edge<int, float>( centerIndex, nRef, weight ) );
        }
    }

    // Sort the edges.
    std::sort( edges.begin(), edges.end() );

    QUICK_TIME_END(te)

    std::cout << "create_edges_from_points() in " << te << "ms. " << std::endl;
    std::cout << "referenceIndices.size() = " << referenceIndices.size() << std::endl;
    std::cout << "pg.size() = " << pg.size() << std::endl;
    std::cout << "edges.size() = " << edges.size() << std::endl;
}

void HBDetector::make_disjoint_sets_from_edges(
        const std::vector<Edge<int, float>>& edges,
        const std::vector<int>& references,
        std::vector<std::vector<int>>& disjointSets ) {
    QUICK_TIME_START(te)

    // ========== Preparation. ==========
    // The maps.
    typedef std::map<int, std::size_t> Rank_t;
    typedef std::map<int, int> Parent_t;
    typedef boost::associative_property_map<Rank_t> PropMapRank_t;
    typedef boost::associative_property_map<Parent_t> PropMapParent_t;

    Rank_t          mapRank;
    Parent_t        mapParent;
    PropMapRank_t   propMapRank(mapRank);
    PropMapParent_t propMapParent(mapParent);

    // The disjoint set.
    boost::disjoint_sets<PropMapRank_t, PropMapParent_t> djs( propMapRank, propMapParent );

    // Make a disjoint set with all the vertices as sub-set containing single element.
    for ( const int& v : references ) {
        djs.make_set( v );
    }

    // ========== Process. ==========
    for ( const Edge<int, float>& e : edges ) {
        // Get the sub-sets contain the two vertices of the current edge.
        auto u = djs.find_set(e.v0);
        auto v = djs.find_set(e.v1);

        // Check if it makes a circle in the MST.
        if ( u != v) {
            // Not a circle.
            djs.link(u, v);
        } else {
            // A circle will be made if we contain this edge into the MST.
            // Do nothing.
        }
    }

    // Flatten.
    djs.compress_sets( references.begin(), references.end() );

    // Test use.
    std::cout << "Disjoint set has " << djs.count_sets(references.begin(), references.end()) << " sub-sets. " << std::endl;

    // Find all the representatives.
    std::set<int> representatives;
    for ( const auto& m : mapParent ) {
        representatives.insert( m.second );
    }

    // Test use.
    std::cout << "Number of representatives: " << representatives.size() << std::endl;

    // Make a map from representative to index.
    std::map<int, int> r2i;
    int count = 0;
    for ( int r : representatives ) {
        r2i.insert( std::pair<int, int>(r, count) );
        count++;
    }

    // Make the disjoint sets.
    disjointSets.resize( count );
    for ( const auto& m : mapParent ) {
        disjointSets[ r2i[m.second] ].push_back( m.first );
    }

    QUICK_TIME_END(te)

    std::cout << "make_disjoint_sets_from_edges() in " << te << "ms. " << std::endl;
}

void HBDetector::make_disjoint_boundary_candidates() {
    QUICK_TIME_START(te)

    // Clear the current disjoint sets.
    disjointBoundaryCandidates.clear();

    // Extract the current boundary candidate points.
    PC_t::Ptr candidatePoints ( new PC_t );
    extract_points<P_t>( pInput, candidatePoints, boundaryCandidates );

    // Make a temporary proximity graph based on radius search.
    RProximityGraph<P_t> rpg;
    rpg.set_r(pgR);
    rpg.set_point_cloud(candidatePoints);
    rpg.process(std::max( candidatePoints->size()/4, static_cast<std::size_t>(1) ));

    // Update the indices in the proximity graph.
//    map_indices_in_proximity_graph( rpg, boundaryCandidates );

    // Create the edges according to the weights.
    std::vector<Edge<int, float>> edges;
    create_edges_from_points( boundaryCandidates, rpg, edges );

    // Make the disjoint sets.
    make_disjoint_sets_from_edges( edges, boundaryCandidates, disjointBoundaryCandidates );

    QUICK_TIME_END(te)

    std::cout << "make_disjoint_boundary_candidates() in " << te << "ms. " << std::endl;
}

template < typename pT >
static void compute_cluster_normal( const typename pcl::PointCloud<pT>::Ptr pInput,
        const pcl::PointIndices::Ptr pIndices,
        pcl::PointNormal& pn ) {
    EIGEN_ALIGN16 Eigen::Matrix3f convMat;
    EIGEN_ALIGN16 Eigen::Vector4f centroid;

    float normalCurvature[4];

    // Get the centroid and normal.
    if (0 == pcl::computeMeanAndCovarianceMatrix( *pInput, *pIndices, convMat, centroid ) ) {
        std::stringstream ss;
        ss << "Failed to compute a normal. ";
        throw( std::runtime_error( ss.str() ) );
    }

    pcl::solvePlaneParameters( convMat,
            normalCurvature[0], normalCurvature[1], normalCurvature[2], normalCurvature[3] );

    pn.x = centroid(0); pn.y = centroid(1); pn.z = centroid(2);
    pn.normal_x = normalCurvature[0]; pn.normal_y = normalCurvature[1]; pn.normal_z = normalCurvature[2];
    pn.curvature = normalCurvature[3];
}

template < typename pT >
static void average_cluster_normal( const typename pcl::PointCloud<pT>::Ptr pInput,
                                    const pcl::PointIndices::Ptr pIndices,
                                    pcl::PointNormal& pn ) {
    pT point;

    pn.x = 0.0f; pn.y = 0.0f; pn.z = 0.0f;
    pn.normal_x = 0.0f; pn.normal_y = 0.0f; pn.normal_z = 0.0f;

    for ( const auto& idx : pIndices->indices ) {
        point = pInput->at(idx);

        pn.x += point.x;
        pn.y += point.y;
        pn.z += point.z;
        pn.normal_x  += point.normal_x;
        pn.normal_y  += point.normal_y;
        pn.normal_z  += point.normal_z;
        pn.curvature += point.curvature;
    }

    const auto N = static_cast<float>( pIndices->indices.size() );

    pn.x = pn.x / N;
    pn.y = pn.y / N;
    pn.z = pn.z / N;
    pn.normal_x  = pn.normal_x / N;
    pn.normal_y  = pn.normal_y / N;
    pn.normal_z  = pn.normal_z / N;
    pn.curvature = pn.curvature / N;
}

void HBDetector::compute_centroid_and_equivalent_normal() {
    pEquivalentNormal->clear();

    for ( const auto& d : disjointBoundaryCandidates ) {
//        if ( d.size() < 3 ) {
//            continue;
//        }

        // Extract points.
        pcl::PointIndices::Ptr pclIndices ( new pcl::PointIndices );
        convert_vector_2_pcl_indices( d, pclIndices );

        pcl::PointNormal pn;

        if ( d.size() > equivalentNormalAveragingLimit ) {
            compute_cluster_normal<P_t>(pInput, pclIndices, pn);
        } else {
            average_cluster_normal<P_t>(pInput, pclIndices, pn);
        }

        pEquivalentNormal->push_back( pn );
    }

    std::cout << pEquivalentNormal->size() << " equivalent normals found. " << std::endl;
}

void HBDetector::process(){
    // Build the proximity graph.
    build_proximity_graph();

    // Compute the criteria.
    compute_criteria();

    // Coherence filter.
    coherence_filter();

    // Make disjoint sets from the boundary candidates.
    make_disjoint_boundary_candidates();

    // Compute the centroid and equivalent normal of the disjoint sets.
    compute_centroid_and_equivalent_normal();
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

    for ( int i = 0; i < pOutput->size(); ++i ) {
        pOutput->at(i).rgba = 0xFFFFFFFF; // Wight.
    }

    for ( const int& i : boundaryCandidates ) {
        pOutput->at(i).rgba = 0xFFFF0000; // Red.
    }

    QUICK_TIME_END(te)

    std::cout << "Create RGB representation by boundary candidates in " << te << "ms. " << std::endl;
}

void HBDetector::create_rgb_representation_by_disjoint_candidates(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput ) {
    QUICK_TIME_START(te)

    assert( pInput.get() != nullptr );
    assert( pInput->size() == criteria.rows() );

    // Copy the input to the output.
    pcl::copyPointCloud( *pInput, *pOutput );

    for ( int i = 0; i < pOutput->size(); ++i ) {
        pOutput->at(i).rgba = 0xFFFFFFFF; // Wight.
    }

    // Color object.
    CommonColor color;

    for ( const auto& d : disjointBoundaryCandidates ) {
        const std::uint32_t c = color.next();

        for ( const int i : d ) {
            pOutput->at(i).rgba = c;
        }
    }

    QUICK_TIME_END(te)

    std::cout << "Create RGB representation by disjoint candidates in " << te << "ms. " << std::endl;
}

HBDetector::PC_t::Ptr HBDetector::get_equivalent_normal() {
    if ( pEquivalentNormal->empty() ) {
        std::stringstream ss;
        ss << "pEquivalentNormal has zero points. ";
        throw( std::runtime_error( ss.str() ) );
    }

    return pEquivalentNormal;
}

template < typename vT >
static void write_array_json(std::ofstream& ofs,
        const std::string& name, const std::vector<vT>& v,
        int lineBreak=0, const std::string& indent="    ") {
    const std::size_t N = v.size();

    ofs << "\"" << name << "\": [ ";

    if ( 0 == lineBreak ) {
        for ( std::size_t i = 0; i < N-1; ++i ) {
            ofs << v[i] << ", ";
        }
    } else {
        for ( std::size_t i = 0; i < N-1; ++i ) {
            ofs << v[i] << ", ";

            if ( 0 == (i+1)%lineBreak && i != 0 ) {
                ofs << std::endl << indent;
            }
        }
    }

    ofs << v[N-1] << " ]";
}

template < typename T >
static void write_array_json(std::ofstream& ofs,
        const std::string& name, const T* a, int n,
        int lineBreak=0, const std::string& indent="    ") {
    ofs << "\"" << name << "\": [ ";

    if ( 0 == lineBreak ) {
        for ( std::size_t i = 0; i < n-1; ++i ) {
            ofs << a[i] << ", ";
        }
    } else {
        for ( std::size_t i = 0; i < n-1; ++i ) {
            ofs << a[i] << ", ";

            if ( 0 == (i+1)%lineBreak && i != 0 ) {
                ofs << std::endl << indent;
            }
        }
    }

    ofs << a[n-1] << " ]";
}

void HBDetector::write_disjoint_sets_and_normal_as_json( const std::string& fn ) {
    const std::string TAB = "    "; // 4-space tab character.
    const std::string TAB3 = TAB + TAB + TAB;
    const std::size_t N = disjointBoundaryCandidates.size();

    std::ofstream ofs(fn);

    if ( !ofs.good() ) {
        std::stringstream ss;
        ss << fn << " is not good. ";
        throw( std::runtime_error( ss.str() ) );
    }

    // The first line.
    ofs << "{" << std::endl;

    // The root element.
    ofs << "\"disjointSets\": [" << std::endl;

    for ( std::size_t i = 0; i < N; ++i ) {
        const P_t point = pEquivalentNormal->at(i);

        ofs << TAB << "{" << std::endl;

        ofs << TAB << TAB << "\"id\": " << i << "," << std::endl;

        ofs << TAB << TAB;
        write_array_json(ofs, "centroid", point.data, 3);
        ofs << "," << std::endl << TAB << TAB;
        write_array_json(ofs, "normal", point.normal, 3);
        ofs << "," << std::endl << TAB << TAB;
        write_array_json(ofs, "indices", disjointBoundaryCandidates[i], 10, TAB3);

        if ( i == N - 1 ) {
            ofs << std::endl << TAB << "}" << std::endl;
        } else {
            ofs << std::endl << TAB << "}," << std::endl;
        }
    }

    // The last line.
    ofs << "]}" << std::endl;

    ofs.close();
}