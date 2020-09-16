//
// Created by yaoyu on 9/4/20.
//

#include <algorithm> // std::sort(), std::copy().
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "Args/ArgsParser.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/extraction.hpp"
#include "PCCommon/IO.hpp"
#include "Profiling/ScopeTimer.hpp"

// Typedefs.
typedef pcl::PointNormal P_t;
typedef pcl::PointCloud<P_t> PC_t;

// Local class.
template < typename T >
class Buffer {
public:
    explicit Buffer(int n) : s{n} {
        buffer.resize(s);
    }

    ~Buffer() = default;

    void set_size(int n) {
        if ( n > buffer.size() ) {
            std::stringstream ss;
            ss << "buffer.size() = " << buffer.size() << ". It is smaller than the new size " << n << ". ";
            throw std::runtime_error( ss.str() );
        }

        s = n;
    }

    int size() const {
        return s;
    }

public:
    std::vector<T> buffer;
    int s;
};

static ap::Args handle_args( int argc, char** argv ) {
    ap::Args args;

    args.add_positional<std::string>("in-cloud", "Input point cloud with normal. ");
    args.add_positional<std::string>("out-dir", "Output directory. ");
    args.add_positional<std::string>("out-name", "Output filename relative to out-dir. ");
    args.add_default<int>("knn", "The number of KNNs. ", 4);
    args.add_default<std::string>("flip-thres",
                            "A negative number, (-1, 0). If the inner product of the normals is smaller than this threshold, then the test normal will be flipped. ",
                            "-0.75");

    args.parse_args( argc, argv );

    std::cout << args;

    return args;
}

template < typename PC_T >
static Eigen::Vector4f find_centroid( PC_T &pc ) {
    // Returning an Eigen object is not efficient. However, we only do it once.

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid( pc, centroid );
    return centroid;
}

template < typename PC_T >
static std::vector< std::pair< float, int > > get_distance_vector( const PC_T &pc, const Eigen::Vector4f &centroid ) {
    FUNCTION_SCOPE_TIMER

    // Make a copy on the stack.
    const auto cx = centroid(0);
    const auto cy = centroid(1);
    const auto cz = centroid(2);

    // Prepare the container.
    const int N = pc.size();
    std::vector< std::pair< float, int > > distVec;
    distVec.reserve( N );

    for ( int i = 0; i < N; ++i ) {
        const typename PC_T::PointType &point = pc[i];

        distVec.emplace_back(
                (point.x - cx) * (point.x - cx) + ( point.y - cy ) * ( point.y - cy ) + ( point.z - cz ) * ( point.z - cz ),
                i );
    }

    // Sort the container.
    std::sort( distVec.begin(), distVec.end() );

    return distVec;
}

template < typename P_T >
static typename pcl::PointCloud<P_T>::Ptr extract_point_cloud_from_dist_pair_vec(
        const typename pcl::PointCloud<P_T>::Ptr pCloud,
        const std::vector< std::pair< float, int > > &distVec ) {
    // Make a copy of the indices.
    // Because pcl's point cloud extraction method cannot preserve the order of points.
    // We will manually insert the points into a new point cloud.
//    pcl::PointIndices::Ptr indices( new pcl::PointIndices );

//    std::transform( distVec.begin(), distVec.end(), std::back_inserter(indices->indices),
//                    []( const std::pair<float, int> &p ) { return p.second; } );

//    return pcu::extract_points<P_T>( pCloud, indices );

    typename pcl::PointCloud<P_T>::Ptr pOut( new pcl::PointCloud<P_T> );
    for ( const auto &p : distVec ) {
        pOut->push_back( (*pCloud)[ p.second ] );
    }

    return pOut;
}

static std::set<int> create_index_set( const std::vector< std::pair< float, int > > &distVec ) {
    std::set<int> indexSet;
    std::transform( distVec.begin(), distVec.end(), std::inserter( indexSet, indexSet.begin() ),
                    []( const std::pair< float, int > &p ){ return p.second; } );

    return indexSet;
}

static std::set<int> create_index_set(int n) {
    std::set<int> indexSet;
    for ( int i = 0; i < n; ++i ) {
        indexSet.insert(i);
    }
    return indexSet;
}

static void split_neighbors( const std::vector<int> &knnIndices,
                             const std::vector<bool> &flags,
                             Buffer<int> &procN,
                             Buffer<int> &nProcN ) {
    const int N = knnIndices.size();

    int  p = 0;
    int np = 0;

    for ( int i = 0; i < N; ++i ) {
        if ( flags[ knnIndices[i] ] ) {
            procN.buffer[p] = i;
            p++;
        } else {
            nProcN.buffer[np] = i;
            np++;
        }
    }

    procN.set_size(p);
    nProcN.set_size(np);
}

static void average_normal_from_neighbors(
        const PC_t &pc,
        const std::vector<int> &knnIndices,
        const Buffer<int> &procN,
        P_t &avgNormal ) {
    avgNormal.normal_x  = 0;
    avgNormal.normal_y  = 0;
    avgNormal.normal_z  = 0;
    avgNormal.curvature = 0;

    const int N = procN.size();
    for ( int i = 0; i < N; ++i ) {
        const auto &point = pc[ knnIndices[ procN.buffer[i] ] ];
        avgNormal.normal_x  += point.normal_x;
        avgNormal.normal_y  += point.normal_y;
        avgNormal.normal_z  += point.normal_z;
        avgNormal.curvature += point.curvature;
    }

    avgNormal.normal_x  = avgNormal.normal_x  / N;
    avgNormal.normal_y  = avgNormal.normal_y  / N;
    avgNormal.normal_z  = avgNormal.normal_z  / N;
    avgNormal.curvature = avgNormal.curvature / N;
}

static void compare_and_flip_normal( const P_t &reference, P_t &source, float flipThres ) {
    if ( reference.normal_x * source.normal_x +
         reference.normal_y * source.normal_y +
         reference.normal_z * source.normal_z < flipThres ) {
        // Flip.
        source.normal_x = -source.normal_x;
        source.normal_y = -source.normal_y;
        source.normal_z = -source.normal_z;
    }
}

static int flip_non_processed_neighbors(
        PC_t &pc,
        const std::vector<int> &knnIndices,
        const std::vector<float> &knnSqdDist,
        const Buffer<int> &nProcN,
        const P_t &targetNormal,
        std::vector<bool> &flags,
        float flipThres ) {
    float maxDist = 0;
    int maxDistGlobalIdx = knnIndices[nProcN.buffer[0]];

    const int N = nProcN.size();

    for ( int i = 0; i < N; ++i ) {
        const int localIdx = nProcN.buffer[i];
        const int globalIdx = knnIndices[localIdx];
        if ( flags[globalIdx] ) {
            // This point is processed.
            continue;
        }

        P_t &point = pc[globalIdx];
        compare_and_flip_normal( targetNormal, point, flipThres );

        // Update maxDist.
        const float sqdDist = knnSqdDist[localIdx];

        if ( sqdDist > maxDist ) {
            maxDistGlobalIdx = globalIdx;
            maxDist = sqdDist;
        }

        // Update the flags.
        flags[globalIdx] = true;
    }

    return maxDistGlobalIdx;
}

/**
 * This function flip the normals of a point cloud. The first point of the point cloud
 * is used as the seeding point. It is assumed that the point cloud is already sorted
 * based on the distance to a specific point. The sort is done in ascending order.
 *
 * @p pc will be changed after calling this function.
 *
 * @param pc The point cloud.
 * @param knn Number of KNNs.
 * @param flipThres The threshold for flipping. Negative (-1, 0).
 */
static void flip_all_normals(
        PC_t::Ptr &pPC, int knn=4, float flipThres=-0.75) {
    // Create an index set.
    auto remainingIndices = create_index_set(pPC->size());

    // Create a flag vector.
    std::vector<bool> flags( pPC->size(), false );

    // Prepare the kdTree.
    pcl::KdTreeFLANN<P_t>::Ptr tree ( new pcl::KdTreeFLANN<P_t> );
    tree->setInputCloud(pPC);

    // Prepare the containers for KNN search.
    std::vector<int> knnIndices(knn);
    std::vector<float> knnSqdDist(knn);

    // Prepare the index buffer.
    Buffer<int> procN(knn); // Processed neighbors local index including the current point.
    Buffer<int> nProcN(knn); // Non-processed neighbors local index including the current point.

    // Target normal.
    P_t targetNormal;

    // Loop.
    auto currentIdx = *(remainingIndices.begin());
    while ( !remainingIndices.empty() ) {
        // Find the KNNs of currentIdx.
        P_t currentPoint = (*pPC)[currentIdx];
        int fn = tree->nearestKSearch( currentPoint, knn, knnIndices, knnSqdDist );
        if ( fn == 0 ) {
            std::stringstream ss;
            ss << "Search at " << (*pPC)[currentIdx] << " returns zero points.";
            throw std::runtime_error( ss.str() );
        }

        // Check if there are points that are already processed.
        split_neighbors( knnIndices, flags, procN, nProcN );

        if ( procN.size() != 0 ) {
            // Have processed points in the neighborhood.
            if ( procN.size() == knn ) {
                // All points in the neighborhood are processed.
                currentIdx = *(remainingIndices.begin());
                continue;
            } else {
                // Some of the points in the neighborhood are processed.
                average_normal_from_neighbors( *pPC, knnIndices, procN, targetNormal );
            }
        } else {
            // No processed points in the neighborhood.
            targetNormal = currentPoint;
        }

        // Flip.
        const int farthermostIdx =
                flip_non_processed_neighbors( *pPC, knnIndices, knnSqdDist, nProcN, targetNormal, flags, flipThres );

        // Update remainingIndices.
        const int numNProcN = nProcN.size();
        for ( int i = 0; i < numNProcN; ++i ) {
            remainingIndices.erase( knnIndices[ nProcN.buffer[i] ] );
        }

        // Select the currentIdx.
        currentIdx = farthermostIdx;
    }
}

int main( int argc, char **argv ) {
    NAMED_SCOPE_TIMER(main)

    std::cout << "Hello, Tool_FlipNormal! \n";
    auto args = handle_args( argc, argv );

    std::string inCloudFn = args.arguments<std::string>["in-cloud"]->get();
    std::string outDir    = args.arguments<std::string>["out-dir"]->get();
    std::string outFn     = args.arguments<std::string>["out-name"]->get();
    const int knn         = args.arguments<int>["knn"]->get();
    std::string flipThresStr = args.arguments<std::string>["flip-thres"]->get();
    float flipThres;
    {
        std::stringstream ss;
        ss << flipThresStr;
        ss >> flipThres;
    }

    assert( flipThres < 0 && flipThres > -1 );

    test_directory( outDir );

    // Load the input point cloud.
    auto pInCloud = pcu::read_point_cloud<P_t>(inCloudFn);

    // The centroid.
    auto centroid = find_centroid( *pInCloud );
    std::cout << "centroid = \n" << centroid << "\n";

    // Sorted distance and indices.
    auto distVec = get_distance_vector( *pInCloud, centroid );

    // Extract new point cloud.
    auto pExtracted = extract_point_cloud_from_dist_pair_vec<P_t>( pInCloud, distVec );

//    // Test use.
//    {
//        std::stringstream ss;
//        ss << outDir << "/FlipTest.ply";
//        pcu::write_point_cloud<P_t>( ss.str(), pExtracted );
//    }

//    // Test use.
//    const int N = pInCloud->size();
//    std::cout << "pInCloud->at(distVec[N-1].second) = " << pInCloud->at( distVec[N-1].second ) << "\n";
//    std::cout << "pExtracted->at(N-1) = " << pExtracted->at(N-1) << "\n";

    // Flip normals.
    flip_all_normals( pExtracted, knn, flipThres );

    // Save the flipped point cloud.
    {
        std::stringstream ss;
        ss << outDir << "/" << outFn;
        pcu::write_point_cloud<P_t>(ss.str(), pExtracted);
    }

    return 0;
}