//
// Created by yaoyu on 3/27/20.
//

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include "Args/Args.hpp"
#include "PCCommon/IO.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/SimpleTime.hpp"

// Namespaces.
namespace bpo = boost::program_options;
namespace bpt = boost::posix_time;

class Args
{
public:
    Args() = default;

    ~Args() = default;

    bool validate() {
        bool flag = true;

        if ( tolerance <= 0 ) {
            flag = false;
            std::cout << "Tolerance must be positive. " << std::endl;
        }

        if ( minSize <= 0 ) {
            flag = false;
            std::cout << "The minimum size must be positive. " << std::endl;
        }

        if ( minSize >= maxSize ) {
            flag = false;
            std::cout << "The maximum size must be larger than the minimum size. " << std::endl;
        }

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_CLOUD << ": " << args.inFile << std::endl;
        out << Args::AS_OUT_FILE << ": " << args.outFile << std::endl;
        out << Args::AS_TOLERANCE << ": " << args.tolerance << std::endl;
        out << Args::AS_MIN_SIZE << ": " << args.minSize << std::endl;
        out << Args::AS_MAX_SIZE << ": " << args.maxSize << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_CLOUD; // AS stands for argument string
    static const std::string AS_OUT_FILE;
    static const std::string AS_TOLERANCE;
    static const std::string AS_MIN_SIZE;
    static const std::string AS_MAX_SIZE;

public:
    std::string inFile; // The input point cloud file.
    std::string outFile; // The output directory.
    double tolerance;
    int minSize;
    int maxSize;
};

const std::string Args::AS_IN_CLOUD   = "infile";
const std::string Args::AS_OUT_FILE  = "outfile";
const std::string Args::AS_TOLERANCE = "tolerance";
const std::string Args::AS_MIN_SIZE  = "min-size";
const std::string Args::AS_MAX_SIZE  = "max-size";

static void parse_args(int argc, char* argv[], Args& args) {

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                (Args::AS_IN_CLOUD.c_str(), bpo::value< std::string >(&args.inFile)->required(), "Input file.")
                (Args::AS_OUT_FILE.c_str(), bpo::value< std::string >(&args.outFile)->required(), "Output file.")
                (Args::AS_TOLERANCE.c_str(), bpo::value< double >(&args.tolerance)->required(), "The tolerance.")
                (Args::AS_MIN_SIZE.c_str(), bpo::value< int >(&args.minSize)->required(), "Cluster smaller than this size will be removed.")
                (Args::AS_MAX_SIZE.c_str(), bpo::value< int >(&args.maxSize)->required(), "Maximum cluster size.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(
                Args::AS_IN_CLOUD.c_str(), 1
                ).add(Args::AS_OUT_FILE.c_str(), 1
                ).add(Args::AS_TOLERANCE.c_str(), 1
                ).add(Args::AS_MIN_SIZE.c_str(), 1
                ).add(Args::AS_MAX_SIZE.c_str(), 1);

        bpo::variables_map optVM;
        bpo::store(bpo::command_line_parser(argc, argv).
                options(optDesc).positional(posOptDesc).run(), optVM);
        bpo::notify(optVM);
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        throw(e);
    }

    if ( !args.validate() ) {
        std::cout << args << std::endl;

        std::stringstream ss;
        ss << "Argument validation failed. ";
        throw( std::runtime_error( ss.str() ) );
    }
}

template < typename T >
static void append_2_vector( const std::vector<T>& from, std::vector<T>& to ) {
    const auto nFrom = from.size();

    if ( 0 == nFrom ) {
        return;
    }

    const auto nTo = to.size();

    to.resize( nTo + nFrom );

    std::copy( from.begin(), from.end(), to.begin() + nTo );
}

template < typename pT >
static void process( const typename pcl::PointCloud<pT>::Ptr& pInput,
        double tolerance, int minSize, int maxSize,
        typename pcl::PointCloud<pT>::Ptr& pOutput ) {
    QUICK_TIME_START(te)

    assert( tolerance > 0 );
    assert( minSize > 0 );
    assert( maxSize > minSize );

    // Tree.
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree ( new pcl::search::KdTree<pcl::PointXYZRGB> );

    // The indices.
    std::vector< pcl::PointIndices > clusterIndices;

    // The extractor.
    pcl::EuclideanClusterExtraction<pT> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pInput);
    ec.extract(clusterIndices);

    const auto N = clusterIndices.size();

    if ( 0 == N ) {
        std::stringstream ss;
        ss << "No clusters found. "
           << "tolerance: " << tolerance << ", "
           << "minSize: " << minSize << ", "
           << "maxSize: " << maxSize << ". ";
        throw( std::runtime_error( ss.str() ) );
    }

    pcl::PointIndices::Ptr indices ( new pcl::PointIndices );

    for ( const auto& ids : clusterIndices ) {
        if ( ids.indices.size() <= minSize ) {
            append_2_vector( ids.indices, indices->indices );
        }
    }

    std::cout << N << " small clusters with less than " << minSize << " points found. "
              << indices->indices.size() << " points to be removed. " << std::endl;

    // Remove the points.
    pcl::ExtractIndices<pT> extract;
    extract.setInputCloud( pInput );
    extract.setIndices( indices );
    extract.setNegative( true );
    extract.filter( *pOutput );

    QUICK_TIME_END(te)

    std::cout << "Filter small clusters in " << te << "ms. " << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "Hello, ClusterFilter!" << std::endl;

    QUICK_TIME_START(te)

    // Handle the command line.
    Args args;
    parse_args(argc, argv, args);

    std::cout << "args: " << std::endl;
    std::cout << args << std::endl;

    // Load the input point cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pInput ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcu::read_point_cloud<pcl::PointXYZRGB>( args.inFile, pInput );

    // The output directory.
    test_directory_by_filename(args.outFile);

    // Process.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOutput ( new pcl::PointCloud<pcl::PointXYZRGB> );
    process<pcl::PointXYZRGB>( pInput, args.tolerance, args.minSize, args.maxSize, pOutput );

    // Save the output point cloud.
    pcu::write_point_cloud<pcl::PointXYZRGB>( args.outFile, pOutput );

    QUICK_TIME_END(te)

    std::cout << "ClusterFilter in " << te << "ms. " << std::endl;

    return 0;
}