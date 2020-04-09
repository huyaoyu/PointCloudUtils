//
// Created by yaoyu on 3/16/20.
//

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

//#include <boost/assert.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <Eigen/Dense>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "Args/Args.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/SimpleTime.hpp"

// Namespaces.
namespace bpo = boost::program_options;
namespace bpt = boost::posix_time;

class Args
{
public:
    Args();
    ~Args() {}

    bool validate(void) {
        bool flag = true;

        bool flagPCType = validate_string_with_trimming(pcType, pcTypeValidList);

        if ( !flagPCType ) {
            std::cout << "Validate pcType: " << pcType << std::endl;
            std::cout << "Valid types are: " << std::endl;
            show_vector_as_list(pcTypeValidList);
        }

        flag = flag && flagPCType;

        return flag;
    }
public:
    static const std::string PC_TYPE_XYZRGB;
    static const std::string PC_TYPE_XYZRGBA;
    static const std::string PC_TYPE_Normal;

public:
    std::string inFile; // A file contains a list of files.
    std::string outFile; // The single output filename.
    std::vector<double> corners; // The two corners of the bounding box. Order: x0, y0, z0, x1, y1, z1.
    std::string pcType; // The type of the point cloud. Supported types are XYZRGB, XYZRGBA, Normal.
    std::vector<std::string> pcTypeValidList;
    std::string inputPrefix; // The prefix of the files listed in "inFile".
    bool flagDownsample;
    std::vector<float> leafSize; // The leaf size of the voxel down-sample filter. 3-element vector.
    bool flagIslandFilter;
    float islandRadius;
};

const std::string Args::PC_TYPE_XYZRGB  = "XYZRGB";
const std::string Args::PC_TYPE_XYZRGBA = "XYZRGBA";
const std::string Args::PC_TYPE_Normal  = "Normal";

Args::Args()
: pcType(PC_TYPE_XYZRGB), pcTypeValidList({PC_TYPE_XYZRGB, PC_TYPE_XYZRGBA, PC_TYPE_Normal}),
  flagDownsample(false)
{ }

void parse_args(int argc, char* argv[], Args& args) {
    std::string bBoxString; // Temporary string storing the command line arguments.
    std::string leafSizeString; // Temporary string storing the leaf size.

    try
    {
        bpo::options_description optDesc("Remove outliers and smooth a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                ("infile", bpo::value< std::string >(&(args.inFile))->required(), "Input file.")
                ("outfile", bpo::value< std::string >(&(args.outFile))->required(), "Output file.")
                ("pc-type", bpo::value< std::string >(&args.pcType)->default_value("XYZRGB"), "The Type of the point cloud. Currently support: XYZRGB, XYZRGBA, Normal.")
                ("input-prefix", bpo::value< std::string >(&args.inputPrefix)->default_value("."), "The prefix of the input files listed in the input file.")
                ("bbox", bpo::value< std::string >(&bBoxString), "x0, y0, z0, x1, y1, z1")
                ("down-sample", bpo::value< int >()->implicit_value(1), "Set this flag to enable down-sample.")
                ("leaf-size", bpo::value< std::string >(&leafSizeString)->default_value("0.1, 0.1, 0.1"), "leaf size")
                ("island-filter", bpo::value< bool >()->implicit_value(false), "Set this flag to enable island-filter.")
                ("island-radius", bpo::value< float >(&args.islandRadius)->default_value(0.05), "The radius of island filter.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add("infile", 1).add("outfile", 1).add("bbox", 1);

        bpo::variables_map optVM;
        bpo::store(bpo::command_line_parser(argc, argv).
                options(optDesc).positional(posOptDesc).run(), optVM);
        bpo::notify(optVM);

        if ( optVM.count("down-sample") ) {
            args.flagDownsample = true;
        } else {
            args.flagDownsample = false;
        }

        if ( optVM.count("island-filter") ) {
            args.flagIslandFilter = true;
        } else {
            args.flagIslandFilter = false;
        }
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        throw(e);
    }

    // Extract the coordinates for the corners.
    args.corners = extract_number_from_string<double>(bBoxString, 6);

    // Extract the numbers for the leaf size.
    args.leafSize = extract_number_from_string<float>(leafSizeString, 3);

    // Validate the point cloud type specification.
    if ( !args.validate() ) {
        std::stringstream ss;
        ss << "args.validate() returns false. ";
        throw(std::runtime_error(ss.str()));
    }
}

template < typename pT, typename rT>
static void down_sample( const typename pcl::PointCloud<pT>::Ptr& pInput,
                         typename pcl::PointCloud<pT>::Ptr& pOutput,
                         const std::vector<rT>& leafSize,
                         rT leafSizeFactor=1.0) {
    QUICK_TIME_START(teFilter);
    pcl::VoxelGrid<pT> sor;
    sor.setLeafSize(leafSize[0]*leafSizeFactor, leafSize[1]*leafSizeFactor, leafSize[2]*leafSizeFactor);
    sor.setInputCloud(pInput);
    sor.filter(*pOutput);
    QUICK_TIME_END(teFilter);

    std::cout << "Down-sampled to " << pOutput->size() << " points. " << std::endl;
    std::cout << "Voxel filter in " << teFilter << "ms. " << std::endl;
}

template <typename T>
static void read_downsample(const std::string& fn, const std::vector<float>& leafSize,
        typename pcl::PointCloud<T>::Ptr& pOutCloud, const bool flagDownsample=false) {
    // ========== Read the point cloud from the file. ==========
    typename pcl::PointCloud<T>::Ptr pInput( new pcl::PointCloud<T> );

    std::cout << "Loading points from " << fn << " ... " << std::endl;

    QUICK_TIME_START(teReadPointCloud);
    if ( pcl::io::loadPLYFile<T>(fn, *pInput) == -1 ) {
        std::stringstream ss;
        ss << "Failed to read: " << fn;
        throw( std::runtime_error(ss.str()) );
    }
    QUICK_TIME_END(teReadPointCloud);

    std::cout << pInput->size() << " points loaded in " << teReadPointCloud << "ms. " << std::endl;

    if ( flagDownsample ) {
        // ========== Down-sample by a voxel grid filter. ==========

        down_sample<T, float>(pInput, pOutCloud, leafSize, 1.0);
    } else {
        pOutCloud = pInput;
    }
}

static std::vector<double> get_ordered_corner(const std::vector<double>& corner)
{
    std::vector<double> ordered;

    // First corner point.
    ordered.push_back(std::min( corner[0], corner[3] ));
    ordered.push_back(std::min( corner[1], corner[4] ));
    ordered.push_back(std::min( corner[2], corner[5] ));

    // Second corner point.
    ordered.push_back(std::max( corner[0], corner[3] ));
    ordered.push_back(std::max( corner[1], corner[4] ));
    ordered.push_back(std::max( corner[2], corner[5] ));

    return ordered;
}

template <typename T>
static void crop_by_PassThrough( const typename pcl::PointCloud<T>::Ptr& inCloud,
        typename pcl::PointCloud<T>::Ptr& outCloud,
        const std::vector<double>& corners )
{
    // Get the ordered corners.
    auto ordered = get_ordered_corner(corners);

    std::cout << "ordered: " << std::endl;
    show_numeric_vector(ordered);
    std::cout << std::endl;

    // Temporary point cloud.
    typename pcl::PointCloud<T>::Ptr pTempX (new pcl::PointCloud<T>);
    typename pcl::PointCloud<T>::Ptr pTempY (new pcl::PointCloud<T>);

    pcl::PassThrough<T> pass;
    pass.setFilterFieldName( "x" );
    pass.setFilterLimits( ordered[0], ordered[3] );
    pass.setInputCloud(inCloud);
    pass.filter(*pTempX);

    std::cout << "pTempX->size() = " << pTempX->size() << std::endl;

    pass.setFilterFieldName( "y" );
    pass.setFilterLimits( ordered[1], ordered[4] );
    pass.setInputCloud(pTempX);
    pass.filter(*pTempY);

    std::cout << "pTempY->size() = " << pTempY->size() << std::endl;

    pass.setFilterFieldName( "z" );
    pass.setFilterLimits( ordered[2], ordered[5] );
    pass.setInputCloud(pTempY);
    pass.filter(*outCloud);
}

template <typename T>
static void crop_by_CropBox( const typename pcl::PointCloud<T>::Ptr& inCloud,
                  typename pcl::PointCloud<T>::Ptr& outCloud,
                  const std::vector<double>& corners )
{
    QUICK_TIME_START(te);

    // Get the ordered corners.
    auto ordered = get_ordered_corner(corners);

    pcl::CropBox<T> pass;
    pass.setMin( Eigen::Vector4f( ordered[0], ordered[1], ordered[2], 1.0 ) );
    pass.setMax( Eigen::Vector4f( ordered[3], ordered[4], ordered[5], 1.0 ) );
    pass.setInputCloud(inCloud);
    pass.filter(*outCloud);

    QUICK_TIME_END(te);

    std::cout << "Cropped " << outCloud->size() << " points. " << std::endl;
    std::cout << "Crop in " << te << "ms. " << std::endl;
}

template < typename rT >
static rT equivalent_leaf_size(const std::vector<rT>& leafSizes) {
    auto acc = static_cast<rT>(0);

    for ( const auto& s : leafSizes ) {
        acc += s*s;
    }

    return std::sqrt( acc );
}

template < typename pT, typename rT >
static void filter_out_overlapping_points( const typename pcl::PointCloud<pT>::Ptr& pInput,
        typename pcl::PointCloud<pT>::Ptr& pOutput, rT dist ) {
    QUICK_TIME_START(te)

    typename pcl::KdTreeFLANN<pT>::Ptr tree ( new pcl::KdTreeFLANN<pT> );
    tree->setInputCloud(pInput);

    const auto n = pInput->size();

    std::vector<bool> deleteFlags(n);
    for ( int i = 0; i < n; ++i ) {
        deleteFlags[i] = false;
    }

    pcl::PointIndices::Ptr indices (new pcl::PointIndices() );

    std::vector<int> indexR;
    std::vector<float> squaredDistanceR;

    int idx;
    int fn = 0;
    int sn = 0;

    for ( int i = 0; i < n; ++i ) {
        if ( deleteFlags[i] ) {
            continue;
        }

        indexR.clear();
        squaredDistanceR.clear();

        fn = tree->radiusSearch( *pInput, i, dist, indexR, squaredDistanceR );

        sn = indexR.size();

        if ( 0 == fn ) {
            continue;
        }

        for ( int j = 0; j < sn; ++j ) {
            idx = indexR[j];

            if ( idx != i ) {
                deleteFlags[ idx ] = true;
                indices->indices.push_back( idx );
            }
        }
    }

    if ( 0 != indices->indices.size() ) {
        pcl::ExtractIndices<pT> extract;
        extract.setInputCloud( pInput );
        extract.setIndices( indices );
        extract.setNegative( true );
        extract.filter( *pOutput );

        std::cout << "Filtering results in " << pOutput->size() << " points. " << std::endl;
    } else {
        std::cout << "No overlapping points found. " << std::endl;
        pOutput = pInput;
    }

    QUICK_TIME_END(te)

    std::cout << "Filter out overlapping points in " << te << "ms. " << std::endl;
}

static int num_points_in_quater_circle(float radius, float step) {
    assert( step > 0 );
    assert( radius >= step );

    const auto steps = static_cast<int>( std::floor( radius/step ) );

    int n = 0;

    for ( int i = 0; i < steps; ++i ) {
        n += static_cast<int>( std::floor( std::sqrt( radius*radius - (i*step)*(i*step) ) / step ) );
    }

    return n;
}

template < typename pT, typename rT >
static void filter_out_island_points( const typename pcl::PointCloud<pT>::Ptr& pInput,
        typename pcl::PointCloud<pT>::Ptr& pOutput, rT dist, int nLimit ) {
    QUICK_TIME_START(te)

    typename pcl::KdTreeFLANN<pT>::Ptr tree ( new pcl::KdTreeFLANN<pT> );
    tree->setInputCloud(pInput);

    const auto n = pInput->size();

    pcl::PointIndices::Ptr indices (new pcl::PointIndices() );

    std::vector<int> indexR;
    std::vector<float> squaredDistanceR;

    int fn = 0;

    for ( int i = 0; i < n; ++i ) {
        indexR.clear();
        squaredDistanceR.clear();

        fn = tree->radiusSearch( *pInput, i, dist, indexR, squaredDistanceR );

        if ( fn < nLimit ) {
            indices->indices.push_back( i );
        }
    }

    if ( 0 != indices->indices.size() ) {
        pcl::ExtractIndices<pT> extract;
        extract.setInputCloud( pInput );
        extract.setIndices( indices );
        extract.setNegative( true );
        extract.filter( *pOutput );

        std::cout << "Removed island points: " << indices->indices.size() << std::endl;
        std::cout << "Filtering results in " << pOutput->size() << " points. " << std::endl;
    } else {
        std::cout << "No island points found. " << std::endl;
        pOutput = pInput;
    }

    QUICK_TIME_END(te)

    std::cout << "Filter out island points in " << te << "ms. " << std::endl;
}

template <typename T>
static void typed_process(const Args& args) {
    // Read the input file list.
    auto fList = read_file_list(args.inFile);

    typename pcl::PointCloud<T>::Ptr pMerged( new pcl::PointCloud<T> );

    std::cout << "Start processing..." << std::endl << std::endl;

    QUICK_TIME_START(teProcess);

    for ( std::string f : fList ) {
        // Prepare the file name.
        std::string fn = args.inputPrefix + "/" + f;
        std::cout << fn << std::endl;

        // Read and down sample the point cloud.
        typename pcl::PointCloud<T>::Ptr pDownSampled( new pcl::PointCloud<T> );
        read_downsample<T>( fn, args.leafSize, pDownSampled, args.flagDownsample );

        // Crop the point cloud.
        typename pcl::PointCloud<T>::Ptr pCropped( new pcl::PointCloud<T> );
        crop_by_CropBox<T>(pDownSampled, pCropped, args.corners);

        // Merge the point cloud.
        *pMerged += *pCropped;

        // Show loop info.
        std::cout << "Total accumulated points: " << pMerged->size() << ". " << std::endl;
        std::cout << std::endl;
    }

    typename pcl::PointCloud<T>::Ptr pOutput( new pcl::PointCloud<T> );
    typename pcl::PointCloud<T>::Ptr pNonOverlap( new pcl::PointCloud<T> );

    // Filter out the overlapping points.
    float equivalentLeafSize = equivalent_leaf_size(args.leafSize);
    std::cout << "Filter the overlapping points..." << std::endl;
    filter_out_overlapping_points<T>( pMerged, pNonOverlap, equivalentLeafSize / 4.f);

//    down_sample<T, float>( pMerged, pOutput, args.leafSize, 1.0 );

    // Island-filter.
    if ( args.flagIslandFilter ) {
        int nLimit = num_points_in_quater_circle( args.islandRadius, args.leafSize[0] );
        nLimit = static_cast<int>(nLimit * 1.5);
        std::cout << "nLimit for island filter = " << nLimit << std::endl;
        filter_out_island_points<T>(pNonOverlap, pOutput, args.islandRadius, nLimit);
    } else {
        pOutput = pNonOverlap;
    }

    // Save the filtered point cloud.
    QUICK_TIME_START(teWrite);
    pcl::PLYWriter writer;
    std::cout << "Saving the filtered point cloud." << std::endl;
    writer.write(args.outFile, *pOutput, true, false);
    QUICK_TIME_END(teWrite);

    std::cout << "Write point cloud in " << teWrite << " ms. " << std::endl;

    QUICK_TIME_END(teProcess);

    std::cout << "Total time: " << teProcess << "ms. " << std::endl;
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "Hello, CropByBBox!" << std::endl;

    // Handle the command line.
    Args args;
    parse_args(argc, argv, args);

    // Show the corner specification of the bounding box.
    std::cout << "The bounds of the BBox are: " << std::endl;
    show_numeric_vector(args.corners);
    std::cout << std::endl << std::endl;

    // Show the leaf size specification.
    std::cout << "The leaf sizes are: " << std::endl;
    show_numeric_vector(args.leafSize);
    std::cout << std::endl << std::endl;

    // Read the input file list.
    auto fList = read_file_list(args.inFile);

    // Show the content of the file list.
    std::cout << "The input file list is: " << std::endl;
    show_vector_as_list(fList, "\n", args.inputPrefix + "/");
    std::cout << std::endl;

    // Process according to different types.
    if ( args.pcType == Args::PC_TYPE_XYZRGB ) {
        typed_process<pcl::PointXYZRGB>(args);
    } else if ( args.pcType == Args::PC_TYPE_XYZRGBA ) {
        typed_process<pcl::PointXYZRGBA>(args);
    } else if ( args.pcType == Args::PC_TYPE_Normal ) {
        typed_process<pcl::PointNormal>(args);
    } else {
        // Should never be here.
        std::stringstream ss;
        ss << "Unexpected args.pcType: " << args.pcType;
        throw(std::runtime_error(ss.str()));
    }

    return 0;
}