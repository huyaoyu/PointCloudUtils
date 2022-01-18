//
// Created by yaoyu on 3/20/20.
//

//
// Created by yaoyu on 3/20/20.
//

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <Eigen/Dense>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h> // From copyPointCloud().
#include <pcl/common/point_tests.h>
#include <pcl/surface/mls.h>

#include "Args/Args.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/NormalHelpers.hpp"
#include "Profiling/SimpleTime.hpp"

// Namespaces.
namespace bpo = boost::program_options;

class Args
{
public:
    Args();
    ~Args() = default;

    bool validate() {
        bool flag = true;

        bool flagPCType = validate_string_with_trimming(computeType, validComputeTypeList);

        if ( !flagPCType ) {
            std::cout << "Validate computation type: " << computeType << std::endl;
            std::cout << "Valid types are: " << std::endl;
            show_vector_as_list(validComputeTypeList);
        }

        flag = flag && flagPCType;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_CLOUD << ": " << args.inFile << std::endl;
        out << Args::AS_OUT_FILE << ": " << args.outFile << std::endl;
        out << Args::AS_COMPUTE_TYPE << ": " << args.computeType << std::endl;
        out << Args::AS_FLIP_OPPOSITE << ": " << args.flagFlipOpposite << std::endl;
        out << Args::AS_POLY_ORDER << ": " << args.polyOrder << std::endl;
        out << Args::AS_RADIUS << ": " << args.radius << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_CLOUD; // AS stands for argument string
    static const std::string AS_OUT_FILE;
    static const std::string AS_COMPUTE_TYPE;
    static const std::string AS_FLIP_OPPOSITE;
    static const std::string AS_POLY_ORDER;
    static const std::string AS_RADIUS;

    static const std::string VAS_COMPUTE_TYPE_MLS;
    static const std::string VAS_COMPUTE_TYPE_NE;

public:
    std::string inFile; // The input point cloud file.
    std::string outFile; // The output file.
    std::string computeType; // The type of computation performed for estimating the normal.
    bool flagFlipOpposite; // A flag enables opposite flip.
    std::vector<std::string> validComputeTypeList;
    int polyOrder; // The order of the polynomial.
    double radius; // The search radius.
};

const std::string Args::AS_IN_CLOUD      = "infile";
const std::string Args::AS_OUT_FILE      = "outfile";
const std::string Args::AS_COMPUTE_TYPE  = "compute-type";
const std::string Args::AS_FLIP_OPPOSITE = "flip-opposite";
const std::string Args::AS_POLY_ORDER    = "poly-order";
const std::string Args::AS_RADIUS        = "radius";

const std::string Args::VAS_COMPUTE_TYPE_MLS = "MLS";
const std::string Args::VAS_COMPUTE_TYPE_NE  = "NE";

Args::Args()
: polyOrder(2), radius(0.05),
  computeType(VAS_COMPUTE_TYPE_MLS), validComputeTypeList({ VAS_COMPUTE_TYPE_MLS, VAS_COMPUTE_TYPE_NE })
{ }

static void parse_args(int argc, char* argv[], Args& args) {

    int flagFlipValue = 0;

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_CLOUD.c_str(), bpo::value< std::string >(&(args.inFile))->required(), "Input file.")
                (Args::AS_OUT_FILE.c_str(), bpo::value< std::string >(&(args.outFile))->required(), "Output file.")
                (Args::AS_COMPUTE_TYPE.c_str(), bpo::value< std::string >(&args.computeType)->default_value(Args::VAS_COMPUTE_TYPE_MLS), "The computation type, MLS or NE.")
                (Args::AS_FLIP_OPPOSITE.c_str(), bpo::value< int >( &flagFlipValue )->default_value(0)->implicit_value(1), "Set this flag to enable opposite flip. ")
                (Args::AS_POLY_ORDER.c_str(), bpo::value< int >(&args.polyOrder)->default_value(2), "The order of the polynomial.")
                (Args::AS_RADIUS.c_str(), bpo::value<double>(&args.radius)->default_value(0.05), "The search radius.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(Args::AS_IN_CLOUD.c_str(), 1).add(Args::AS_OUT_FILE.c_str(), 1);

        bpo::variables_map optVM;
        bpo::store(bpo::command_line_parser(argc, argv).
                options(optDesc).positional(posOptDesc).run(), optVM);
        bpo::notify(optVM);

        if ( optVM.count(Args::AS_FLIP_OPPOSITE.c_str()) ) {
            args.flagFlipOpposite = true;
        } else {
            args.flagFlipOpposite = false;
        }
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        throw(e);
    }

    // Validate the point cloud type specification.
    if ( !args.validate() ) {
        std::stringstream ss;
        ss << "args.validate() returns false. ";
        throw(std::runtime_error(ss.str()));
    }
}

template <typename T>
static void read_point_cloud(const std::string& fn, typename pcl::PointCloud<T>::Ptr& pOutCloud) {
    // ========== Read the point cloud from the file. ==========
    std::cout << "Loading points from " << fn << " ... " << std::endl;

    QUICK_TIME_START(teReadPointCloud)
    if ( pcl::io::loadPLYFile<T>(fn, *pOutCloud) == -1 ) {
        std::stringstream ss;
        ss << "Failed to read: " << fn;
        throw( std::runtime_error(ss.str()) );
    }
    QUICK_TIME_END(teReadPointCloud)

    std::cout << pOutCloud->size() << " points loaded in " << teReadPointCloud << "ms. " << std::endl;
}

template <typename T>
static void compute_normal(const typename pcl::PointCloud<T>::Ptr& pInput,
        pcl::PointCloud<pcl::Normal>::Ptr& pOutput,
        const double radius=0.05) {
    QUICK_TIME_START(te)

    // Compute the centroid of the point cloud.
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pInput, centroid);

    std::cout << "The centroid point is " << centroid << std::endl;

    // Create a KD-Tree.
    typename pcl::search::KdTree<T>::Ptr tree ( new pcl::search::KdTree<T> );

    // The normal estimator.
    pcl::NormalEstimation<T, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    ne.setInputCloud(pInput);
    ne.setViewPoint( centroid(0), centroid(1), centroid(2) );

    pcl::PointCloud<pcl::Normal>::Ptr temp ( new pcl::PointCloud<pcl::Normal> );
    ne.compute(*pOutput);

    QUICK_TIME_END(te)

    std::cout << "Compute normal in " << te << "ms. " << std::endl;
}

template <typename inT, typename outT>
static void moving_least_square(const typename pcl::PointCloud<inT>::Ptr& pInput,
        typename pcl::PointCloud<outT>::Ptr& pOutput,
        const int order, const double radius, const bool flagNormal=true ) {
    QUICK_TIME_START(te)

    // Create a KD-Tree.
    typename pcl::search::KdTree<inT>::Ptr tree ( new pcl::search::KdTree<inT> );

    // The moving least square object.
    pcl::MovingLeastSquares<inT, outT> mls;
    mls.setComputeNormals(flagNormal);
    mls.setPolynomialOrder(order);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setInputCloud(pInput);

    mls.process(*pOutput);

    QUICK_TIME_END(te)

    std::cout << "Moving least square in " << te << "ms. " << std::endl;
}

int main(int argc, char* argv[]) {
    QUICK_TIME_START(teMain)

    std::cout << "Hello, NormalEstimation!" << std::endl;

    // Handle the command line.
    Args args;
    parse_args(argc, argv, args);
    std::cout << "args: " << std::endl;
    std::cout << args << std::endl;

    // Define the point cloud object.
    typedef pcl::PointXYZRGB P_t;
    typedef pcl::PointCloud<P_t> PC_t;
    PC_t::Ptr pInput(new PC_t);

    // Read the point cloud.
    read_point_cloud<P_t>(args.inFile, pInput);

    // The output.
    pcl::PointCloud<pcl::PointNormal>::Ptr pOutput ( new pcl::PointCloud<pcl::PointNormal> );

    // Compute normal.
    if ( args.computeType == Args::VAS_COMPUTE_TYPE_NE ) {
        pcl::PointCloud<pcl::Normal>::Ptr pNormal (new pcl::PointCloud<pcl::Normal>);
        compute_normal<P_t>(pInput, pNormal, args.radius);

        // Copy point cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr pXYZ (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*pInput, *pXYZ);

        // Concatenate fields.
        pcl::concatenateFields(*pXYZ, *pNormal, *pOutput);
    } else if ( args.computeType == Args::VAS_COMPUTE_TYPE_MLS ) {
        // Moving least square.
        moving_least_square<pcl::PointXYZRGB, pcl::PointNormal>(pInput, pOutput, args.polyOrder, args.radius);

        // Flip the normal.
        // Compute the centroid of the point cloud.
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*pInput, centroid);
        pcu::flip_normal_inplace<pcl::PointNormal>(pOutput, centroid, args.flagFlipOpposite);
    } else {
        // Should never be here.
        std::stringstream ss;
        ss << "Unexpected args.computeType value " << args.computeType;
        throw(std::runtime_error(ss.str()));
    }

    // Get the file parts.
    auto parts = get_file_parts(args.outFile);

    // Test the output directory.
    test_directory( parts[0] );

    // Save the point cloud.
    QUICK_TIME_START(teWrite)
    pcl::PLYWriter writer;
    std::cout << "Saving the filtered point cloud." << std::endl;
    writer.write(args.outFile, *pOutput, true, false);
    QUICK_TIME_END(teWrite)

    std::cout << "Write point cloud in " << teWrite << " ms. " << std::endl;

    QUICK_TIME_END(teMain)

    std::cout << "Total time is " << teMain << "ms. " << std::endl;

    return 0;
}
