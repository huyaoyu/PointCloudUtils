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
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "Args/Args.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/SimpleTime.hpp"

// Namespaces.
namespace bpo = boost::program_options;
namespace bpt = boost::posix_time;

/**
 * This function is copied from
 * https://www.boost.org/doc/libs/1_60_0/libs/program_options/example/options_description.cpp
 *
 * @tparam T
 * @param os
 * @param v
 * @return
 */
template<class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
    return os;
}

class Args
{
public:
    Args(): polyOrder(2), radius(0.05) {}
    ~Args() = default;

    bool validate(void) {
        bool flag = true;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << args.AS_IN_FILE << ": " << args.inFile << std::endl;
        out << args.AS_OUT_FILE << ": " << args.outFile << std::endl;
        out << args.AS_POLY_ORDER << ": " << args.polyOrder << std::endl;
        out << args.AS_RADIUS << ": " << args.radius << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_FILE; // AS stands for argument string
    static const std::string AS_OUT_FILE;
    static const std::string AS_POLY_ORDER;
    static const std::string AS_RADIUS;

public:
    std::string inFile; // The input point cloud file.
    std::string outFile; // The output file.
    int polyOrder; // The order of the polynomial.
    double radius; // The search radius.
};

const std::string Args::AS_IN_FILE    = "infile";
const std::string Args::AS_OUT_FILE   = "outfile";
const std::string Args::AS_POLY_ORDER = "poly-order";
const std::string Args::AS_RADIUS     = "radius";

static void parse_args(int argc, char* argv[], Args& args) {

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_FILE.c_str(), bpo::value< std::string >(&(args.inFile))->required(), "Input file.")
                (Args::AS_OUT_FILE.c_str(), bpo::value< std::string >(&(args.outFile))->required(), "Output file.")
                (Args::AS_POLY_ORDER.c_str(), bpo::value< int >(&args.polyOrder)->default_value(2), "The order of the polynomial.")
                (Args::AS_RADIUS.c_str(), bpo::value<double>(&args.radius)->default_value(0.05), "The search radius.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(Args::AS_IN_FILE.c_str(), 1).add(Args::AS_OUT_FILE.c_str(), 1);

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
}

template <typename T>
static void read_point_cloud(const std::string& fn, typename pcl::PointCloud<T>::Ptr& pOutCloud) {
    // ========== Read the point cloud from the file. ==========
    std::cout << "Loading points from " << fn << " ... " << std::endl;

    QUICK_TIME_START(teReadPointCloud);
    if ( pcl::io::loadPLYFile<T>(fn, *pOutCloud) == -1 ) {
        std::stringstream ss;
        ss << "Failed to read: " << fn;
        throw( std::runtime_error(ss.str()) );
    }
    QUICK_TIME_END(teReadPointCloud);

    std::cout << pOutCloud->size() << " points loaded in " << teReadPointCloud << "ms. " << std::endl;
}

template <typename T>
static void compute_normal(const typename pcl::PointCloud<T>::Ptr& pInput,
        pcl::PointCloud<pcl::Normal>::Ptr& pOutput,
        const double radius=0.05) {
    QUICK_TIME_START(te);

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

    QUICK_TIME_END(te);

    std::cout << "Compute normal in " << te << "ms. " << std::endl;
}

template <typename inT, typename outT>
static void moving_least_square(const typename pcl::PointCloud<inT>::Ptr& pInput,
        typename pcl::PointCloud<outT>::Ptr& pOutput,
        const int order, const double radius, const bool flagNormal=true ) {
    QUICK_TIME_START(te);

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

    QUICK_TIME_END(te);

    std::cout << "Moving least square in " << te << "ms. " << std::endl;
}

int main(int argc, char* argv[]) {
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

    // Compute normal.
//    pcl::PointCloud<pcl::Normal>::Ptr pNormal (new pcl::PointCloud<pcl::Normal>);
//    compute_normal<P_t>(pInput, pNormal, args.radius);
//
//    // Copy point cloud.
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pXYZ (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::copyPointCloud(*pInput, *pXYZ);
//
//    // Concatenate fields.
//    pcl::PointCloud<pcl::PointNormal>::Ptr pPN ( new pcl::PointCloud<pcl::PointNormal> );
//    pcl::concatenateFields(*pXYZ, *pNormal, *pPN);
//
    pcl::PointCloud<pcl::PointNormal>::Ptr pOutput ( new pcl::PointCloud<pcl::PointNormal> );
//
//    // Moving least square.
//    moving_least_square<pcl::PointNormal, pcl::PointNormal>(pPN, pOutput, args.polyOrder, args.radius*0.5);

    moving_least_square<pcl::PointXYZRGB, pcl::PointNormal>(pInput, pOutput, args.polyOrder, args.radius);

    // Get the file parts.
    auto parts = get_file_parts(args.outFile);

    // Test the output directory.
    test_directory( parts[0] );

    // Save the point cloud.
    QUICK_TIME_START(teWrite);
    pcl::PLYWriter writer;
    std::cout << "Saving the filtered point cloud." << std::endl;
    writer.write(args.outFile, *pOutput, true, false);
    QUICK_TIME_END(teWrite);

    std::cout << "Write point cloud in " << teWrite << " ms. " << std::endl;

    return 0;
}
