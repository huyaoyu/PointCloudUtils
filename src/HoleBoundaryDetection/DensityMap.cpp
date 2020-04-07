//
// Created by yaoyu on 4/5/20.
//

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "Args/Args.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/IO.hpp"
#include "Visualization/tinycolormap/include/tinycolormap.hpp"

// Namespaces.
namespace bpo = boost::program_options;

class Args
{
public:
    Args() = default;

    ~Args() = default;

    bool validate() {
        bool flag = true;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_MVS << ": " << args.inMVS << std::endl;
        out << Args::AS_IN_LIDAR << ": " << args.inLiDAR << std::endl;
        out << Args::AS_OUT_FILE << ": " << args.outFile << std::endl;
        out << Args::AS_RADIUS << ": " << args.radius << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_MVS; // AS stands for argument string
    static const std::string AS_IN_LIDAR;
    static const std::string AS_OUT_FILE;
    static const std::string AS_RADIUS;

public:
    std::string inMVS; // The target point cloud.
    std::string inLiDAR; // The reference point cloud.
    std::string outFile; // The output file.
    float radius; // The search radius of the kd-tree.
};

const std::string Args::AS_IN_MVS = "inMVS";
const std::string Args::AS_IN_LIDAR = "inLiDAR";
const std::string Args::AS_OUT_FILE = "outFile";
const std::string Args::AS_RADIUS = "radius";

static void parse_args(int argc, char* argv[], Args& args) {

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_MVS.c_str(), bpo::value< std::string >(&args.inMVS)->required(), "The input MVS point cloud.")
                (Args::AS_IN_LIDAR.c_str(), bpo::value< std::string >(&args.inLiDAR)->required(), "The input LiDAR point cloud.")
                (Args::AS_OUT_FILE.c_str(), bpo::value< std::string >(&args.outFile)->required(), "The output file.")
                (Args::AS_RADIUS.c_str(), bpo::value< float >(&args.radius)->required(), "The radius for the kd-tree");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(Args::AS_IN_MVS.c_str(), 1
        ).add(Args::AS_IN_LIDAR.c_str(), 1
        ).add(Args::AS_OUT_FILE.c_str(), 1);

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

template < typename pT0, typename pT1 >
static pcl::PointCloud<pcl::PointXYZI>::Ptr create_density_map(
        const typename pcl::PointCloud<pT0>::Ptr p0,
        const typename pcl::PointCloud<pT1>::Ptr p1,
        float radius ) {
    QUICK_TIME_START(te)

    assert( radius > 0 );

    // Create the kd-tree for p0.
    typename pcl::KdTreeFLANN<pT0> tree;
    tree.setInputCloud(p0);

    // Create the output XYZI point cloud.
    pcl::PointCloud<pcl::PointXYZI>::Ptr pOutput ( new pcl::PointCloud<pcl::PointXYZI> );
    const std::size_t N = p1->size();
    pOutput->resize( p1->size() );

    // The vairables holding the output of the search.
    std::vector<int> idxSearch;
    std::vector<float> squaredDist;

    // Loop over all the point in p1.
    for ( std::size_t i = 0; i< N; ++i) {
        pcl::PointXYZ point;
        pT1 &point1 = p1->at(i);
        point.x = point1.x;
        point.y = point1.y;
        point.z = point1.z;

        int n = tree.radiusSearch( point, radius, idxSearch, squaredDist );

        pcl::PointXYZI outPoint;
        outPoint.x = point.x;
        outPoint.y = point.y;
        outPoint.z = point.z;
        outPoint.intensity = static_cast<float>(n);
        pOutput->at(i) = outPoint;
    }

    QUICK_TIME_END(te)

    std::cout << "Create density map in " << te << " ms. " << std::endl;

    return pOutput;
}

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr convert_intensity_2_rgb(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr pInput,
        float minVal, float maxVal ) {

    assert( minVal < maxVal );

    const std::size_t N = pInput->size();
    const float span = maxVal - minVal;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pRGB ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pRGB->resize(N);

    for ( std::size_t i = 0; i < N; ++i ) {
        const pcl::PointXYZI &point = pInput->at(i);
        float value = ( point.intensity - minVal ) / span;
        if ( value > 1.0f ) {
            value = 1.0f;
        } else if ( value < 0.0f ) {
            value = 0.0f;
        }

        const tinycolormap::Color color = tinycolormap::GetColor( value, tinycolormap::ColormapType::Heat );

        pcl::PointXYZRGB xyzrgb;
        xyzrgb.x = point.x;
        xyzrgb.y = point.y;
        xyzrgb.z = point.z;
        xyzrgb.r = static_cast<uint8_t >(color.r()*255);
        xyzrgb.g = static_cast<uint8_t >(color.g()*255);
        xyzrgb.b = static_cast<uint8_t >(color.b()*255);

        pRGB->at(i) = xyzrgb;
    }

    return pRGB;
}

int main( int argc, char* argv[] ) {
    QUICK_TIME_START(te)

    std::cout << "Hello, DensityMap! " << std::endl;

    // Handle the command line.
    Args args;
    parse_args(argc, argv, args);

    std::cout << "args: " << std::endl;
    std::cout << args << std::endl;

    // Load the two point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pMVS =
            pcu::read_point_cloud<pcl::PointXYZ>( args.inMVS );

    pcl::PointCloud<pcl::PointXYZ>::Ptr pLiDAR =
            pcu::read_point_cloud<pcl::PointXYZ>( args.inLiDAR );

    // Build the density map.
    pcl::PointCloud<pcl::PointXYZI>::Ptr pOutput =
            create_density_map<pcl::PointXYZ, pcl::PointXYZ>( pMVS, pLiDAR, args.radius );

    // Save the point cloud.
    test_directory_by_filename(args.outFile);

    pcu::write_point_cloud<pcl::PointXYZI>( args.outFile, pOutput );

    // Convert the intensity point cloud into RGB representation.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pRGB = convert_intensity_2_rgb( pOutput, 0, 30 );
    std::vector<std::string> parts = get_file_parts(args.outFile);
    std::string outRGB = parts[0] + "/" + parts[1] + "_RGB" + parts[2];
    pcu::write_point_cloud<pcl::PointXYZRGB>(outRGB, pRGB);

    QUICK_TIME_END(te)

    std::cout << "Build density map in " << te << " ms. " << std::endl;

    return 0;
}