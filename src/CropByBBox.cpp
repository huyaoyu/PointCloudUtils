//
// Created by yaoyu on 3/16/20.
//

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
//#include <boost/assert.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <Eigen/Dense>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

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

typedef struct Args {
    std::string inFile;
    std::string outFile;
    std::vector<double> coners;
    std::string inputPrefix;
} Args_t;

void parse_args(int argc, char* argv[], Args_t& args) {
    std::string bBoxString;

    try
    {
        bpo::options_description optDesc("Remove outliers and smooth a point cloud.");

        optDesc.add_options()
                ("help", "produce help message")
                ("infile", bpo::value< std::string >(&(args.inFile))->required(), "input file")
                ("outfile", bpo::value< std::string >(&(args.outFile))->required(), "input file")
                ("input-prefix", bpo::value< std::string >(&args.inputPrefix)->default_value("."), "The prefix of the input files listed in the input file.")
                ("bbox", bpo::value< std::string >(&bBoxString), "x0, y0, z0, x1, y1, z1");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add("infile", 1).add("outfile", 1).add("bbox", 1);

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

    // Extract the leaf size;
    args.coners = extract_number_from_string<double>(bBoxString, 6);
}

static std::vector<double> get_ordered_corner(const std::vector<double>& corner)
{
    std::vector<double> ordered;
    ordered.push_back(std::min( corner[0], corner[3] ));
    ordered.push_back(std::min( corner[1], corner[4] ));
    ordered.push_back(std::min( corner[2], corner[5] ));

    ordered.push_back(std::max( corner[0], corner[3] ));
    ordered.push_back(std::max( corner[1], corner[4] ));
    ordered.push_back(std::max( corner[2], corner[5] ));

    return ordered;
}

template <typename T>
static void crop( const typename pcl::PointCloud<T>::Ptr& inCloud,
        typename pcl::PointCloud<T>::Ptr& outCloud,
        const std::vector<double>& corners )
{
    // Get the ordered corners.
    auto ordered = get_ordered_corner(corners);

    std::cout << "ordered: " << std::endl;
    for ( double c : ordered ) {
        std::cout << c << ", ";
    }
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
    // Get the ordered corners.
    auto ordered = get_ordered_corner(corners);

    pcl::CropBox<T> pass;
    pass.setMin( Eigen::Vector4f( ordered[0], ordered[1], ordered[2], 1.0 ) );
    pass.setMax( Eigen::Vector4f( ordered[3], ordered[4], ordered[5], 1.0 ) );
    pass.setInputCloud(inCloud);
    pass.filter(*outCloud);
}

int main(int argc, char* argv[]) {
    std::cout << "Hello, PCL!" << std::endl;

    Args_t args;

    parse_args(argc, argv, args);

    std::cout << "The bounds of the BBox are: " << std::endl;
    for (double c : args.coners) {
        std::cout << c << ", ";
    }
    std::cout << std::endl << std::endl;

    // Read the input file list.
    auto fList = read_file_list(args.inFile);

    std::cout << "The input file list is: " << std::endl;
    for ( std::string f : fList ) {
        std::cout << args.inputPrefix << "/" << f << std::endl;
    }
    std::cout << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pInput( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOutput( new pcl::PointCloud<pcl::PointXYZRGB> );

    std::string fn = args.inputPrefix + "/" + fList[0];

    std::cout << "Loading points from " << fn << " ... " << std::endl;

    QUICK_TIME_START(teReadPointCloud);

    if ( pcl::io::loadPLYFile<pcl::PointXYZRGB>(fn, *pInput) == -1 ) {
        std::cerr << "Failed to read: " << fn << std::endl;
        return -1;
    }

    QUICK_TIME_END(teReadPointCloud);

    std::cout << "Load " << pInput->size() << " points from " << fn
              << " in " << teReadPointCloud << " ms." << std::endl;

    QUICK_TIME_START(teCrop);

    crop_by_CropBox<pcl::PointXYZRGB>(pInput, pOutput, args.coners);

    QUICK_TIME_END(teCrop);

    std::cout << "Crop in " << teCrop << " ms. " << std::endl;
    std::cout << "Point cloud contains " << pOutput->size() << " points." << std::endl;

    // Save the filtered point cloud.
    QUICK_TIME_START(teWrite);
    pcl::PLYWriter writer;
    std::cout << "Saving the filtered point cloud." << std::endl;
    writer.write(args.outFile, *pOutput, true, false);
    QUICK_TIME_END(teWrite);

    std::cout << "Write point cloud in " << teWrite << " ms." << std::endl;

    return 0;
}