#include <iostream>
#include <sstream>
#include <string>
#include <vector>

//#include <boost/assert.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "Args/Args.hpp"

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
    std::vector<float> leafSize;
        } Args_t;

void parse_args(int argc, char* argv[], Args_t& args) {
    std::string leafSizeStr;

    try
    {
        bpo::options_description optDesc("Down-sample a point cloud with a voxel filter.");

        optDesc.add_options()
                ("help", "produce help message")
                ("infile", bpo::value< std::string >(&(args.inFile))->required(), "input file")
                ("outfile", bpo::value< std::string >(&(args.outFile))->required(), "input file")
                ("leaf-size", bpo::value< std::string >(&leafSizeStr)->required(), "leaf size");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add("infile", 1).add("outfile", 1).add("leaf-size", 1);

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
    args.leafSize = extract_number_from_string<float>(leafSizeStr, 3);
}

int main(int argc, char* argv[]) {
    std::cout << "Hello, PCL!" << std::endl;

    Args_t args;

    parse_args(argc, argv, args);

    std::cout << "Leaf size: [ "
              << args.leafSize[0] << ", "
              << args.leafSize[1] << ", "
              << args.leafSize[2] << " ]. "
              << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB> input;
    pcl::PointCloud<pcl::PointXYZRGB> output;

    std::cout << "Loading points from " << args.inFile << " ... " << std::endl;

    if ( pcl::io::loadPLYFile<pcl::PointXYZRGB>(args.inFile, input) == -1 ) {
        std::cerr << "Failed to read: " << args.inFile << std::endl;
        return -1;
    } else {
        std::cout << "Load " << input.size() << " points from " << args.inFile << std::endl;
    }

    // Apply the voxel filter .
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setInputCloud(input.makeShared());
    grid.setLeafSize( args.leafSize[0], args.leafSize[1], args.leafSize[2] );
    std::cout << "Filtering... " << std::endl;
    grid.filter(output);

    // Save the filtered point cloud.
    pcl::PLYWriter writer;
    std::cout << "Saving the filtered point cloud." << std::endl;
    writer.write(args.outFile, output, true, false);

    return 0;
}
