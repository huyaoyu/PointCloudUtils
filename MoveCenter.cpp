//
// Created by yaoyu on 3/4/20.
//

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
//#include <boost/assert.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Namespaces.
namespace bpo = boost::program_options;
namespace bpt = boost::posix_time;

std::vector<std::string> get_file_parts(const std::string& p) {
    boost::filesystem::path pp{p};

    std::vector<std::string> parts{
        pp.parent_path().string(), pp.stem().string(), pp.extension().string()};

    return parts;
}

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
    std::string outDir;
} Args_t;

void parse_args(int argc, char* argv[], Args_t& args) {
    try
    {
        bpo::options_description optDesc("Down-sample a point cloud with a voxel filter.");

        optDesc.add_options()
                ("help", "produce help message")
                ("infile", bpo::value< std::string >(&(args.inFile))->required(), "input file")
                ("outdir", bpo::value< std::string >(&(args.outDir))->required(), "input file");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add("infile", 1).add("outdir", 1);

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

int main(int argc, char* argv[]) {
    std::cout << "Hello, PCL!" << std::endl;

    Args_t args;

    parse_args(argc, argv, args);

    pcl::PointCloud<pcl::PointXYZRGB> input;
    pcl::PointCloud<pcl::PointXYZRGB> output;

    std::cout << "Loading points from " << args.inFile << " ... " << std::endl;

    if ( pcl::io::loadPLYFile<pcl::PointXYZRGB>(args.inFile, input) == -1 ) {
        std::cerr << "Failed to read: " << args.inFile << std::endl;
        return -1;
    } else {
        std::cout << "Load " << input.size() << " points from " << args.inFile << std::endl;
    }

    // Compute the 3D centroid of the point cloud.
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(input, centroid);

    std::cout << "centroid = " << std::endl << centroid << std::endl;

    // Transform the point cloud.
    Eigen::Matrix4f tMat = Eigen::Matrix4f::Identity();
    tMat(0, 3) = -1 * centroid(0);
    tMat(1, 3) = -1 * centroid(1);
    tMat(2, 3) = -1 * centroid(2);

    std::cout << "Transform. " << std::endl;
    pcl::transformPointCloud(input, output, tMat);

    if ( !boost::filesystem::is_directory(args.outDir) ) {
        boost::filesystem::path outDir{args.outDir};
        boost::filesystem::create_directories(outDir);
    }

    auto parts = get_file_parts(args.inFile);

    // Save the transformed point cloud.
    pcl::PLYWriter writer;
    std::string outFile = parts[0] + "/" + parts[1] + "_Moved.ply";
    std::cout << "Saving the transformed point cloud to " << outFile << std::endl;
    writer.write(outFile, output, true, false);

    // Save the center coordinates.
    outFile = parts[0] + "/" + parts[1] + "_Center.txt";
    std::ofstream ofs(outFile);
    if ( ofs.is_open() ) {
        ofs << centroid;
    } else {
        std::stringstream ss;
        ss << outFile << " could not be opened. " << std::endl;
        throw std::runtime_error(ss.str());
    }

    std::cout << outFile << " saved. " << std::endl;

    return 0;
}
