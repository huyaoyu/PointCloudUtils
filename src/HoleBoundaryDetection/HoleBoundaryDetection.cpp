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
#include <pcl/kdtree/kdtree_flann.h>

#include "Args/Args.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/SimpleTime.hpp"

#include "HoleBoundaryDetection/HoleBoundaryDetector.hpp"

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
    Args() = default;
    ~Args() = default;

    bool validate(void) {
        bool flag = true;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << args.AS_IN_FILE << ": " << args.inFile << std::endl;
        out << args.AS_OUT_DIR << ": " << args.outDir << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_FILE; // AS stands for argument string
    static const std::string AS_OUT_DIR;

public:
    std::string inFile; // The input point cloud file.
    std::string outDir; // The output directory.
};

const std::string Args::AS_IN_FILE = "infile";
const std::string Args::AS_OUT_DIR = "outdir";

static void parse_args(int argc, char* argv[], Args& args) {

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_FILE.c_str(), bpo::value< std::string >(&(args.inFile))->required(), "Input file.")
                (Args::AS_OUT_DIR.c_str(), bpo::value< std::string >(&(args.outDir))->required(), "Output directory.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(Args::AS_IN_FILE.c_str(), 1).add(Args::AS_OUT_DIR.c_str(), 1);

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

int main(int argc, char* argv[]) {
    std::cout << "Hello, HoleBoundaryDetection!" << std::endl;

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

    // The hole boundary detector.
    auto hbd = pcu::HBDetector();

//    hbd.set_point_cloud(pInput);

    return 0;
}