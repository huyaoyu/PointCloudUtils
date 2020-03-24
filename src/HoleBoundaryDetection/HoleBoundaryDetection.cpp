//
// Created by yaoyu on 3/20/20.
//

#include <iostream>
#include <set>
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
    Args()
    : pgK(AI_PROXIMITY_GRAPH_K), pgR(AI_PROXIMITY_GRAPH_R), pgSDB(AI_PROXIMITY_GRAPH_SDB),
      cStartIdx(AI_C_START_IDX)
    {}

    ~Args() = default;

    bool validate() {
        bool flag = true;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_FILE << ": " << args.inFile << std::endl;
        out << Args::AS_OUT_DIR << ": " << args.outDir << std::endl;
        out << Args::AS_PROXIMITY_GRAPH_K << ": " << args.pgK << std::endl;
        out << Args::AS_PROXIMITY_GRAPH_R << ": " << args.pgR << std::endl;
        out << Args::AS_PROXIMITY_GRAPH_SDB << ": " << args.pgSDB << std::endl;
        out << Args::AS_C_START_IDX << ": " << args.cStartIdx << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_FILE; // AS stands for argument string
    static const std::string AS_OUT_DIR;
    static const std::string AS_PROXIMITY_GRAPH_K;
    static const std::string AS_PROXIMITY_GRAPH_R;
    static const std::string AS_PROXIMITY_GRAPH_SDB;
    static const std::string AS_C_START_IDX;

    static const int AI_PROXIMITY_GRAPH_K; // AI stands for argument initial value.
    static const double AI_PROXIMITY_GRAPH_R;
    static const int AI_PROXIMITY_GRAPH_SDB;
    static const int AI_C_START_IDX;

public:
    std::string inFile; // The input point cloud file.
    std::string outDir; // The output directory.
    int    pgK;
    double pgR;
    int    pgSDB;
    int    cStartIdx; // Criteria computation starting index.
};

const std::string Args::AS_IN_FILE = "infile";
const std::string Args::AS_OUT_DIR = "outdir";
const std::string Args::AS_PROXIMITY_GRAPH_K   = "pg-k";
const std::string Args::AS_PROXIMITY_GRAPH_R   = "pg-r";
const std::string Args::AS_PROXIMITY_GRAPH_SDB = "pg-sdb";
const std::string Args::AS_C_START_IDX = "c-start-index";

const int    Args::AI_PROXIMITY_GRAPH_K   = 10;
const double Args::AI_PROXIMITY_GRAPH_R   = 0.02;
const int    Args::AI_PROXIMITY_GRAPH_SDB = 100000;
const int    Args::AI_C_START_IDX         = 0;

static void parse_args(int argc, char* argv[], Args& args) {

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_FILE.c_str(), bpo::value< std::string >(&(args.inFile))->required(), "Input file.")
                (Args::AS_OUT_DIR.c_str(), bpo::value< std::string >(&(args.outDir))->required(), "Output directory.")
                (Args::AS_PROXIMITY_GRAPH_K.c_str(), bpo::value< int >(&args.pgK)->default_value(Args::AI_PROXIMITY_GRAPH_K), "The k value for the proximity graph.")
                (Args::AS_PROXIMITY_GRAPH_R.c_str(), bpo::value< double >(&args.pgR)->default_value(Args::AI_PROXIMITY_GRAPH_R), "The radius for the proximity graph.")
                (Args::AS_PROXIMITY_GRAPH_SDB.c_str(), bpo::value< int >(&args.pgSDB)->default_value(Args::AI_PROXIMITY_GRAPH_SDB), "The show-detail base for the proximity graph.")
                (Args::AS_C_START_IDX.c_str(), bpo::value< int >(&args.cStartIdx)->default_value(Args::AI_C_START_IDX), "The starting index of the computation of the boundary criteria.");

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

template <typename pT>
static void test_show_proximity_graph_vertex_neighbors(
        typename pcl::PointCloud<pT>::Ptr& pInput,
        pcu::HBDetector& hbd, int index) {
    std::set<int>& neighbor = hbd.get_proximity_graph().get_neighbors(index);
    pT point = (*pInput)[index];
    std::cout << "Vertex " << index << " ("
              << point.x << ", "
              << point.y << ", "
              << point.z << ") " << std::endl;

    std::cout << "The neighbors of vertex " << index << " are:" << std::endl;
    for ( const auto& n : neighbor ) {
        std::cout << n << ", ";
    }
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "Hello, HoleBoundaryDetection!" << std::endl;

    // Handle the command line.
    Args args;
    parse_args(argc, argv, args);

    std::cout << "args: " << std::endl;
    std::cout << args << std::endl;

    // Define the point cloud object.
    typedef pcl::PointNormal P_t;
    typedef pcl::PointCloud<P_t> PC_t;
    PC_t::Ptr pInput(new PC_t);

    // Read the point cloud.
    read_point_cloud<P_t>(args.inFile, pInput);

    // The hole boundary detector.
    auto hbd = pcu::HBDetector();

    hbd.set_point_cloud(pInput);
    hbd.set_proximity_graph_params(args.pgK, args.pgR, args.pgSDB);
    hbd.set_criteria_computation_start_index(args.cStartIdx);

    hbd.process();

//    // Test use.
//    const int index = 245930;
//    test_show_proximity_graph_vertex_neighbors<P_t>(pInput, hbd, index);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored ( new pcl::PointCloud<pcl::PointXYZRGB> );
    hbd.create_rgb_representation(colored);

    // Test the output directory.
    test_directory(args.outDir);

    // Save the point cloud.
    QUICK_TIME_START(teWrite)
    std::string outFn = args.outDir + "/Colored.ply";
    pcl::PLYWriter writer;
    std::cout << "Saving the filtered point cloud." << std::endl;
    writer.write(outFn, *colored, true, false);
    QUICK_TIME_END(teWrite)

    std::cout << "Write point cloud in " << teWrite << " ms. " << std::endl;

    return 0;
}