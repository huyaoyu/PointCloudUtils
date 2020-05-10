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
#include "PCCommon/IO.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/SimpleTime.hpp"
#include "Visualization/Print.hpp"

#include "HoleBoundaryDetection/HoleBoundaryDetector.hpp"

// Namespaces.
namespace bpo = boost::program_options;
namespace bpt = boost::posix_time;

class Args
{
public:
    Args()
    : pgK(AI_PROXIMITY_GRAPH_K), pgR(AI_PROXIMITY_GRAPH_R), pgSDB(AI_PROXIMITY_GRAPH_SDB),
      cStartIdx(AI_C_START_IDX),
      enLimit(AI_EN_LIMIT),
      flagSectionBorder(false), sectionBorderLimit(AI_SEC_BORDER_LIMIT)
    {}

    ~Args() = default;

    bool validate() {
        bool flag = true;

        if ( cStartIdx < 0 ) {
            flag = false;
            std::cout << "Wrong " << AS_C_START_IDX << " value " << cStartIdx << ". Must be non-negative. " << std::endl;
        }

        if ( enLimit < 0 ) {
            flag = false;
            std::cout << "Wrong " << AS_EN_LIMIT << " value " << enLimit << ". Must be non-negative. " << std::endl;
        }

        if ( flagSectionBorder && sectionBorderLimit <= 0 ) {
            flag = false;
            std::cout << "wrong " << AS_SEC_BORDER_LIMIT << " value " << sectionBorderLimit << ". Must be positive. " << std::endl;
        }

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_CLOUD << ": " << args.inFile << std::endl;
        out << Args::AS_OUT_DIR << ": " << args.outDir << std::endl;
        out << Args::AS_PROXIMITY_GRAPH_K << ": " << args.pgK << std::endl;
        out << Args::AS_PROXIMITY_GRAPH_R << ": " << args.pgR << std::endl;
        out << Args::AS_PROXIMITY_GRAPH_SDB << ": " << args.pgSDB << std::endl;
        out << Args::AS_C_START_IDX << ": " << args.cStartIdx << std::endl;
        out << Args::AS_EN_LIMIT << ": " << args.enLimit << std::endl;
        out << Args::AS_FLAG_SEC_BORDER << ": " << args.flagSectionBorder << std::endl;
        if ( args.flagSectionBorder ) {
            out << Args::AS_SEC_BORDER_LIMIT << ": " << args.sectionBorderLimit << std::endl;
            out << Args::AS_SEC_BORDER_CORNERS << ": " << std::endl;
            show_numeric_vector(args.sectionBorderCorners);
            out << std::endl;
        }

        return out;
    }

public:
    static const std::string AS_IN_CLOUD; // AS stands for argument string
    static const std::string AS_OUT_DIR;
    static const std::string AS_PROXIMITY_GRAPH_K;
    static const std::string AS_PROXIMITY_GRAPH_R;
    static const std::string AS_PROXIMITY_GRAPH_SDB;
    static const std::string AS_C_START_IDX;
    static const std::string AS_EN_LIMIT;
    static const std::string AS_FLAG_SEC_BORDER;
    static const std::string AS_SEC_BORDER_CORNERS;
    static const std::string AS_SEC_BORDER_LIMIT;

    static const int AI_PROXIMITY_GRAPH_K; // AI stands for argument initial value.
    static const double AI_PROXIMITY_GRAPH_R;
    static const int AI_PROXIMITY_GRAPH_SDB;
    static const int AI_C_START_IDX;
    static const int AI_EN_LIMIT;
    static const int AI_SEC_BORDER_LIMIT;

public:
    std::string inFile; // The input point cloud file.
    std::string outDir; // The output directory.
    int    pgK;
    double pgR;
    int    pgSDB;
    int    cStartIdx; // Criteria computation starting index.
    int    enLimit; // The limit number of points for equivalent normal computation.
    bool   flagSectionBorder; // True if filter points near section borders.
    std::vector<float> sectionBorderCorners;
    float  sectionBorderLimit; // Set this value if flagSectionBorder is set.
};

const std::string Args::AS_IN_CLOUD = "infile";
const std::string Args::AS_OUT_DIR = "outdir";
const std::string Args::AS_PROXIMITY_GRAPH_K   = "pg-k";
const std::string Args::AS_PROXIMITY_GRAPH_R   = "pg-r";
const std::string Args::AS_PROXIMITY_GRAPH_SDB = "pg-sdb";
const std::string Args::AS_C_START_IDX = "c-start-index";
const std::string Args::AS_EN_LIMIT = "en-limit";
const std::string Args::AS_FLAG_SEC_BORDER = "sec-border";
const std::string Args::AS_SEC_BORDER_CORNERS = "sec-border-corners";
const std::string Args::AS_SEC_BORDER_LIMIT = "sec-border-limit";

const int    Args::AI_PROXIMITY_GRAPH_K   = 10;
const double Args::AI_PROXIMITY_GRAPH_R   = 0.02;
const int    Args::AI_PROXIMITY_GRAPH_SDB = 100000;
const int    Args::AI_C_START_IDX         = 0;
const int    Args::AI_EN_LIMIT            = 50;
const int    Args::AI_SEC_BORDER_LIMIT    = 0.05;

static void parse_args(int argc, char* argv[], Args& args) {
    std::string secBorderCornersString;

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_CLOUD.c_str(), bpo::value< std::string >(&(args.inFile))->required(), "Input file.")
                (Args::AS_OUT_DIR.c_str(), bpo::value< std::string >(&(args.outDir))->required(), "Output directory.")
                (Args::AS_PROXIMITY_GRAPH_K.c_str(), bpo::value< int >(&args.pgK)->default_value(Args::AI_PROXIMITY_GRAPH_K), "The k value for the proximity graph.")
                (Args::AS_PROXIMITY_GRAPH_R.c_str(), bpo::value< double >(&args.pgR)->default_value(Args::AI_PROXIMITY_GRAPH_R), "The radius for the proximity graph.")
                (Args::AS_PROXIMITY_GRAPH_SDB.c_str(), bpo::value< int >(&args.pgSDB)->default_value(Args::AI_PROXIMITY_GRAPH_SDB), "The show-detail base for the proximity graph.")
                (Args::AS_C_START_IDX.c_str(), bpo::value< int >(&args.cStartIdx)->default_value(Args::AI_C_START_IDX), "The starting index of the computation of the boundary criteria.")
                (Args::AS_EN_LIMIT.c_str(), bpo::value< int >(&args.enLimit)->default_value(Args::AI_EN_LIMIT), "The limit number of points for normal averaging.")
                (Args::AS_FLAG_SEC_BORDER.c_str(), bpo::value< bool >( &args.flagSectionBorder )->implicit_value(true), "Set this to enable section border filter. Must set --sec-border-limit.")
                (Args::AS_SEC_BORDER_LIMIT.c_str(), bpo::value< float >( &args.sectionBorderLimit )->default_value(Args::AI_SEC_BORDER_LIMIT), "The limit distance to classify a point to be near the border of the point cloud section. ")
                (Args::AS_SEC_BORDER_CORNERS.c_str(), bpo::value< std::string >( &secBorderCornersString ), "The section border corner points, x0, y0, z0, x1, y1, z1.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(Args::AS_IN_CLOUD.c_str(), 1).add(Args::AS_OUT_DIR.c_str(), 1);

        bpo::variables_map optVM;
        bpo::store(bpo::command_line_parser(argc, argv).
                options(optDesc).positional(posOptDesc).run(), optVM);
        bpo::notify(optVM);

        if ( optVM.count(Args::AS_FLAG_SEC_BORDER.c_str()) ) {
            args.flagSectionBorder = true;
        }

        args.sectionBorderCorners = extract_number_from_string<float>(
                        secBorderCornersString, 6, "," );
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        throw(e);
    }

    if ( !args.validate() ) {
        EXCEPTION_INVALID_ARGUMENTS(args)
    }
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
    MAIN_COMMON_LINES(argc, argv, args)

    QUICK_TIME_START(teMain)

    // Define the point cloud object.
    typedef pcl::PointNormal P_t;
    typedef pcl::PointCloud<P_t> PC_t;
    PC_t::Ptr pInput(new PC_t);

    // Read the point cloud.
    print_bar("Load the point cloud.");
    pcu::read_point_cloud<P_t>(args.inFile, pInput);

    // The hole boundary detector.
    print_bar("Boundary detection.");
    auto hbd = pcu::HBDetector();

    hbd.set_point_cloud(pInput);
    hbd.set_proximity_graph_params(args.pgK, args.pgR, args.pgSDB);
    hbd.set_criterion_computation_start_index(args.cStartIdx);
    hbd.set_equivalent_normal_averaging_limit(args.enLimit);

    if ( args.flagSectionBorder ) {
        hbd.set_section_border_corners(args.sectionBorderCorners);
        hbd.set_section_border_threshold(args.sectionBorderLimit);
    }

    hbd.process();

//    // Test use.
//    const int index = 245930;
//    test_show_proximity_graph_vertex_neighbors<P_t>(pInput, hbd, index);

    // Test the output directory.
    print_bar("Write results.");
    test_directory(args.outDir);

    {
        // Colored point cloud by criteria.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredByCriteria ( new pcl::PointCloud<pcl::PointXYZRGB> );
        hbd.create_rgb_representation_by_criteria(coloredByCriteria);

        // Save the point cloud.
        QUICK_TIME_START(teWrite_ColorByCriteria)
        std::string outFn = args.outDir + "/ColoredByCriteria.ply";
        pcl::PLYWriter writer;
        std::cout << "Saving the filtered point cloud." << std::endl;
        writer.write(outFn, *coloredByCriteria, true, false);
        QUICK_TIME_END(teWrite_ColorByCriteria)

        std::cout << "Write colored point cloud by criteria in " << teWrite_ColorByCriteria << " ms. " << std::endl;
    }

    {
        // Colored point cloud by boundary candidates.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredByBoundaryCandidates ( new pcl::PointCloud<pcl::PointXYZRGB> );
        hbd.create_rgb_representation_by_boundary_candidates(coloredByBoundaryCandidates);

        // Save the point cloud.
        QUICK_TIME_START(teWrite_ColorByBounadryCandidates)
        std::string outFn = args.outDir + "/ColoredByBoundaryCandidates.ply";
        pcl::PLYWriter writer;
        std::cout << "Saving the filtered point cloud." << std::endl;
        writer.write(outFn, *coloredByBoundaryCandidates, true, false);
        QUICK_TIME_END(teWrite_ColorByBounadryCandidates)

        std::cout << "Write point cloud by boundary candidates in " << teWrite_ColorByBounadryCandidates << " ms. " << std::endl;
    }

    {
        // Colored point cloud by disjoint candidates.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored ( new pcl::PointCloud<pcl::PointXYZRGB> );
        hbd.create_rgb_representation_by_disjoint_candidates(colored);

        // Save the point cloud.
        QUICK_TIME_START(te)
        std::string outFn = args.outDir + "/ColoredByDisjointCandidates.ply";
        pcl::PLYWriter writer;
        std::cout << "Saving the filtered point cloud." << std::endl;
        writer.write(outFn, *colored, true, false);
        QUICK_TIME_END(te)

        std::cout << "Write point cloud by disjoint candidates in " << te << " ms. " << std::endl;
    }

    // Equivalent normal.
    {
        std::string outFn = args.outDir + "/EquivalentNormal.ply";
        pcu::write_point_cloud<pcl::PointNormal>( outFn, hbd.get_equivalent_normal(), false );
    }

    // The JSON file.
    {
        std::string outFn = args.outDir + "/DisjointSets.json";
        hbd.write_disjoint_sets_and_normal_as_json(outFn);
    }

    QUICK_TIME_END(teMain)
    std::cout << "HoleBoundaryDetection in " << teMain << " ms. " << std::endl;

    return 0;
}