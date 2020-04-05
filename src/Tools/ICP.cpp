//
// Created by yaoyu on 4/4/20.
//

#include <iostream>
#include <sstream>
#include <string>

#include <boost/program_options.hpp>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "Args/Args.hpp"
#include "PCCommon/IO.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/SimpleTime.hpp"

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
        out << Args::AS_TGT << ": " << args.tgt << std::endl;
        out << Args::AS_SRC << ": " << args.src << std::endl;
        out << Args::AS_OUT << ": " << args.outfile << std::endl;

        return out;
    }

public:
    static const std::string AS_TGT; // AS stands for argument string
    static const std::string AS_SRC;
    static const std::string AS_OUT;

public:
    std::string tgt; // The target point cloud.
    std::string src; // The reference point cloud.
    std::string outfile; // The output directory.
};

const std::string Args::AS_TGT = "tgt";
const std::string Args::AS_SRC = "src";
const std::string Args::AS_OUT = "outdir";

static void parse_args(int argc, char* argv[], Args& args) {

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_TGT.c_str(), bpo::value< std::string >(&args.tgt)->required(), "The target point cloud.")
                (Args::AS_SRC.c_str(), bpo::value< std::string >(&args.src)->required(), "The reference point cloud.")
                (Args::AS_OUT.c_str(), bpo::value< std::string >(&args.outfile)->required(), "The output file.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(Args::AS_TGT.c_str(), 1
        ).add(Args::AS_SRC.c_str(), 1
        ).add(Args::AS_OUT.c_str(), 1);

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

template < typename pTR, typename pTT >
static typename pcl::PointCloud<pTT>::Ptr icp_align(
        const typename pcl::PointCloud<pTR>::Ptr pSrc,
        const typename pcl::PointCloud<pTT>::Ptr pTgt ) {
    QUICK_TIME_START(te)

    // The aligned point cloud.
    typename  pcl::PointCloud<pTT>::Ptr pAligned ( new pcl::PointCloud<pTT> );

    // The ICP object.
    pcl::IterativeClosestPoint< pTR, pTT > icp;
    icp.setInputSource(pSrc);
    icp.setInputTarget(pTgt);
    icp.align(*pAligned);

    // Show ICP info.
    std::cout << "icp.hasConverged() = " << icp.hasConverged() << ", "
              << "score " << icp.getFitnessScore() << std::endl;

    std::cout << "icp.getFinalTransformation() = " << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    QUICK_TIME_END(te)

    std::cout << "icp_align in " << te << " ms. " << std::endl;

    return pAligned;
}

int main( int argc, char* argv[] ) {
    QUICK_TIME_START(te)

    std::cout << "Hello, ICP! " << std::endl;

    // Handle the command line.
    Args args;
    parse_args(argc, argv, args);

    std::cout << "args: " << std::endl;
    std::cout << args << std::endl;

    // Load the reference/target point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pTgt =
            pcu::read_point_cloud<pcl::PointXYZ>( args.tgt );

    // Load the source point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSrc =
            pcu::read_point_cloud<pcl::PointXYZ>( args.src );

    pcl::PointCloud<pcl::PointXYZ>::Ptr pAligned =
            icp_align<pcl::PointXYZ, pcl::PointXYZ>( pSrc, pTgt );

    // Save the aligned point cloud.
    test_directory_by_filename(args.outfile);

    pcu::write_point_cloud<pcl::PointXYZ>(args.outfile, pAligned);

    QUICK_TIME_END(te)

    std::cout << "ICP in " << te << " ms. " << std::endl;

    return 0;
}