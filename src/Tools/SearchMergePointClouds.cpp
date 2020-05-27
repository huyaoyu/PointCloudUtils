//
// Created by yaoyu on 5/27/20.
//

#include <iostream>
#include <vector>

#include "Args/Args.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/IO.hpp"
#include "Profiling/SimpleTime.hpp"

// Namespaces.
namespace bpo = boost::program_options;

// Local typedefs.
typedef pcl::PointXYZ P_t;
typedef pcl::PointCloud<P_t> PC_t;
typedef PC_t::Ptr PCPtr_t;

/************************ Command line arguments. ****************************/
class Args
{
public:
    Args() = default;
    ~Args() = default;

    bool validate() const {
        bool flag = true;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << args.AS_IN_DIR  << ": " << args.inDir   << "\n";
        out << args.AS_PATTERN << ": " << args.pattern << "\n";
        out << args.AS_OUT_DIR << ": " << args.outDir  << "\n";

        return out;
    }

    void parse_args(int argc, char* argv[]) {
        try
        {
            bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

            optDesc.add_options()
                    ("help", "Produce help message.")
                    (AS_IN_DIR.c_str(), bpo::value<std::string>(&inDir)->required(), "The input directory. ")
                    (AS_PATTERN.c_str(), bpo::value<std::string>(&pattern)->required(), "The search pattern. ")
                    (AS_OUT_DIR.c_str(), bpo::value<std::string>(&outDir)->required(), "The output file. ");

            bpo::positional_options_description posOptDesc;
            posOptDesc.add(AS_IN_DIR.c_str(), 1
            ).add(AS_PATTERN.c_str(), 1
            ).add(AS_OUT_DIR.c_str(), 1);

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

        if ( !validate() ) {
            EXCEPTION_INVALID_ARGUMENTS_IN_CLASS()
        }
    }

public:
    const std::string AS_IN_DIR  = "in-dir";
    const std::string AS_PATTERN = "pattern";
    const std::string AS_OUT_DIR = "out-dir";

    std::string inDir;
    std::string pattern;
    std::string outDir; // The output directory.
};

int main( int argc, char** argv ) {
    QUICK_TIME_START(teMain)
    std::cout << "Hello, SearchMergePointClouds! \n";
    MAIN_COMMON_LINES_ONE_CLASS(argc, argv, args)

    std::vector<std::string> files = find_files_recursively( args.inDir, args.pattern );

    if ( files.empty() ) {
        std::cout << "No files found from " << args.inDir
                  << " with pattern " << args.pattern << "\n";
        return 1;
    }

    PCPtr_t pMerged (new PC_t);

    for ( const auto& f : files ) {
        auto pInCloud = pcu::read_point_cloud<P_t>(f);
        *pMerged += *pInCloud;
    }

    std::cout << pMerged->size() << " points from "
              << files.size() << " point clouds. \n";

    std::string outFn = args.outDir + "/FilledMerged.ply";
    pcu::write_point_cloud<P_t>(outFn, pMerged);

    QUICK_TIME_SHOW(teMain, "SearchMergePointClouds")
    return 0;
}