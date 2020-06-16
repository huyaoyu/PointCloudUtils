//
// Created by yaoyu on 6/15/20.
//

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>

#include "Args/Args.hpp"
#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "DataInterfaces/JSONHelper/Reader.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/IO.hpp"
#include "Profiling/SimpleTime.hpp"

// Namespaces.
namespace bpo = boost::program_options;
using JSON = nlohmann::json;

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
        out << args.AS_IN_JSON << ": " << args.inJSON << "\n";
        out << args.AS_OUT_FN << ": " << args.outFn  << "\n";

        return out;
    }

    void parse_args(int argc, char* argv[]) {
        try
        {
            bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

            optDesc.add_options()
                    ("help", "Produce help message.")
                    (AS_IN_JSON.c_str(), bpo::value< std::string >(&inJSON)->required(), "The input JSON file contains the point clouds need to be merged. ")
                    (AS_OUT_FN.c_str(), bpo::value< std::string >(&outFn)->required(), "The output file. ");

            bpo::positional_options_description posOptDesc;
            posOptDesc.add(Args::AS_IN_JSON.c_str(), 1
            ).add(Args::AS_OUT_FN.c_str(), 1);

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
    const std::string AS_IN_JSON = "in-json";
    const std::string AS_OUT_FN = "out-fn";

    std::string inJSON;
    std::string outFn; // The output directory.
};

int main( int argc, char** argv ) {
    QUICK_TIME_START(teMain)

    std::cout << "Hello, Merge! \n";
    MAIN_COMMON_LINES_ONE_CLASS(argc, argv, args)

    std::vector<std::string> parts = get_file_parts( args.inJSON );

    std::shared_ptr<JSON> pParams = read_json( args.inJSON );

    std::vector<std::string> files = (*pParams)["clouds"].get< std::vector<std::string> >();

    pcl::PointCloud<pcl::PointXYZ>::Ptr pMerged ( new pcl::PointCloud<pcl::PointXYZ> );

    for ( const auto &f : files ) {
        std::string fn = parts[0] + "/" + f;
        std::cout << fn << "\n";

        pcl::PointCloud<pcl::PointXYZ>::Ptr pInCloud = pcu::read_point_cloud<pcl::PointXYZ>( fn );

        *pMerged += (*pInCloud);
    }

    {
        // Test the output directory.
        test_directory_by_filename(args.outFn);
        pcu::write_point_cloud<pcl::PointXYZ>( args.outFn, pMerged);
    }

    QUICK_TIME_SHOW(teMain, "Merge")
    return 0;
}