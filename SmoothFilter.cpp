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

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    int meanK;
    double stdDevMulThresh;
} Args_t;

void parse_args(int argc, char* argv[], Args_t& args) {
    std::string leafSizeStr;

    try
    {
        bpo::options_description optDesc("Remove outliers and smooth a point cloud.");

        optDesc.add_options()
                ("help", "produce help message")
                ("infile", bpo::value< std::string >(&(args.inFile))->required(), "input file")
                ("outfile", bpo::value< std::string >(&(args.outFile))->required(), "input file")
                ("mean-k", bpo::value< int >(&args.meanK)->default_value(10), "mean k, number of nearest neighbours")
                ("std-dev-mul-t", bpo::value< double >(&args.stdDevMulThresh)->default_value(1.0), "standard deviation multiplier");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add("infile", 1).add("outfile", 1);

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

    std::cout << "mean-k = " << args.meanK << std::endl;
    std::cout << "std-dev-mul-t = " << args.stdDevMulThresh << std::endl;

    pcl::PointCloud<pcl::PointNormal>::Ptr pInput( new pcl::PointCloud<pcl::PointNormal> );
    pcl::PointCloud<pcl::PointNormal>::Ptr pOutput( new pcl::PointCloud<pcl::PointNormal> );

    std::cout << "Loading points from " << args.inFile << " ... " << std::endl;

    if ( pcl::io::loadPLYFile<pcl::PointNormal>(args.inFile, *pInput) == -1 ) {
        std::cerr << "Failed to read: " << args.inFile << std::endl;
        return -1;
    } else {
        std::cout << "Load " << pInput->size() << " points from " << args.inFile << std::endl;
    }

    // Apply the statistical outlier removal filter .
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
    sor.setInputCloud(pInput);
    sor.setMeanK(args.meanK);
    sor.setStddevMulThresh(args.stdDevMulThresh);

    std::cout << "Filtering... " << std::endl;
    sor.filter(*pOutput);

    // Save the filtered point cloud.
    pcl::PLYWriter writer;
    std::cout << "Saving the filtered point cloud." << std::endl;
    writer.write(args.outFile, *pOutput, true, false);

    return 0;
}
