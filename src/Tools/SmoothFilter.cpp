//
// Created by yaoyu on 3/16/20.
//

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

//#include <boost/assert.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "Args/Args.hpp"

// Namespaces.
namespace bpo = boost::program_options;
namespace bpt = boost::posix_time;

typedef struct Args {
    std::string inFile;
    std::string outFile;
    int mlsPolyOrder;
    double mlsRadius;
    bool flagRemoveOutlier;
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
                ("mls-poly-order", bpo::value< int >(&args.mlsPolyOrder)->default_value(2), "The order of the polynomial for the MLS filter.")
                ("mls-radius", bpo::value< double >(&args.mlsRadius)->default_value(0.05), "The search radius of the MLS. Unit m.")
                ("remove-outlier", bpo::value< int >()->implicit_value(1), "Set this flag to enable outlier removal.")
                ("mean-k", bpo::value< int >(&args.meanK)->default_value(10), "mean k, number of nearest neighbours")
                ("std-dev-mul-t", bpo::value< double >(&args.stdDevMulThresh)->default_value(1.0), "standard deviation multiplier");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add("infile", 1).add("outfile", 1);

        bpo::variables_map optVM;
        bpo::store(bpo::command_line_parser(argc, argv).
                options(optDesc).positional(posOptDesc).run(), optVM);
        bpo::notify(optVM);

        if ( optVM.count("remove-outlier") ) {
            args.flagRemoveOutlier = true;
        } else {
            args.flagRemoveOutlier = false;
        }
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

    pcl::PointCloud<pcl::PointNormal>::Ptr tempPC( new pcl::PointCloud<pcl::PointNormal> );

    if ( args.flagRemoveOutlier ) {
        // Apply the statistical outlier removal filter .
        pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
        sor.setInputCloud(pInput);
        sor.setMeanK(args.meanK);
        sor.setStddevMulThresh(args.stdDevMulThresh);

        std::cout << "Filtering... " << std::endl;
        sor.filter(*tempPC);
    } else {
        tempPC = pInput;
    }

    // Create a KD-Tree.
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree ( new pcl::search::KdTree<pcl::PointNormal> );

    // MLS.
    pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    mls.setInputCloud(tempPC);
    mls.process(*pOutput);

    // Save the filtered point cloud.
    pcl::PLYWriter writer;
    std::cout << "Saving the filtered point cloud." << std::endl;
    writer.write(args.outFile, *pOutput, true, false);

    return 0;
}
