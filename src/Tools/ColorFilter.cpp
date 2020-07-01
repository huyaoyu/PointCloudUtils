//
// Created by yaoyu on 6/30/20.
//

#include <iostream>
#include <vector>

#include <pcl/point_types.h>

#include "Args/Args.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/extraction.hpp"
#include "PCCommon/IO.hpp"
#include "Profiling/SimpleTime.hpp"

// Namespaces.
namespace bpo = boost::program_options;

// Local typedef.
typedef pcl::PointXYZRGB P_t;
typedef pcl::PointCloud<P_t> PT_t;
typedef PT_t::Ptr PTPtr;

template < typename T >
static void show_vector(const std::vector<T> &vec,
        const std::string &name, const std::string &delimiter=", ") {
    std::cout << name << ": \n";

    for ( const auto& v : vec ) {
        std::cout << v << delimiter;
    }

    std::cout << "\n";
}

template < typename T >
static std::ostream& show_vector(std::ostream& out,
        const std::vector<T> &vec ) {
    for ( const auto& v : vec ) {
        out << v << ", ";
    }

    out << "\n";

    return out;
}

/************************ Command line arguments. ****************************/
class Args
{
public:
    Args() = default;
    ~Args() = default;

    bool validate() const {
        bool flag = true;

        for ( int i = 0; i < 3; ++i ) {
            auto t0 = rgbMin[i];
            auto t1 = rgbMax[i];

            if ( t0 >= 255 ||
                 t1 <= 0   ||
                 t0 >= t1 ) {
                flag = false;
                std::cout << "The RGB thresholds are wrong. \n";
                show_vector( rgbMin, "rgbMin" );
                show_vector( rgbMax, "rgbMax" );
                break;
            }
        }

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << args.AS_IN_CLOUD  << ": " << args.inCloud  << "\n";
        out << args.AS_OUT_CLOUD << ": " << args.outCloud << "\n";

        out << args.AS_RGB_MIN << ": ";
        show_vector(out, args.rgbMin);

        out << args.AS_RGB_MAX << ": ";
        show_vector(out, args.rgbMax);

        return out;
    }

    void parse_args(int argc, char* argv[]) {
        std::string rgbMinStr;
        std::string rgbMaxStr;

        try
        {
            bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

            optDesc.add_options()
                    ("help", "Produce help message.")
                    (AS_IN_CLOUD.c_str(), bpo::value< std::string >(&inCloud)->required(), "The input MVS point cloud. ")
                    (AS_OUT_CLOUD.c_str(), bpo::value< std::string >(&outCloud)->required(), "The input LiDAR point cloud. ")
                    (AS_RGB_MIN.c_str(), bpo::value< std::string >(&rgbMinStr)->required(), "The min RGB values.")
                    (AS_RGB_MAX.c_str(), bpo::value< std::string >(&rgbMaxStr)->required(), "The max RGB values.");

            bpo::positional_options_description posOptDesc;
            posOptDesc.add(Args::AS_IN_CLOUD.c_str(), 1
            ).add(Args::AS_OUT_CLOUD.c_str(), 1
            ).add(Args::AS_RGB_MIN.c_str(), 1
            ).add(Args::AS_RGB_MAX.c_str(), 1);

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

        // Extract the RGB thresholds.
        rgbMin = extract_number_from_string<int>(rgbMinStr, 3);
        rgbMax = extract_number_from_string<int>(rgbMaxStr, 3);

        if ( !validate() ) {
            EXCEPTION_INVALID_ARGUMENTS_IN_CLASS()
        }
    }

public:
    const std::string AS_IN_CLOUD  = "in-cloud";
    const std::string AS_OUT_CLOUD = "out-cloud";
    const std::string AS_RGB_MIN   = "rgb-min";
    const std::string AS_RGB_MAX   = "rgb-max";

    std::string inCloud;
    std::string outCloud;
    std::vector<int> rgbMin;
    std::vector<int> rgbMax;
};

static pcl::PointIndices::Ptr filter_indices_by_RGB(
        const PTPtr pInput,
        const std::vector<int> &rgbMin,
        const std::vector<int> &rgbMax ) {

    // Local copy of the thresholds.
    int rMin = rgbMin[0];
    int gMin = rgbMin[1];
    int bMin = rgbMin[2];

    int rMax = rgbMax[0];
    int gMax = rgbMax[1];
    int bMax = rgbMax[2];

    pcl::PointIndices::Ptr pIndices ( new pcl::PointIndices );

    const int N = pInput->size();

    for ( int i = 0; i < N; ++i ) {
        const auto& point = pInput->at(i);

        if ( point.r < rMin &&
             point.g < gMin &&
             point.b < bMin ) {
            continue;
        }

        if ( point.r > rMax &&
             point.g > gMax &&
             point.b > bMax ) {
            continue;
        }

        pIndices->indices.push_back(i);
    }

    return pIndices;
}

int main( int argc, char** argv ) {
    QUICK_TIME_START(teMain)

    std::cout << "Hello, ColorFilter! \n";
    MAIN_COMMON_LINES_ONE_CLASS(argc, argv, args)

    // Load the point cloud.
    PTPtr pInCloud = pcu::read_point_cloud<P_t>( args.inCloud );

    // Filter for the indices.
    pcl::PointIndices::Ptr pIndices = filter_indices_by_RGB( pInCloud, args.rgbMin, args.rgbMax );

    std::cout << "pIndices->indices.size() = " << pIndices->indices.size() << "\n";

    // Extract the points.
    PTPtr pExtracted = pcu::extract_points<P_t>( pInCloud, pIndices );

    // Save the extracted points.
    test_directory_by_filename(args.outCloud);
    pcu::write_point_cloud<P_t>(args.outCloud, pExtracted);

    QUICK_TIME_SHOW(teMain, "ColorFilter")

    return 0;
}