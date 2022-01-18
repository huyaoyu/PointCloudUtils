//
// Created by yaoyu on 11/29/20.
//

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/array.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/property_map.h>
#include <CGAL/Real_timer.h>

// Point cloud processing headers.
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/remove_outliers.h>

// Local headers.
#include "Args/ArgsParser.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/ScopeTimer.hpp"

// CGAL typedefs.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
//typedef CGAL::Simple_cartesian<float>  Kernel_t;
typedef Kernel_t::Point_3              Point_t;
typedef std::array< unsigned char, 3 > Color_t;

// CGAL point cloud customized typedef.
typedef std::tuple< Point_t, Color_t > P_t;
typedef CGAL::Nth_of_tuple_property_map<0, P_t> PointMap_t;
typedef CGAL::Nth_of_tuple_property_map<1, P_t> ColorMap_t;
typedef std::vector< P_t > PC_t;

typedef CGAL::Parallel_if_available_tag ConcurrencyTag_t;

static PC_t read_ply_point_cloud( const std::string& fn ) {
    FUNCTION_SCOPE_TIMER

    std::ifstream ifs(fn);
    if (!ifs) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for writing. ";
        throw std::runtime_error( ss.str() );
    }

    PC_t cloud;

    if ( !CGAL::read_ply_points_with_properties(
            ifs,
            std::back_inserter(cloud),
            CGAL::make_ply_point_reader( PointMap_t() ),
            std::make_tuple( ColorMap_t(), CGAL::Construct_array(), // The number of the PLY_property<>s must be the same with the definition of Color_t.
                             CGAL::PLY_property<unsigned char>("red"),
                             CGAL::PLY_property<unsigned char>("green"),
                             CGAL::PLY_property<unsigned char>("blue") ) ) ) {
        ifs.close();
        std::stringstream ss;
        ss << "Read from PLY file " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ifs.close();

    return cloud;
}

namespace CGAL {
    template < typename T >
    struct Output_rep< ::Color_t, T > {
        const ::Color_t& c;
        static const bool is_specialized = true;

        Output_rep( const ::Color_t& c ) : c{c} {}

        std::ostream& operator() ( std::ostream& out ) const {
            if ( is_ascii( out ) ) {
                out << static_cast<int>(c[0]) << " "
                    << static_cast<int>(c[1]) << " "
                    << static_cast<int>(c[2]);
            } else {
                out.write( reinterpret_cast<const char*>(&c), sizeof(c) );
            }

            return out;
        }
    };
}

static void write_ply_point_clouds( const std::string& fn, const PC_t& cloud, bool flagBinary=true ) {
    FUNCTION_SCOPE_TIMER

    std::ofstream ofs;

    if ( flagBinary ) {
        ofs.open( fn, std::ios::binary );
        CGAL::set_binary_mode(ofs);
    } else {
        ofs.open( fn );
    }

    if (!ofs) {
        std::stringstream ss;
        ss << "Failed to open " << fn << " for writing. ";
        throw std::runtime_error( ss.str() );
    }

    if ( !CGAL::write_ply_points_with_properties(
            ofs,
            cloud,
            CGAL::make_ply_point_writer(PointMap_t()),
            std::make_tuple( ColorMap_t(),
                             CGAL::PLY_property<unsigned char>("red"),
                             CGAL::PLY_property<unsigned char>("green"),
                             CGAL::PLY_property<unsigned char>("blue") ) ) ) {
        ofs.close();
        std::stringstream ss;
        ss << "Write to " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ofs.close();
}

template < typename F_T >
static void write_average_spacing( const std::string& fn, F_T averageSpacing ) {
    std::ofstream ofs(fn);
    if (!ofs) {
        std::stringstream ss;
        ss << "Failed to open " << fn << " for writing. ";
        throw std::runtime_error( ss.str() );
    }

    ofs.precision(12);
    ofs << std::scientific << std::showpos << averageSpacing << "\n";

    ofs.close();
}

struct ProgressStdCerrCallback {
    explicit ProgressStdCerrCallback( const std::string& name )
            : name{name}, nb{0}, iAdv{0}, printCount{0} {
        timer.start();
        tStart  = timer.time();
        tLatest = tStart;
    }

    bool operator()(double advancement) {
        ++nb;
        if ( nb == 1 ) {
            std::cerr << name << ": ";
        }

        if ( advancement != 1 && nb % 100 != 0 ) return true;

        double t = timer.time();

        if ( ( t - tLatest ) > 0.1 ) {
            const int adv = static_cast<int>(advancement * 100);
            if ( adv != iAdv ) {
                if ( printCount % 10 == 0 ) std::cerr << "\n";

                std::cerr << adv << "%, ";
                iAdv = adv;
                printCount++;
            }
            tLatest = t;
        } else if ( advancement == 1) {
            std::cerr << static_cast<int>(advancement * 100) << "%\n";
            printCount++;
            tLatest = t;
        }

        return true;
    }

    const std::string name;
    int nb;
    int iAdv;
    int printCount;
    CGAL::Real_timer timer;
    double tStart;
    double tLatest;
};

static float compute_average_spacing( PC_t& points, int nb=24 ) {
    FUNCTION_SCOPE_TIMER
    return static_cast<float>( CGAL::compute_average_spacing<ConcurrencyTag_t>(
            points, nb,
            CGAL::parameters::point_map( PointMap_t() )
            .callback( ProgressStdCerrCallback("Average spacing") )) );
}

static float simplify_point_cloud( PC_t& points, int nb=24, float factor=2.f ) {
    FUNCTION_SCOPE_TIMER
    std::cout << "Simplification: Starts computing the average spacing. \n";
    const auto averageSpacing = compute_average_spacing( points, nb );
    std::cout << "Simplification: averageSpacing = " << averageSpacing << "\n";
    std::cout << "Simplification: Starts simplification. \n";
    points.erase( CGAL::grid_simplify_point_set( points, factor * averageSpacing,
                                                 CGAL::parameters::point_map( PointMap_t() )
                                                 .callback(ProgressStdCerrCallback("Grid simplification") ) ),
                  points.end() );
    std::cerr << "\n";

    return static_cast<float>( averageSpacing );
}

static std::pair<PC_t, float> remove_outliers( PC_t& points, int nb=24 ) {
    FUNCTION_SCOPE_TIMER
    std::cout << "OutlierRemoval: Starts computing the average spacing. \n";
    const auto averageSpacing = compute_average_spacing( points, nb );
    typename PC_t::iterator firstToRemove =
            CGAL::remove_outliers< ConcurrencyTag_t >(
                    points, nb,
                    CGAL::parameters::point_map( PointMap_t() )
                        .threshold_percent(100)
                        .threshold_distance(2.0 * averageSpacing) );

    std::cout << 100.0 * std::distance( firstToRemove, points.end() ) / points.size()
              << "% of the point cloud are identified as outliers. "
              << "Average spacing is " << averageSpacing << ". \n";

    PC_t newPoints( std::distance( points.begin(), firstToRemove ) );
    std::copy( points.begin(), firstToRemove, newPoints.begin() );

    return { newPoints, averageSpacing };
}

static void smooth_point_cloud( PC_t& points, int nb=24 ) {
    FUNCTION_SCOPE_TIMER
    std::cout << "Smoothing: Starts smoothing. \n";
    CGAL::jet_smooth_point_set<ConcurrencyTag_t>( points, nb,
                                                  CGAL::parameters::point_map( PointMap_t() )
                                                  .callback(ProgressStdCerrCallback("Smoothing") ) );
    std::cerr << "\n";
}

static ap::Args handle_args( int argc, char** argv ) {
    ap::Args args;

    args.add_positional<std::string>("in-cloud", "Input point cloud with color. ");
    args.add_positional<std::string>("out-dir", "Output directory. ");
    args.add_positional<std::string>("out-name", "Output point cloud filename relative to out-dir. ");

    args.add_default<int>("simplification-neighbors", "The number of neighbors used for simplification. ", 16);
    args.add_default<float>("simplification-factor", "The factor multiplied to the average spacing for simplification. ", 2.f);

    args.add_flag("remove-outlier", "Set this flag to remove outliers.");
    args.add_default<int>("remove-outlier-neighbors", "The number of neighbors used for removing outliers. ", 24);

    args.add_flag("smoothing", "Set this flag to enable smoothing. ");
    args.add_default<int>("smoothing-neighbors", "The number of neighbors used for smoothing. ", 24);

    args.parse_args( argc, argv );

    std::cout << args;

    return args;
}

int main( int argc, char** argv ){
    std::cout << "Hello, PointCloudSimplification! \n";

    // Handle the arguments.
    auto args = handle_args(argc, argv);

    // Retrieve the arguments.
    const std::string inCloudFn       = args.arguments<std::string>["in-cloud"]->get();
    const std::string outDir          = args.arguments<std::string>["out-dir"]->get();
    const std::string outName         = args.arguments<std::string>["out-name"]->get();
    const int simplificationNeighbors = args.arguments<int>["simplification-neighbors"]->get();
    const float simplificationFactor  = args.arguments<float>["simplification-factor"]->get();

    const bool flagRemoveOutlier     = args.arguments<bool>["remove-outlier"]->get();
    const int removeOutlierNeighbors = args.arguments<int>["remove-outlier-neighbors"]->get();

    const bool flagSmoothing     = args.arguments<bool>["smoothing"]->get();
    const int smoothingNeighbors = args.arguments<int>["smoothing-neighbors"]->get();

    // Prepare the output directory.
    test_directory( outDir );

    // Read the point cloud.
    auto cloud = read_ply_point_cloud( inCloudFn );
    std::cout << "cloud.size() = " << cloud.size() << "\n";

    // Simplification.
    float averageSpacing = simplify_point_cloud( cloud, simplificationNeighbors, simplificationFactor );

    // Outlier removal.
    if ( flagRemoveOutlier ) {
        std::pair<PC_t, float> res = remove_outliers( cloud, removeOutlierNeighbors );
        cloud = std::move(res.first);
        averageSpacing = res.second;
    }

    // Smoothing.
    if ( flagSmoothing ) {
        smooth_point_cloud( cloud, smoothingNeighbors );
    }

    // Write the point cloud.
    {
        std::stringstream outFullFn;
        outFullFn << outDir << "/" << outName;
        write_ply_point_clouds( outFullFn.str(), cloud );
    }

    // Write the average spacing.
    {
        std::stringstream outFullFn;
        outFullFn << outDir << "/AverageSpacing.dat";
        write_average_spacing( outFullFn.str(), averageSpacing );
    }

    return 0;
}