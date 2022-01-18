//
// Created by yaoyu on 12/4/20.
//

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Surface_mesh.h>

// Local headers.
#include "Args/ArgsParser.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/ScopeTimer.hpp"

#include "CGALCommon/IO.hpp"

// Namespace.
namespace cc  = cgal_common;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::FT                Float_t;
typedef Kernel_t::Point_3           Point_t;
typedef Kernel_t::Vector_3          Vector_t;
typedef CGAL::Surface_mesh<Point_t> SurfaceMesh_t;

typedef std::tuple< Point_t, Vector_t, pcu::Color_t > PCPoint_t;
typedef std::vector< PCPoint_t >                      PC_t;

constexpr int PC_IDX_POINT  = 0;
constexpr int PC_IDX_NORMAL = 1;
constexpr int PC_IDX_COLOR  = 2;
typedef CGAL::Nth_of_tuple_property_map< PC_IDX_POINT,  PCPoint_t > PCMap_Point_t;
typedef CGAL::Nth_of_tuple_property_map< PC_IDX_NORMAL, PCPoint_t > PCMap_Normal_t;
typedef CGAL::Nth_of_tuple_property_map< PC_IDX_COLOR,  PCPoint_t > PCMap_Color_t;

// Shape detection.
typedef CGAL::Shape_detection::Efficient_RANSAC_traits<
    Kernel_t, PC_t, PCMap_Point_t, PCMap_Normal_t > ERTraits_t;
typedef CGAL::Shape_detection::Efficient_RANSAC< ERTraits_t > EfficientRansac_t;
typedef CGAL::Shape_detection::Plane< ERTraits_t >            ERPlane_t;

typedef std::array< Float_t, 4 > PlaneCoeff_t;
typedef std::vector< PlaneCoeff_t > PlaneContainer_t;

// Global variable.
pcu::CommonColor gCommonColor;

static void assign_white_color( PC_t& points ) {
    const pcu::Color_t white { 255, 255, 255, 255 };
    for ( auto& point : points ) {
        std::get<PC_IDX_COLOR>(point) = white;
    }
}

struct EfficientRANSACWrapper {
    EfficientRANSACWrapper( double prob, int minPoints, double eps, double epsCluster, double normThres ) {
        er.add_shape_factory<ERPlane_t>();

        erParams.probability      = prob;
        erParams.min_points       = minPoints;
        erParams.epsilon          = eps;
        erParams.cluster_epsilon  = epsCluster;
        erParams.normal_threshold = normThres;
    }

    void detect( PC_t& points ) {
        FUNCTION_SCOPE_TIMER

        er.set_input( points );

        std::cout << "Pre-processing...\n";
        er.preprocess();

        std::cout << "Detecting...\n";
        er.detect(erParams);

        EfficientRansac_t::Shape_range erShapes = er.shapes();
        EfficientRansac_t::Shape_range::iterator ersIter = erShapes.begin();

        std::cout << "Assign planes...\n";
        planes.clear();
        pointPlaneIndices.assign( points.size(), -1 );
        for ( int i = 0; ersIter != erShapes.end(); ersIter++, i++ ) {
            boost::shared_ptr< EfficientRansac_t::Shape > shape = *ersIter;
            auto pPlane = reinterpret_cast<ERPlane_t*>( shape.get() );
            const auto &normal = pPlane->plane_normal();

            planes.push_back( {
                          static_cast<Float_t>(normal.x()),
                          static_cast<Float_t>(normal.y()),
                          static_cast<Float_t>(normal.z()),
                          static_cast<Float_t>(pPlane->d()) } );

            auto ptIdxIter = shape->indices_of_assigned_points().begin();
            while ( ptIdxIter != shape->indices_of_assigned_points().end() ) {
                pointPlaneIndices[ *ptIdxIter ] = i;
                ptIdxIter++;
            }
        }
    }

    EfficientRansac_t er;
    EfficientRansac_t::Parameters erParams;
    PlaneContainer_t planes;
    std::vector<int> pointPlaneIndices;
};

static void move_points(
        const PlaneContainer_t& planes,
        const std::vector<int> pointPlaneIndices,
        PC_t& points ) {
    FUNCTION_SCOPE_TIMER

    const int N = points.size();

    for ( int i = 0; i < N; i++ ) {
        const auto planeIdx = pointPlaneIndices[i];
        if ( -1 == planeIdx ) continue;

        auto& plane = planes[ planeIdx ];
        auto& point = points[i];

        auto& coor = std::get<PC_IDX_POINT>(point);
        const auto a = plane[0];
        const auto b = plane[1];
        const auto c = plane[2];
        const auto t =
                ( -plane[3]
                  - a * coor.x() - b * coor.y() - c * coor.z()
                  ) / ( a * a + b * b + c * c );
        std::get<PC_IDX_POINT>(point) = Point_t(
                coor.x() + t * a, coor.y() + t * b, coor.z() + t * c );
    }
}

static ap::Args handle_args( int argc, char** argv ) {
    ap::Args args;

    args.add_positional<std::string>("in-cloud", "Input point cloud with normal. ");
    args.add_positional<std::string>("out-dir", "Output directory. ");
    args.add_positional<std::string>("out-name", "Output mesh filename relative to out-dir. ");

    args.add_default<double>("ransac-prob", "The main probability of the RANSAC algorithm. ", 0.05);
    args.add_default<int>("ransac-min-points", "The minimum points of the RANSAC algorithm. ", 2000);
    args.add_default<double>("ransac-epsilon", "The epsilon parameter of the RANSAC algorithm. ", 0.01);
    args.add_default<double>("ransac-cluster-epsilon", "The cluster epsilon parameter of the RANSAC algorithm. ", 0.1);
    args.add_default<double>("ransac-normal-thres", "The normal threshold of the RANSAC algorithm. ", 0.9);

    args.add_flag("move", "Set this flag to move the points. ");
    args.add_flag( "remove", "Set this flag to remove points without plane association. " );

    args.parse_args( argc, argv );

    std::cout << args;

    return args;
}

int main( int argc, char** argv ) {
    std::cout << "Hello, MovingPointsByPlaneDetection! \n";

    // Handle the arguments.
    auto args = handle_args( argc, argv );
    const std::string inCloudFn = args.arguments<std::string>["in-cloud"]->get();
    const std::string outDir    = args.arguments<std::string>["out-dir"]->get();
    const std::string outFn     = args.arguments<std::string>["out-name"]->get();

    const double ransacProb        = args.arguments<double>["ransac-prob"]->get();
    const int    ransacMinPoints   = args.arguments<int>["ransac-min-points"]->get();
    const double ransacEpsilon     = args.arguments<double>["ransac-epsilon"]->get();
    const double ransacClusterEps  = args.arguments<double>["ransac-cluster-epsilon"]->get();
    const double ransacNormalThres = args.arguments<double>["ransac-normal-thres"]->get();

    const bool flagMove   = args.arguments<bool>["move"]->get();
    const bool flagRemove = args.arguments<bool>["remove"]->get();

    // Test the output directory.
    test_directory(outDir);

    // Read the point cloud.
    PC_t points = cc::read_points_normal_from_ply<PCPoint_t, PCMap_Point_t, PCMap_Normal_t>(inCloudFn);
    std::cout << "points.size() = " << points.size() << "\n";

    // Assing white color to all the points.
    assign_white_color( points );

    // Shape detection.
    EfficientRANSACWrapper erw { ransacProb, ransacMinPoints, ransacEpsilon, ransacClusterEps, ransacNormalThres };
    erw.detect( points );


    if ( flagMove ) {
        std::cout << "Move points... \n";
        move_points( erw.planes, erw.pointPlaneIndices, points );
    }

    std::cout << "Coloring the points...\n";
    EfficientRansac_t::Shape_range erShapes = erw.er.shapes();
    EfficientRansac_t::Shape_range::iterator ersIter = erShapes.begin();

    PC_t* outPC = nullptr;
    PC_t removed;

    if ( flagRemove ) {
        for ( int i = 0; ersIter != erShapes.end(); ersIter++, i++ ) {
            boost::shared_ptr< EfficientRansac_t::Shape > shape = *ersIter;
//            std::cout << i << ": " << shape->info() << "\n";

            auto ptIdxIter = shape->indices_of_assigned_points().begin();

            // Prepare the color.
            const auto color = pcu::next_color(gCommonColor);

            while ( ptIdxIter != shape->indices_of_assigned_points().end() ) {
                PCPoint_t &point = points[ *ptIdxIter ];
                std::get<PC_IDX_COLOR>(point) = color;
                removed.push_back(point);
                ptIdxIter++;
            }
        }

        outPC = &removed;
    } else {
        for ( int i = 0; ersIter != erShapes.end(); ersIter++, i++ ) {
            boost::shared_ptr< EfficientRansac_t::Shape > shape = *ersIter;
//            std::cout << i << ": " << shape->info() << "\n";

            auto ptIdxIter = shape->indices_of_assigned_points().begin();

            // Prepare the color.
            const auto color = pcu::next_color(gCommonColor);

            while ( ptIdxIter != shape->indices_of_assigned_points().end() ) {
                PCPoint_t &point = points[ *ptIdxIter ];
                std::get<PC_IDX_COLOR>(point) = color;
                ptIdxIter++;
            }
        }

        outPC = &points;
    }

    // Write the colored point cloud.
    std::stringstream outCloudSS;
    outCloudSS << outDir << "/" << outFn;
    cc::write_points_normal_color_ply<
            PCPoint_t, PCMap_Point_t, PCMap_Normal_t, PCMap_Color_t>( outCloudSS.str(), *outPC );

    return 0;
}