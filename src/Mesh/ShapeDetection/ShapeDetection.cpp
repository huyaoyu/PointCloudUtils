//
// Created by yaoyu on 9/15/20.
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

#include "Visualization/Color.hpp"

// Namespace.
namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3                                   Point3_t;
typedef CGAL::Surface_mesh<Point3_t>                        SurfaceMesh_t;

typedef std::uint8_t                                        ColorByte_t;
typedef std::array< ColorByte_t, 4 >                        Color_t;
typedef std::tuple< Point3_t, Kernel_t::Vector_3, Color_t > PNC_t;
typedef std::vector< PNC_t >                                PNCVector_t;
constexpr int PNC_IDX_POINT  = 0;
constexpr int PNC_IDX_NORMAL = 1;
constexpr int PNC_IDX_COLOR  = 2;
typedef CGAL::Nth_of_tuple_property_map< PNC_IDX_POINT,  PNC_t > PNCMap_Point_t;
typedef CGAL::Nth_of_tuple_property_map< PNC_IDX_NORMAL, PNC_t > PNCMap_Normal_t;
typedef CGAL::Nth_of_tuple_property_map< PNC_IDX_COLOR,  PNC_t > PNCMap_Color_t;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits<
    Kernel_t, PNCVector_t, PNCMap_Point_t, PNCMap_Normal_t >   ERTraits_t;
typedef CGAL::Shape_detection::Efficient_RANSAC< ERTraits_t > EfficientRansac_t;
typedef CGAL::Shape_detection::Plane< ERTraits_t >            ERPlane_t;

// For write color to a PLY file.
namespace CGAL {
    template < typename F >
    struct Output_rep< ::Color_t, F > {
        const ::Color_t &c;
        static const bool is_specialized = true;

        explicit Output_rep( const ::Color_t &c ) : c{c} { }

        std::ostream& operator() ( std::ostream &out ) const {
            if ( is_ascii(out) ) {
                out << int( c[0] ) << " "
                    << int( c[1] ) << " "
                    << int( c[2] ) << " "
                    << int( c[3] );
            } else {
                out.write( reinterpret_cast<const char*>( &c ), sizeof(c) );
            }
        }
    };
}

static PNCVector_t read_points_normal_from_ply( const std::string &fn ) {
    std::ifstream ifs { fn };
    if ( !ifs ) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for reading. ";
        throw std::runtime_error( ss.str() );
    }

    PNCVector_t pncVector;
    if ( !CGAL::read_ply_points_with_properties( ifs,
                                                 std::back_inserter( pncVector ),
                                                 CGAL::make_ply_point_reader( PNCMap_Point_t() ),
                                                 CGAL::make_ply_normal_reader( PNCMap_Normal_t() )) ) {
        std::stringstream ss;
        ss << "read_ply() from " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ifs.close();

    return pncVector;
}

static void write_points_normal_color_ply( const std::string &fn, const PNCVector_t &points ) {
    std::ofstream ofs(fn, std::ios::binary);

    if ( !ofs ) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for writing. ";
        throw std::runtime_error( ss.str() );
    }

    CGAL::set_binary_mode(ofs);

    if ( !CGAL::write_ply_points_with_properties( ofs,
                                                  points,
                                                  CGAL::make_ply_point_writer( PNCMap_Point_t() ),
                                                  CGAL::make_ply_normal_writer( PNCMap_Normal_t() ),
                                                  std::make_tuple(
                                                          PNCMap_Color_t(),
                                                          CGAL::PLY_property<std::uint8_t>("red"),
                                                          CGAL::PLY_property<std::uint8_t>("green"),
                                                          CGAL::PLY_property<std::uint8_t>("blue"),
                                                          CGAL::PLY_property<std::uint8_t>("alpha") )
                                                  ) ) {
        std::stringstream ss;
        ss << "CGAL::write_ply_points_with_properties() failed. ";
        throw std::runtime_error( ss.str() );
    }

    ofs.close();
}

pcu::CommonColor gCommonColor;

union RGBA {
    std::uint32_t bgra;
    struct {
        std::uint8_t b;
        std::uint8_t g;
        std::uint8_t r;
        std::uint8_t a;
    };
};

static Color_t next_color() {
    RGBA color;
    color.bgra = gCommonColor.next();

    std::cout << "color = [ "
              << static_cast<int>( color.b ) << ", "
              << static_cast<int>( color.g ) << ", "
              << static_cast<int>( color.r ) << ", "
              << static_cast<int>( color.a ) << " ]\n";

    return { color.r, color.g, color.b, color.a };
}

int main(int argc, char **argv) {
    assert( argc >= 3 );

    std::cout << "Hello, Mesh_ShapeDetection! \n";

    std::string inCloudFn  = argv[1];
    std::string outCloudFn = argv[2];

    auto pncVector = read_points_normal_from_ply( inCloudFn );

    // Debug use.
    std::cout << "pncVector.size() = " << pncVector.size() << "\n";
    std::cout << "std::get<0>(pncVector[0]) = " << std::get<PNC_IDX_POINT>(pncVector[0]) << "\n";
    std::cout << "std::get<1>(pncVector[0]) = " << std::get<PNC_IDX_NORMAL>(pncVector[0]) << "\n";

    // Shape detection engine.
    EfficientRansac_t er;
    er.set_input( pncVector );
    er.add_shape_factory<ERPlane_t>();

    std::cout << "Pre-processing...\n";
    er.preprocess();

    EfficientRansac_t::Parameters erParams;
    erParams.probability      = 0.05;
    erParams.min_points       = 1000;
    erParams.epsilon          = 0.002;
    erParams.cluster_epsilon  = 0.1;
    erParams.normal_threshold = 0.9;

    std::cout << "Detecting...\n";
    er.detect(erParams);

    EfficientRansac_t::Shape_range erShapes = er.shapes();
    EfficientRansac_t::Shape_range::iterator ersIter = erShapes.begin();

    std::cout << "Coloring the points...\n";
    for ( int i = 0; ersIter != erShapes.end(); ersIter++, i++ ) {
        boost::shared_ptr< EfficientRansac_t::Shape > shape = *ersIter;
        std::cout << i << ": " << shape->info() << "\n";

        auto ptIdxIter = shape->indices_of_assigned_points().begin();

        // Prepare the color.
        const auto color = next_color();

        while ( ptIdxIter != shape->indices_of_assigned_points().end() ) {
            PNC_t &point = pncVector[ *ptIdxIter ];
            std::get<PNC_IDX_COLOR>(point) = color;
            ptIdxIter++;
        }
    }

    // Write the colored point cloud.
    write_points_normal_color_ply( outCloudFn, pncVector );

    return 0;
}