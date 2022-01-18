//
// Created by yaoyu on 7/12/20.
//

#ifndef INCLUDES_CGALCOMMON_IO_HPP
#define INCLUDES_CGALCOMMON_IO_HPP

#include <fstream>
#include <iostream>
#include <string>

#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/property_map.h>

#include <CGAL/Surface_mesh/IO/OFF.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/write_ply_points.h>

#include "Visualization/Color.hpp"

template < typename PT >
void read_mesh_ply(const std::string &fn,
                   CGAL::Surface_mesh<PT> &sm ) {

    std::ifstream ifs { fn };

    if ( !ifs ) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for reading. ";
        throw std::runtime_error( ss.str() );
    }

    if ( !CGAL::IO::read_PLY( ifs, sm ) ) {
        std::stringstream ss;
        ss << "read_ply() from " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ifs.close();
}

template < typename PT >
void write_mesh_ply(const std::string &fn,
                    CGAL::Surface_mesh<PT> &sm,
                    bool flagBinary= true) {
    std::ofstream ofs;

    if ( flagBinary ) {
        ofs.open( fn, std::ios::binary );
    } else {
        ofs.open( fn );
    }

    if ( !ofs ) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for output. ";
        throw std::runtime_error( ss.str() );
    }

    CGAL::IO::write_PLY( ofs, sm );

    ofs.close();
}

template < typename KT >
void write_mesh_ply( const std::string &fn,
        CGAL::Polyhedron_3<KT> &polyhedron,
        bool flagBinary=true ) {

    CGAL::Surface_mesh<typename KT::Point_3> surfaceMesh;
    CGAL::copy_face_graph( polyhedron, surfaceMesh );

    write_mesh_ply(fn, surfaceMesh);
}

template < typename PT >
void write_mesh_off( const std::string &fn,
        CGAL::Surface_mesh<PT> &sm,
        bool flagBinary=true ) {
    std::ofstream ofs;

    if ( flagBinary ) {
        ofs.open( fn, std::ios::binary );
    } else {
        ofs.open( fn );
    }

    if ( !ofs ) {
        std::stringstream ss;
        ss << "Cannot open file " << fn;
        throw std::runtime_error( ss.str() );
    }

    CGAL::IO::write_OFF( ofs, sm );

    ofs.close();
}

template < typename PointRange >
void write_points_ply( const std::string &fn,
        const PointRange &points,
        bool flagBinary=true) {

    std::ofstream ofs;

    if ( flagBinary ) {
        ofs.open( fn, std::ios::binary );
    } else {
        ofs.open( fn );
    }

    CGAL::write_ply_points( ofs, points );

    ofs.close();
}

// For write color to a PLY file.
namespace CGAL {
    template < typename F >
    struct Output_rep< pcu::Color_t, F > {
        const pcu::Color_t &c;
        static const bool is_specialized = true;

        explicit Output_rep( const pcu::Color_t &c ) : c{c} { }

        std::ostream& operator() ( std::ostream &out ) const {
            if ( is_ascii(out) ) {
                out << int( c[0] ) << " "
                    << int( c[1] ) << " "
                    << int( c[2] ) << " "
                    << int( c[3] );
            } else {
                out.write( reinterpret_cast<const char*>( &c ), sizeof(c) );
            }

            return out;
        }
    };
}

namespace cgal_common
{

template < typename PCPoint_T, typename PCPointMap_T >
std::vector<PCPoint_T> read_points_from_ply( const std::string &fn ) {
    std::ifstream ifs { fn };
    if ( !ifs ) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for reading. ";
        throw std::runtime_error( ss.str() );
    }

    std::vector<PCPoint_T> points;

    if ( !CGAL::IO::read_PLY( ifs,
                                 std::back_inserter( points ),
                                 CGAL::parameters::point_map( PCPointMap_T() ) ) ) {
        std::stringstream ss;
        ss << "read_ply() from " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ifs.close();

    return points;
}

template < typename PCPoint_T, typename PCPointMap_T, typename PCNormalMap_T >
std::vector<PCPoint_T> read_points_normal_from_ply( const std::string &fn ) {
    std::ifstream ifs { fn };
    if ( !ifs ) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for reading. ";
        throw std::runtime_error( ss.str() );
    }

    std::vector<PCPoint_T> points;

    if ( !CGAL::IO::read_PLY_with_properties( ifs,
                                                 std::back_inserter( points ),
                                                 CGAL::IO::make_ply_point_reader( PCPointMap_T() ),
                                                 CGAL::IO::make_ply_normal_reader( PCNormalMap_T() )) ) {
        std::stringstream ss;
        ss << "read_ply() from " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ifs.close();

    return points;
}

template < typename PCPoint_T, typename PCPointMap_T, typename PCNormalMap_T, typename PCColorMap_T >
void write_points_normal_color_ply( const std::string &fn, const std::vector<PCPoint_T>& points ) {
    std::ofstream ofs(fn, std::ios::binary);

    if ( !ofs ) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for writing. ";
        throw std::runtime_error( ss.str() );
    }

    CGAL::set_binary_mode(ofs);

    if ( !CGAL::IO::write_PLY_with_properties( ofs,
                                                  points,
                                                  CGAL::make_ply_point_writer( PCPointMap_T() ),
                                                  CGAL::make_ply_normal_writer( PCNormalMap_T() ),
                                                  std::make_tuple(
                                                          PCColorMap_T(),
                                                          CGAL::IO::PLY_property<std::uint8_t>("red"),
                                                          CGAL::IO::PLY_property<std::uint8_t>("green"),
                                                          CGAL::IO::PLY_property<std::uint8_t>("blue"),
                                                          CGAL::IO::PLY_property<std::uint8_t>("alpha") )
    ) ) {
        std::stringstream ss;
        ss << "CGAL::write_ply_points_with_properties() failed. ";
        throw std::runtime_error( ss.str() );
    }

    ofs.close();
}

} // namespace cgal_common

#endif //INCLUDES_CGALCOMMON_IO_HPP
