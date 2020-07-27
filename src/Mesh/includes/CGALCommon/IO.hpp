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

#include <CGAL/IO/write_ply_points.h>

template < typename PT >
void read_mesh_ply(const std::string &fn,
                   CGAL::Surface_mesh<PT> &sm ) {

    std::ifstream ifs { fn };

    if ( !ifs ) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for reading. ";
        throw std::runtime_error( ss.str() );
    }

    if ( !CGAL::read_ply( ifs, sm ) ) {
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

    CGAL::write_ply( ofs, sm );

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

    CGAL::write_off( ofs, sm );

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

#endif //INCLUDES_CGALCOMMON_IO_HPP
