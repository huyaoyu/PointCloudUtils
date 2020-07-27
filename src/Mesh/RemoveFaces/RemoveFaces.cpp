//
// Created by yaoyu on 7/13/20.
//

#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_set>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>

#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Handle_hash_function.h>

#include <boost/unordered_set.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3              Point_t;
typedef CGAL::Surface_mesh<Point_t>    Mesh_t;
typedef Mesh_t::Vertex_index           VertexIdx_t;
typedef Mesh_t::Face_index             FaceIdx_t;
typedef Mesh_t::Halfedge_index         HalfedgeIdx_t;

typedef boost::graph_traits<Mesh_t>::face_descriptor   FaceDesc_t;
typedef boost::graph_traits<Mesh_t>::edge_descriptor   EdgeDesc_t;
typedef boost::graph_traits<Mesh_t>::vertex_descriptor VertexDesc_t;

namespace pmp = CGAL::Polygon_mesh_processing;

static void smooth_mesh( Mesh_t &mesh ) {
    typedef boost::property_map<Mesh_t, CGAL::edge_is_feature_t>::type EIFMap;
    EIFMap eif = get( CGAL::edge_is_feature, mesh );

    pmp::detect_sharp_edges( mesh, 60, eif );

    int sharpCount = 0;
    for ( EdgeDesc_t e : edges(mesh) ) {
        if ( get(eif, e) ) {
            sharpCount++;
        }
    }

    std::cout << "sharpCount = " << sharpCount << "\n";

    const unsigned int nbIters = 10;

    std::cout << "Smoothing...\n";

    pmp::smooth_mesh( mesh, pmp::parameters::number_of_iterations(nbIters)
            .use_safety_constraints(false)
            .edge_is_constrained_map(eif) );
    std::cout << "Smoothing done. \n";
}

template < typename PT >
static void write_mesh_off( const std::string &fn,
       CGAL::Surface_mesh<PT> &mesh ) {
    std::ofstream ofs { fn, std::ios::binary };

    if ( !ofs ) {
        std::stringstream ss;
        ss << "Cannot open file " << fn << ". ";
        throw std::runtime_error( ss.str() );
    }

    CGAL::write_off( ofs, mesh );

    ofs.close();
}

int main( int argc, char **argv ) {
    std::cout << "Hello, RemoveFaces! \n";

    // Load the mesh.
    Mesh_t surfaceMesh;

    std::ifstream ifs( argv[1], std::ios::binary );

    if ( !ifs ) {
        throw std::runtime_error("Cannot open file. ");
    }

    CGAL::read_off( ifs, surfaceMesh );
    ifs.close();

    // Loop over all the intersected faces and remove them.
    std::cout << "Collecting intersected faces. \n";
    std::vector< std::pair< FaceDesc_t, FaceDesc_t > > intersectedFacePairs;
    pmp::self_intersections( surfaceMesh, std::back_inserter( intersectedFacePairs ) );

//    std::set<FaceDesc_t> intersectedFaceSet;
    std::unordered_set<FaceDesc_t, CGAL::Handle_hash_function> intersectedFaceSet;
    for ( const auto &p : intersectedFacePairs ) {
        intersectedFaceSet.insert( p.first );
        intersectedFaceSet.insert(p.second);
    }

    std::cout << intersectedFaceSet.size() << " faces to be removed. \n";

    std::cout << "Begin removing intersected faces. \n";
    for ( const auto &f : intersectedFaceSet ) {
//            for ( HalfedgeIdx_t h : CGAL::halfedges_around_face( surfaceMesh.halfedge(f), surfaceMesh ) ) {
//                if ( !surfaceMesh.is_border(h) ) CGAL::Euler::remove_face(h, surfaceMesh);
//            }
            CGAL::Euler::remove_face( halfedge(f, surfaceMesh), surfaceMesh );
//        surfaceMesh.remove_face(f);
    }
//    surfaceMesh.collect_garbage();

    write_mesh_off( "FacesRemoved.off", surfaceMesh );
    std::cout << "File saved to FacesRemoved.off. \n";

    std::vector< std::vector< VertexDesc_t > > duplicatedVertices;
    int newVerticesNB = pmp::duplicate_non_manifold_vertices(
            surfaceMesh, CGAL::parameters::output_iterator( std::back_inserter( duplicatedVertices ) ) );
    std::cout << "newVerticesNB = " << newVerticesNB << "\n";
    write_mesh_off( "NonManifoldVertexDuplicated.off", surfaceMesh );

    // Smoothing.
    smooth_mesh( surfaceMesh );

    write_mesh_off( "Smoothed.off", surfaceMesh );
    std::cout << "File saved to Smoothed.off. \n";

    return 0;
}