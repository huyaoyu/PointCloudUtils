//
// Created by yaoyu on 7/12/20.
//

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <set>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>

#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/facets_in_complex_2_to_triangle_mesh.h>

#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/draw_polyhedron.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

#include "CGALCommon/IO.hpp"
#include "Profiling/ScopeTimer.hpp"

// CGAL local typedefs.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3              Point_t;
typedef CGAL::Surface_mesh<Point_t>    Mesh_t;
typedef Mesh_t::Vertex_index           VertexIdx_t;
typedef Mesh_t::Face_index             FaceIdx_t;
typedef Mesh_t::Halfedge_index         HalfedgeIdx_t;

// For surface mesh generation.
typedef CGAL::Surface_mesh_default_triangulation_3 Tr_t;
typedef CGAL::Complex_2_in_triangulation_3<Tr_t>   C2t3_t;

typedef Tr_t::Geom_traits GT_t;
typedef GT_t::Sphere_3    Sphere_t;
typedef GT_t::Point_3     SPoint_t;
typedef GT_t::FT          FT_t;

typedef FT_t (*Function)(SPoint_t);

typedef CGAL::Implicit_surface_3<GT_t, Function> ImpSurface_t;

// For Polyhedron_3.
typedef CGAL::Polyhedron_3<Kernel_t> Polyhedron_t;
typedef Polyhedron_t::Halfedge_const_handle PHCHalfedgeHandle_t;
typedef Polyhedron_t::Vertex_const_handle   PHCVertexHandle_t;

// For AABB Tree.
typedef Kernel_t::Ray_3 Ray_t;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron_t> AABBPrimitive_t;
typedef CGAL::AABB_traits<Kernel_t, AABBPrimitive_t>           AABBTraits_t;
typedef CGAL::AABB_tree<AABBTraits_t>                          AABBTree_t;

SPoint_t g_SphereCenter { 0.0, 0.0, 0.0 };
FT_t g_RadiusSqd = 1.0;

FT_t sphere_function ( SPoint_t p ) {
    const FT_t dx = p.x() - g_SphereCenter.x();
    const FT_t dy = p.y() - g_SphereCenter.y();
    const FT_t dz = p.z() - g_SphereCenter.z();

    const FT_t x2 = dx * dx, y2 = dy * dy, z2 = dz * dz;
    return x2 + y2 + z2 - g_RadiusSqd;
}

template < typename KT, typename PT >
static void compute_non_close_centroid(
        const CGAL::Polyhedron_3<KT> polyhedron,
        PT &centroid,
        double &maxDistSqd) {

    typename CGAL::Polyhedron_3<KT>::Vertex_const_iterator vertexIter    = polyhedron.vertices_begin();
    typename CGAL::Polyhedron_3<KT>::Vertex_const_iterator vertexIterEnd = polyhedron.vertices_end();

    double x = 0.0, y = 0.0, z = 0.0;

    for ( ; vertexIter != vertexIterEnd; vertexIter++ ) {
        x += vertexIter->point().x();
        y += vertexIter->point().y();
        z += vertexIter->point().z();
    }

    const int N = polyhedron.size_of_vertices();
    const double cx = x/N;
    const double cy = y/N;
    const double cz = z/N;
    centroid = PT( cx, cy, cz );

    maxDistSqd = 0.0;
    vertexIter = polyhedron.vertices_begin();
    for ( ; vertexIter != vertexIterEnd; vertexIter++ ) {
        x = vertexIter->point().x() - cx; x = x * x;
        y = vertexIter->point().y() - cy; y = y * y;
        z = vertexIter->point().z() - cz; z = z * z;

        // Re-use x;
        x = x + y + z;

        if ( x > maxDistSqd ) maxDistSqd = x;
    }
}

template < typename KT0, typename KT1 >
static void create_envelop_mesh(
        const CGAL::Polyhedron_3<KT0> &ph0,
        CGAL::Polyhedron_3<KT1> &ph1,
        typename KT0::Point_3 &centroid ) {
    ScopeTimer timer(__func__);

    double maxDistSqd = 0.0;
    compute_non_close_centroid( ph0, centroid, maxDistSqd );

    std::cout << "centroid = " << centroid << "\n";
    std::cout << "maxDistSqd = " << maxDistSqd << "\n";

    g_SphereCenter = SPoint_t( centroid.x(), centroid.y(), centroid.z() );
    g_RadiusSqd    = maxDistSqd;

    FT_t bound = std::sqrt( maxDistSqd ) + 0.1;
    bound = bound * bound;

    Tr_t tr;
    C2t3_t c2t3(tr);

    ImpSurface_t impSurface(
            sphere_function,
            Sphere_t( g_SphereCenter, bound ) );

    CGAL::Surface_mesh_default_criteria_3<Tr_t> criteria( 30., // Angular bound.
                                                        0.1,     // Radius bound.
                                                        0.1 ); // Distance bound.
    std::cout << "Generating surface mesh for the sphere... \n";
    CGAL::make_surface_mesh(c2t3, impSurface, criteria, CGAL::Manifold_tag());

    CGAL::facets_in_complex_2_to_triangle_mesh( c2t3, ph1 );
}

template < typename PT >
static int surface_mesh_border_vertex_count( const CGAL::Surface_mesh<PT> &sm ) {
    int count = 0;
    typedef typename CGAL::Surface_mesh<PT>::vertex_index VT;

    // Loop over all the halfedges.
    for ( const VT &vt : sm.vertices() ) {
        if ( sm.is_border(vt, true) ) {
            count++;
        }
    }

    return count;
}

template < typename KT >
static int polyhedron_mesh_border_vertex_count( const CGAL::Polyhedron_3<KT> &polyhedron ) {
    typedef typename CGAL::Polyhedron_3<KT>::Halfedge_const_iterator HCIter;
    typedef typename CGAL::Polyhedron_3<KT>::Vertex_const_handle VCH;

    HCIter iter = polyhedron.border_edges_begin();
    HCIter iterEnd = polyhedron.halfedges_end();

    std::set<VCH> borderVertexSet;
    for ( ; iter != iterEnd; iter++ ) {
        if ( iter->is_border() ) {
            borderVertexSet.insert( iter->vertex() );
        }
    }

    return borderVertexSet.size();
}

int main( int argc, char **argv ) {
    std::cout << "Hello, EnvelopSphere! \n";

    if ( argc == 1 ) {
        std::stringstream ss;
        throw std::runtime_error("Not enough arguments. ");
    }

    // Load the surface mesh.
    Mesh_t surfaceMesh;
    read_mesh_ply(argv[1], surfaceMesh);

    std::cout << "surfaceMesh.number_of_vertices() = " << surfaceMesh.number_of_vertices() << "\n";
    std::cout << "Number of border vertices is "
              << surface_mesh_border_vertex_count(surfaceMesh) << "\n";

    // Copy the surface mesh to polyhedron mesh.
    Polyhedron_t polyhedron;
    CGAL::copy_face_graph( surfaceMesh, polyhedron );
    polyhedron.normalize_border();

    std::cout << "polyhedron.size_of_vertices() = " << polyhedron.size_of_vertices() << "\n";
    std::cout << "Number of border vertices is " << polyhedron_mesh_border_vertex_count(polyhedron) << "\n";

    // Generate an envelope sphere mesh.
    Polyhedron_t sphere;
    Point_t centroid;
    create_envelop_mesh(polyhedron, sphere, centroid);
    write_mesh_ply("EnvelopSphere.ply", sphere, true);

    // AABB tree.
    AABBTree_t aabbTree( faces(polyhedron).first, faces(polyhedron).second, polyhedron );

    Polyhedron_t::Vertex_const_iterator pvcIter    = polyhedron.vertices_begin();
    Polyhedron_t::Vertex_const_iterator PVCIterEnd = polyhedron.vertices_end();

    int intersectCount = 0;
    for ( ; pvcIter != PVCIterEnd; pvcIter++ ) {
        Ray_t rayQuery( centroid, pvcIter->point() );
        if ( aabbTree.do_intersect( rayQuery ) ) {
            intersectCount++;
        }
    }

    std::cout << "intersectCount = " << intersectCount << "\n";

//    CGAL::draw(surfaceMesh);
//    CGAL::draw(polyhedron);
    CGAL::draw(sphere);

    return 0;
}