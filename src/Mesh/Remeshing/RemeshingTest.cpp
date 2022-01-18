//
// Created by yaoyu on 9/6/20.
//

#include <fstream>
#include <iostream>
#include <string>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Surface_mesh.h>

// Namespace.
namespace PMP = CGAL::Polygon_mesh_processing;

// CGAL typedefs.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3                                   Point3_t;
typedef CGAL::Surface_mesh<Point3_t>                        SurfaceMesh_t;

typedef boost::graph_traits< SurfaceMesh_t >::face_descriptor DescFace_t;
typedef boost::graph_traits< SurfaceMesh_t >::edge_descriptor DescEdge_t;
typedef boost::graph_traits< SurfaceMesh_t >::halfedge_descriptor DescHalfedge_t;
typedef boost::graph_traits< SurfaceMesh_t >::vertex_descriptor DescVert_t;

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

struct halfedge2edge
{
    halfedge2edge(const SurfaceMesh_t &m, std::vector<DescEdge_t> &edges)
            : m_mesh(m), m_edges(edges)
    {}

    void operator()(const DescHalfedge_t &h) const
    {
        m_edges.push_back(edge(h, m_mesh));
    }

    const SurfaceMesh_t& m_mesh;
    std::vector<DescEdge_t>& m_edges;
};

static void isotropic_remesh( SurfaceMesh_t &mesh, double targetLength, int iters ) {
    // Border.
    std::cout << "Split border...\n";
    std::vector<DescEdge_t> border;
    PMP::border_halfedges(faces(mesh),
                          mesh,
                          boost::make_function_output_iterator(halfedge2edge(mesh, border)));
    PMP::split_long_edges(border, targetLength, mesh);

    // Perform smooth.
    std::cout << "Start remeshing... \n";
    PMP::isotropic_remeshing(
            faces( mesh ),
            targetLength,
            mesh,
            PMP::parameters::number_of_iterations(iters)
                    .protect_constraints(true) );
}

int main( int argc, char **argv ) {
    std::cout << "Hello, RemeshingTest! \n";

    // Load the mesh from the PLY file.
    SurfaceMesh_t mesh;
    read_mesh_ply( argv[1], mesh);

    auto res = CGAL::is_valid_polygon_mesh(mesh);
    std::cout << "rs = " << res << "\n";

    // Remesh.
    isotropic_remesh( mesh, 0.02, 1 );

    return 0;
}