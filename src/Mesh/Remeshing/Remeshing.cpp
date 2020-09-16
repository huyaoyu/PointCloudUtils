//
// Created by yaoyu on 9/5/20.
//

#include <iostream>
#include <string>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Surface_mesh.h>

#include "Args/ArgsParser.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/ScopeTimer.hpp"

#include "CGALCommon/IO.hpp"

// Namespace.
namespace PMP = CGAL::Polygon_mesh_processing;

// CGAL typedefs.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3                                   Point3_t;
typedef CGAL::Surface_mesh<Point3_t>                        SurfaceMesh_t;

typedef boost::graph_traits< SurfaceMesh_t >::face_descriptor DescFace_t;
typedef boost::graph_traits< SurfaceMesh_t >::edge_descriptor DescEdge_t;
typedef boost::graph_traits< SurfaceMesh_t >::halfedge_descriptor DescHalfedge_t;

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
    FUNCTION_SCOPE_TIMER

    typedef boost::property_map< SurfaceMesh_t, CGAL::face_patch_id_t<int> >::type PatchIDPMap_t;

    PatchIDPMap_t fPMap = get( CGAL::face_patch_id_t<int>(), mesh );
    bool fPMapValid = false;
    {
        for ( DescFace_t f : faces(mesh) ) {
            if ( get(fPMap, f) != 1 ) {
                fPMapValid = true;
                break;
            }
        }
    }

    std::cout << "fPMapValid = " << fPMapValid << "\n";

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

static ap::Args handle_args( int argc, char** argv ) {
    ap::Args args;

    args.add_positional<std::string>("in-cloud", "Input point cloud with normal. ");
    args.add_positional<std::string>("out-dir", "Output directory. ");
    args.add_positional<std::string>("out-name", "Output mesh filename relative to out-dir. ");
    args.add_positional<double>("target-length", "The target edge length of the remeshed mesh. ");
    args.add_default<int>("iters", "Number of iterations. ", 1);

    args.parse_args( argc, argv );

    std::cout << args;

    return args;
}

int main( int argc, char **argv ) {
    NAMED_SCOPE_TIMER(main)
    std::cout << "Hello, Mesh_Remeshing! \n";

    auto args = handle_args( argc, argv );

    std::string inCloudFn = args.arguments<std::string>["in-cloud"]->get();
    std::string outDir    = args.arguments<std::string>["out-dir"]->get();
    std::string outFn     = args.arguments<std::string>["out-name"]->get();
    const double targetLength = args.arguments<double>["target-length"]->get();
    const int iters = args.arguments<int>["iters"]->get();

    test_directory( outDir );

    // Load the mesh from the PLY file.
    SurfaceMesh_t mesh;
    read_mesh_ply( inCloudFn, mesh);

    // Check the mesh.
    if ( !CGAL::is_valid_polygon_mesh(mesh) ) {
        std::stringstream ss;
        ss << "Not a valid mesh. ";
        throw std::runtime_error( ss.str() );
    }

    // Remesh.
    isotropic_remesh( mesh, targetLength, iters );

    // Save the mesh.
    {
        std::stringstream ss;
        ss << outDir << "/" << outFn;
        write_mesh_ply( ss.str(), mesh );
    }

    return 0;
}
