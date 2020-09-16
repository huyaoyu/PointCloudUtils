//
// Created by yaoyu on 9/5/20.
//
// Some of the codes are copied from
// https://doc.cgal.org/latest/Advancing_front_surface_reconstruction/Advancing_front_surface_reconstruction_2reconstruction_surface_mesh_8cpp-example.html
// Some of the codes are copied from
// cgal/Polyhedron/demo/Polyhedron/Plugins/Point_set/Surface_reconstruction_advancing_front_impl.cpp
//

#include <algorithm>
#include <iostream>
#include <string>

#include <boost/math/constants/constants.hpp>

#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/array.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Surface_mesh.h>

#include <pcl/point_types.h>

#include "Args/ArgsParser.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/IO.hpp"
#include "Profiling/ScopeTimer.hpp"

#include "CGALCommon/IO.hpp"

// Namespace.
namespace PMP = CGAL::Polygon_mesh_processing;

// Global constants.
const auto G_PI = boost::math::constants::pi<double>();

// Typedefs for CGAL.
typedef std::array< std::size_t, 3 > Facet_t;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3                                   Point3_t;
typedef CGAL::Surface_mesh<Point3_t>                        SurfaceMesh_t;

struct MeshConstructorByPointIndices {
    typedef typename boost::property_map< SurfaceMesh_t, boost::vertex_point_t >::type VPMap_t;

    SurfaceMesh_t &mesh;
    VPMap_t       vpMap;
    std::vector< typename boost::graph_traits<SurfaceMesh_t>::vertex_descriptor > vertices;

    template < typename Point_T >
    MeshConstructorByPointIndices( SurfaceMesh_t &m, const std::vector<Point_T> &points )
    : mesh{m} {
        vpMap = get(boost::vertex_point, mesh);
        for ( const auto& p : points ) {
            boost::graph_traits<SurfaceMesh_t>::vertex_descriptor v;
            v = add_vertex( mesh );
            vertices.push_back(v);
            put( vpMap, v, p );
        }
    }

    MeshConstructorByPointIndices& operator = ( const Facet_t f ) {
        typedef boost::graph_traits<SurfaceMesh_t>::vertex_descriptor DescV_t;
        std::vector< DescV_t > facet(3);

        facet[0] = vertices[ f[0] ];
        facet[1] = vertices[ f[1] ];
        facet[2] = vertices[ f[2] ];

        CGAL::Euler::add_face( facet, mesh );

        return *this;
    }

    MeshConstructorByPointIndices& operator * () { return *this; }
    MeshConstructorByPointIndices& operator ++ () { return *this; }
    MeshConstructorByPointIndices  operator ++ (int) { return *this; }
};

struct PriorityRadius {
    double bound;

    explicit PriorityRadius ( double b )
    : bound{b} {}

    template < typename AdvFront_T, typename CellHandle_T >
    double operator () ( const AdvFront_T &adv, CellHandle_T &c, const int &index ) const {
        if ( 0 == bound ) {
            return adv.smallest_radius_delaunay_sphere( c, index );
        }

        double d = 0;
        d = sqrt( CGAL::squared_distance( c->vertex( (index+1)%4 )->point(),
                                          c->vertex( (index+2)%4 )->point() ) );
        if ( d > bound ) return adv.infinity();

        d = sqrt( CGAL::squared_distance( c->vertex( (index+2)%4 )->point(),
                                          c->vertex( (index+3)%4 )->point() ) );
        if ( d > bound ) return adv.infinity();

        d = sqrt( CGAL::squared_distance( c->vertex( (index+1)%4 )->point(),
                                          c->vertex( (index+3)%4 )->point() ) );
        if ( d > bound ) return adv.infinity();

        return adv.smallest_radius_delaunay_sphere( c, index );
    }
};

static std::vector<Point3_t> read_points_from_ply( const std::string &fn ) {
    typedef pcl::PointXYZ           pclP_t;
    typedef pcl::PointCloud<pclP_t> pclPC_t;

    pclPC_t::Ptr pInPCL = pcu::read_point_cloud<pclP_t>(fn);

    const int N = pInPCL->size();
    std::vector<Point3_t> cgalPoints;
    cgalPoints.reserve( N );

    for ( int i = 0; i < N; ++i ) {
        const auto &pclPoint = (*pInPCL)[i];
        cgalPoints.emplace_back( pclPoint.x, pclPoint.y, pclPoint.z );
    }

    return cgalPoints;
}

static void reconstruct_surface_mesh(
        const std::vector<Point3_t> &points, SurfaceMesh_t &mesh,
        double longestEdge=0.01, double radiusRatioBound=5.0, double betaDeg=30.0 ) {
    FUNCTION_SCOPE_TIMER
    std::cout << "Begin reconstruction by AFS. \n";
    PriorityRadius priority(longestEdge);
    MeshConstructorByPointIndices meshConstructor( mesh, points );
    std::cout << "AFS... \n";
    CGAL::advancing_front_surface_reconstruction(
            points.begin(), points.end(),
            meshConstructor, priority,
            radiusRatioBound, betaDeg/180.0*G_PI );
}

static void remove_self_intersection_and_duplicate( SurfaceMesh_t &mesh ) {
    FUNCTION_SCOPE_TIMER

    typedef boost::graph_traits< SurfaceMesh_t >::face_descriptor DescFace_t;
    typedef boost::graph_traits< SurfaceMesh_t >::vertex_descriptor DescVert_t;

    std::cout << "Collecting intersected faces. \n";
    std::vector< std::pair< DescFace_t, DescFace_t > > intersectedFacePairs;
    PMP::self_intersections( mesh, std::back_inserter( intersectedFacePairs ) );

    std::unordered_set<DescFace_t , CGAL::Handle_hash_function> intersectedFaceSet;
    for ( const auto &p : intersectedFacePairs ) {
        intersectedFaceSet.insert( p.first );
        intersectedFaceSet.insert( p.second );
    }

    std::cout << intersectedFaceSet.size() << " faces to be removed. \n";

    std::cout << "Begin removing intersected faces. \n";
    for ( const auto &f : intersectedFaceSet ) {
        CGAL::Euler::remove_face( halfedge(f, mesh), mesh );
    }

    std::vector< std::vector< DescVert_t > > duplicatedVertices;
    int newVerticesNB = PMP::duplicate_non_manifold_vertices(
            mesh, CGAL::parameters::output_iterator( std::back_inserter( duplicatedVertices ) ) );
    std::cout << "newVerticesNB = " << newVerticesNB << "\n";
}

static ap::Args handle_args( int argc, char** argv ) {
    ap::Args args;

    args.add_positional<std::string>("in-cloud", "Input point cloud with normal. ");
    args.add_positional<std::string>("out-dir", "Output directory. ");
    args.add_positional<std::string>("out-name", "Output mesh filename relative to out-dir. ");
    args.add_default<double>("longest-edge", "The longest edge of the reconstructed surface mesh. ", 0.1);
    args.add_default<double>("radius-ratio-bound", "The radius ratio bound. ", 5.0);
    args.add_default<double>("beta-deg", "The beta angle in degree. ", 30.0);
    args.add_flag("clean-mesh", "Clean self-intersections and non-manifold vertices. ");

    args.parse_args( argc, argv );

    std::cout << args;

    return args;
}

int main( int argc, char **argv ) {
    NAMED_SCOPE_TIMER(main)
    std::cout << "Hello, Mesh_Reconstruction! \n";

    auto args = handle_args( argc, argv );

    std::string inCloudFn = args.arguments<std::string>["in-cloud"]->get();
    std::string outDir    = args.arguments<std::string>["out-dir"]->get();
    std::string outFn     = args.arguments<std::string>["out-name"]->get();
    const double longestEdge      = args.arguments<double>["longest-edge"]->get();
    const double radiusRatioBound = args.arguments<double>["radius-ratio-bound"]->get();
    const double betaDeg          = args.arguments<double>["beta-deg"]->get();
    const bool   flagCleanMesh    = args.arguments<bool>["clean-mesh"]->get();

    test_directory( outDir );

    // Load the PLY point cloud by PCL and convert the points into CGAL point representation.
    auto points = read_points_from_ply( inCloudFn );

//    // Test.
//    const int N = points.size();
//    std::cout << "points.size() = " << N << "\n";

    SurfaceMesh_t mesh;
    reconstruct_surface_mesh( points, mesh, longestEdge, radiusRatioBound, betaDeg );
    auto res = CGAL::is_valid_polygon_mesh(mesh);
    std::cout << "res = " << res << "\n";

    // Remove isolated points.
    auto nIsolated = PMP::remove_isolated_vertices( mesh );
    std::cout << "nIsolated = " << nIsolated << "\n";

    // Handle self-intersection and non-manifold vertices.
    if ( flagCleanMesh ) remove_self_intersection_and_duplicate( mesh );

    // Save the surface mesh as a PLY file.
    {
        std::stringstream ss;
        ss << outDir << "/" << outFn;
        write_mesh_ply( ss.str(), mesh );
    }

    return 0;
}
