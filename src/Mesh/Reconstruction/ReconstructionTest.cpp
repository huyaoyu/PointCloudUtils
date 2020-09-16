//
// Created by yaoyu on 9/8/20.
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
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Surface_mesh.h>

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

    MeshConstructorByPointIndices& operator *  () { return *this; }
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
    std::ifstream ifs { fn };
    if ( !ifs ) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for reading. ";
        throw std::runtime_error( ss.str() );
    }

    std::vector<Point3_t> cgalPoints;
    if ( !CGAL::read_ply_points( ifs, std::back_inserter( cgalPoints ) ) ) {
        std::stringstream ss;
        ss << "read_ply() from " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ifs.close();

    return cgalPoints;
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

static void reconstruct_surface_mesh(
        const std::vector<Point3_t> &points, SurfaceMesh_t &mesh,
        double longestEdge=0.1, double radiusRatioBound=5.0, double betaDeg=30.0 ) {
    std::cout << "Begin reconstruction by AFS. \n";
    PriorityRadius priority(longestEdge);
    MeshConstructorByPointIndices meshConstructor( mesh, points );
    std::cout << "AFS... \n";
    CGAL::advancing_front_surface_reconstruction(
            points.begin(), points.end(),
            meshConstructor, priority,
            radiusRatioBound, betaDeg/180.0*G_PI );
}

int main( int argc, char **argv ) {
    std::cout << "Hello, Mesh_Reconstruction! \n";

    if ( argc != 3 ) {
        std::stringstream ss;
        ss << "Expecting 3 input arguments. " << argc << " given. ";
        throw std::runtime_error( ss.str() );
    }

    std::string inPointsFn = argv[1];
    std::string outMeshFn  = argv[2];

    // Load the PLY point cloud by PCL and convert the points into CGAL point representation.
    auto points = read_points_from_ply( inPointsFn );

    SurfaceMesh_t mesh;
    reconstruct_surface_mesh( points, mesh );

    // Check the reconstructed mesh.
    auto res = CGAL::is_valid_polygon_mesh(mesh);
    std::cout << "rs = " << res << "\n";

    // Remove isolated points.
    auto nIsolated = PMP::remove_isolated_vertices( mesh );
    std::cout << "nIsolated = " << nIsolated << "\n";

    // Save the mesh.
    write_mesh_ply( outMeshFn, mesh );

    return 0;
}
