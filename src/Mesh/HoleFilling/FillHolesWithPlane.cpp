//
// Created by yaoyu on 9/16/20.
//

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <set>
#include <unordered_set>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Handle_hash_function.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Args/ArgsParser.hpp"
#include "CGALCommon/IO.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/IO.hpp"
#include "Profiling/ScopeTimer.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3              Point_t;
typedef Kernel_t::Vector_3             Vector_t;
typedef CGAL::Surface_mesh<Point_t>    Mesh_t;
typedef Mesh_t::Vertex_index           VertexIdx_t;
typedef Mesh_t::Face_index             FaceIdx_t;

typedef boost::graph_traits<Mesh_t>::face_descriptor     FaceDesc_t;
typedef boost::graph_traits<Mesh_t>::edge_descriptor     EdgeDesc_t;
typedef boost::graph_traits<Mesh_t>::halfedge_descriptor HalfedgeDesc_t;
typedef boost::graph_traits<Mesh_t>::vertex_descriptor   VertexDesc_t;

constexpr int IDX_POINT  = 0;
constexpr int IDX_NORMAL = 1;
constexpr int IDX_VDESC  = 2;
typedef std::tuple< Point_t, Vector_t, VertexDesc_t > MeshPoint_t;
typedef CGAL::Nth_of_tuple_property_map< IDX_POINT,  MeshPoint_t > MP_Point_t;
typedef CGAL::Nth_of_tuple_property_map< IDX_NORMAL, MeshPoint_t > MP_Normal_t;
typedef CGAL::Nth_of_tuple_property_map< IDX_VDESC,  MeshPoint_t > MP_VDesc_t;
typedef std::vector< MeshPoint_t > MeshPointCloud_t;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits<
        Kernel_t, MeshPointCloud_t, MP_Point_t, MP_Normal_t > ERTraits_t;
typedef CGAL::Shape_detection::Efficient_RANSAC< ERTraits_t > EfficientRansac_t;
typedef CGAL::Shape_detection::Plane< ERTraits_t >            ERPlane_t;

namespace pmp = CGAL::Polygon_mesh_processing;

const uint8_t FILLED_TAG_INITIAL = 0;
const uint8_t FILLED_TAG_FILLED  = 99;

const std::string VERTEX_NORMAL_PROP_NAME = "v:normals";
const std::string VERTEX_PLANE_PROP_NAME  = "v:plane";
const std::string FILLED_TAG_NAME         = "f:filled";

template < typename PT >
static void load_surface_mesh_and_show_info(
        const std::string &fn,
        CGAL::Surface_mesh<PT> &sm ) {
    read_mesh_ply(fn, sm);

    std::cout << "sm.number_of_vertices() = " << sm.number_of_vertices() << "\n";
    std::cout << "sm.number_of_facets()   = " << sm.number_of_faces() << "\n";
}

template < typename MeshT >
static void duplicate_non_manifold_vertices(
        MeshT &sm ) {
    typedef typename boost::graph_traits< MeshT >::vertex_descriptor VertexDesc_t;

    std::vector< std::vector< VertexDesc_t > > duplicatedVertices;
    int newVertices = pmp::duplicate_non_manifold_vertices(
            sm, CGAL::parameters::output_iterator( std::back_inserter( duplicatedVertices ) ) );
    std::cout << "newVertices = " << newVertices << "\n";
}

static void estimate_vertex_normal(Mesh_t &mesh, const std::string &normalPropName="v:normals") {
    FUNCTION_SCOPE_TIMER
    auto vertexNormals = mesh.add_property_map< VertexDesc_t, Vector_t >( normalPropName, CGAL::NULL_VECTOR ).first;
    pmp::compute_vertex_normals( mesh, vertexNormals );
}

static MeshPointCloud_t make_mesh_point_cloud( const Mesh_t &mesh, const std::string &normalPropName="v:normals" ) {
    FUNCTION_SCOPE_TIMER
    // Vertex point property.
    Mesh_t::Property_map< VertexDesc_t , Point_t > vertexPoints;
    bool found;
    boost::tie( vertexPoints, found ) = mesh.property_map< VertexDesc_t, Point_t >( "v:point" );
    assert( found );

    // Vertex normal property.
    auto vertexNormals = mesh.property_map< VertexDesc_t, Vector_t >( normalPropName ).first;

    // Mesh point cloud.
    MeshPointCloud_t meshPC;

    for ( const VertexDesc_t &vDesc : mesh.vertices() ) {
        meshPC.emplace_back( vertexPoints[vDesc], vertexNormals[vDesc], vDesc );
    }

    return meshPC;
}

static void shape_detection( MeshPointCloud_t &meshPC,
                             std::vector<int> &pointPlaneMap,
                             std::vector<std::array<Kernel_t::FT, 4>> &planes,
                             std::vector<std::array<Kernel_t::FT, 3>> &planeCentroids ) {
    FUNCTION_SCOPE_TIMER
    // Shape detection engine.
    EfficientRansac_t er;
    er.set_input( meshPC );
    er.add_shape_factory<ERPlane_t>();

    std::cout << "=== Shape detection. ===\n";
    std::cout << "Pre-processing...\n";
    er.preprocess();

    EfficientRansac_t::Parameters erParams;
    erParams.probability      = 0.05;
    erParams.min_points       = 2000;
    erParams.epsilon          = 0.002;
    erParams.cluster_epsilon  = 0.1;
    erParams.normal_threshold = 0.9;

    std::cout << "Detecting...\n";
    er.detect(erParams);

    EfficientRansac_t::Shape_range erShapes = er.shapes();
    EfficientRansac_t::Shape_range::iterator ersIter = erShapes.begin();

    std::cout << "Assign planes to the points...\n";
    pointPlaneMap.resize( meshPC.size() );
    planes.clear();
    for ( int i = 0; ersIter != erShapes.end(); ersIter++, i++ ) {
        boost::shared_ptr< EfficientRansac_t::Shape > shape = *ersIter;
        auto pPlane = reinterpret_cast<ERPlane_t*>( shape.get() );
        const auto &normal = pPlane->plane_normal();

        planes.push_back( {
            static_cast<Kernel_t::FT>(normal.x()),
            static_cast<Kernel_t::FT>(normal.y()),
            static_cast<Kernel_t::FT>(normal.z()),
            static_cast<Kernel_t::FT>(pPlane->d()) } );

//        std::cout << i << ": " << shape->info() << "\n";

        Kernel_t::FT x = 0, y = 0, z = 0;

        auto ptIdxIter = shape->indices_of_assigned_points().begin();
        while ( ptIdxIter != shape->indices_of_assigned_points().end() ) {
            pointPlaneMap[ *ptIdxIter ] = i;
            const auto& point = std::get<IDX_POINT>( meshPC[*ptIdxIter] );
            x += point.x();
            y += point.y();
            z += point.z();
            ptIdxIter++;
        }

        const int NP = shape->indices_of_assigned_points().size();
        planeCentroids.push_back( { x/NP, y/NP, z/NP } );

        // Test Use.
        std::cout << i << ": [ "
                  << planes[i][0] << ", "
                  << planes[i][1] << ", "
                  << planes[i][2] << ", "
                  << planes[i][2] << "]. "
                  << NP << " points. "
                  << "Centroid [ "
                  << planeCentroids[i][0] << ", "
                  << planeCentroids[i][1] << ", "
                  << planeCentroids[i][2] << " ]. \n";
    }
}

static void write_planes_and_centroids(
        const std::string &fn,
        const std::vector< std::array< Kernel_t::FT, 4 > > &planes,
        const std::vector< std::array< Kernel_t::FT, 3 > > &planeCentroids ) {
    const int N = planes.size();
    assert( N == planeCentroids.size() );

    std::ofstream ofs(fn);
    if (!ofs) {
        std::stringstream ss;
        ss << "Open " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    for ( int i = 0; i < N; ++i ) {
        ofs << std::showpos << std::setprecision(12) << std::scientific
            << planeCentroids[i][0] << ", "
            << planeCentroids[i][1] << ", "
            << planeCentroids[i][2] << ", "
            << planes[i][0] << ", "
            << planes[i][1] << ", "
            << planes[i][2] << "\n";
    }

    ofs.close();
}

static void add_plane_index_property_2_mesh(
        Mesh_t &mesh,
        const MeshPointCloud_t &meshPC,
        const std::vector<int> &pointPlaneMap,
        const std::string &vertexPlanePropName="v:plane" ) {
    FUNCTION_SCOPE_TIMER
    // Assign -1 as default plane index (no plane).
    auto vertexPlane = mesh.add_property_map< VertexDesc_t, int >( vertexPlanePropName, -1 ).first;

    const int N = pointPlaneMap.size();
    assert( N == meshPC.size() );
    for ( int i = 0; i < N; i++ ) {
        const auto &vDesc = std::get<IDX_VDESC>( meshPC[i] );
        vertexPlane[ vDesc ] = pointPlaneMap[i];
    }
}

static std::set<int> collect_planes_along_hole(
        Mesh_t &mesh, HalfedgeDesc_t hd, const std::string &vertexPlanePropName="v:plane" ) {
    typedef CGAL::Halfedge_around_face_circulator<Mesh_t> HalfedgeAroundFaceCirculator_t;

    // Get the property.
    auto vertexPlanes = mesh.property_map< VertexDesc_t, int >( vertexPlanePropName ).first;

    HalfedgeAroundFaceCirculator_t circ( hd, mesh ), done(circ);

    std::set<int> planeIdxSet;
    do {
        const int idx = vertexPlanes[ target(*circ, mesh) ];
        if ( idx == -1 ) {
            continue;
        } else {
            planeIdxSet.insert( idx );
        }
    } while (++circ != done);

    return planeIdxSet;
}

template < typename MeshT >
static void fill_holes(
        MeshT &mesh,
        std::vector<
                std::unordered_set<
                        typename boost::graph_traits<MeshT>::face_descriptor,
                        CGAL::Handle_hash_function> > &vPatchFacets,
        std::vector<std::set<int>> &holePlaneSets,
        const std::string &vertexPlanePropName="v:plane"
) {
    FUNCTION_SCOPE_TIMER

    typedef typename boost::graph_traits<MeshT>::face_descriptor FaceDesc_t;
    typedef typename boost::graph_traits<MeshT>::vertex_descriptor VertexDesc_t;
    typedef typename boost::graph_traits<MeshT>::halfedge_descriptor HalfedgeDesc_t;

    int nFilledHoles = 0;
    holePlaneSets.clear();

    for ( HalfedgeDesc_t h : halfedges( mesh ) ) {
        if ( !is_border(h, mesh)  ) {
            continue;
        }

        // Find all the planes associated with this hole.
        std::set<int> holePlanes = collect_planes_along_hole( mesh, h, vertexPlanePropName );

        std::unordered_set< FaceDesc_t, CGAL::Handle_hash_function > patchFacets;
        std::vector< VertexDesc_t > patchVertices;

        pmp::triangulate_and_refine_hole(
                mesh,
                h,
                std::inserter( patchFacets, patchFacets.begin() ),
                std::back_inserter( patchVertices ),
                pmp::parameters::vertex_point_map( get( CGAL::vertex_point, mesh ) )
                        .density_control_factor(2)
                        .use_delaunay_triangulation(true)
                        .geom_traits( Kernel_t() )
        );

        ++nFilledHoles;
        std::cout << "Fill No. " << nFilledHoles << ". \n";
        std::cout << "Facets in patch: " << patchFacets.size() << ". \n";
        std::cout << "holePlanes.size() = " << holePlanes.size() << "\n";

        holePlaneSets.push_back( std::move( holePlanes ) );
        vPatchFacets.push_back( std::move( patchFacets ) );
    }

    std::cout << nFilledHoles << " holes filled. \n";
}

template < typename MeshT, typename FacetSetContainerT >
static void add_hole_tag_face_property(
        MeshT &mesh, FacetSetContainerT &facetSetContainer,
        const std::string &patchFacetPropName="FILLED_TAG_NAME") {
    FUNCTION_SCOPE_TIMER

    typedef typename boost::graph_traits<MeshT>::face_descriptor FaceDesc_T;

    // New property map.
    std::cout << "Create a new face property map for the filled facets identification. \n";
    typename MeshT::template Property_map<FaceDesc_T, std::uint8_t> facetProperties;
    bool created;
    boost::tie(facetProperties, created) =
            mesh.template add_property_map<FaceDesc_T, std::uint8_t>(patchFacetPropName, FILLED_TAG_INITIAL);
    assert(created);

    for (auto &facetSet : facetSetContainer) {
        if (facetSet.size() < 10) continue;

        for (auto &facet : facetSet) {
            facetProperties[facet] = FILLED_TAG_FILLED;
        }
    }
    std::cout << "Face property map created. \n";
}

/**
 * The inner product between @p normal and the plane is first evaluated.
 * If the absolute value of the inner product is larger than 0.5, then
 * the projection is performed. Otherwise, the projected point is the
 * same with @p point.
 *
 * This function returns true if the point is actually projected.
 * It returns false when the projected point is the same with @p point.
 *
 * NOTE: Assuming unit @p normal and unit normal representation in
 * @p planeDesc.
 *
 * @tparam PointT Type of CGAL point.
 * @tparam NormalT Type of CGAL vector.
 * @tparam PlaneDescT A 4-element object, can be indexed by positive integer.
 * @param point The point.
 * @param normal The normal direction at the point.
 * @param planeDesc The target plane description, 4-element.
 * @return true if actual projection happens. false when @p projection = @p point.
 */
template < typename PointT, typename NormalT, typename PlaneDescT >
static bool project_point(
        const PointT &point, const NormalT &normal,
        const PlaneDescT &planeDesc,
        Kernel_t::FT &x, Kernel_t::FT &y, Kernel_t::FT &z, Kernel_t::FT &sqd ) {
    const auto npx = planeDesc[0];
    const auto npy = planeDesc[1];
    const auto npz = planeDesc[2];

    const auto innNpNl = npx * normal.x() + npy * normal.y() + npz * normal.z();
    if ( -0.5 < innNpNl && innNpNl < 0.5 ) {
        x = point.x(); y = point.y(); z = point.z();
        return false;
    }

    const auto s = -( npx*point.x() + npy*point.y() + npz*point.z() + planeDesc[3] ) / innNpNl;

    x = point.x() + s*normal.x();
    y = point.y() + s*normal.y();
    z = point.z() + s*normal.z();

    sqd = s * s;

    return true;
}

template < typename PointT, typename NormalT, typename PlaneDescT >
static bool project_point_along_plane(
        const PointT &point, const NormalT &normal,
        const PlaneDescT &planeDesc,
        Kernel_t::FT &x, Kernel_t::FT &y, Kernel_t::FT &z, Kernel_t::FT &sqd ) {
    const auto npx = planeDesc[0];
    const auto npy = planeDesc[1];
    const auto npz = planeDesc[2];

    const auto D = -( npx*point.x() + npy*point.y() + npz*point.z() + planeDesc[3] );

    x = point.x() + D*npx;
    y = point.y() + D*npy;
    z = point.z() + D*npz;

    const auto innNpNl = npx * normal.x() + npy * normal.y() + npz * normal.z();
    sqd = 1-innNpNl*innNpNl;
    sqd = std::max( sqd, static_cast<Kernel_t::FT>(0) );
    sqd = D*D*sqd;

    return true;
}

template < typename VertexDescsT, typename HolePlaneSetT, typename PlaneDescriptionsT >
static void move_vertices(
        Mesh_t &mesh,
        const VertexDescsT &vertices,
        const HolePlaneSetT &holePlanes,
        const PlaneDescriptionsT &planes,
        const std::string &vertexNormalPropName="v:normals" ) {
    if ( holePlanes.size() == 0 ) return;

    auto vertexPointMap  = mesh.points();
    auto vertexNormalMap = mesh.property_map< VertexDesc_t, Vector_t >( vertexNormalPropName ).first;

    for ( const auto &v : vertices ) {
        auto &point  = vertexPointMap[v];
        auto &normal = vertexNormalMap[v];

        Kernel_t::FT bx, by, bz, bs=std::numeric_limits<Kernel_t::FT>::max(); // Best values.
        Kernel_t::FT x, y, z, s;

        for ( const auto &planeIdx : holePlanes ) {
            s = bs;
//            project_point( point, normal, planes[planeIdx], x, y, z, s );
            project_point_along_plane( point, normal, planes[planeIdx], x, y, z, s );
            if ( s < bs ) {
                bx = x; by = y; bz = z; bs = s;
            }
        }

        put( vertexPointMap, v, Point_t( bx, by, bz ) );
    }
}

template < typename FacetSetT, typename HolePlaneSetT, typename PlaneDescriptionsT >
static void move_facets_to_nearest_plane(
        Mesh_t &mesh,
        std::vector<FacetSetT> &facetSetContainer,
        const std::vector<HolePlaneSetT> &holePlaneSets,
        const PlaneDescriptionsT &planes,
        const std::string &vertexNormalPropName="v:normals" ) {
    FUNCTION_SCOPE_TIMER
    const int N = facetSetContainer.size();
    assert( N == holePlaneSets.size() );

    typedef CGAL::Vertex_around_face_iterator<Mesh_t> VertexAroundFaceIter_t;
    VertexAroundFaceIter_t vi, ve;

    for ( int i = 0; i < N; i++ ) {
        const auto &planeIndices = holePlaneSets[i];

        // Gather the vertices.
        std::unordered_set< VertexDesc_t, CGAL::Handle_hash_function > vertexSet;
        for ( auto &facet : facetSetContainer[i] ) {
            for ( boost::tie(vi, ve) = vertices_around_face( mesh.halfedge(facet), mesh ); vi != ve; ++vi ) {
                vertexSet.insert( *vi );
            }
        }

        // Move.
        move_vertices( mesh, vertexSet, planeIndices, planes, vertexNormalPropName );
    }
}

template < typename MeshT, typename FacetSetContainerT >
static int remesh_by_facet_set(
        MeshT &mesh, FacetSetContainerT &facetSetContainer,
        const std::string &patchFacetPropName,
        double targetLength=0.01 ) {
    FUNCTION_SCOPE_TIMER

    typedef typename boost::graph_traits<MeshT>::face_descriptor FaceDesc_T;

    // New property map.
    std::cout << "Create a new face property map for the filled facets identification. \n";
    typename MeshT::template Property_map< FaceDesc_T , std::uint8_t > facetProperties;
    bool found;
    boost::tie( facetProperties, found ) =
            mesh.template property_map< FaceDesc_T, std::uint8_t >(patchFacetPropName);
    assert( found );

    int idx = 0;
    int count = 0;
    for ( auto& facetSet : facetSetContainer ) {
        if ( facetSet.size() < 10 ) {
            std::cout << "Hole area " << idx << " is too small ("
                      << facetSet.size() << "). Skip this hole area. \n";
            ++idx;
            continue;
        }

        pmp::isotropic_remeshing(
                facetSet,
                targetLength,
                mesh,
                pmp::parameters::number_of_iterations(1)
                        .protect_constraints(false)
                        .face_patch_map( facetProperties )
        );

        std::cout << "Hole area " << idx << " ( " << facetSet.size() << " facets ) remeshed. \n";
        ++idx;
        ++count;
    }

    return count;
}

template < typename MeshT, typename FacetSetContainerT >
static void pb_remesh(
        MeshT &mesh, FacetSetContainerT &facetSetContainer, double targetLength=0.01 ) {

    std::cout << "Remesh the larger areas. \n";

    int count = remesh_by_facet_set( mesh, facetSetContainer, FILLED_TAG_NAME, targetLength );

    std::cout << count << " hole areas remeshed in total. \n";
}

template < typename PointSetIteratorT >
static void pb_write_point_sets( const std::string outDir,
                                 PointSetIteratorT iter, PointSetIteratorT iterEnd ) {
    FUNCTION_SCOPE_TIMER

    std::cout << "Write the point sets. \n";

    for ( int i = 0; iter != iterEnd; ++iter, ++i ) {
        std::stringstream ss;
        ss << outDir << "/PointSet_" << i << ".ply";

        write_points_ply( ss.str(), *iter );
    }
}

template < typename MeshT, typename TagValueT >
static std::vector<typename MeshT::Point> collect_points_by_facet_tag(
        MeshT &mesh, const std::string &tagName, const TagValueT &tagValue ) {
    FUNCTION_SCOPE_TIMER

    typedef typename boost::graph_traits<MeshT>::face_descriptor FaceDesc_T;
    typedef typename boost::graph_traits<MeshT>::vertex_descriptor VertexDesc_t;
    typedef typename CGAL::Vertex_around_face_iterator<MeshT> VertexAroundFaceIter_t;

    typename MeshT::template Property_map< FaceDesc_T , TagValueT > facetProperties;
    bool found;
    boost::tie( facetProperties, found ) =
            mesh.template property_map< FaceDesc_T, TagValueT >(tagName);
    assert( found );

    std::unordered_set< VertexDesc_t, CGAL::Handle_hash_function > vertexSet;
    VertexAroundFaceIter_t vi, ve;

    for ( auto &facet : mesh.faces() ) {
        if ( tagValue != facetProperties[facet] ) {
            continue;
        }

        for ( boost::tie(vi, ve) = vertices_around_face( mesh.halfedge(facet), mesh ); vi != ve; ++vi ) {
            vertexSet.insert( *vi );
        }
    }

    std::vector< typename MeshT::Point > points;

    if ( 0 == vertexSet.size() ) return points;

    points.reserve( vertexSet.size() );

    typename MeshT::template Property_map<
            typename MeshT::Vertex_index, typename MeshT::Point > pointProperties = mesh.points();

    for ( auto &vertex : vertexSet ) {
        points.push_back( pointProperties[vertex] );
    }

    return points;
}

template < typename MeshT, typename TagValueT >
static std::vector<typename MeshT::Point> pb_collect_points_by_facet_tag(
        MeshT &mesh, const std::string &tagName, const TagValueT &tagValue ) {
    std::cout << "Collect points from the filled in facets. \n";

    std::vector< typename MeshT::Point > points;
    points = collect_points_by_facet_tag( mesh, tagName, tagValue );

    std::cout << points.size() << " points collected. \n";

    return points;
}

template < typename CGALPoint_T, typename PCLPoint_T >
static typename pcl::PointCloud<PCLPoint_T>::Ptr convert_cgal_point_2_pcl( const std::vector<CGALPoint_T> &cgalPoints ) {
    assert( cgalPoints.size() > 0 );

    typename pcl::PointCloud<PCLPoint_T>::Ptr pCloud( new pcl::PointCloud<PCLPoint_T> );

    for ( const auto & point : cgalPoints ) {
        PCLPoint_T pclPoint;
        pclPoint.x = static_cast<float>( point.x() );
        pclPoint.y = static_cast<float>( point.y() );
        pclPoint.z = static_cast<float>( point.z() );
        pCloud->push_back( pclPoint );
    }

    return pCloud;
}

static ap::Args handle_args( int argc, char** argv ) {
    ap::Args args;

    args.add_positional<std::string>("in-mesh", "Input surface mesh. ");
    args.add_positional<std::string>("working-dir", "Working directory. ");
    args.add_positional<std::string>("out-points", "The filename for the output point cloud. ");
    args.add_default<double>("filled-smooth-length",
                             "The target edge length of the holefilled region after remeshing",
                             0.01);

    args.add_flag("project", "Set this flag to enable plane projection. ");

    args.parse_args( argc, argv );

    std::cout << args;

    return args;
}

int main( int argc, char** argv ) {
    std::cout << "Hello, FillHolesWithPlane! \n";

    ap::Args args = handle_args( argc, argv );

    std::string inMeshFn      = args.arguments<std::string>["in-mesh"]->get();
    std::string workingDir    = args.arguments<std::string>["working-dir"]->get();
    std::string outPointsFn   = args.arguments<std::string>["out-points"]->get();
    double filledSmoothLength = args.arguments<double>["filled-smooth-length"]->get();

    const bool flagProject = args.arguments<bool>["project"]->get();

    test_directory(workingDir);

    Mesh_t surfaceMesh;
    load_surface_mesh_and_show_info(inMeshFn, surfaceMesh);
    duplicate_non_manifold_vertices( surfaceMesh );

    // Estimate surface normals.
    estimate_vertex_normal(surfaceMesh, VERTEX_NORMAL_PROP_NAME);

    auto meshPC = make_mesh_point_cloud( surfaceMesh, VERTEX_NORMAL_PROP_NAME );

    // Detect planes.
    std::vector<int> pointPlaneMap;
    std::vector<std::array<Kernel_t::FT, 4>> planes;
    std::vector<std::array<Kernel_t::FT, 3>> planeCentroids;
    shape_detection( meshPC, pointPlaneMap, planes, planeCentroids );

    // Save the planes and plane centroids to file for debugging.
    {
        std::stringstream ss;
        ss << workingDir << "/PlanePointNormals.csv";
        write_planes_and_centroids( ss.str(), planes, planeCentroids );
    }

    // Add new vertex property to the mesh.
    add_plane_index_property_2_mesh( surfaceMesh, meshPC, pointPlaneMap, VERTEX_PLANE_PROP_NAME );

    // Fill holes.
    typedef boost::graph_traits<Mesh_t>::face_descriptor PHFaceDesc_t;
    typedef std::unordered_set<PHFaceDesc_t , CGAL::Handle_hash_function> PatchFacetSet_t;

    std::vector< std::set<int> > holePlaneSets;
    std::vector< PatchFacetSet_t > vPatchFacets;
    fill_holes( surfaceMesh, vPatchFacets, holePlaneSets, VERTEX_PLANE_PROP_NAME );
    {
        std::stringstream ss;
        ss << workingDir << "/HoleFilled.ply";
        write_mesh_ply( ss.str(), surfaceMesh );
    }

    // Add hole tag face property.
    add_hole_tag_face_property( surfaceMesh, vPatchFacets, FILLED_TAG_NAME );

    if ( flagProject ) {
        // Compute the vertex normal again.
        estimate_vertex_normal( surfaceMesh, VERTEX_NORMAL_PROP_NAME );

        // Move vertices.
        move_facets_to_nearest_plane( surfaceMesh, vPatchFacets, holePlaneSets, planes, VERTEX_NORMAL_PROP_NAME );
    }

//    assert(false);

    // Remesh the large hole areas.
    pb_remesh( surfaceMesh, vPatchFacets, filledSmoothLength );
    {
        std::stringstream ss;
        ss << workingDir << "/HoleFilledRemeshed.ply";
        write_mesh_ply( ss.str(), surfaceMesh );
    }

    // Collect the vertices.
    std::vector< Point_t > points =
            pb_collect_points_by_facet_tag( surfaceMesh, FILLED_TAG_NAME, FILLED_TAG_FILLED );

//    write_points_ply( outPointsFn, points );

    auto pCloud = convert_cgal_point_2_pcl<Point_t, pcl::PointXYZ>( points );

    pcu::write_point_cloud<pcl::PointXYZ>( outPointsFn, pCloud );

    return 0;
}
