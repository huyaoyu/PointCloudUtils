//
// Created by yaoyu on 7/15/20.
//

#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Handle_hash_function.h>

#include "Args/ArgsParser.hpp"
#include "CGALCommon/IO.hpp"
#include "Filesystem/Filesystem.hpp"
#include "Profiling/ScopeTimer.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3              Point_t;
typedef CGAL::Surface_mesh<Point_t>    Mesh_t;
typedef Mesh_t::Vertex_index           VertexIdx_t;
typedef Mesh_t::Face_index             FaceIdx_t;

typedef boost::graph_traits<Mesh_t>::face_descriptor   FaceDesc_t;
typedef boost::graph_traits<Mesh_t>::edge_descriptor   EdgeDesc_t;
typedef boost::graph_traits<Mesh_t>::vertex_descriptor VertexDesc_t;

namespace pmp = CGAL::Polygon_mesh_processing;

const uint8_t FILLED_TAG_INITIAL = 0;
const uint8_t FILLED_TAG_FILLED  = 99;

const std::string FILLED_TAG_NAME = "f:filled";

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

template < typename MeshT >
static void fill_holes(
        MeshT &mesh,
        std::vector<
                std::unordered_set<
                        typename boost::graph_traits<MeshT>::face_descriptor,
                        CGAL::Handle_hash_function> > &vPatchFacets
                        ) {
    FUNCTION_SCOPE_TIMER

    typedef typename boost::graph_traits<MeshT>::face_descriptor FaceDesc_t;
    typedef typename boost::graph_traits<MeshT>::vertex_descriptor VertexDesc_t;
    typedef typename boost::graph_traits<MeshT>::halfedge_descriptor HalfedgeDesc_t;

    int nFilledHoles = 0;

    for ( HalfedgeDesc_t h : halfedges( mesh ) ) {
        if ( !is_border(h, mesh)  ) {
            continue;
        }

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

        vPatchFacets.push_back( std::move( patchFacets ) );
    }

    std::cout << nFilledHoles << " holes filled. \n";
}

template < typename MeshT, typename FacetIteratorT >
static std::pair< typename MeshT::Point, typename MeshT::Point> find_bbox(
        MeshT &mesh,
        FacetIteratorT facetIter,
        FacetIteratorT facetIterEnd ) {

    typedef typename MeshT::Point Point;
    typedef typename Point::FT    FT;

    int pointCount = 0;
    FT x0 = 0, y0 = 0, z0 = 0;
    FT x1 = 0, y1 = 0, z1 = 0;

    for ( ; facetIter != facetIterEnd; facetIter++ ) {
        pmp::Face_location<MeshT, FT> faceLoc { *facetIter, { 0.5, 0.5, 0.5 } };
        Point point = pmp::construct_point( faceLoc, mesh );

        if ( 0 == pointCount ) {
            x0 = point.x(); y0 = point.y(); z0 = point.z();
            x1 = x0; y1 = y0; z1 = z0;
        } else {
            if ( point.x() < x0 ) {
                x0 = point.x();
            } else if ( point.x() > x1 ) {
                x1 = point.x();
            }

            if ( point.y() < y0 ) {
                y0 = point.y();
            } else if ( point.y() > y1 ) {
                y1 = point.y();
            }

            if ( point.z() < z0 ) {
                z0 = point.z();
            } else if ( point.z() > z1 ) {
                z1 = point.z();
            }
        }

        ++pointCount;
    }

    std::pair< typename MeshT::Point, typename MeshT::Point> bbox { Point( x0, y0, z0 ), Point( x1, y1, z1 ) };
    return bbox;
}

template < typename MeshT, typename FacetSetIteratorT >
static std::vector< std::pair< typename MeshT::Point, typename MeshT::Point > > find_bboxes(
        MeshT &mesh,
        FacetSetIteratorT facetSetIter,
        FacetSetIteratorT facetSetIterEnd,
        int sizeLimit ) {
    FUNCTION_SCOPE_TIMER

    typedef typename std::pair< typename MeshT::Point, typename MeshT::Point > BBox_t;
    std::vector< BBox_t > bboxes;

    for ( ; facetSetIter != facetSetIterEnd; facetSetIter++ ) {
        if ( facetSetIter->size() < sizeLimit ) continue;

        BBox_t bbox = find_bbox( mesh,
                facetSetIter->begin(), facetSetIter->end() );
        bboxes.push_back( std::move( bbox ) );
    }

    return bboxes;
}

template < typename MeshT, typename FacetSetIteratorT >
static std::vector< std::pair< typename MeshT::Point, typename MeshT::Point > > pb_find_bboxes(
        MeshT &mesh, FacetSetIteratorT iter, FacetSetIteratorT iterEnd, int sizeLimit=10 ) {

    std::cout << "Find the bboxes. \n";
    std::vector< std::pair< typename MeshT::Point, typename MeshT::Point > > bboxes =
            find_bboxes( mesh, iter, iterEnd, sizeLimit );

    std::cout << "Find " << bboxes.size() << " bboxes. \n";
    for ( int i = 0; i < bboxes.size(); ++i ) {
        std::cout << "BBox " << i << ": "
                  << bboxes[i].first << ", " << bboxes[i].second << "\n";
    }

    return bboxes;
}

template < typename MeshT, typename FacetSetContainerT >
static int remesh_by_facet_set(
        MeshT &mesh, FacetSetContainerT &facetSetContainer,
        const std::string &patchFacetPropName) {
    FUNCTION_SCOPE_TIMER

    typedef typename boost::graph_traits<MeshT>::face_descriptor FaceDesc_T;
//    typedef typename MeshT::Face_index                           FaceDesc_T;

    // New property map.
    std::cout << "Create a new face property map for the filled facets identification. \n";
    typename MeshT::template Property_map< FaceDesc_T , std::uint8_t > facetProperties;
    bool created;
    boost::tie( facetProperties, created ) =
            mesh.template add_property_map< FaceDesc_T, std::uint8_t >(patchFacetPropName, FILLED_TAG_INITIAL);
    assert( created );

    for ( auto &facetSet : facetSetContainer ) {
        if ( facetSet.size() < 10 ) continue;

        for ( auto &facet : facetSet ) {
            facetProperties[facet] = FILLED_TAG_FILLED;
        }
    }
    std::cout << "Face property map created. \n";

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
                0.05,
                mesh,
                pmp::parameters::number_of_iterations(3)
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
        MeshT &mesh, FacetSetContainerT &facetSetContainer ) {

    std::cout << "Remesh the larger areas. \n";

    int count = remesh_by_facet_set( mesh, facetSetContainer, FILLED_TAG_NAME );

    std::cout << count << " hole areas remeshed in total. \n";
}

template < typename MeshT, typename BBoxPointPairT >
static std::vector< typename MeshT::Point > collect_points_in_bbox(
        MeshT &mesh,
        const BBoxPointPairT &bboxPointPair ) {
    typedef typename MeshT::Point::FT FT;
    const FT x0 = bboxPointPair.first.x();
    const FT y0 = bboxPointPair.first.y();
    const FT z0 = bboxPointPair.first.z();

    const FT x1 = bboxPointPair.second.x();
    const FT y1 = bboxPointPair.second.y();
    const FT z1 = bboxPointPair.second.z();

//    // Debug use.
//    std::cout << "BBox corners: [ "
//              << x0 << ", " << y0 << ", " << z0 << " ], [ "
//              << x1 << ", " << y1 << ", " << z1 << " ]. \n";

    std::vector< typename MeshT::Point > points;

    typename MeshT::template Property_map<
            typename MeshT::Vertex_index, typename MeshT::Point > pointProperties = mesh.points();

    for ( typename MeshT::Vertex_index v : mesh.vertices() ) {
        const FT x = pointProperties[v].x();
        const FT y = pointProperties[v].y();
        const FT z = pointProperties[v].z();

        if ( x >= x0 && y >= y0 && z >= z0 &&
             x <= x1 && y <= y1 && z <= z1 ) {
            points.push_back( pointProperties[v] );
        }
    }

    return points;
}

template < typename MeshT, typename BBoxIteratorT >
static std::vector< std::vector<typename MeshT::Point> > collect_points_by_bboxes(
        MeshT &mesh,
        BBoxIteratorT iter,
        BBoxIteratorT iterEnd ) {
    FUNCTION_SCOPE_TIMER

    std::vector< std::vector<typename MeshT::Point> > pointSets;

    for ( ; iter != iterEnd; ++iter ) {
        std::vector< typename MeshT::Point > points =
                collect_points_in_bbox( mesh, *iter );
        pointSets.push_back( std::move( points ) );
    }

    return pointSets;
}

template < typename MeshT, typename BBoxIteratorT >
static std::vector< std::vector<typename MeshT::Point> > pb_collect_points_by_bboxes(
        MeshT &mesh,
        BBoxIteratorT iter,
        BBoxIteratorT iterEnd ) {
    std::cout << "Collect ponts by the bboxes. \n";

    std::vector< std::vector<typename MeshT::Point> > pointSets =
            collect_points_by_bboxes( mesh, iter, iterEnd );

    // Show the number of points collected.
    for ( int i = 0; i < pointSets.size(); ++i ) {
        std::cout << "Point set " << i << ", "
                  << pointSets[i].size() << " points. \n";
    }

    return pointSets;
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

static ap::Args handle_args( int argc, char** argv ) {
    ap::Args args;

    args.add_positional<std::string>("in-mesh", "Input surface mesh. ");
    args.add_positional<std::string>("working-dir", "Working directory. ");

    args.parse_args( argc, argv );

    std::cout << args;

    return args;
}

int main( int argc, char** argv ) {
    std::cout << "Hello, FillHoles! \n";

    ap::Args args = handle_args( argc, argv );

    std::string inMeshFn   = args.arguments<std::string>["in-mesh"]->get();
    std::string workingDir = args.arguments<std::string>["working-dir"]->get();

    test_directory(workingDir);

    Mesh_t surfaceMesh;
    load_surface_mesh_and_show_info(inMeshFn, surfaceMesh);
    duplicate_non_manifold_vertices( surfaceMesh );

    // Fill holes.
    typedef boost::graph_traits<Mesh_t>::face_descriptor PHFaceDesc_t;
    typedef std::unordered_set<PHFaceDesc_t , CGAL::Handle_hash_function> PatchFacetSet_t;

    std::vector< PatchFacetSet_t > vPatchFacets;
    fill_holes( surfaceMesh, vPatchFacets );
    {
        std::stringstream ss;
        ss << workingDir << "/HoleFilled.ply";
        write_mesh_ply( ss.str(), surfaceMesh );
    }

    // Remesh the large hole areas.
    pb_remesh( surfaceMesh, vPatchFacets );
    {
        std::stringstream ss;
        ss << workingDir << "/HoleFilledRemeshed.ply";
        write_mesh_ply( ss.str(), surfaceMesh );
    }

    // Collect the vertices.
    std::vector< Point_t > points =
            pb_collect_points_by_facet_tag( surfaceMesh, FILLED_TAG_NAME, FILLED_TAG_FILLED );

    {
        std::stringstream ss;
        ss << workingDir << "/HoleFilledCollectePoints.ply";
        write_points_ply( ss.str(), points );
    }

    return 0;
}
