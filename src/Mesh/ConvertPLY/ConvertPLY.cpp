//
// Created by yaoyu on 7/11/20.
//

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <unordered_set>

// Boost.
#include <boost/function_output_iterator.hpp>
#include <boost/math/constants/constants.hpp>

// CGAL includes.
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>

#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Handle_hash_function.h>

#include "tinyply.h"
#include "CGALCommon/IO.hpp"
#include "Profiling/ScopeTimer.hpp"

// Local typedefs.
//typedef CGAL::Simple_cartesian<double> Kernel_t;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3              Point_t;
typedef CGAL::Surface_mesh<Point_t>    Mesh_t;
typedef Mesh_t::Vertex_index           VertexIdx_t;
typedef Mesh_t::Face_index             FaceIdx_t;
typedef Mesh_t::Halfedge_index         HalfedgeIdx_t;

typedef boost::graph_traits<Mesh_t>::face_descriptor     faceDesc;
typedef boost::graph_traits<Mesh_t>::halfedge_descriptor halfedgeDesc;
typedef boost::graph_traits<Mesh_t>::edge_descriptor     EdgeDesc;
typedef boost::graph_traits<Mesh_t>::vertex_descriptor   VertexDesc;

namespace pmp = CGAL::Polygon_mesh_processing;

struct Halfedge2Edge {
    Halfedge2Edge( const Mesh_t &m, std::vector<EdgeDesc> &edges )
    : mesh(m), edges(edges) {}

    void operator () ( const halfedgeDesc &h ) const {
        edges.push_back( edge(h, mesh) );
    }

    const Mesh_t &mesh;
    std::vector<EdgeDesc> &edges;
};

struct XYZV {
    float x;
    float y;
    float z;
    float v;
};

struct FaceIdx {
    int v0;
    int v1;
    int v2;
};

// These functions are copied from tinyply Github repo.
// https://github.com/ddiakopoulos/tinyply/blob/ca7b279fb6c9af931ffdaed96a3b11ca3ccd79ea/source/example-utils.hpp#L22
inline std::vector<uint8_t> read_file_binary(const std::string & pathToFile)
{
    std::ifstream file(pathToFile, std::ios::binary);
    std::vector<uint8_t> fileBufferBytes;

    if (file.is_open())
    {
        file.seekg(0, std::ios::end);
        size_t sizeBytes = file.tellg();
        file.seekg(0, std::ios::beg);
        fileBufferBytes.resize(sizeBytes);
        if (file.read((char*)fileBufferBytes.data(), sizeBytes)) return fileBufferBytes;
    }
    else throw std::runtime_error("could not open binary ifstream to path " + pathToFile);
    return fileBufferBytes;
}

struct memory_buffer : public std::streambuf
{
    char * p_start {nullptr};
    char * p_end {nullptr};
    size_t size;

    memory_buffer(char const * first_elem, size_t size)
            : p_start(const_cast<char*>(first_elem)), p_end(p_start + size), size(size)
    {
        setg(p_start, p_start, p_end);
    }

    pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override
    {
        if (dir == std::ios_base::cur) gbump(static_cast<int>(off));
        else setg(p_start, (dir == std::ios_base::beg ? p_start : p_end) + off, p_end);
        return gptr() - p_start;
    }

    pos_type seekpos(pos_type pos, std::ios_base::openmode which) override
    {
        return seekoff(pos, std::ios_base::beg, which);
    }
};

struct memory_stream : virtual memory_buffer, public std::istream
{
    memory_stream(char const * first_elem, size_t size)
            : memory_buffer(first_elem, size), std::istream(static_cast<std::streambuf*>(this)) {}
};

struct SeparatePLY {
    // This struct should, by default, has a
    // compiler generated move constructor.
    std::vector<XYZV> xyzv;
    std::vector<FaceIdx> faceIdx;
};

static SeparatePLY read_ply_and_convert( const std::string &fn ) {
    std::unique_ptr<std::istream> pIfs;
    std::vector<uint8_t> byteBuffer;

    byteBuffer = read_file_binary( fn );

    pIfs.reset( new memory_stream( (char*)byteBuffer.data(), byteBuffer.size() ) );

    if ( !pIfs || pIfs->fail() ) {
        std::stringstream ss;
        ss << "Failed to read from " << fn;
        throw std::runtime_error(ss.str());
    }

    pIfs->seekg(0, std::ios::end);
    const float sizeMB = pIfs->tellg() * 1e-6f;
    pIfs->seekg(0, std::ios::beg);

    std::cout << "sizeMB = " << sizeMB << "\n";

    tinyply::PlyFile plyFile;

    plyFile.parse_header( *pIfs );

    std::cout << "plyFile.is_binary_file() = " << plyFile.is_binary_file() << "\n";

    for ( const auto &c : plyFile.get_info() ) {
        std::cout << "Info: " << c << "\n";
    }

    for ( const auto &e : plyFile.get_elements() ) {
        std::cout << "element: " << e.name << ", size = " << e.size << "\n";
        for ( const auto &p : e.properties ) {
            std::cout << "\tproperty: " << p.name << ", "
                      << "type = " << tinyply::PropertyTable[p.propertyType].str << ". ";
            if ( p.isList ) {
                std::cout << tinyply::PropertyTable[p.listType].str << ". ";
            }
            std::cout << "\n";
        }
    }

    std::shared_ptr<tinyply::PlyData> pVertices, pFaces;

    try {
        pVertices = plyFile.request_properties_from_element("vertex", {"x", "y", "z", "value"});
    } catch ( const std::exception &ex ) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }

    try {
        pFaces = plyFile.request_properties_from_element("face", { "vertex_indices" }, 3);
    } catch ( const std::exception &ex ) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }

    std::cout << "Reading from PLY file... \n";

    {
        ScopeTimer timer("PLY read");
        plyFile.read(*pIfs);
    }

    if ( pVertices ) std::cout << "pVertices->count = " << pVertices->count << "\n";
    if ( pFaces ) std::cout << "pFaces->count = " << pFaces->count << "\n";

    SeparatePLY sPLY;

    sPLY.xyzv.resize( pVertices->count );
    std::memcpy( sPLY.xyzv.data(), pVertices->buffer.get(), pVertices->buffer.size_bytes() );

    sPLY.faceIdx.resize( pFaces->count );
    std::memcpy( sPLY.faceIdx.data(), pFaces->buffer.get(), pFaces->buffer.size_bytes() );

    return sPLY;
}

template < typename PT, typename Countable >
static void copy_from_separate_ply_2_polygon_soup(
        const SeparatePLY &sPLY,
        std::vector<PT> &points,
        std::vector<Countable> &polygons ) {
    points.clear();
    points.reserve( sPLY.xyzv.size() );

    for ( const auto &xyzv : sPLY.xyzv ) {
        points.emplace_back( xyzv.x, xyzv.y, xyzv.z );
    }

    polygons.clear();
    polygons.reserve( sPLY.faceIdx.size() );

    for ( const auto &faceIdx : sPLY.faceIdx ) {
        Countable face { faceIdx.v0, faceIdx.v1, faceIdx.v2 };
        polygons.push_back( std::move(face) );
    }
}

template < typename PT >
static void repair_separate_ply_and_convert(
        const SeparatePLY &sPLY,
        CGAL::Surface_mesh<PT> &sm ) {
    FUNCTION_SCOPE_TIMER

    // Copy the data in sPLY into Point_t objects and a polygon soup.
    std::vector<PT> points;
    typedef std::vector<int> Countable;
    std::vector<Countable> polygons;

    copy_from_separate_ply_2_polygon_soup( sPLY, points, polygons );

    pmp::repair_polygon_soup( points, polygons );
    pmp::orient_polygon_soup( points, polygons );

    pmp::polygon_soup_to_polygon_mesh( points, polygons, sm );

    // Self intersection.
    bool isSelfIntersection =
            pmp::does_self_intersect( sm, pmp::parameters::vertex_point_map( get( CGAL::vertex_point, sm ) ) );

    std::cout << "isSelfIntersection = " << isSelfIntersection << "\n";

    typedef typename boost::graph_traits<CGAL::Surface_mesh<PT>>::face_descriptor FD_t;
    typedef CGAL::Surface_mesh<PT> SM_t;

    if ( isSelfIntersection ) {
        std::vector< std::pair< FD_t, FD_t > > intersectedFacePairs;
        pmp::self_intersections( sm, std::back_inserter( intersectedFacePairs ) );

        std::unordered_set<FD_t, CGAL::Handle_hash_function> intersectedFaceSet;
        for ( const auto &p : intersectedFacePairs ) {
            intersectedFaceSet.insert(p.first);
            intersectedFaceSet.insert(p.second);
        }

        std::cout << "Begin removing intersected faces. \n";
        int removedFaceCount = 0;
        for ( const auto &f : intersectedFaceSet ) {
            CGAL::Euler::remove_face( halfedge(f, sm), sm );
            ++removedFaceCount;
        }
        std::cout << removedFaceCount << " intersecting facets are removed. \n";
    }

    isSelfIntersection =
            pmp::does_self_intersect( sm, pmp::parameters::vertex_point_map( get( CGAL::vertex_point, sm ) ) );

    std::cout << "isSelfIntersection = " << isSelfIntersection << "\n";

    std::vector< std::vector< VertexDesc > > duplicatedVertices;
    int newVerticesNB = pmp::duplicate_non_manifold_vertices(
            sm, CGAL::parameters::output_iterator( std::back_inserter( duplicatedVertices ) ) );
    std::cout << "newVerticesNB = " << newVerticesNB << "\n";
}

static void convert_separate_ply_2_cgal_mesh(
        const SeparatePLY &sPLY,
        Mesh_t &mesh ) {
    ScopeTimer timer( __func__ );

    const int NV = sPLY.xyzv.size();
    std::vector<VertexIdx_t> vertexIdx;
    vertexIdx.reserve(NV);

    // Add all the vertices into the mesh.
    for ( const auto &xyzv : sPLY.xyzv ) {
        VertexIdx_t idx = mesh.add_vertex( Point_t( xyzv.x, xyzv.y, xyzv.z ) );
        vertexIdx.push_back(idx);
    }

    // Add all the faces into the mesh;
    for ( const auto &face : sPLY.faceIdx ) {
        // Get the actual vertex index.
        const VertexIdx_t vi0 = vertexIdx[ face.v0 ];
        const VertexIdx_t vi1 = vertexIdx[ face.v1 ];
        const VertexIdx_t vi2 = vertexIdx[ face.v2 ];

        FaceIdx_t ft = mesh.add_face( vi0, vi1, vi2 );

        if ( ft == Mesh_t::null_face() ) {
            std::cerr << "Face [ "
                      << face.v0 << ", " << face.v1 << ", " << face.v2
                      << " ] cannot be added because of an orientation error. Try again. \n";
            ft = mesh.add_face( vi0, vi2, vi1 );
            assert( ft != Mesh_t::null_face() );
        }
    }
}

static void remesh( Mesh_t &mesh,
        double targetEdgeLength=0.05,
        unsigned int nbIter=3,
        bool protectBorder=true) {
    ScopeTimer timer(__func__);

    std::cout << "Split border...\n";

    std::vector< EdgeDesc > border;
    pmp::border_halfedges( faces( mesh ), mesh,
            boost::make_function_output_iterator( Halfedge2Edge( mesh, border ) ));
    pmp::split_long_edges( border, targetEdgeLength, mesh );

    std::cout << "Split border done. \n";

    std::cout << "Remeshing...\n";

    pmp::isotropic_remeshing( faces(mesh),
            targetEdgeLength, mesh,
            pmp::parameters::number_of_iterations(nbIter)
                .protect_constraints(protectBorder)
                .number_of_relaxation_steps(3));

    // Debug use.
    write_mesh_off( "InitialConverted.off", mesh );

    // Find all dangling faces.
    const auto PI = boost::math::constants::pi<double>();
    const double targetFaceArea = 0.1 * 0.5 * std::sin(PI/3) * targetEdgeLength * targetEdgeLength;
    int nRemovedFaces = 0;
    for ( const FaceIdx_t &f : mesh.faces() ) {
//        int borderCount = 0;
//        for ( const HalfedgeIdx_t &h : CGAL::halfedges_around_face( mesh.halfedge(f), mesh ) ) {
//                if ( mesh.is_border(h) ) borderCount++;
//        }
//
//        if ( 3 == borderCount ) {
//            mesh.remove_face(f);
//            nRemovedFaces++;
//        }

        if ( pmp::face_area( f, mesh ) < targetFaceArea ) {
            CGAL::Euler::remove_face( halfedge(f, mesh), mesh );
            nRemovedFaces++;
        }
    }

    std::cout << nRemovedFaces << " faces removed. \n";
    std::cout << "Remesh done. \n";
}

static void smooth_mesh( Mesh_t &mesh ) {
    typedef boost::property_map<Mesh_t, CGAL::edge_is_feature_t>::type EIFMap;
    EIFMap eif = get( CGAL::edge_is_feature, mesh );

    ScopeTimer timer(__func__);

    pmp::detect_sharp_edges( mesh, 60, eif );

    int sharpCount = 0;
    for ( EdgeDesc e : edges(mesh) ) {
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

int main( int argc, char **argv ) {
    std::cout << "Hello, ConvertPLY! \n";

    SeparatePLY sPLY = read_ply_and_convert( argv[1] );

    std::cout << "sPLY.xyzv[0] = { "
              << "x = " << sPLY.xyzv[0].x << ", "
              << "y = " << sPLY.xyzv[0].y << ", "
              << "z = " << sPLY.xyzv[0].z << ", "
              << "v = " << sPLY.xyzv[0].v << " }\n";

    std::cout << "sPLY.faceIndex[0] = { "
              << sPLY.faceIdx[0].v0 << ", "
              << sPLY.faceIdx[0].v1 << ", "
              << sPLY.faceIdx[0].v2 << " }\n";

    Mesh_t surfaceMesh;

//    convert_separate_ply_2_cgal_mesh(sPLY, surfaceMesh);
    repair_separate_ply_and_convert(sPLY, surfaceMesh);

    remesh(surfaceMesh, 0.1, 3, false);
    write_mesh_ply("Remeshed.ply", surfaceMesh);
//    write_mesh_ply( "RemeshedBorderMoved.ply", surfaceMesh );

//    if ( surfaceMesh.has_garbage() ) {
//        surfaceMesh.collect_garbage();
//    }

    smooth_mesh( surfaceMesh );
    write_mesh_ply("Smoothed.ply", surfaceMesh);

    CGAL::draw( surfaceMesh );

    return 0;
}