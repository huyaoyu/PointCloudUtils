//
// Created by yaoyu on 3/31/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_HOLEBOUNDARYPROJECTION_HPP
#define POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_HOLEBOUNDARYPROJECTION_HPP

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>

#include "PCCommon/extraction.hpp"
#include "PCCommon/Projection.hpp"
#include "CameraGeometry/CameraProjection.hpp"
#include "Profiling/SimpleTime.hpp"

namespace pcu
{

template < typename rT >
class HoleBoundaryPoints {
public:
    HoleBoundaryPoints() = default;
    ~HoleBoundaryPoints() = default;

    void write_json_content( std::ofstream &ofs,
        const std::string& indent, int baseIndentNum ) const;

    friend std::ostream& operator << ( std::ostream &out, const HoleBoundaryPoints<rT> & hbp ) {
        out << "{" << std::endl;
        out << "\"id\": " << hbp.id << "," << std::endl;
        out << "\"centroid\": [ "
            << hbp.equivalentNormal.x << ", "
            << hbp.equivalentNormal.y << ", "
            << hbp.equivalentNormal.z << " ]," << std::endl;
        out << "\"normal\": [ "
            << hbp.equivalentNormal.normal_x << ", "
            << hbp.equivalentNormal.normal_y << ", "
            << hbp.equivalentNormal.normal_z << " ]," << std::endl;
        out << "\"curvature\": " << hbp.equivalentNormal.curvature << "," << std::endl;
        out << "\"polygonIndices\": [ ";

        const int lineBreak = 10;
        const int nPI = hbp.polygonIndices.size();
        for ( int i = 0; i < nPI; ++i ) {
            if ( i == nPI - 1 ) {
                out << hbp.polygonIndices[i] << " ]," << std::endl;
                break;
            }

            out << hbp.polygonIndices[i] << ", ";

            if ( (i+1) % lineBreak == 0 && i != 0 ) {
                out << std::endl;
            }
        }

        out << "\"camProj\": ";

        out << hbp.camProj << std::endl;

        out << std::endl << "}";
    }

public:
    int id;
    std::vector<int> polygonIndices;
    pcl::PointNormal equivalentNormal;
    CameraProjection<rT> camProj;
};

template < typename rT >
void HoleBoundaryPoints<rT>::write_json_content( std::ofstream &ofs,
        const std::string& indent, int baseIndentNum ) const {
    std::string baseIndent = "";

    for ( int i = 0; i < baseIndentNum; ++i ) {
        baseIndent += indent;
    }

    const std::string baseIndentPlus = baseIndent + indent;
    const std::string arrayLineSpace = "     ";

    ofs << "{" << std::endl;
    ofs << baseIndentPlus << "\"id\": " << id << "," << std::endl;
    ofs << baseIndentPlus << "\"centroid\": [ "
        << equivalentNormal.x << ", " << equivalentNormal.y << ", " << equivalentNormal.z << " ]," << std::endl;
    ofs << baseIndentPlus << "\"normal\": [ "
        << equivalentNormal.normal_x << ", "
        << equivalentNormal.normal_y << ", "
        << equivalentNormal.normal_z << " ]," << std::endl;
    ofs << baseIndentPlus << "\"curvature\": " << equivalentNormal.curvature << "," << std::endl;
    ofs << baseIndentPlus << "\"polygonIndices\": [ ";

    const int lineBreak = 10;
    const int nPI = polygonIndices.size();
    for ( int i = 0; i < nPI; ++i ) {
        if ( i == nPI - 1 ) {
            ofs << polygonIndices[i] << " ]," << std::endl;
            break;
        }

        ofs << polygonIndices[i] << ", ";

        if ( (i+1) % lineBreak == 0 && i != 0 ) {
            ofs << std::endl << baseIndentPlus << arrayLineSpace;
        }
    }

    ofs << baseIndentPlus << "\"camProj\": ";

    camProj.write_json_content(ofs, indent, baseIndentNum+1);

    ofs << std::endl << baseIndent << "}";
}

template < typename pT, typename rT >
class HoleBoundaryProjector {
public:
    typedef pcl::PointCloud<pT> PC_t;
    typedef pcl::PointCloud<pcl::PointNormal> PN_t;
    typedef std::vector< std::vector<int> > DS_t; // Disjoint set type.
//    typedef std::shared_ptr<DS_t> DSPtr;

public:
    HoleBoundaryProjector() = default;
    ~HoleBoundaryProjector() = default;

    void process( const typename PC_t::Ptr pInCloud,
            const DS_t& DS,
            const PN_t::Ptr pEquivalentNormals,
            const std::vector< CameraProjection<rT> >& cameraProjections,
            std::vector< HoleBoundaryPoints<rT> >& hbp);

protected:
    int find_best_camera( const pcl::PointNormal& en,
            const std::vector< CameraProjection<rT> >& cameraProjections );

    void plane_projection_and_convex_hull(
            const typename PC_t::Ptr pInput,
            const pcl::PointIndices::Ptr pIndices,
            const pcl::PointNormal &pn,
            std::vector<int> &polygonIndices );
};

template < typename pT, typename rT >
int HoleBoundaryProjector<pT, rT>::find_best_camera(
        const pcl::PointNormal& en,
        const std::vector< CameraProjection<rT> >& cameraProjections ) {

    const int nCams = cameraProjections.size();

    // Convert the equivalent normal's centroid and normal to Eigen vectors.
    Eigen::Vector3<rT> centroid;
    centroid << en.x, en.y, en.z;
    Eigen::Vector3<rT> normal;
    normal << en.normal_x, en.normal_y, en.normal_z;

    Eigen::Vector3<rT> pixel;
    rT angleCosine;
    rT pixelDistance;

    rT bestDistance = 0.0f;
    int bestCamIdx = -1;

    // Loop over all cameras.
    for ( int i = 0; i< nCams; ++i ) {
        // Get the camera projection object.
        const CameraProjection<rT>& cProj = cameraProjections[i];

//        if ( 330 == cProj.id ) {
//            std::cout << "Debug id = " << 330 << std::endl;
//            std::cout << "centroid: " << std::endl << centroid << std::endl;
//            std::cout << "normal: " << std::endl << normal << std::endl;
//
//            std::cout << "cProj.R: " << std::endl << cProj.R << std::endl;
//        }

        // Check if the equivalent normal's centroid point
        // could be projected to the cameras image region.
        if ( !cProj.is_world_point_in_image(centroid) ) {
            // Not observable by this camera.
            continue;
        }

        // Get the cosine of the 3D angle between the equivalent normal
        // and the camera normal.
        angleCosine = cProj.angle_cosine_between_camera_normal(normal);

        if ( angleCosine >= -0.707 ) {
            // Filter out normal directions that a smaller than 135 degrees.
            continue;
        }

        // Get the camera projection pixel coordinate and the distance from the pincipal point.
        cProj.world_2_pixel(centroid, pixel);
        pixelDistance = cProj.pixel_distance_from_principal_point(pixel);

        pixelDistance = pixelDistance * std::exp( 1.0f + angleCosine );

        if ( -1 == bestCamIdx ) {
            bestDistance = pixelDistance;
            bestCamIdx = i;
            continue;
        }

        if ( pixelDistance < bestDistance ) {
            bestDistance = pixelDistance;
            bestCamIdx = i;
        }
    }

    return bestCamIdx;
}

template < typename pT, typename rT >
void HoleBoundaryProjector<pT, rT>::plane_projection_and_convex_hull(
        const typename PC_t::Ptr pInput,
        const pcl::PointIndices::Ptr pIndices,
        const pcl::PointNormal &pn,
        std::vector<int> &polygonIndices ) {

    // Extract the points.
    typename PC_t::Ptr pPoints ( new PC_t );
    extract_points<pT>( pInput, pPoints, pIndices );

    // Projection.
    typename PC_t::Ptr pProjected = project_2_plane<pT>(pPoints, pn);

    // Convex hull.
    typename PC_t::Ptr pConvexHull ( new PC_t );
    pcl::ConvexHull<pT> ch;
    ch.setInputCloud(pProjected);
    ch.reconstruct(*pConvexHull);

    pcl::PointIndices::Ptr pChIndices (new pcl::PointIndices);
    ch.getHullPointIndices(*pChIndices);

    // Copy the indices.
    const auto N = pChIndices->indices.size();
    polygonIndices.resize( N );

    for ( int i = 0; i < N; ++i ) {
        polygonIndices[i] = pIndices->indices[ pChIndices->indices[i] ];
    }
}

template < typename pT, typename rT >
void HoleBoundaryProjector<pT, rT>::process(const typename PC_t::Ptr pInCloud,
        const DS_t& DS,
        const PN_t::Ptr pEquivalentNormals,
        const std::vector< CameraProjection<rT> >& cameraProjections,
        std::vector<HoleBoundaryPoints<rT>> &hbp) {

    hbp.clear();

    const int nEN = pEquivalentNormals->size();

    // Loop over all the equivalent normals.
    for ( int i = 0; i < nEN; ++i ) {
        // ========== Find the best camera that observe this equivalent normal.
        // Get the equivalent normal.
        pcl::PointNormal &en = pEquivalentNormals->at(i);
        int bestCamIdx = find_best_camera(en, cameraProjections);

        if ( -1 == bestCamIdx ) {
            std::cout << i << ": "
                      << "Equivalent normal " << en
                      << " has no valid camera pose. " << std::endl;
            continue;
        }

        // Test use.
        {
            std::cout << i << ": "
                      << "Best camera id for equivalent normal " << en
                      << " is " << cameraProjections[bestCamIdx].id << std::endl;
        }

        // Get the best camera projectiong object.
        const CameraProjection<rT> &cProj = cameraProjections[bestCamIdx];

        // Project the points to the camera plane and
        // find the 2D polygon of the projected boundary points.
        pcl::PointNormal cameraPlaneNormal; cameraPlaneNormal.curvature = 0.0f;
        cProj.get_center( cameraPlaneNormal.x,
                cameraPlaneNormal.y, cameraPlaneNormal.z );
        cProj.get_z_axis( cameraPlaneNormal.normal_x,
                cameraPlaneNormal.normal_y, cameraPlaneNormal.normal_z );

        // The HoleBoundaryPoints object.
        HoleBoundaryPoints<rT> newHBP;

        pcl::PointIndices::Ptr pIndicesOnInput = convert_vector_2_pcl_indices( DS[i] );
        plane_projection_and_convex_hull( pInCloud, pIndicesOnInput, cameraPlaneNormal, newHBP.polygonIndices );

        // Copy values.
        newHBP.id = i;
        newHBP.equivalentNormal = en;
        newHBP.camProj = cProj;

        hbp.push_back( newHBP );
    }
}

}

#endif //POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_HOLEBOUNDARYPROJECTION_HPP
