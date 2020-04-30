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

#include "PCCommon/common.hpp"
#include "PCCommon/extraction.hpp"
#include "PCCommon/Projection.hpp"
#include "CameraGeometry/CameraProjection.hpp"
#include "Geometry/SimpleIntersection.hpp"
#include "Profiling/SimpleTime.hpp"

namespace pcu
{

template < typename rT >
class HoleBoundaryPoints {
public:
    HoleBoundaryPoints() : pCamProj(nullptr) {}
    HoleBoundaryPoints( const HoleBoundaryPoints<rT> &other ) : pCamProj(nullptr) {
        this->id               = other.id;
        this->polygonIndices   = other.polygonIndices;
        this->equivalentNormal = other.equivalentNormal;
        if ( nullptr != other.pCamProj.get() ) {
            this->pCamProj     = std::make_shared<CameraProjection<rT>>(*(other.pCamProj));
        }
    }

    ~HoleBoundaryPoints() = default;

    HoleBoundaryPoints<rT>& operator = ( const HoleBoundaryPoints<rT> &other ) {
        if ( &other == this ) {
            return *this;
        }

        this->id               = other.id;
        this->polygonIndices   = other.polygonIndices;
        this->equivalentNormal = other.equivalentNormal;
        if ( nullptr != other.pCamProj.get() ) {
            this->pCamProj     = std::make_shared<CameraProjection<rT>>(*(other.pCamProj));
        } else {
            this->pCamProj.reset();
        }

        return *this;
    }

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

        if ( nullptr != hbp.pCamProj.get() ) {
            out << "\"camProj\": ";

            out << *(hbp.pCamProj) << std::endl;
        }

        out << std::endl << "}";
    }

public:
    int id;
    std::vector<int> polygonIndices;
    pcl::PointNormal equivalentNormal;
    std::shared_ptr< CameraProjection<rT> > pCamProj;
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

    if ( nullptr != pCamProj.get() ) {
        ofs << baseIndentPlus << "\"haveCamProj\": true," << std::endl;
        ofs << baseIndentPlus << "\"camProj\": ";
        pCamProj->write_json_content(ofs, indent, baseIndentNum+1);
    } else {
        ofs << baseIndentPlus << "\"haveCamProj\": false";
    }

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
    HoleBoundaryProjector() : projectionNumberLimit(3) {}
    ~HoleBoundaryProjector() = default;

    void set_projection_number_limit(int v);
    void set_projection_curvature_limit(rT v);

    void process( const typename PC_t::Ptr pInCloud,
            const DS_t& DS,
            const PN_t::Ptr pEquivalentNormals,
            const std::vector< CameraProjection<rT> >& cameraProjections,
            std::vector< HoleBoundaryPoints<rT> >& hbp);

protected:
    int find_best_camera( const pcl::PointNormal& en,
            const std::vector< CameraProjection<rT> >& cameraProjections,
            bool flagDebug = false);

    bool are_points_visible_2_camera(
            const typename pcl::PointCloud<pT>::Ptr pInput,
            const CameraProjection<rT> &camProj );

    void plane_projection_and_convex_hull(
            const typename PC_t::Ptr pInput,
            const pcl::PointIndices::Ptr pIndices,
            const pcl::PointNormal &pn,
            std::vector<int> &polygonIndices );

    void find_and_add_polygon( const typename pcl::PointCloud<pT>::Ptr pSubSet,
                               const pcl::PointIndices::Ptr pOriginalIndices,
                               const pcl::PointNormal &planePointNormal,
                               int id,
                               const pcl::PointNormal &equivalentNormal,
                               const CameraProjection<rT> &cProj,
                               std::vector<HoleBoundaryPoints<rT>> &hbp );

    void find_and_add_polygon( const typename pcl::PointCloud<pT>::Ptr pSubSet,
                               const pcl::PointIndices::Ptr pOriginalIndices,
                               const pcl::PointNormal &planePointNormal,
                               int id,
                               const pcl::PointNormal &equivalentNormal,
                               std::vector<HoleBoundaryPoints<rT>> &hbp );

private:
    int projectionNumberLimit;
    rT projectionCurvatureLimit;
};

template < typename pT, typename rT >
void HoleBoundaryProjector<pT, rT>::set_projection_number_limit(int v) {
    assert( v >= 3 );
    projectionNumberLimit = v;
}

template < typename pT, typename rT >
void HoleBoundaryProjector<pT, rT>::set_projection_curvature_limit(rT v) {
    assert( v > 0 );
    projectionCurvatureLimit = v;
}

template < typename pT, typename rT >
int HoleBoundaryProjector<pT, rT>::find_best_camera(
        const pcl::PointNormal& en,
        const std::vector< CameraProjection<rT> >& cameraProjections,
        bool flagDebug ) {

    const int nCams = cameraProjections.size();

    // Convert the equivalent normal's centroid and normal to Eigen vectors.
    Eigen::Vector3<rT> centroid;
    centroid << en.x, en.y, en.z;
    Eigen::Vector3<rT> normal;
    normal << en.normal_x, en.normal_y, en.normal_z;

    Eigen::Vector3<rT> imgPlanePoint;
    rT angleCosine;
    rT imgPlaneDistance;

    rT bestDistance = 0.0f;
    int bestCamIdx = -1;

    const int debugIdx0 = 1449;
    const int debugIdx1 = 1450;

    // Loop over all cameras.
    for ( int i = 0; i< nCams; ++i ) {
        // Get the camera projection object.
        const CameraProjection<rT>& cProj = cameraProjections[i];

        if ( flagDebug && ( debugIdx0 == cProj.id || debugIdx1 == cProj.id ) ) {
            std::cout << "Debug id = " << cProj.id << std::endl;
            std::cout << "centroid: " << std::endl << centroid << std::endl;
            std::cout << "normal: " << std::endl << normal << std::endl;

            std::cout << "cProj: " << std::endl << cProj << std::endl;
        }

        // Check if the equivalent normal's centroid point
        // could be projected to the cameras image region.
        if ( !cProj.is_world_point_in_image(centroid) ) {
            if ( flagDebug && ( debugIdx0 == cProj.id || debugIdx1 == cProj.id ) ) {
                std::cout << "Equivalent centroid is not in the FOV of the camera. " << std::endl;
            }
            // Not observable by this camera.
            continue;
        }

        // Get the cosine of the 3D angle between the equivalent normal
        // and the vector from camera centroid to the equivalent normal centroid.
//        angleCosine = cProj.angle_cosine_between_camera_normal(normal);
        angleCosine = normal.dot( centroid - cProj.T );

        if ( angleCosine >= -0.5 ) {
            if ( flagDebug && ( debugIdx0 == cProj.id || debugIdx1 == cProj.id ) ) {
                std::cout << "Equivalent normal has angleCoseine = " << angleCosine << ". " << std::endl;
            }
            // Filter out normal directions that a smaller than 120 degrees.
            continue;
        }

        // Get the intersection of the equivalent normal and the camera sensor plane.
        auto camNormal = cProj.get_z_axis();

        if (! line_plane_directed_intersection( centroid, normal, cProj.T, camNormal, imgPlanePoint ) ) {
            if ( flagDebug && ( debugIdx0 == cProj.id || debugIdx1 == cProj.id ) ) {
                std::cout << "Equivalent normal has no positive intersection. " << std::endl;
            }
            // No positive intersection.
            continue;
        }

        imgPlaneDistance = ( imgPlanePoint - cProj.T ).norm();

        if ( flagDebug && ( debugIdx0 == cProj.id || debugIdx1 == cProj.id ) ) {
            std::cout << "camNormal: " << std::endl << camNormal << std::endl;
            std::cout << "imgPlanePoint: " << std::endl << imgPlanePoint << std::endl;
            std::cout << "imgPlaneDistance: " << std::endl << imgPlaneDistance << std::endl;
        }

        if ( -1 == bestCamIdx ) {
            bestDistance = imgPlaneDistance;
            bestCamIdx = i;
            continue;
        }

        if (imgPlaneDistance < bestDistance ) {
            bestDistance = imgPlaneDistance;
            bestCamIdx = i;
        }
    }

    return bestCamIdx;
}

template < typename pT, typename rT >
bool HoleBoundaryProjector<pT, rT>::are_points_visible_2_camera(
        const typename pcl::PointCloud<pT>::Ptr pInput,
        const CameraProjection<rT> &camProj ) {
    // Convert PCL point cloud to Eigen matrix.
    Eigen::MatrixX<rT> wp;
    convert_pcl_2_eigen_matrix<pT, rT>( pInput, wp );

    // Check visibility.
    return camProj.are_world_points_in_image( wp );
}

template < typename pT, typename rT >
void HoleBoundaryProjector<pT, rT>::plane_projection_and_convex_hull(
        const typename PC_t::Ptr pInput,
        const pcl::PointIndices::Ptr pIndices,
        const pcl::PointNormal &pn,
        std::vector<int> &polygonIndices ) {

    // Extract the points.
    typename PC_t::Ptr pPoints ( new PC_t );
//    extract_points<pT>( pInput, pPoints, pIndices );
    pPoints = pInput; // pInput is the extracted points already.

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
void HoleBoundaryProjector<pT, rT>::find_and_add_polygon( const typename pcl::PointCloud<pT>::Ptr pSubSet,
                           const pcl::PointIndices::Ptr pOriginalIndices,
                           const pcl::PointNormal &planePointNormal,
                           int id,
                           const pcl::PointNormal &equivalentNormal,
                           std::vector<HoleBoundaryPoints<rT>> &hbp ) {
    // The HoleBoundaryPoints object.
    HoleBoundaryPoints<rT> newHBP;

    // Convex hull.
    plane_projection_and_convex_hull( pSubSet, pOriginalIndices, planePointNormal, newHBP.polygonIndices );

    // Copy values.
    newHBP.id = id;
    newHBP.equivalentNormal = equivalentNormal;

    hbp.push_back( newHBP );
}

template < typename pT, typename rT >
void HoleBoundaryProjector<pT, rT>::find_and_add_polygon( const typename pcl::PointCloud<pT>::Ptr pSubSet,
                                                          const pcl::PointIndices::Ptr pOriginalIndices,
                                                          const pcl::PointNormal &planePointNormal,
                                                          int id,
                                                          const pcl::PointNormal &equivalentNormal,
                                                          const CameraProjection<rT> &cProj,
                                                          std::vector<HoleBoundaryPoints<rT>> &hbp ) {
    find_and_add_polygon( pSubSet, pOriginalIndices, planePointNormal, id, equivalentNormal, hbp );
    hbp[ hbp.size() - 1 ].pCamProj = std::make_shared<CameraProjection<rT>>( cProj );
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
    bool flagDebug = false;
    for ( int i = 0; i < nEN; ++i ) {
        // Check if we have enough points to perform a projection.
        if (DS[i].size() < projectionNumberLimit ) {
            std::cout << i << ": "
                      << "Set contains " << DS[i].size()
                      << " points which is less than the limit ("
                      << projectionNumberLimit << "). " << std::endl;
            continue;
        }

        // Get the points.
        pcl::PointIndices::Ptr pIndicesOnInput = convert_vector_2_pcl_indices( DS[i] );
        typename pcl::PointCloud<pT>::Ptr pPointSubSet =
                extract_points<pT>( pInCloud, pIndicesOnInput );

        // Get the equivalent normal.
        pcl::PointNormal &en = pEquivalentNormals->at(i);

        // ========== Find the best camera that observe this equivalent normal.
//        flagDebug = ( 15 == i ) ? true : false;
        int bestCamIdx = find_best_camera(en, cameraProjections, flagDebug);

        if ( -1 != bestCamIdx ) {
            std::cout << i << ": "
                      << "Best camera id for equivalent normal " << en
                      << " is " << cameraProjections[bestCamIdx].id << std::endl;

            // Get the best camera projection object.
            const CameraProjection<rT> &cProj = cameraProjections[bestCamIdx];

            // Check visibility.
            if ( !are_points_visible_2_camera( pPointSubSet, cProj ) ) {
                std::cout << i << ": "
                          << "Set contains points out of the FOV of the camera. " << std::endl;
            } else {
                // Project the points to the camera plane and
                // find the 2D polygon of the projected boundary points.
                pcl::PointNormal cameraPlaneNormal;
                cameraPlaneNormal.curvature = 0.0f;
                cProj.get_center( cameraPlaneNormal.x,
                                  cameraPlaneNormal.y, cameraPlaneNormal.z );
                cProj.get_z_axis( cameraPlaneNormal.normal_x,
                                  cameraPlaneNormal.normal_y, cameraPlaneNormal.normal_z );

                find_and_add_polygon( pPointSubSet, pIndicesOnInput, cameraPlaneNormal, i, en, cProj, hbp );

                continue;
            }
        } else {
            std::cout << i << ": "
                      << "Equivalent normal " << en
                      << " has no valid camera pose. " << std::endl;
        }

        if ( en.curvature <= projectionCurvatureLimit ) {
            std::cout << i << ": Equivalent normal has a curvature lower than the limit. " << std::endl;
            find_and_add_polygon( pPointSubSet, pIndicesOnInput, en, i, en, hbp );
            continue;
        } else {
            std::cout << i << ": Equivalent normal has a curvature (" << en.curvature << ") higher than the limit ("
                      << projectionCurvatureLimit << "). " << std::endl;
        }
    }
}

} // Namespace pcu.

#endif //POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_HOLEBOUNDARYPROJECTION_HPP
