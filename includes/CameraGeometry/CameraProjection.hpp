//
// Created by yaoyu on 3/29/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP
#define POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "DataInterfaces/JSONHelper/Reader.hpp"
#include "DataInterfaces/Plain/FromVector.hpp"
#include "Exception/Common.hpp"
#include "Profiling/Instrumentor.hpp"

struct NoCamProjFound : virtual exception_common_base {};

template < typename rT >
class CameraProjection {
public:
    CameraProjection()
    : K( Eigen::Matrix3<rT>::Identity() ),
      RC( Eigen::Matrix3<rT>::Identity() ),
      R( Eigen::Matrix3<rT>::Identity() ),
      T( Eigen::Vector3<rT>::Zero() ),
      Q( Eigen::Quaternion<rT>( 1.0, 0.0, 0.0, 0.0 ) ),
      height(3008), width(4112), id(-1) {
        RCtRt = RC.transpose() * R.transpose();
    }

    CameraProjection( const CameraProjection<rT>& other ) {
        this->K  = other.K;
        this->RC = other.RC;
        this->R  = other.R;
        this->T  = other.T;
        this->Q  = other.Q;

        this->height = other.height;
        this->width  = other.width;

        this->id = other.id;

        this->RCtRt = this->RC.transpose() * this->R.transpose();

        for ( int i = 0; i < 4; ++i ) {
            this->frustumNormals[i] = other.frustumNormals[i];
        }
    }

    CameraProjection( CameraProjection<rT>&& other ) noexcept {
        this->swap(other);
    }

    ~CameraProjection() = default;

    CameraProjection<rT>& operator = ( const CameraProjection<rT>& other ) {
        if ( this == &other ) {
            return *this;
        }

        this->K  = other.K;
        this->RC = other.RC;
        this->R  = other.R;
        this->T  = other.T;
        this->Q  = other.Q;

        this->height = other.height;
        this->width  = other.width;

        this->id = other.id;

        this->RCtRt = this->RC.transpose() * this->R.transpose();

        for ( int i = 0; i < 4; ++i ) {
            this->frustumNormals[i] = other.frustumNormals[i];
        }

        return *this;
    }

    CameraProjection<rT>& operator= ( CameraProjection<rT>&& other ) noexcept {
        this->swap( other );
        return *this;
    }

    void swap( CameraProjection<rT>& other ) noexcept {
        std::swap( K,  other.K );
        std::swap( RC, other.RC );
        std::swap( R,  other.R );
        std::swap( T,  other.T );
        std::swap( Q,  other.Q );
        std::swap( height, other.height );
        std::swap( width,  other.width );
        std::swap( id, other.id );
        std::swap( RCtRt, other.RCtRt );
        std::swap( frustumNormals, other.frustumNormals );
    }

    void set_R(const Eigen::Matrix3<rT> &r);
    void set_RC(const Eigen::Matrix3<rT> &rc);
    void update_RCtRt();

    void update_frustum_normals(void);
    void scale_intrinsics(rT s);

    void set_rotation_by_quaternion( const Eigen::Quaternion<rT> &q );

    void get_center( rT &x, rT &y, rT &z ) const;

    Eigen::Vector3<rT> get_z_axis() const;
    void get_z_axis( rT &nx, rT &ny, rT &nz ) const;

    void world_2_camera( const Eigen::Vector3<rT>& wp,
                        Eigen::Vector3<rT>& cp ) const;

    void camera_2_pixel( const Eigen::Vector3<rT>& cp,
                        Eigen::Vector3<rT>& pixel ) const;

    void sensor_2_pixel( const Eigen::Vector3<rT>& sp,
            Eigen::Vector3<rT> &pixel ) const;

    void world_2_sensor( const Eigen::Vector3<rT> &wp,
                         Eigen::Vector3<rT> &sp ) const;

    void world_2_pixel( const Eigen::Vector3<rT>& wp,
                       Eigen::Vector3<rT>& pixel ) const;

    void world_2_pixel( const Eigen::Vector3<rT> &wp,
                        Eigen::Vector3<rT> &sp,
                        Eigen::Vector3<rT> &pixel ) const;

    void pixel_2_sensor( const Eigen::Vector3<rT> &pixelWithDepth,
            Eigen::Vector3<rT> &sp ) const;

    void pixel_2_camera( const Eigen::Vector3<rT>& pixelWithDepth,
                         Eigen::Vector3<rT>& cp ) const;

    void camera_2_world( const Eigen::Vector3<rT>& cp,
                        Eigen::Vector3<rT>& wp ) const;

    void pixel_2_world(const Eigen::Vector3<rT>& pixelWithDepth,
                       Eigen::Vector3<rT>& wp ) const;

    void pixel_2_world(const Eigen::MatrixX<rT> &pixelsWithDepth,
            Eigen::MatrixX<rT> &wp) const;

    bool is_camera_point_in_image( const Eigen::Vector3<rT>& cp ) const;
    bool is_world_point_in_image( const Eigen::Vector3<rT>& wp ) const;
    bool are_world_points_in_image( const Eigen::MatrixX<rT> &wp ) const;

    template < typename Derived >
    bool are_world_points_outside_frustum( const Eigen::MatrixBase<Derived> &wp ) const;

    rT angle_cosine_between_camera_normal( const Eigen::Vector3<rT> &wn ) const;
    rT pixel_distance_from_principal_point( rT x, rT y ) const ;
    rT pixel_distance_from_principal_point( const Eigen::Vector3<rT> &pixel ) const;

    void write_json_content( std::ofstream &ofs, const std::string& indent, int baseIndentNum ) const;

    friend std::ostream& operator << ( std::ostream &out, const CameraProjection<rT> & cp ) {
        out << "{" << std::endl;
        out << "\"id\": " << cp.id << "," << std::endl;
        out << "\"height\": " << cp.height << "," << std::endl;
        out << "\"width\": " << cp.width << "," << std::endl;
        out << "\"K\": [ "
            << cp.K(0, 0) << ", " << cp.K(0, 1) << ", " << cp.K(0, 2) << ", " << std::endl
            << cp.K(1, 0) << ", " << cp.K(1, 1) << ", " << cp.K(1, 2) << ", " << std::endl
            << cp.K(2, 0) << ", " << cp.K(2, 1) << ", " << cp.K(2, 2) << " ]," << std::endl;
        out << "\"RC\": [ "
            << cp.RC(0, 0) << ", " << cp.RC(0, 1) << ", " << cp.RC(0, 2) << ", " << std::endl
            << cp.RC(1, 0) << ", " << cp.RC(1, 1) << ", " << cp.RC(1, 2) << ", " << std::endl
            << cp.RC(2, 0) << ", " << cp.RC(2, 1) << ", " << cp.RC(2, 2) << " ]," << std::endl;
        out << "\"R\": [ "
            << cp.R(0, 0) << ", " << cp.R(0, 1) << ", " << cp.R(0, 2) << ", " << std::endl
            << cp.R(1, 0) << ", " << cp.R(1, 1) << ", " << cp.R(1, 2) << ", " << std::endl
            << cp.R(2, 0) << ", " << cp.R(2, 1) << ", " << cp.R(2, 2) << " ]," << std::endl;
        out << "\"T\": [ "
            << cp.T(0) << ", " << cp.T(1) << ", " << cp.T(2) << " ]," << std::endl;
        out << "\"Q\": [ "
            << cp.Q.w() << ", " << cp.Q.x() << ", " << cp.Q.y() << ", " << cp.Q.z() << " ]," << std::endl;

        out << "\"frustumNormals\": [" << std::endl;
        for ( int i = 0; i < 3; ++i ) {
            out << "[ "
                << cp.frustumNormals[i](0) << ", "
                << cp.frustumNormals[i](1) << ", "
                << cp.frustumNormals[i](2) << " ],"
                << std::endl;
        }
        out << "[ "
            << cp.frustumNormals[3](0) << ", "
            << cp.frustumNormals[3](1) << ", "
            << cp.frustumNormals[3](2) << " ] ]"
            << std::endl;

        out << "}";

        return out;
    }

public:
    Eigen::Matrix3<rT> K;
    Eigen::Matrix3<rT> RC;
    Eigen::Matrix3<rT> R;
    Eigen::Vector3<rT> T;

    Eigen::Quaternion<rT> Q;

    int height;
    int width;

    int id;

    Eigen::Matrix3<rT> RCtRt;
    Eigen::Vector3<rT> frustumNormals[4];
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template < typename rT >
void CameraProjection<rT>::set_R(const Eigen::Matrix3<rT> &r) {
    R = r;
    update_RCtRt();
}

template < typename rT >
void CameraProjection<rT>::set_RC(const Eigen::Matrix3<rT> &rc) {
    RC = R;
    update_RCtRt();
}

template < typename rT >
void CameraProjection<rT>::update_RCtRt() {
    RCtRt = RC.transpose() * R.transpose();
}

template < typename rT >
void CameraProjection<rT>::update_frustum_normals(void) {
    // The 4 3D corner points of the pixel-plane in the sensor frame (z-axis forwards).
    Eigen::Vector3<rT> ic[4];
    // Small errors using width-1 and height-1.
    ic[0] <<         0 - K(0, 2),          0 - K(1, 2), K(0,0);
    ic[1] <<     width - K(0, 2),          0 - K(1, 2), K(0,0);
    ic[2] <<     width - K(0, 2),     height - K(1, 2), K(0,0);
    ic[3] <<         0 - K(0, 2),     height - K(1, 2), K(0,0);

    // The normal vectors of the frustum faces.
    frustumNormals[0] = ic[1].cross( ic[0] ); frustumNormals[0].normalize();
    frustumNormals[1] = ic[2].cross( ic[1] ); frustumNormals[1].normalize();
    frustumNormals[2] = ic[3].cross( ic[2] ); frustumNormals[2].normalize();
    frustumNormals[3] = ic[0].cross( ic[3] ); frustumNormals[3].normalize();
}

template < typename rT >
void CameraProjection<rT>::scale_intrinsics(rT s) {
    assert( s > 0 );

    height = static_cast<int>( std::round( s * height ) );
    width  = static_cast<int>( std::round( s * width ) );

    K *= s;
    K(2,2) = static_cast<rT>(1.0);

    update_frustum_normals();
}

template < typename rT >
void CameraProjection<rT>::set_rotation_by_quaternion( const Eigen::Quaternion<rT> &q ) {
    R = q.toRotationMatrix();
    Q = q;
}

template < typename rT >
void CameraProjection<rT>::get_center( rT &x, rT &y, rT &z ) const {
    x = T(0);
    y = T(1);
    z = T(2);
}

template < typename rT >
Eigen::Vector3<rT> CameraProjection<rT>::get_z_axis() const {
    Eigen::Vector3<rT> vz;

    vz << RC(0,2), RC(1,2), RC(2,2);
    vz = R * vz.eval();

    return vz;
}

template < typename rT >
void CameraProjection<rT>::get_z_axis( rT &nx, rT &ny, rT &nz ) const {
    Eigen::Vector3<rT> normal = get_z_axis();

    nx = normal(0);
    ny = normal(1);
    nz = normal(2);
}

template < typename rT >
void CameraProjection<rT>::world_2_camera(const Eigen::Vector3<rT> &wp, Eigen::Vector3<rT> &cp) const {
//    PROFILE_FUNCTION()
    cp = R.transpose() * ( wp.eval() - T);
}

template < typename rT >
void CameraProjection<rT>::camera_2_pixel(const Eigen::Vector3<rT> &cp, Eigen::Vector3<rT> &pixel) const {
    Eigen::Vector3<rT> sensorPoint = RC.transpose() * cp;
    sensor_2_pixel(sensorPoint, pixel);
}

template < typename rT >
void CameraProjection<rT>::sensor_2_pixel( const Eigen::Vector3<rT>& sp,
                     Eigen::Vector3<rT> &pixel ) const {
//    PROFILE_FUNCTION()
    if ( sp(2) <= 0 ) {
        std::stringstream ss;
        ss << "Point " << sp << " is located at the back of the camera. ";
        throw( std::runtime_error( ss.str() ) );
    }

    pixel = K * sp.eval();

    pixel(0) = pixel(0) / pixel(2);
    pixel(1) = pixel(1) / pixel(2);
    pixel(2) = static_cast<rT>(1.0);
}

template < typename rT >
void CameraProjection<rT>::world_2_pixel(const Eigen::Vector3<rT> &wp, Eigen::Vector3<rT> &pixel) const {
    Eigen::Vector3<rT> cp;
    world_2_camera(wp, cp);
    camera_2_pixel(cp, pixel);
}

template < typename rT >
void CameraProjection<rT>::world_2_pixel( const Eigen::Vector3<rT> &wp,
                    Eigen::Vector3<rT> &sp,
                    Eigen::Vector3<rT> &pixel ) const {

    world_2_sensor(wp, sp);
    sensor_2_pixel(sp, pixel);
}

template < typename rT >
void CameraProjection<rT>::world_2_sensor( const Eigen::Vector3<rT> &wp,
                     Eigen::Vector3<rT> &sp ) const {
    sp = RCtRt * ( wp.eval() - T);
}

template < typename rT >
void CameraProjection<rT>::pixel_2_sensor( const Eigen::Vector3<rT> &pixelWithDepth,
                     Eigen::Vector3<rT> &sp ) const {
    if ( pixelWithDepth(2) <= 0 ) {
        std::stringstream ss;
        ss << "Non-positive depth is not allowed. pixelWidthDepth = " << pixelWithDepth;
        throw( std::runtime_error( ss.str() ) );
    }

    sp(0) = ( pixelWithDepth(0) - K(0, 2) ) / K(0, 0) * pixelWithDepth(2);
    sp(1) = ( pixelWithDepth(1) - K(1, 2) ) / K(1, 1) * pixelWithDepth(2);
    sp(2) = pixelWithDepth(2);
}

template < typename rT >
void CameraProjection<rT>::pixel_2_camera(const Eigen::Vector3<rT> &pixelWithDepth, Eigen::Vector3<rT> &cp) const {
    Eigen::Vector3<rT> sensorPoint;

    pixel_2_sensor( pixelWithDepth, sensorPoint );

    cp = RC * sensorPoint;
}

template < typename rT >
void CameraProjection<rT>::camera_2_world(const Eigen::Vector3<rT> &cp, Eigen::Vector3<rT> &wp) const {
    wp = R * cp.eval() + T;
}

template < typename rT >
void CameraProjection<rT>::pixel_2_world(const Eigen::Vector3<rT> &pixelWithDepth, Eigen::Vector3<rT> &wp) const {
    Eigen::Vector3<rT> cp;
    pixel_2_camera(pixelWithDepth, cp);
    camera_2_world(cp, wp);
}

template < typename rT >
void CameraProjection<rT>::pixel_2_world(const Eigen::MatrixX<rT> &pixelsWithDepth,
                   Eigen::MatrixX<rT> &wp) const {
    // Copy input and reuse.
    wp = pixelsWithDepth;

    // Pixel to sensor.
    wp.row(0).array() -= K(0, 2);
    wp.row(0).array() /= K(0, 0);
    wp.row(0).array() *= pixelsWithDepth.row(2).array();

    wp.row(1).array() -= K(1, 2);
    wp.row(1).array() /= K(1, 1);
    wp.row(1).array() *= pixelsWithDepth.row(2).array();

    // Sensor to world.
    wp = (R * RC * wp).eval().colwise() + T;
}

template < typename rT >
bool CameraProjection<rT>::is_camera_point_in_image(const Eigen::Vector3<rT> &cp) const {
//    PROFILE_FUNCTION()
    Eigen::Vector3<rT> sensorPoint = RC.transpose() * cp;
    if ( sensorPoint(2) <= 0 ) {
        return false;
    }

    Eigen::Vector3<rT> pixel;
    sensor_2_pixel(sensorPoint, pixel);

    // Using pixel(0) >= width may introduce small amount of error.
    if ( pixel(0) < 0 || pixel(0) > width ) {
        return false;
    }
    // Using pixel(1) >= height may introduce small amount of error.
    if ( pixel(1) < 0 || pixel(1) > height ) {
        return false;
    }

    return true;
}

template < typename rT >
bool CameraProjection<rT>::is_world_point_in_image(const Eigen::Vector3<rT> &wp) const {
    Eigen::Vector3<rT> cp;
    world_2_camera(wp, cp);
    return is_camera_point_in_image(cp);
}

template < typename rT >
bool CameraProjection<rT>::are_world_points_in_image( const Eigen::MatrixX<rT> &wps ) const {
    assert( 3 == wps.rows() );

    Eigen::Vector3<rT> wp;
    Eigen::Vector3<rT> cp;

    bool flag = true;

    for ( int i = 0; i < wps.cols(); ++i ) {
        wp << wps(0, i), wps(1, i), wps(2, i);
        world_2_camera(wp, cp);

        if ( !is_camera_point_in_image(cp) ) {
            flag = false;
            break;
        }
    }

    return flag;
}

template < typename rT >
template < typename Derived >
bool CameraProjection<rT>::are_world_points_outside_frustum( const Eigen::MatrixBase<Derived> &wps ) const {
    // Assuming the columns are the points.
    const int N = wps.cols();
    bool res = false; // The return value.

    for ( int f = 0; f < 4; ++f ) {
        int i = 0;

        for ( ; i < N; ++i ) {
            Eigen::Vector3<rT> v;
            v << wps(0,i), wps(1,i), wps(2,i);

            // Transfer the world point to the sensor frame.
            world_2_camera( v, v );
            v = RC.transpose() * v.eval();

            const rT p = frustumNormals[f].dot( v ); // The inner-product.

            if ( p <= 0 ) {
                break;
            }
        }

        if ( i == N ) {
            res = true;
            break;
        }
    }

    return res;
}

template < typename rT >
rT CameraProjection<rT>::angle_cosine_between_camera_normal( const Eigen::Vector3<rT> &wn ) const {
    // NOTE: We use the z-axis as the normal of the camera.
    // Normal of the camera.

    Eigen::Vector3<rT> camNormal = get_z_axis();

    return camNormal.dot( wn );
}

template < typename rT >
rT CameraProjection<rT>::pixel_distance_from_principal_point( rT x, rT y ) const {
    const rT dx = x - K(0, 2);
    const rT dy = y - K(1, 2);

    return std::sqrt( dx * dx + dy * dy );
}

template < typename rT >
rT CameraProjection<rT>::pixel_distance_from_principal_point( const Eigen::Vector3<rT> &pixel ) const {
    return pixel_distance_from_principal_point( pixel(0), pixel(1) );
}

template < typename rT >
void CameraProjection<rT>::write_json_content( std::ofstream &ofs,
        const std::string& indent, int baseIndentNum ) const {
    std::string baseIndent = "";

    for ( int i = 0; i < baseIndentNum; ++i ) {
        baseIndent += indent;
    }

    const std::string baseIndentPlus = baseIndent + indent;
    const std::string arrayLineSpace = "       ";

    ofs << "{" << std::endl;
    ofs << baseIndentPlus << "\"id\": " << id << "," << std::endl;
    ofs << baseIndentPlus << "\"height\": " << height << "," << std::endl;
    ofs << baseIndentPlus << "\"width\": " << width << "," << std::endl;
    ofs << baseIndentPlus << "\"K\": [ "
                                            << K(0, 0) << ", " << K(0, 1) << ", " << K(0, 2) << ", " << std::endl
        << baseIndentPlus << arrayLineSpace << K(1, 0) << ", " << K(1, 1) << ", " << K(1, 2) << ", " << std::endl
        << baseIndentPlus << arrayLineSpace << K(2, 0) << ", " << K(2, 1) << ", " << K(2, 2) << " ]," << std::endl;
    ofs << baseIndentPlus << "\"RC\": [ "
                                            << RC(0, 0) << ", " << RC(0, 1) << ", " << RC(0, 2) << ", " << std::endl
        << baseIndentPlus << arrayLineSpace << RC(1, 0) << ", " << RC(1, 1) << ", " << RC(1, 2) << ", " << std::endl
        << baseIndentPlus << arrayLineSpace << RC(2, 0) << ", " << RC(2, 1) << ", " << RC(2, 2) << " ]," << std::endl;
    ofs << baseIndentPlus << "\"R\": [ "
                                            << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2) << ", " << std::endl
        << baseIndentPlus << arrayLineSpace << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2) << ", " << std::endl
        << baseIndentPlus << arrayLineSpace << R(2, 0) << ", " << R(2, 1) << ", " << R(2, 2) << " ]," << std::endl;
    ofs << baseIndentPlus << "\"T\": [ "
        << T(0) << ", " << T(1) << ", " << T(2) << " ]," << std::endl;
    ofs << baseIndentPlus << "\"Q\": [ "
        << Q.w() << ", " << Q.x() << ", " << Q.y() << ", " << Q.z() << " ]" << std::endl;
    ofs << baseIndent << "}";
}

// ========== Utility function. ==========
template < typename rT >
void convert_from_quaternion_translation(
        int id,
        const Eigen::MatrixX<rT>& quat,
        const Eigen::MatrixX<rT>& pos,
        const Eigen::Matrix3<rT>& K,
        CameraProjection<rT>& cProj ) {
    cProj.K = K;

    Eigen::Vector4<rT> qv;
    // The input is w, x, y, z.
    // For qv, it has to be x, y, z, w.
    qv << quat(0, 1), quat(0, 2), quat(0, 3), quat(0,0);
    qv.normalize();

    Eigen::Quaternion<rT> q(qv);

    cProj.set_rotation_by_quaternion(q);

    cProj.T << pos(0,0), pos(0,1), pos(0,2);

    cProj.id = id;
}

template < typename rT >
void convert_from_quaternion_translation_table(
        const Eigen::VectorXi& id,
        const Eigen::MatrixX<rT>& quat,
        const Eigen::MatrixX<rT>& pos,
        const Eigen::Matrix3<rT>& K,
        std::vector< CameraProjection<rT> >& cps ) {
    const int N = id.rows();

    assert( N == quat.rows() );
    assert( N == pos.rows() );

    cps.resize( N );

    for ( int i = 0; i < N; ++i ) {
        Eigen::MatrixX<rT> quatEntry = quat(i, Eigen::all);
        Eigen::MatrixX<rT> posEntry  = pos(i, Eigen::all);

        convert_from_quaternion_translation( id(i), quatEntry, posEntry, K, cps[i] );
    }

}

template < typename rT >
std::shared_ptr< CameraProjection<rT> > create_camera_projection(
        int id, int height, int width,
        const std::vector<rT> &vK,
        const std::vector<rT> &vRC,
        const std::vector<rT> &vR,
        const std::vector<rT> &vT,
        const std::vector<rT> &vQ ) {
    auto pCamProj = std::make_shared<CameraProjection<rT>>();

    pCamProj->id = id;
    pCamProj->height = height;
    pCamProj->width = width;

    convert_vector_2_eigen_mat3( vK,  pCamProj->K );
    convert_vector_2_eigen_mat3( vRC, pCamProj->RC );
    convert_vector_2_eigen_mat3( vR,  pCamProj->R );
    convert_vector_2_eigen_vector( vT, pCamProj->T );

    pCamProj->Q = Eigen::Quaternion<rT>( vQ[0], vQ[1], vQ[2], vQ[3] );

    return pCamProj;
}

template < typename rT >
std::vector<CameraProjection<rT>> read_cam_proj_from_json( const std::string &fn ) {
    using JSON = nlohmann::json;
    std::vector<CameraProjection<rT>> camProjs;

    std::shared_ptr<JSON> pJson = read_json( fn );

    const auto& jCamProjs = (*pJson)["camProjs"];
    const int N = jCamProjs.size();

    if ( 0 == N ) {
        BOOST_THROW_EXCEPTION( NoCamProjFound() << ExceptionInfoString("No camera projection objects found in the JSON file.") );
    }

    for ( int i = 0; i < N; ++i ) {
        CameraProjection<rT> camProj;
        const auto& jCamProj = jCamProjs[i];

        camProj.id     = jCamProj["id"];
        camProj.height = jCamProj["height"];
        camProj.width  = jCamProj["width"];

        convert_vector_2_eigen_mat3(   jCamProj["K"].get<  std::vector<rT> >(), camProj.K );
        convert_vector_2_eigen_mat3(   jCamProj["RC"].get< std::vector<rT> >(), camProj.RC );
        convert_vector_2_eigen_mat3(   jCamProj["R"].get<  std::vector<rT> >(), camProj.R );
        convert_vector_2_eigen_vector( jCamProj["T"].get<  std::vector<rT> >(), camProj.T );

        std::vector<rT> qv = jCamProj["Q"].get< std::vector<rT> >();

        camProj.Q = Eigen::Quaternion<rT>( qv[0], qv[1], qv[2], qv[3] );

        camProj.update_RCtRt();
        camProj.update_frustum_normals();

        camProjs.emplace_back( camProj );
    }

    return camProjs;
}


template < typename rT >
void read_cam_proj_from_json( const std::string &fn,
                              std::vector<CameraProjection<rT>> &camProjs) {
    using JSON = nlohmann::json;

    std::shared_ptr<JSON> pJson = read_json( fn );

    const auto& jCamProjs = (*pJson)["camProjs"];
    const int N = jCamProjs.size();

    if ( 0 == N ) {
        BOOST_THROW_EXCEPTION( NoCamProjFound() << ExceptionInfoString("No camera projection objects found in the JSON file.") );
    }

    for ( int i = 0; i < N; ++i ) {
        CameraProjection<rT> camProj;
        const auto& jCamProj = jCamProjs[i];

        camProj.id     = jCamProj["id"];
        camProj.height = jCamProj["height"];
        camProj.width  = jCamProj["width"];

        convert_vector_2_eigen_mat3(   jCamProj["K"].get<  std::vector<rT> >(), camProj.K );
        convert_vector_2_eigen_mat3(   jCamProj["RC"].get< std::vector<rT> >(), camProj.RC );
        convert_vector_2_eigen_mat3(   jCamProj["R"].get<  std::vector<rT> >(), camProj.R );
        convert_vector_2_eigen_vector( jCamProj["T"].get<  std::vector<rT> >(), camProj.T );

        std::vector<rT> qv = jCamProj["Q"].get< std::vector<rT> >();

        camProj.Q = Eigen::Quaternion<rT>( qv[0], qv[1], qv[2], qv[3] );

        camProj.update_RCtRt();
        camProj.update_frustum_normals();

        camProjs.emplace_back( camProj );
    }
}

#endif //POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP
