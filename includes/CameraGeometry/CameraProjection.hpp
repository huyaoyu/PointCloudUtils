//
// Created by yaoyu on 3/29/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP
#define POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

template < typename rT >
class CameraProjection {
public:
    CameraProjection()
    : K( Eigen::Matrix3<rT>::Identity() ),
      R( Eigen::Matrix3<rT>::Identity() ),
      T( Eigen::Vector3<rT>::Zero() ),
      height(3008), width(4112) {}

    CameraProjection( const CameraProjection<rT>& other ) {
        this->K = other.K;
        this->R = other.R;
        this->T = other.T;

        this->height = other.height;
        this->width  = other.width;
    }

    ~CameraProjection() = default;

    CameraProjection<rT>& operator = ( const CameraProjection<rT>& other ) {
        if ( this == &other ) {
            return *this;
        }

        this->K = other.K;
        this->R = other.R;
        this->T = other.T;

        this->height = other.height;
        this->width  = other.width;

        return *this;
    }

    void world_2_camera( const Eigen::Vector3<rT>& wp,
                        Eigen::Vector3<rT>& cp );

    void camera_2_pixel( const Eigen::Vector3<rT>& cp,
                        Eigen::Vector3<rT>& pixel );

    void world_2_pixel( const Eigen::Vector3<rT>& wp,
                       Eigen::Vector3<rT>& pixel );

    void pixel_2_camera( const Eigen::Vector3<rT>& pixelWithDepth,
                         Eigen::Vector3<rT>& cp );

    void camera_2_world( const Eigen::Vector3<rT>& cp,
                        Eigen::Vector3<rT>& wp );

    void pixel_2_world(const Eigen::Vector3<rT>& pixelWithDepth,
                       Eigen::Vector3<rT>& wp );

    bool is_camera_point_in_image( const Eigen::Vector3<rT>& cp );
    bool is_world_point_in_image( const Eigen::Vector3<rT>& wp );

public:
    Eigen::Matrix3<rT> K;
    Eigen::Matrix3<rT> R;
    Eigen::Vector3<rT> T;

    int height;
    int width;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template < typename rT >
void CameraProjection<rT>::world_2_camera(const Eigen::Vector3<rT> &wp, Eigen::Vector3<rT> &cp) {
    cp = R.transpose() * ( wp - T);
}

template < typename rT >
void CameraProjection<rT>::camera_2_pixel(const Eigen::Vector3<rT> &cp, Eigen::Vector3<rT> &pixel) {
    if ( cp(2) <= 0 ) {
        std::stringstream ss;
        ss << "Point " << cp << " is located at the back of the camera. ";
        throw( std::runtime_error( ss.str() ) );
    }

    pixel = K * cp;

    pixel(0) = pixel(0).eval() / pixel(2).eval();
    pixel(1) = pixel(1).eval() / pixel(2).eval();
    pixel(2) = static_cast<rT>(1.0);
}

template < typename rT >
void CameraProjection<rT>::world_2_pixel(const Eigen::Vector3<rT> &wp, Eigen::Vector3<rT> &pixel) {
    Eigen::Vector3<rT> cp;
    world_2_camera(wp, cp);
    camera_2_pixel(cp, pixel);
}

template < typename rT >
void CameraProjection<rT>::pixel_2_camera(const Eigen::Vector3<rT> &pixelWithDepth, Eigen::Vector3<rT> &cp) {
    if ( pixelWithDepth(2) <= 0 ) {
        std::stringstream ss;
        ss << "Non-positive depth is not allowed. pixelWidthDepth = " << pixelWithDepth;
        throw( std::runtime_error( ss.str() ) );
    }

    cp(0) = ( pixelWithDepth(0) - K(0, 2) ) / K(0, 0) * pixelWithDepth(2);
    cp(1) = ( pixelWithDepth(1) - K(1, 2) ) / K(1, 1) * pixelWithDepth(2);
    cp(2) = pixelWithDepth(2);
}

template < typename rT >
void CameraProjection<rT>::camera_2_world(const Eigen::Vector3<rT> &cp, Eigen::Vector3<rT> &wp) {
    wp = R * cp + T;
}

template < typename rT >
void CameraProjection<rT>::pixel_2_world(const Eigen::Vector3<rT> &pixelWithDepth, Eigen::Vector3<rT> &wp) {
    Eigen::Vector3<rT> cp;
    pixel_2_camera(pixelWithDepth, cp);
    camera_2_world(cp, wp);
}

template < typename rT >
bool CameraProjection<rT>::is_camera_point_in_image(const Eigen::Vector3<rT> &cp) {
    if ( cp(2) <= 0 ) {
        return false;
    }

    Eigen::Vector3<rT> pixel;
    camera_2_pixel(cp, pixel);

    if ( pixel(0) < 0 || pixel(0) >= width ) {
        return false;
    }

    if ( pixel(1) < 0 || pixel(1) >= height ) {
        return false;
    }

    return true;
}

template < typename rT >
bool CameraProjection<rT>::is_world_point_in_image(const Eigen::Vector3<rT> &wp) {
    Eigen::Vector3<rT> cp;
    world_2_camera(wp, cp);

    return is_camera_point_in_image(cp);
}

// ========== Utility function. ==========
template < typename rT >
void convert_from_quaternion_translation( const Eigen::MatrixX<rT>& quat,
        const Eigen::MatrixX<rT>& pos,
        const Eigen::Matrix3<rT>& K,
        CameraProjection<rT>& cProj ) {
    cProj.K = K;

    Eigen::Vector4<rT> qv;
    qv << quat(0,0), quat(0, 1), quat(0, 2), quat(0, 3);
    qv.normalize();

    Eigen::Quaternion<rT> q(qv);

    cProj.R = q.toRotationMatrix();

    cProj.T << pos(0,0), pos(0,1), pos(0,2);
}

template < typename rT >
void convert_from_quaternion_translation_table( const Eigen::MatrixX<rT>& quat,
        const Eigen::MatrixX<rT>& pos,
        const Eigen::Matrix3<rT>& K,
        std::vector< CameraProjection<rT> >& cps ) {
    const int N = quat.rows();

    assert( N == pos.rows() );

    cps.resize( N );

    for ( int i = 0; i < N; ++i ) {
        Eigen::MatrixX<rT> quatEntry = quat(i, Eigen::all);
        Eigen::MatrixX<rT> posEntry  = pos(i, Eigen::all);

        convert_from_quaternion_translation( quatEntry, posEntry, K, cps[i] );
    }

}

#endif //POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP
