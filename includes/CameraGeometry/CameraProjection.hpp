//
// Created by yaoyu on 3/29/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP
#define POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP

#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Dense>

template < typename rT >
class CameraProjection {
public:
    CameraProjection()
    : K( Eigen::Matrix3<rT>::Identity() ),
      R( Eigen::Matrix3<rT>::Identity() ),
      T( Eigen::Vector3<rT>::Zero() ),
      height(3008), width(4112) {}

    ~CameraProjection() = default;

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

#endif //POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP
