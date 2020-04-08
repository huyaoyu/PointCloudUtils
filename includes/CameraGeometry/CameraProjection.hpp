//
// Created by yaoyu on 3/29/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP
#define POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP

#include <cmath>
#include <fstream>
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
      RC( Eigen::Matrix3<rT>::Identity() ),
      R( Eigen::Matrix3<rT>::Identity() ),
      T( Eigen::Vector3<rT>::Zero() ),
      Q( Eigen::Quaternion<rT>( 1.0, 0.0, 0.0, 0.0 ) ),
      height(3008), width(4112), id(-1) {}

    CameraProjection( const CameraProjection<rT>& other ) {
        this->K  = other.K;
        this->RC = other.RC;
        this->R  = other.R;
        this->T  = other.T;
        this->Q  = other.Q;

        this->height = other.height;
        this->width  = other.width;

        this->id = other.id;
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

        return *this;
    }

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

    void world_2_pixel( const Eigen::Vector3<rT>& wp,
                       Eigen::Vector3<rT>& pixel ) const;

    void pixel_2_sensor( const Eigen::Vector3<rT> &pixelWithDepth,
            Eigen::Vector3<rT> &sp ) const;

    void pixel_2_camera( const Eigen::Vector3<rT>& pixelWithDepth,
                         Eigen::Vector3<rT>& cp ) const;

    void camera_2_world( const Eigen::Vector3<rT>& cp,
                        Eigen::Vector3<rT>& wp ) const;

    void pixel_2_world(const Eigen::Vector3<rT>& pixelWithDepth,
                       Eigen::Vector3<rT>& wp ) const;

    bool is_camera_point_in_image( const Eigen::Vector3<rT>& cp ) const;
    bool is_world_point_in_image( const Eigen::Vector3<rT>& wp ) const;

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
            << cp.Q.w() << ", " << cp.Q.x() << ", " << cp.Q.y() << ", " << cp.Q.z() << " ]" << std::endl;
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
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template < typename rT >
void CameraProjection<rT>::scale_intrinsics(rT s) {
    assert( s > 0 );

    height = static_cast<int>( s * height );
    width  = static_cast<int>( s * width );

    K *= s;
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
    cp = R.transpose() * ( wp - T);
}

template < typename rT >
void CameraProjection<rT>::camera_2_pixel(const Eigen::Vector3<rT> &cp, Eigen::Vector3<rT> &pixel) const {
    Eigen::Vector3<rT> sensorPoint = RC.transpose() * cp;

    sensor_2_pixel(sensorPoint, pixel);
}

template < typename rT >
void CameraProjection<rT>::sensor_2_pixel( const Eigen::Vector3<rT>& sp,
                     Eigen::Vector3<rT> &pixel ) const {
    if ( sp(2) <= 0 ) {
        std::stringstream ss;
        ss << "Point " << sp << " is located at the back of the camera. ";
        throw( std::runtime_error( ss.str() ) );
    }

    pixel = K * sp;

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
    wp = R * cp + T;
}

template < typename rT >
void CameraProjection<rT>::pixel_2_world(const Eigen::Vector3<rT> &pixelWithDepth, Eigen::Vector3<rT> &wp) const {
    Eigen::Vector3<rT> cp;
    pixel_2_camera(pixelWithDepth, cp);
    camera_2_world(cp, wp);
}

template < typename rT >
bool CameraProjection<rT>::is_camera_point_in_image(const Eigen::Vector3<rT> &cp) const {
    Eigen::Vector3<rT> sensorPoint = RC.transpose() * cp;

    if ( sensorPoint(2) <= 0 ) {
        return false;
    }

    Eigen::Vector3<rT> pixel;
    sensor_2_pixel(sensorPoint, pixel);

    if ( pixel(0) < 0 || pixel(0) >= width ) {
        return false;
    }

    if ( pixel(1) < 0 || pixel(1) >= height ) {
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

#endif //POINTCLOUDUTILS_INCLUDES_CAMERAGEOMETRY_CAMERAPROJECTION_HPP
