//
// Created by yaoyu on 4/13/20.
//

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/math/constants/constants.hpp>
#include <boost/exception/all.hpp>
#include <boost/program_options.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>
#include <glog/logging.h>

#include "Args/Args.hpp"
#include "CameraGeometry/CameraProjection.hpp"
#include "CVCommon/All.hpp"
#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "DataInterfaces/JSONHelper/Reader.hpp"
#include "DataInterfaces/NumPy/IO.hpp"
#include "Exception/Common.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/common.hpp"
#include "PCCommon/IO.hpp"

// Namespaces.
namespace bpo = boost::program_options;
using JSON = nlohmann::json;

static const int FLAG_MVS_POINT=1;
static const int FLAG_LDR_POINT=2;

template < typename rT >
static std::shared_ptr< CameraProjection<rT> > json_2_camera_projection(
        const JSON &param ) {
    // Read all the parameters associated with an CameraProjection object.

    int id     = param["camProj"]["id"];
    int height = param["camProj"]["height"];
    int width  = param["camProj"]["width"];

    auto vK  = param["camProj"]["K"].get< std::vector<rT> >();
    auto vRC = param["camProj"]["RC"].get< std::vector<rT> >();
    auto vR  = param["camProj"]["R"].get< std::vector<rT> >();
    auto vT  = param["camProj"]["T"].get< std::vector<rT> >();
    auto vQ  = param["camProj"]["Q"].get< std::vector<rT> >();

    std::shared_ptr<CameraProjection<rT>> pCamProj =
            create_camera_projection( id, height, width,
                                      vK, vRC, vR, vT, vQ );

    // Scale.
    rT s = param["camScale"];

    pCamProj->scale_intrinsics(s);

    return pCamProj;
}

template < typename rT >
static void make_initial_depth_map(
        const CameraProjection<rT> &camProj,
        const Eigen::MatrixX<rT> &tableMVS,
        const Eigen::MatrixX<rT> &tableLiDAR,
        Eigen::MatrixX<rT> &img,
        Eigen::MatrixXi &flagMat ){
    assert( tableMVS.rows() > 0 );
    assert( tableLiDAR.rows() > 0 );

    // Create a blank image and a flag image.
    img.resize( camProj.height, camProj.width );
    flagMat.resize( camProj.height, camProj.width );

    // Clear the content of img and flagMat.
    img     = Eigen::MatrixXf::Zero( camProj.height, camProj.width );
    flagMat = Eigen::MatrixXi::Zero( camProj.height, camProj.width );

    for ( int i = 0; i < tableMVS.rows(); ++i ) {
        // Get the pixel coordinates.
        const int x = static_cast<int>( std::round( tableMVS(i, 0) ) );
        const int y = static_cast<int>( std::round( tableMVS(i, 1) ) );

        img(y, x) = tableMVS( i, 2 );
        flagMat(y, x) = FLAG_MVS_POINT;
    }

    for ( int i = 0; i < tableLiDAR.rows(); ++i ) {
        // Get the pixel coordinates.
        const int x = static_cast<int>( std::round( tableLiDAR(i, 0) ) );
        const int y = static_cast<int>( std::round( tableLiDAR(i, 1) ) );

        // Get the LiDAR depth value.
        rT v = tableLiDAR(i, 2);

        // Check overlapping.
        if ( 0 == img(y, x) ) {
            img( y, x )   = v;
            flagMat(y, x) = FLAG_LDR_POINT;
        } else {
            if ( v < img(y, x) ) {
                img(y, x) = v;
                flagMat(y, x) = FLAG_LDR_POINT;
            }
        }
    }
}

template < typename rT >
static void get_image_plane_bbox( const Eigen::MatrixX<rT> &img,
        int &x0, int &y0, int &x1, int &y1 ) {
    x0 = img.cols(); y0 = img.rows();
    x1 = 0; y1 = 0;

    for ( int i = 0; i < img.cols(); ++i ) {
        for ( int j = 0; j < img.rows(); ++j ) {
            if ( img( j, i ) > 0 ) {
                if ( i < x0 ) {
                    x0 = i;
                }

                if ( i > x1 ) {
                    x1 = i;
                }

                if ( j < y0 ) {
                    y0 = j;
                }

                if ( j > y1 ) {
                    y1 = j;
                }
            }
        }
    }
}

template < typename rT >
static void get_average_initial_guess(
        const Eigen::MatrixX<rT> &inDepthMap,
        const Eigen::MatrixXi &inFlagMap,
        Eigen::MatrixX<rT> &guess ) {
    const int rows = inDepthMap.rows();
    const int cols = inDepthMap.cols();

    // Allocate memory.
    guess.resize( rows, cols );

    // The average value.
    auto avg = static_cast<rT>(0);
    int count = 0;

    for ( int i = 0; i < cols; ++i ) {
        for ( int j = 0; j < rows; ++j ) {
            if ( FLAG_MVS_POINT == inFlagMap(j, i) ) {
                avg += inDepthMap(j, i);
                count++;
            }
        }
    }

    guess = Eigen::MatrixX<rT>::Constant( rows, cols, avg/count );

    std::cout << "average depth is " << avg/count << std::endl;
}

struct CF_MVS {
    CF_MVS( double d, double f ) : mvsD(d), factor(f) {}

    template < typename rT >
    bool operator () ( const rT* const depth, rT* residual ) const {
        residual[0] =  rT(factor) * ( rT(mvsD) - depth[0] );

        return true;
    }

private:
    const double mvsD;
    const double factor;
};

struct CF_LDR {
    CF_LDR( double d, double f ) : ldrD(d), factor(f) {}

    template < typename rT >
    bool operator() ( const rT* const depth, rT* residual ) const {
        residual[0] = rT(factor) * ( rT(ldrD) - depth[0] );

        return true;
    }

private:
    const double ldrD;
    const double factor;
};

struct CF_Prior {

    template < typename rT >
    bool operator () ( const rT* const neighborDepth, rT* residual ) const {
        residual[0] =
                ceres::abs( neighborDepth[3] - 2 * neighborDepth[0] + neighborDepth[1] ) +
                ceres::abs( neighborDepth[4] - 2 * neighborDepth[0] + neighborDepth[2] );

        return true;
    }

};

struct CF_Prior_5 {

    CF_Prior_5(double f) : factor(f) {}

    template < typename rT >
    bool operator () ( const rT* const n0,
            const rT* const n1,
            const rT* const n2,
            const rT* const n3,
            const rT* const n4,
            rT* residual ) const {
        residual[0] = rT(0.25*factor) * (
                ceres::abs( n3[0] - n0[0] ) +
                ceres::abs( n1[0] - n0[0] ) +
                ceres::abs( n4[0] - n0[0] ) +
                ceres::abs( n2[0] - n0[0] ) );

        residual[0] += rT(factor) * (
                ceres::abs( n3[0] - rT(2) * n0[0] + n1[0] ) +
                ceres::abs( n4[0] - rT(2) * n0[0] + n2[0] ) );

        return true;
    }

private:
    const double factor;
};

static void build_op_problem(
        const Eigen::MatrixXd &inDepthMap,
        const Eigen::MatrixXd &inGuessMap,
        const Eigen::MatrixXi &inFlagMap,
        ceres::Problem &problem,
        Eigen::MatrixXd &paddedSolution,
        double fMVS=1.0, double fLiDAR=1.0, double fPrior=1.0) {
    typedef double rT;

    // Check if inDepthMap is a row-major matrix.
    if ( inDepthMap.IsRowMajor ) {
        EXCEPTION_EIGEN_ROW_MAJOR(inDepthMap)
    }

    const int rows = inDepthMap.rows();
    const int cols = inDepthMap.cols();
    const int paddedRows = rows + 2;
    const int paddedCols = cols + 2;

    // Allocate solution.
    paddedSolution.resize( rows+2, cols+2 );
    paddedSolution.block(1,1,rows,cols) = inGuessMap;

    for ( int i = 0; i < cols; ++i ) {
        const int paddedI = i + 1;

        for ( int j = 0; j < rows; ++j ) {
            const int paddedJ = j + 1;

            std::vector<rT*> vDepth;

            // Center depth.
            vDepth.push_back( ( paddedSolution.data() + paddedI*paddedRows + paddedJ ) );

            // Neighbor x0.
            if ( 0 == i ) {
                paddedSolution(paddedJ,0) = inGuessMap(j,i);
            }

            vDepth.push_back( ( paddedSolution.data() + (paddedI-1)*paddedRows + paddedJ ) );

            // Neighbor y0.
            if ( 0 == j ) {
                paddedSolution(0,paddedI) = inGuessMap(j,i);
            }

            vDepth.push_back( ( paddedSolution.data() + paddedI*paddedRows + (paddedJ-1) ) );

            // Neighbor x1.
            if ( cols-1 == i ) {
                paddedSolution(paddedJ,paddedI+1) = inGuessMap(j,i);
            }

            vDepth.push_back( ( paddedSolution.data() + (paddedI+1)*paddedRows + paddedJ ) );

            // Neighbor y1.
            if ( rows-1 == j ) {
                paddedSolution(paddedJ+1, paddedI) = inGuessMap(j,i);
            }

            vDepth.push_back( ( paddedSolution.data() + paddedI*paddedRows + (paddedJ+1) ) );

            // Data term.
            if ( FLAG_MVS_POINT == inFlagMap(j,i) ) {
                ceres::CostFunction* cf =
                        new ceres::AutoDiffCostFunction<CF_MVS, 1, 1>( new CF_MVS( inDepthMap(j, i), fMVS ) );

                problem.AddResidualBlock( cf, nullptr, vDepth[0] );
            } else if ( FLAG_LDR_POINT == inFlagMap(j, i) ) {
                ceres::CostFunction* cf =
                        new ceres::AutoDiffCostFunction<CF_LDR, 1, 1>( new CF_LDR( inDepthMap(j, i), fLiDAR ) );
                problem.AddResidualBlock( cf, nullptr, vDepth[0] );
            }

            // prior term.
            {
                ceres::CostFunction* cf =
                        new ceres::AutoDiffCostFunction<CF_Prior_5, 1, 1, 1, 1, 1, 1>( new CF_Prior_5(fPrior) );
                problem.AddResidualBlock( cf, nullptr, vDepth );
            }
        }
    }
}

template < typename rT >
static void fill_depth_map(
        const Eigen::MatrixX<rT> &inDepthMap,
        const Eigen::MatrixX<rT> &initialGuess,
        const Eigen::MatrixXi &inFlagMap,
        Eigen::MatrixX<rT> &filledDepthMap,
        double fMVS=1.0, double fLiDAR=1.0, double fPrior=1.0) {

    std::cout << "Build the ceres problem. " << std::endl;

    // Convert data type.
    Eigen::MatrixXd inDepthD, initGuessD;
    inDepthD   = inDepthMap.template cast<double>();
    initGuessD = initialGuess.template cast<double>();

    // Test use.
    std::cout << "initGuessD.mean() = " << initGuessD.mean() << std::endl;

    Eigen::MatrixXd paddedFilledD;

    // Ceres problem.
    ceres::Problem problem;
    build_op_problem( inDepthD, initGuessD, inFlagMap, problem, paddedFilledD,
            fMVS, fLiDAR, fPrior );

    ceres::Solver::Options options;
    options.max_num_iterations = 10000;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 4;

    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    options.function_tolerance  = 1e-6;
    options.gradient_tolerance  = 1e-10;
    options.parameter_tolerance = 1e-8;

    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );

    // Remove padding.
    Eigen::MatrixXd filledD = paddedFilledD.block( 1, 1, inDepthMap.rows(), inDepthMap.cols() );

    // Convert back.
    filledDepthMap = filledD.cast<rT>();

    std::cout << summary.FullReport() << std::endl;
}

template < typename rT >
static void find_non_mvs_pixels(
        const Eigen::MatrixX<rT> &inMap,
        const Eigen::MatrixXi &inFlagMap,
        Eigen::MatrixX<rT> &nonMVSPixels ) {
    assert( !nonMVSPixels.IsRowMajor );

    std::vector<rT> vPixels;

    const int rows = inMap.rows();
    const int cols = inMap.cols();

    assert( rows == inFlagMap.rows() );
    assert( cols == inFlagMap.cols() );

    for ( int i = 0; i < cols; ++i ) {
        for ( int j = 0; j < rows; ++j ) {
            if ( FLAG_MVS_POINT == inFlagMap(j, i) ) {
                continue;
            }

            vPixels.push_back( static_cast<rT>(i) );
            vPixels.push_back( static_cast<rT>(j) );
            vPixels.push_back( inMap(j, i) );
        }
    }

    // Copy the data from the vPixels to nonMVSPixels.
    nonMVSPixels.resize( 3, vPixels.size()/3 );
    nonMVSPixels = Eigen::Map< Eigen::MatrixX<rT> >( vPixels.data(), 3, vPixels.size()/3 );
}

template < typename Derived0, typename Derived1, typename Derived2 >
static typename Derived0::Scalar angle_from_3_points(
        const Eigen::MatrixBase<Derived0> &p0,
        const Eigen::MatrixBase<Derived1> &p1,
        const Eigen::MatrixBase<Derived2> &p ) {
    assert( 1 == p0.cols() );
    assert( 1 == p1.cols() );
    assert( 1 == p.cols() );

    typedef typename Derived0::Scalar Real_t;

    Eigen::Vector2<Real_t> v0;
    v0 << p0(0,0) - p(0,0), p0(1, 0) - p(1, 0);
    Eigen::Vector2<Real_t> v1;
    v1 << p1(0, 0) - p(0, 0), p1(1, 0) - p(1, 0);

    const Real_t n0n1 = v0.norm() * v1.norm();
    if ( n0n1 < 1e-4 ) {
        return static_cast<Real_t>(0);
    }

    const Real_t a = std::acos( v0.dot(v1) / n0n1 );

    Eigen::Vector2<Real_t> j0;
    j0 << -v0(1,0), v0(0, 0);

    return a * std::copysign( static_cast<Real_t>(1.0), j0.dot(v1) );
}

template < typename Derived0, typename Derived1 >
static bool is_inside_polygon(
        const Eigen::MatrixBase<Derived0> &polygonPixels,
        const Eigen::MatrixBase<Derived1> &pixel ) {
    typedef typename Derived0::Scalar Real_t;

    const auto pi = boost::math::constants::pi<Real_t>();
    const auto pixel2 = pixel.block(0, 0, 2, 1);

    auto acc = static_cast<Real_t>(0);

    const int N = polygonPixels.cols();

    for ( int i = 0; i < N - 1; ++i ) {
        acc += angle_from_3_points(
                polygonPixels.block(0,   i, 2, 1),
                polygonPixels.block(0, i+1, 2, 1),
                pixel2 );
    }

    acc += angle_from_3_points(
            polygonPixels.block(0, N-1, 2, 1),
            polygonPixels.block(0,   0, 2, 1),
            pixel2 );

    return std::abs(acc) > 1.9 * pi;
}

template < typename rT >
static void find_in_polygon_pixels(
        const Eigen::MatrixX<rT> &pixels,
        const Eigen::MatrixX<rT> &polygonPixels,
        Eigen::MatrixX<rT> &insidePolygonPixels ) {
    const int rows = pixels.rows();
    assert( 2 == rows || 3 == rows );

    std::vector<rT> vInsidePolygonPixels;

    const int N = pixels.cols();

    if ( 3 == rows ) {
        for ( int i = 0; i < N; ++i ) {
            // Test if the current pixel is in side the polygon.
//            Eigen::MatrixX<rT> tempCol = pixels.col(i);
            if ( is_inside_polygon(polygonPixels, pixels.col(i)) ) {
                vInsidePolygonPixels.push_back( pixels(0, i) );
                vInsidePolygonPixels.push_back( pixels(1, i) );
                vInsidePolygonPixels.push_back( pixels(2, i) );
            }
        }
    } else {
        for ( int i = 0; i < N; ++i ) {
            // Test if the current pixel is in side the polygon.
//            Eigen::MatrixX<rT> tempCol = pixels.col(i);
            if ( is_inside_polygon(polygonPixels, pixels.col(i)) ) {
                vInsidePolygonPixels.push_back( pixels(0, i) );
                vInsidePolygonPixels.push_back( pixels(1, i) );
            }
        }
    }

    insidePolygonPixels = Eigen::Map< Eigen::MatrixX<rT> >(
            vInsidePolygonPixels.data(), rows, vInsidePolygonPixels.size()/rows );
}

template < typename rT >
static void collect_filled_points_3d(
        const Eigen::MatrixX<rT> &filled,
        const Eigen::MatrixXi &inFlagMap,
        int shiftX, int shiftY,
        const Eigen::MatrixX<rT> &boundaryPixels,
        const CameraProjection<rT> &camProj,
        Eigen::MatrixX<rT> &filled3D ) {
    // Find all the filled pixels that are not MVS pixels.
    Eigen::MatrixX<rT> nonMVSPixels;
    find_non_mvs_pixels( filled, inFlagMap, nonMVSPixels );

    // Shift the pixel coordinates according to shiftX and shiftY.
    Eigen::Vector3<rT> shift;
    shift << static_cast<rT>(shiftX), static_cast<rT>(shiftY), static_cast<rT>(0);
    nonMVSPixels.colwise() += shift;

    // Find the pixels in side the boundary polygon.
    Eigen::MatrixX<rT> insidePolygonPixels;
    find_in_polygon_pixels( nonMVSPixels, boundaryPixels, insidePolygonPixels );

    // Project the pixels back to the world frame.
    camProj.pixel_2_world( insidePolygonPixels, filled3D );
}

class Args
{
public:
    Args() = default;

    ~Args() = default;

    bool validate() {
        bool flag = true;

        if ( fMVS < 0 ) {
            flag = false;
        }

        if ( fLiDAR < 0 ) {
            flag = false;
        }

        if ( fPrior < 0 ) {
            flag = false;
        }

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_PARAM << ": " << args.inParam << std::endl;
        out << Args::AS_IN_MVS << ": " << args.inMVS << std::endl;
        out << Args::AS_IN_LIDAR << ": " << args.inLiDAR << std::endl;
        out << Args::AS_IN_BP << ": " << args.inBP << std::endl;
        out << Args::AS_OUT_DIR << ": " << args.outDir << std::endl;
        out << Args::AS_F_MVS << ": " << args.fMVS << std::endl;
        out << Args::AS_F_LIDAR << ": " << args.fLiDAR << std::endl;
        out << Args::AS_F_PRIOR << ": " << args.fPrior << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_PARAM; // AS stands for argument string
    static const std::string AS_IN_MVS;
    static const std::string AS_IN_LIDAR;
    static const std::string AS_IN_BP;
    static const std::string AS_OUT_DIR;
    static const std::string AS_F_MVS;
    static const std::string AS_F_LIDAR;
    static const std::string AS_F_PRIOR;

public:
    std::string inParam; // The input file of the parameters.
    std::string inMVS; // Pixel table of the MVS points.
    std::string inLiDAR; // Pixel table of the LiDAR points.
    std::string inBP; // The boundary pixels.
    std::string outDir; // The output directory.
    double fMVS; // Cost factor.
    double fLiDAR;
    double fPrior;
};

const std::string Args::AS_IN_PARAM = "inParam";
const std::string Args::AS_IN_MVS   = "inMVS";
const std::string Args::AS_IN_LIDAR = "inLiDAR";
const std::string Args::AS_IN_BP    = "inBP";
const std::string Args::AS_OUT_DIR  = "outDir";
const std::string Args::AS_F_MVS    = "f-mvs";
const std::string Args::AS_F_LIDAR  = "f-lidar";
const std::string Args::AS_F_PRIOR  = "f-prior";

static void parse_args(int argc, char* argv[], Args& args) {

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                (Args::AS_IN_PARAM.c_str(), bpo::value< std::string >(&args.inParam)->required(), "The input parameters. ")
                (Args::AS_IN_MVS.c_str(), bpo::value< std::string >(&args.inMVS)->required(), "Pixel table of the MVS points. ")
                (Args::AS_IN_LIDAR.c_str(), bpo::value< std::string >(&args.inLiDAR)->required(), "Pixel table of the LiDAR points. ")
                (Args::AS_IN_BP.c_str(), bpo::value< std::string >(&args.inBP)->required(), "The table of boundary pixels.")
                (Args::AS_OUT_DIR.c_str(), bpo::value< std::string >(&args.outDir)->required(), "The output file. ")
                (Args::AS_F_MVS.c_str(), bpo::value< double >(&args.fMVS)->default_value(1.0), "The cost factor for MVS point.")
                (Args::AS_F_LIDAR.c_str(), bpo::value< double >(&args.fLiDAR)->default_value(1.0), "The cost factor for LiDAR point.")
                (Args::AS_F_PRIOR.c_str(), bpo::value< double >(&args.fPrior)->default_value(1.0), "The cost factor for prior.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(Args::AS_IN_PARAM.c_str(), 1
        ).add(Args::AS_IN_MVS.c_str(), 1
        ).add(Args::AS_IN_LIDAR.c_str(), 1
        ).add(Args::AS_IN_BP.c_str(), 1
        ).add(Args::AS_OUT_DIR.c_str(), 1);

        bpo::variables_map optVM;
        bpo::store(bpo::command_line_parser(argc, argv).
                options(optDesc).positional(posOptDesc).run(), optVM);
        bpo::notify(optVM);
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        throw(e);
    }

    if ( !args.validate() ) {
        EXCEPTION_INVALID_ARGUMENTS(args)
    }
}

template < typename rT >
static void write_points_as_pcl( const std::string& fn,
        const Eigen::MatrixX<rT> &points ) {
    // Convert.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pPC =
            pcu::convert_eigen_matrix_2_pcl_xyz<pcl::PointXYZ, rT>(points);

    // Write.
    pcu::write_point_cloud<pcl::PointXYZ>(fn, pPC);
}

int main( int argc, char** argv ) {
    google::InitGoogleLogging(argv[0]);

    QUICK_TIME_START(te)

    std::cout << "Hello, Simple! " << std::endl;

    // Handle the command line.
    MAIN_COMMON_LINES(argc, argv, args)

    // Read the input parameters from the JSON file.
    std::shared_ptr<JSON> pParams = read_json( args.inParam );

    // Create the CameraProjection object.
    auto pCamProj = json_2_camera_projection<float>( *pParams );
    std::cout << "pCamProj = " << std::endl;
    std::cout << *pCamProj << std::endl;

    // Read the tables for MVS points, LiDAr points and boundary pixels.
    Eigen::MatrixXf tableMVS, tableLiDAR, tableBoundaryPixels;
    read_npy_2_eigen_matrix( args.inMVS, tableMVS );
    read_npy_2_eigen_matrix( args.inLiDAR, tableLiDAR );
    read_npy_2_eigen_matrix( args.inBP, tableBoundaryPixels );

    assert( 3 == tableMVS.cols() );
    assert( 3 == tableLiDAR.cols() );
    assert( 3 == tableBoundaryPixels.cols() );

    // Build the initial depth map as an Eigen matrix.
    Eigen::MatrixXf initialDepthMap;
    Eigen::MatrixXi flagMap;

    make_initial_depth_map( *pCamProj, tableMVS, tableLiDAR, initialDepthMap, flagMap );

    // Find the bounding box on the pixel plane.
    int x0, y0, x1, y1;
    get_image_plane_bbox( initialDepthMap, x0, y0, x1, y1 );

    std::cout << "[ x0, y0, x1, y1 ] = [ "
              << x0 << ", " << y0 << ", "
              << x1 << ", " << y1 << " ]. " << std::endl;

    // Crop the initial depth image.
    Eigen::MatrixXf croppedDepthMap = initialDepthMap.block( y0, x0, (y1-y0+1), (x1-x0+1) );
    Eigen::MatrixXi croppedFlagMap  = flagMap.block( y0, x0, (y1-y0+1), (x1-x0+1) );

    // Compute the average depth as the initial guess.
    Eigen::MatrixXf initialGuess;
    get_average_initial_guess( croppedDepthMap, croppedFlagMap, initialGuess );

    // Fill the missing depth.
    Eigen::MatrixXf croppedFilled;
    fill_depth_map( croppedDepthMap, initialGuess, croppedFlagMap, croppedFilled,
            args.fMVS, args.fLiDAR, args.fPrior );
    // Test only.
//    croppedFilled = croppedDepthMap;

    // Get the filled 3D points.
    Eigen::MatrixXf filled3D;
    Eigen::MatrixXf boundaryPixelMatrix = tableBoundaryPixels.transpose();
    collect_filled_points_3d( croppedFilled, croppedFlagMap, x0, y0,
            boundaryPixelMatrix, *pCamProj, filled3D );

    // Test output directory.
    test_directory(args.outDir);

    {
        // Write filled3D matrix as PCL point cloud.
        std::string outFn = args.outDir + "/Filled3D.ply";
        write_points_as_pcl( outFn, filled3D );
    }

    {
        // Write the cropped filled image to a NumPy file.
        std::string outFn = args.outDir + "/CroppedFilledImg.npy";
        write_eigen_matrix_2_npy(outFn, croppedFilled);
    }

    QUICK_TIME_END(te)

    std::cout << "Simple filling in " << te << " ms. " << std::endl;

    return 0;
}