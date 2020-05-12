//
// Created by yaoyu on 4/28/20.
//

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
#include "CVCommon/All.hpp"
#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "DataInterfaces/JSONHelper/Reader.hpp"
#include "DataInterfaces/NumPy/IO.hpp"
#include "Exception/Common.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/common.hpp"
#include "PCCommon/BBox.hpp"
#include "PCCommon/IO.hpp"
#include "PCCommon/Transform.hpp"

// Namespaces.
namespace bpo = boost::program_options;
using JSON = nlohmann::json;

static const int FLAG_MVS_POINT=1;
static const int FLAG_LDR_POINT=2;

/**
 * This function find the nearest multiples of \base around \target
 * such that the result is bigger than target in terms of
 * absolute value. The result will have the same sign of \target.
 * \base must be positive.
 *
 * @tparam bT Type of \base.
 * @tparam tT Type of \target.
 * @param base A positive number.
 * @param target The target number around which to find the multiples of \base.
 * @return The result multiples of \base.
 */
template < typename bT, typename tT>
static tT bigger_multiples(bT base, tT target) {
    assert(base > 0);

    return static_cast<tT>(
            std::copysign( std::ceil( 1.0 * std::abs(target) / base ) * base, target ) );
}

/**
 * Make an initial depth map on the x-y plane.
 *
 * This function assumes the z-axis is the depth direction.
 *
 * @tparam rT Floating number type.
 * @param dx The x-step for the discretized grid.
 * @param dy the y-step for the discretized grid.
 * @param xyLimits A 4-element vector. (x0, y0, x1, y1).
 * @param pointsMVS The MVS points. 3xN.
 * @param pointsLiDAR The LiDAR points. 3xN.
 * @param plane The xy plane to be created.
 * @param flagMat A matrix the same size with \plane storing the flag for types of points.
 * @param gridDef A 4-element vector stores the grid definition. (x0, y0, dx, dy).
 */
template < typename rT >
static void make_initial_depth_plane(
        rT dx, rT dy,
        const Eigen::Vector4<rT> &xyLimits,
        const Eigen::MatrixX<rT> &pointsMVS,
        const Eigen::MatrixX<rT> &pointsLiDAR,
        Eigen::MatrixX<rT> &plane,
        Eigen::MatrixXi &flagMat,
        Eigen::Vector4<rT> &gridDef ){
    assert( dx > 0 );
    assert( dy > 0 );
    assert( xyLimits(2) - xyLimits(0) > 2*dx );
    assert( xyLimits(3) - xyLimits(1) > 2*dy );

    // Figure out the span of the plane.
    const auto x0 = bigger_multiples<rT, rT>( dx, xyLimits(0) );
    const auto y0 = bigger_multiples<rT, rT>( dy, xyLimits(1) );
    const auto x1 = bigger_multiples<rT, rT>( dx, xyLimits(2) );
    const auto y1 = bigger_multiples<rT, rT>( dy, xyLimits(3) );
    const auto nx = static_cast<int>( std::round( ( x1 - x0 ) / dx ) );
    const auto ny = static_cast<int>( std::round( ( y1 - y0 ) / dy ) );

    // Save the grid definition.
    gridDef << x0, y0, dx, dy;

    // Create the blank plane and flag grids.
    plane.resize( ny, nx );
    flagMat.resize( ny, nx );
    Eigen::MatrixXi count = Eigen::MatrixXi::Zero(ny, nx);

    // Clear the content of plane and flagMat.
    plane   = Eigen::MatrixXf::Zero( ny, nx );
    flagMat = Eigen::MatrixXi::Zero( ny, nx );

    rT x, y, z;

    // MVS points.
    for ( int i = 0; i < pointsMVS.cols(); ++i ) {
        x = pointsMVS(0, i);
        y = pointsMVS(1, i);
        z = pointsMVS(2, i);

        if ( x < x0 || x > x1 || y < y0 || y > y1 ) {
            continue;
        }

        // Get the cell coordinates of the grid.
        const auto idxX = static_cast<int>( std::floor( ( x - x0 ) / dx ) );
        const auto idxY = static_cast<int>( std::floor( ( y - y0 ) / dy ) );

        plane(idxY, idxX)   = plane(idxY, idxX) + ( z - plane(idxY, idxX) ) / ( count(idxY, idxX) + 1 );
        flagMat(idxY, idxX) = FLAG_MVS_POINT;
        count(idxY, idxX)  += 1;
    }

    // LiDAR points.
    for ( int i = 0; i < pointsLiDAR.cols(); ++i ) {
        x = pointsLiDAR(0, i);
        y = pointsLiDAR(1, i);
        z = pointsLiDAR(2, i);

        if ( x < x0 || x > x1 || y < y0 || y > y1 ) {
            continue;
        }

        // Get the pixel coordinates.
        const auto idxX = static_cast<int>( std::floor( ( x - x0 ) / dx ) );
        const auto idxY = static_cast<int>( std::floor( ( y - y0 ) / dy ) );

        // Check overlapping with MVS points.
        if ( FLAG_MVS_POINT == flagMat(idxY, idxX) ) {
            continue;
        }

        plane(idxY, idxX)   = plane(idxY, idxX) + ( z - plane(idxY, idxX) ) / ( count(idxY, idxX) + 1 );
        flagMat(idxY, idxX) = FLAG_LDR_POINT;
        count(idxY, idxX)  += 1;
    }
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
static void find_non_mvs_cells(
        const Eigen::MatrixX<rT> &inMap,
        const Eigen::MatrixXi &inFlagMap,
        Eigen::MatrixX<rT> &nonMVSCells ) {
    assert( !nonMVSCells.IsRowMajor );

    std::vector<rT> vCells;

    const int rows = inMap.rows();
    const int cols = inMap.cols();

    assert( rows == inFlagMap.rows() );
    assert( cols == inFlagMap.cols() );

    for ( int i = 0; i < cols; ++i ) {
        for ( int j = 0; j < rows; ++j ) {
            if ( FLAG_MVS_POINT == inFlagMap(j, i) ) {
                continue;
            }

            vCells.push_back( static_cast<rT>(i) );
            vCells.push_back( static_cast<rT>(j) );
            vCells.push_back( inMap(j, i) );
        }
    }

    // Copy the data from the vCells to nonMVSCells.
    nonMVSCells.resize( 3, vCells.size()/3 );
    nonMVSCells = Eigen::Map< Eigen::MatrixX<rT> >( vCells.data(), 3, vCells.size()/3 );
}

template < typename rT >
static void convert_cells_2_points( const Eigen::MatrixX<rT> &cells,
        const Eigen::Vector4<rT> &gridDef,
        Eigen::MatrixX<rT> &points ) {
    assert( 3 == cells.rows() );

    points = cells;

    Eigen::Vector3<rT> v0, vD;
    v0 << gridDef(0) + gridDef(2)/2.0, gridDef(1) + gridDef(3)/2.0, 0;
    vD << gridDef(2), gridDef(3), 1;

    points.array().colwise() *= vD.array();
    points.colwise() += v0;
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
        const Eigen::MatrixBase<Derived0> &polygonPoints,
        const Eigen::MatrixBase<Derived1> &point ) {
    typedef typename Derived0::Scalar Real_t;

    const auto pi = boost::math::constants::pi<Real_t>();
    const auto point2 = point.block(0, 0, 2, 1);

    auto acc = static_cast<Real_t>(0);

    const int N = polygonPoints.cols();

    for ( int i = 0; i < N - 1; ++i ) {
        acc += angle_from_3_points(
                polygonPoints.block(0,   i, 2, 1),
                polygonPoints.block(0, i+1, 2, 1),
                point2 );
    }

    acc += angle_from_3_points(
            polygonPoints.block(0, N-1, 2, 1),
            polygonPoints.block(0,   0, 2, 1),
            point2 );

    return std::abs(acc) > 1.9 * pi;
}

template < typename rT >
static void find_in_polygon_points(
        const Eigen::MatrixX<rT> &points,
        const Eigen::MatrixX<rT> &polygonPoints,
        Eigen::MatrixX<rT> &insidePolygonPoints ) {
    const int rows = points.rows();
    assert( 2 == rows || 3 == rows );

    std::vector<rT> vInsidePolygonPoints;

    const int N = points.cols();

    if ( 3 == rows ) {
        for ( int i = 0; i < N; ++i ) {
            // Test if the current cell is in side the polygon.
            if ( is_inside_polygon(polygonPoints, points.col(i)) ) {
                vInsidePolygonPoints.push_back( points(0, i) );
                vInsidePolygonPoints.push_back( points(1, i) );
                vInsidePolygonPoints.push_back( points(2, i) );
            }
        }
    } else {
        for ( int i = 0; i < N; ++i ) {
            // Test if the current cell is in side the polygon.
            if ( is_inside_polygon(polygonPoints, points.col(i)) ) {
                vInsidePolygonPoints.push_back( points(0, i) );
                vInsidePolygonPoints.push_back( points(1, i) );
            }
        }
    }

    insidePolygonPoints = Eigen::Map< Eigen::MatrixX<rT> >(
            vInsidePolygonPoints.data(), rows, vInsidePolygonPoints.size()/rows );
}

template < typename rT >
static void collect_output_points(
        const Eigen::MatrixX<rT> &filled,
        const Eigen::MatrixXi &inFlagMap,
        const Eigen::MatrixX<rT> &boundaryPoints,
        const Eigen::Vector4<rT> &gridDef,
        Eigen::MatrixX<rT> &output,
        Eigen::MatrixX<rT> &nonMVSPoints) {
    // Find all the cells that are not MVS cells.
    Eigen::MatrixX<rT> nonMVSCells;
    find_non_mvs_cells( filled, inFlagMap, nonMVSCells );

    // Test use.
    std::cout << "nonMVSCells.cols() = " << nonMVSCells.cols() << std::endl;

    // Covert the cells to the points.
    convert_cells_2_points(nonMVSCells, gridDef, nonMVSPoints);;

    // Find the cells inside the boundary polygon.
    find_in_polygon_points( nonMVSPoints, boundaryPoints, output );

    // Test use.
    std::cout << "output.cols() = " << output.cols() << std::endl;
}

template < typename rT >
static void write_points_as_pcl( const std::string& fn,
                                 const Eigen::MatrixX<rT> &points ) {
    // Convert.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pPC =
            pcu::convert_eigen_matrix_2_pcl_xyz(points);

    // Write.
    pcu::write_point_cloud<pcl::PointXYZ>(fn, pPC);
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

        if ( cellDim[0] <= 0 || cellDim[1] <= 0 ) {
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
        out << Args::AS_CELL_DIM << ": [ " << args.cellDim[0] << ", " << args.cellDim[1] << " ]" << std::endl;

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
    static const std::string AS_CELL_DIM;

public:
    std::string inParam; // The input file of the parameters.
    std::string inMVS; // Pixel table of the MVS points.
    std::string inLiDAR; // Pixel table of the LiDAR points.
    std::string inBP; // The boundary pixels.
    std::string outDir; // The output directory.
    double fMVS; // Cost factor.
    double fLiDAR;
    double fPrior;
    std::vector<double> cellDim; // The x,y-length of the grid cell.
};

const std::string Args::AS_IN_PARAM = "inParam";
const std::string Args::AS_IN_MVS   = "inMVS";
const std::string Args::AS_IN_LIDAR = "inLiDAR";
const std::string Args::AS_IN_BP    = "inBP";
const std::string Args::AS_OUT_DIR  = "outDir";
const std::string Args::AS_F_MVS    = "f-mvs";
const std::string Args::AS_F_LIDAR  = "f-lidar";
const std::string Args::AS_F_PRIOR  = "f-prior";
const std::string Args::AS_CELL_DIM = "cell-dim";

static void parse_args(int argc, char* argv[], Args& args) {

    std::string cellDimString;

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
                (Args::AS_F_PRIOR.c_str(), bpo::value< double >(&args.fPrior)->default_value(1.0), "The cost factor for prior.")
                (Args::AS_CELL_DIM.c_str(), bpo::value< std::string >(&cellDimString)->default_value("0.1, 0.1"), "The size of one cell grid.");

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

    args.cellDim = extract_number_from_string<double>( cellDimString, 2 );

    if ( !args.validate() ) {
        EXCEPTION_INVALID_ARGUMENTS(args)
    }
}

int main( int argc, char** argv ) {
    google::InitGoogleLogging(argv[0]);

    QUICK_TIME_START(te)

    std::cout << "Hello, Simple! " << std::endl;

    // Handle the command line.
    MAIN_COMMON_LINES(argc, argv, args)

    // Read the input parameters from the JSON file.
    std::shared_ptr<JSON> pParams = read_json( args.inParam );

    typedef pcl::PointXYZ P_t;

    // Read the MVS points, LiDAr points and boundary points.
    pcl::PointCloud<P_t>::Ptr pMVS   = pcu::read_point_cloud<P_t>(args.inMVS);
    pcl::PointCloud<P_t>::Ptr pLiDAR = pcu::read_point_cloud<P_t>(args.inLiDAR);
    pcl::PointCloud<P_t>::Ptr pBP    = pcu::read_point_cloud<P_t>(args.inBP);

    // Compute the oriented bounding box.
    P_t bbMinPoint, bbMaxPoint, bbPosition;
    Eigen::Matrix3f bbRotMat;
    pcu::get_obb<P_t, float>( pMVS, bbMinPoint, bbMaxPoint, bbPosition, bbRotMat );

    // Test use.
    std::cout << "bbMinPoint = " << bbMinPoint << std::endl;
    std::cout << "bbMaxPoint = " << bbMaxPoint << std::endl;
    std::cout << "bbRotMat = " << std::endl << bbRotMat << std::endl;

    // Check if the bounding box's frame has its xy plane aligned with the point cloud.
    assert( bbMaxPoint.x - bbMinPoint.x > bbMaxPoint.z - bbMinPoint.z );
    assert( bbMaxPoint.y - bbMinPoint.y > bbMaxPoint.z - bbMinPoint.z );

    // Transform the point clouds to the frame of the bounding box.
    pcl::PointCloud<P_t>::Ptr pTransMVS =
            pcu::transform_point_cloud<P_t, float>( pMVS, bbPosition, bbRotMat );
    pcl::PointCloud<P_t>::Ptr pTransLiDAR =
            pcu::transform_point_cloud<P_t, float>( pLiDAR, bbPosition, bbRotMat );
    pcl::PointCloud<P_t>::Ptr pTransBP =
            pcu::transform_point_cloud<P_t, float>( pBP, bbPosition, bbRotMat );

    // Convert the point clouds to Eigen matrices.
    Eigen::MatrixXf pointsMVS, pointsLiDAR, pointsBP;
    pcu::convert_pcl_2_eigen_matrix<P_t, float>(pTransMVS, pointsMVS);
    pcu::convert_pcl_2_eigen_matrix<P_t, float>(pTransLiDAR, pointsLiDAR);
    pcu::convert_pcl_2_eigen_matrix<P_t, float>(pTransBP, pointsBP);

    // Build the initial depth map as an Eigen matrix.
    Eigen::MatrixXf initialDepthMap;
    Eigen::MatrixXi flagMap;

    // Use the bounding box. Fill in MVS and LiDAR points. Get the flags.
    Eigen::Vector4f xyLimits;
    xyLimits << bbMinPoint.x, bbMinPoint.y, bbMaxPoint.x, bbMaxPoint.y;

    // Test use.
    std::cout << "xyLimits = " << xyLimits << std::endl;

    Eigen::Vector4f gridDef;
    make_initial_depth_plane(
            static_cast<float>( args.cellDim[0] ), static_cast<float>( args.cellDim[1] ),
            xyLimits, pointsMVS, pointsLiDAR, initialDepthMap, flagMap, gridDef );

    // Test use.
    std::cout << "Initial map dimension: " << initialDepthMap.rows() << ", "
              << initialDepthMap.cols() << ". " << std::endl;

    // Use zero as the initial guess.
    Eigen::MatrixXf initialGuess = Eigen::MatrixXf::Zero(
            initialDepthMap.rows(), initialDepthMap.cols() );

    // Fill the missing depth.
    Eigen::MatrixXf filled;
    fill_depth_map( initialDepthMap, initialGuess, flagMap, filled,
            args.fMVS, args.fLiDAR, args.fPrior );

    // Get the points in the polygon.
    Eigen::MatrixXf inPolygon, nonMVSPoints;
    collect_output_points( filled, flagMap, pointsBP, gridDef, inPolygon, nonMVSPoints );

    // Convert back to PCL point clouds.
    pcl::PointCloud<P_t>::Ptr inPolygonPC =
            pcu::convert_eigen_matrix_2_pcl_xyz<P_t, float>(inPolygon);

    // Test use.
    pcl::PointCloud<P_t>::Ptr nonMVSPC =
            pcu::convert_eigen_matrix_2_pcl_xyz<P_t, float>(nonMVSPoints);

    // Transform the point clouds back to the original frame.
    Eigen::Matrix4f tsLocal2World =
            pcu::create_eigen_transform_matrix_0( bbPosition, bbRotMat );
    pcl::PointCloud<P_t>::Ptr filledPC =
            pcu::transform_point_cloud<P_t>( inPolygonPC, tsLocal2World );

    // Test use.
    pcl::PointCloud<P_t>::Ptr nonMVSPCWorld =
            pcu::transform_point_cloud<P_t>( nonMVSPC, tsLocal2World );

    // Test output directory.
    test_directory(args.outDir);

    {
        // Write filled3D matrix as PCL point cloud.
        std::string outFn = args.outDir + "/Filled3D.ply";
        pcu::write_point_cloud<P_t>( outFn, filledPC );

        // Test use.
        outFn = args.outDir + "/NonMVSPCWorld.ply";
        pcu::write_point_cloud<P_t>( outFn, nonMVSPCWorld );
    }

    QUICK_TIME_END(te)

    std::cout << "Simple filling in " << te << " ms. " << std::endl;

    return 0;
}