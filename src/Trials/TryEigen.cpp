//
// Created by yaoyu on 4/4/20.
//

#include <iostream>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#define SHOW_MATRIX(x) \
    { \
        std::cout << #x << " = " << std::endl << x << std::endl; \
        std::cout << #x << ".IsRowMajor = " << x.IsRowMajor << std::endl; \
    }

#define SHOW_MATRIX_RAW_DATA(x) \
    { \
        for ( int i = 0; i < x.size(); ++i ) { \
            std::cout << #x << "[" << i << "] = " << *( x.data()+i ) << std::endl; \
        } \
    }

template < typename rT0, typename rT1 >
static void custom_dynamic_cast( const Eigen::MatrixX<rT0> &A ,
        Eigen::MatrixX<rT1> &B ) {

    B = A.template cast<rT1>();
}

template < typename Derived >
static void test_block_as_argument(
        const Eigen::MatrixBase<Derived> &m ) {
    SHOW_MATRIX(m)
    std::cout << "For derived type, must use typename Derived::Scalar to access the underlying scalar type." << std::endl;
}

int main( int argc, char* argv[] ) {
    std::cout << "Hello, TryEigen! " << std::endl;

    Eigen::Matrix4f transMat = Eigen::Matrix4f::Identity();

    SHOW_MATRIX(transMat)

    const auto pi = boost::math::constants::pi<float>();
    const auto angleAxis = Eigen::AngleAxisf( 0.5f * pi, Eigen::Vector3f::UnitZ() );
    Eigen::Quaternionf quat = Eigen::Quaternionf( angleAxis );
    Eigen::Matrix3f rotMat = quat.toRotationMatrix();

    SHOW_MATRIX(rotMat)

    Eigen::Vector4f translation({ 1.0f, 2.0f, 3.0f, 1.0f });

    SHOW_MATRIX(translation)

    transMat.block(0,0,3,3) = rotMat;
//    transMat( Eigen::seq(0,2), Eigen::seq(0,2) ) = rotMat; // Also works.
    transMat.col(3) = translation;

    SHOW_MATRIX(transMat)

    // Test the column broadcasting.
    Eigen::MatrixXf pointMat = Eigen::MatrixXf::Random(3, 5);
    SHOW_MATRIX(pointMat)

    Eigen::MatrixXf transPoint = pointMat.colwise() - Eigen::Vector3f::Ones();
    SHOW_MATRIX(transPoint)

    // Test the mapping mechanism.
    {
        std::vector<double> raw = { 0, 1, 2, 3, 4, 5 };
        // The following line trigger a copy. mapped is a new column-major matrix.
        Eigen::MatrixXd mapped = Eigen::Map< Eigen::MatrixXd >( raw.data(), 2, 3 );
        SHOW_MATRIX(mapped)

        mapped(0, 0) = 10.0; // This will not change the value stored in raw.
        SHOW_MATRIX(mapped)

        std::cout << "raw[0] = " << raw[0] << std::endl;

        // Use the Map object directly.
        Eigen::Map< Eigen::MatrixXd > mapped1( raw.data(), 2, 3 );
        SHOW_MATRIX(mapped1)

        // This will change the value in raw since mapped1 and raw are sharing the same memory.
        mapped1(0, 0) = 10.0;
        SHOW_MATRIX(mapped1)
        std::cout << "raw[0] = " << raw[0] << std::endl;
    }

    {
        // Test the row major memory layout.
        Eigen::MatrixXi A(2,3);
        A <<  0, 1, 2, 3, 4, 5;
        SHOW_MATRIX(A)
        SHOW_MATRIX_RAW_DATA(A)

        // Make copy of A as a new RowMajor matrix.
        Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> B = A;
        SHOW_MATRIX(B)
        SHOW_MATRIX_RAW_DATA(B)
    }

    {
        // Test assignment between different types of matrix.
        Eigen::MatrixXd A = Eigen::MatrixXd::Random(2,3);
        Eigen::MatrixXf B;
        B = A.cast<float>();

        SHOW_MATRIX(A)
        SHOW_MATRIX(B)

        Eigen::MatrixXf C;
        custom_dynamic_cast(A, C);
        SHOW_MATRIX(C)
    }

    {
        // Test row and column scalar arithmetics.
        std::cout << "Row and column scalar arithmetics." << std::endl;
        Eigen::MatrixXf A(2,3);
        A << 0, 1, 2, 3, 4, 5;

        Eigen::MatrixXf B(A);

        SHOW_MATRIX(A)
        SHOW_MATRIX(B)

//        A.row(0) = A.row(0) + 10.0f; // Not compile.
//        B.col(0) += 10.0f; // Not compile.

        Eigen::VectorXf a(2);
        a << 10.0f, 0.0f;
        A.colwise() += a;

        Eigen::VectorXf b(3); // Not compile with row-wise operation.
//        Eigen::MatrixXf b(1, 3); // Not compile with row-wise operation.
        b << 10.0f, 0.0f, 0.0f;
        B.rowwise() += b.transpose();

        SHOW_MATRIX(A)
        SHOW_MATRIX(B)

        Eigen::MatrixXf C(2, 3);
        C << 0, 1, 2, 3, 4, 5;
        SHOW_MATRIX(C)

        C.row(0).array() += 10;
        SHOW_MATRIX(C)

        C.row(1).array() *= 2;
        SHOW_MATRIX(C)
    }

    {
        // Test block as argument.
        std::cout << "Test block as argument. " << std::endl;
        Eigen::MatrixXf A(2,3);
        A << 0, 1, 2, 3, 4, 5;

        test_block_as_argument( A.block(0,0,2,2) );
    }

    return 0;
}