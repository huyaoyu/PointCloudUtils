//
// Created by yaoyu on 4/4/20.
//

#include <iostream>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include "CPP/Types.hpp"

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

template < typename Derived0, typename Derived1 >
static float inner_product( const Eigen::MatrixBase<Derived0> &m0,
        const Eigen::MatrixBase<Derived1> &m1 ) {
    const auto p = m0.transpose() * m1;

    return static_cast<float>( p(0, 0) );
}

template < typename rT >
static void same_objects( const Eigen::VectorX<rT> &v0, Eigen::VectorX<rT> &v1 ) {
    std::cout << "&v0 = " << &v0 << std::endl;
    std::cout << "&v1 = " << &v1 << std::endl;

    Eigen::MatrixX<rT> R = Eigen::MatrixX<rT>::Random(v0.rows(), v0.rows());
    Eigen::VectorX<rT> t = Eigen::VectorX<rT>::Random(v0.rows());

    v1 = R * v0 + t;
}

template < typename Derived >
static void slice_dims( const Eigen::MatrixBase<Derived> &m ) {
    std::cout << __FUNCTION__ << ": m.rows() = " << m.rows() << ", "
              << "m.cols() = " << m.cols() << ". " << std::endl;
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

    {
        // Test inner product.
        std::cout << std::endl << "Test inner product. " << std::endl;
        Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 1);
        Eigen::MatrixXf B(A);

        const auto p = A.transpose() * B;
        std::cout << "decltype(p) is " << type_name<decltype(p)>() << std::endl;
        std::cout << "p(0,0) = " << p(0, 0) << std::endl;

        const Eigen::MatrixXf C = A.transpose() * B;
        std::cout << "decltype(C) is " << type_name<decltype(C)>() << std::endl;
        std::cout << "C(0,0) = " << C(0, 0) << std::endl;

        Eigen::MatrixX3f D = Eigen::Matrix3f::Random();
        const Eigen::MatrixXf E = D.col(0).transpose() * D.col(0);
        std::cout << "E(0, 0) = " << E << std::endl;

        auto F = inner_product( D.col(0), D.col(0) );
        std::cout << "F = " << F << std::endl;
        F = inner_product( D.block(0,0,3,1), D.block(0,0,3,1) );
        std::cout << "F = " << F << std::endl;

        Eigen::Vector3f G = D.block(0,0,3,1);
        F = inner_product( G, D.block(0,0,3,1) );
        std::cout << "F = " << F << std::endl;

        // The following block will cause a runtime error.
        // Mixing double and float for multiplication is not allowed inside inner_product().
//        Eigen::Vector3d J = D.col(0); // This is not allowed.
//        std::cout << "J = " << J << std::endl;
//        F = inner_product(J, D.col(0));
//        std::cout << "F = " << F << std::endl;
        // This is also not allowed.
//        F = G.dot(J);

        Eigen::MatrixXf H = G.transpose() * D.block(0,0,3,1);
        std::cout << "H(0, 0) = " << H(0, 0) << std::endl;

        H = G.transpose() * D.col(0);
        std::cout << "H(0, 0) = " << H(0, 0) << std::endl;
    }

    {
        // Test aliasing.
        Eigen::MatrixXf matA(2,2);
        matA << 2, 0,  0, 2;
        matA.noalias() = matA * matA;
        std::cout << matA << std::endl;

        Eigen::VectorXf v0 = Eigen::VectorXf::Random(3);
        std::cout << "v0 = " << v0 << std::endl;

        same_objects(v0, v0);

        std::cout << "v0 = " << v0 << std::endl;
    }

    {
        // eval() test.
        Eigen::Matrix3f K;
        K << 4877.75, 0, 2117.07007, 0, 4877.75, 1507.23999, 0, 0, 1;
        Eigen::Vector3f sp;
        sp << -0.99140209, 0.218834817, 2.28696632;

        Eigen::Vector3f pixel = K * sp.eval();
        std::cout << "K = \n" << K << std::endl;
        std::cout << "sp = \n" << sp << std::endl;
        std::cout << "pixel = \n" << pixel << std::endl;
    }

    {
        // Test slicing.
        Eigen::MatrixXd M(10, 4);
        slice_dims(M( Eigen::all, Eigen::seq(0, 1) ));
    }

    return 0;
}