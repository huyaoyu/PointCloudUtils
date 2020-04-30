//
// Created by yaoyu on 4/28/20.
//

#include <cmath>
#include <iostream>

int main( int argc, char** argv ) {
    std::cout << "Hello, TryCMath! " << std::endl;

    std::cout << "ceil(10.0/3) = " << std::ceil( 10.0/3 ) << std::endl;
    std::cout << "ceil(-10.0/3) = " << std::ceil( -10.0/3 ) << std::endl;
    std::cout << "std::copysign( std::ceil( std::abs(-10.0) / 3 ), -10.0 ) = "
              << std::copysign( std::ceil( std::abs(-10.0) / 3 ), -10.0 )
              << std::endl;

    return 0;
}