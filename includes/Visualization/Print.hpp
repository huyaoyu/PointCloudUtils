//
// Created by yaoyu on 4/20/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_VISUALIZATION_PRINT_HPP
#define POINTCLOUDUTILS_INCLUDES_VISUALIZATION_PRINT_HPP

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

static void print_bar( const std::string &s, int n=80,
        const std::string &d="=", const std::string &prefix="\n", const std::string &suffix="\n" ) {
    // ========== Prepare the bar. ==========
    int ns = s.size();
    std::stringstream bar;

    if ( ns > n - 4 ) {
        bar << d << d;
    } else {
        const int barLength = ( n - ns + 1 ) / 2; // Integer arithmetic.

        for ( int i = 0; i < barLength; ++i ) {
            bar << d;
        }
    }

    // ========== Print. ==========
    std::cout << prefix << bar.str() << " " << s << " " << bar.str() << suffix << "\n";
}

#endif //POINTCLOUDUTILS_INCLUDES_VISUALIZATION_PRINT_HPP
