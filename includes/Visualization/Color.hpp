//
// Created by yaoyu on 3/25/20.
//

#ifndef POINTCLOUDUTILS_COLOR_HPP
#define POINTCLOUDUTILS_COLOR_HPP

#include <iostream>
#include <vector>

namespace pcu
{

class CommonColor{
public:
    CommonColor(): pos(0) {}
    ~CommonColor() = default;

    void reset() {
        pos = 0;
    }

    std::uint32_t next() {
        auto c = colors[pos];

        pos++;

        if ( pos == colors.size() ) {
            pos = 0;
        }

        return c;
    }

public:
    static std::vector<std::uint32_t> colors;

protected:
    int pos;
};

std::vector<std::uint32_t> CommonColor::colors = {
        0xFF002685,
        0xFF449ADF,
        0xFF4DC7FD,
        0xFF4CDE77,
        0xFF5E53C7,
        0xFF7E77D2,
        0xFFCD1E10,
        0xFFFC007F,
        0xFFFE79D1,
        0xFF763931,
        0xFFF1AB00,
        0xFFFADF00,
        0xFF007E3A,
        0xFF64D13E
};


typedef std::uint8_t                 ColorByte_t;
typedef std::array< ColorByte_t, 4 > Color_t;

union RGBA {
    std::uint32_t bgra;
    struct {
        std::uint8_t b;
        std::uint8_t g;
        std::uint8_t r;
        std::uint8_t a;
    };
};

Color_t next_color(CommonColor& cc) {
    RGBA color;
    color.bgra = cc.next();

//    std::cout << "color = [ "
//              << static_cast<int>( color.b ) << ", "
//              << static_cast<int>( color.g ) << ", "
//              << static_cast<int>( color.r ) << ", "
//              << static_cast<int>( color.a ) << " ]\n";

    return { color.r, color.g, color.b, color.a };
}

}

#endif //POINTCLOUDUTILS_COLOR_HPP
