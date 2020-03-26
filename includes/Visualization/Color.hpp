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

}

#endif //POINTCLOUDUTILS_COLOR_HPP
