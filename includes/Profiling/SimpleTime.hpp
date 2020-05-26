//
// Created by yaoyu on 3/16/20.
//

#ifndef POINTCLOUDUTILS_SIMPLETIME_HPP
#define POINTCLOUDUTILS_SIMPLETIME_HPP

#include <chrono>
#include <cstddef>

#define QUICK_TIME_START(t) \
    auto _stQuickTimeStart_##t = std::chrono::system_clock::now();

#define QUICK_TIME_END(t) \
    auto _stQuickTimeEnd_##t = std::chrono::system_clock::now();\
    auto _stQuickTimeElapsed_##t = std::chrono::duration_cast<std::chrono::milliseconds>( _stQuickTimeEnd_##t - _stQuickTimeStart_##t ); \
    std::size_t t = _stQuickTimeElapsed_##t.count();

#define QUICK_TIME_SHOW(t, msg) \
    QUICK_TIME_END(t) \
    {\
        std::cout << msg << " in " << t << " ms. \n"; \
    }

#endif //POINTCLOUDUTILS_SIMPLETIME_HPP
