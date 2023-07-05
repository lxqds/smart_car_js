//
// Created by lxq on 2023-07-05.
//

#ifndef __SLOWZONE_HPP
#define __SLOWZONE_HPP

#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"

enum flag_slow_e {
    SLOW_NONE = 0,   // 非慢行区模式
    SLOW_DETECTION,  // 检测到慢行区
    SLOW_DOWN,         // 慢行阶段
};
extern flag_slow_e flag_slow;

void check_slow();
void run_slow();

#endif //SMART_CAR_2023_02__SLOWZONE_H
