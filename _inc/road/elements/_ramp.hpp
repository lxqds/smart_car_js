#ifndef _RAMP_HPP_
#define _RAMP_HPP_

enum flag_ramp_e {
    RAMP_NONE = 0,   // 非坡道模式
    RAMP_DETECTION,  // 检测到坡道
    RAMP_UP,         // 上坡阶段
    RAMP_DOWN,       // 下坡阶段
};
extern flag_ramp_e flag_ramp;
extern int ramp_x = 0;           //横坐标
extern int ramp_y = 0;           //纵坐标
void check_ramp();
void run_ramp();

#endif
