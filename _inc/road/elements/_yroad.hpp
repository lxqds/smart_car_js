#ifndef _YROAD_HPP_
#define _YROAD_HPP_

enum flag_yroad_e {
    YROAD_NONE = 0,
    YROAD_DETECTION,  // 检测到泛行区
    YROAD_FOUND_0,      // 找到同侧双角点
    YROAD_FOUND_1,      // 找到普通车辆
    YROAD_FOUND_2,      // 找到嫌疑车辆
    YROAD_FOUND_3,      // 找到危险车辆
    YROAD_OUT,        // 驶离泛行区
};
extern flag_yroad_e flag_yroad;

void check_yroad();
void run_yroad();

#endif
