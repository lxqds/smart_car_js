//
// Created by lxq on 2023-07-05.
//
#include "_slowzone.hpp"
#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"

#undef LOG_TAG
#define LOG_TAG "Slow"

flag_slow_e flag_slow = SLOW_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
#define IMG imgprocess

// 变量定义
float slow_route = 0;  // 慢行区编码器积分

/* ********************************************************************* */

void check_slow() {
    if (flag_slow == SLOW_DETECTION) {
        IMG.flag_track = TRACK_MIDDLE;
        IMG.element_identify = 1;
        slow_route = IMG.encoder.route;
        flag_slow = SLOW_DOWN;
        MAT_LOG("SLOW_DOWN");
    }
}

/* ********************************************************************* */


/* ********************************************************************* */

void run_slow() {
    if (flag_slow == SLOW_DOWN && (IMG.encoder.route - slow_route > 130)) {
        IMG.flag_track = TRACK_MIDDLE;
        flag_slow = SLOW_NONE;
        MAT_LOG("SLOW_NONE");
    }
}

/* ********************************************************************* */