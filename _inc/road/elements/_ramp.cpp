#include "_ramp.hpp"

#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"

#undef LOG_TAG
#define LOG_TAG "Ramp"

flag_ramp_e flag_ramp = RAMP_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
#define IMG imgprocess

// 变量定义
float ramp_route = 0;  // 坡道编码器积分

/* ********************************************************************* */

void check_ramp() {
    if (flag_ramp == RAMP_DETECTION) {
        IMG.flag_track = TRACK_MIDDLE;
        IMG.element_identify = 1;
        IMG.element_over = false;
        ramp_route = IMG.encoder.route;
        flag_ramp = RAMP_UP;
        MAT_LOG("BRIDGE_UP");
    }
}

/* ********************************************************************* */

void run_ramp() {
    if (flag_ramp == RAMP_UP && (IMG.encoder.route - ramp_route > 130)) {
        flag_ramp = RAMP_DOWN;
        MAT_LOG("BRIDGE_DOWN");
        ramp_route = IMG.encoder.route;
    }

    if (flag_ramp == RAMP_DOWN && (IMG.encoder.route - ramp_route > 180)) {
        IMG.element_over = true;
        IMG.element_over_route = IMG.encoder.route;
        flag_ramp = RAMP_NONE;
        MAT_LOG("BRIDGE_NONE");
    }
}

/* ********************************************************************* */