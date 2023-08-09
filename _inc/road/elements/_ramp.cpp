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
        IMG.element_identify = 1;
        ramp_route = IMG.encoder.route;
        flag_ramp = RAMP_UP;
        MAT_LOG("BRIDGE_UP");
    }
}

/* ********************************************************************* */

void run_ramp() {
    if (flag_ramp == RAMP_UP && (IMG.encoder.route - ramp_route > IMG._config->ramp_up_rou)) {
        flag_ramp = RAMP_DOWN;
        MAT_LOG("BRIDGE_DOWN");
        ramp_route = IMG.encoder.route;
    }

    if (flag_ramp == RAMP_DOWN && (IMG.encoder.route - ramp_route > ramp_down_rou)) {
        IMG.element_over = true;
        IMG.element_over_route = IMG.encoder.route;
        flag_ramp = RAMP_NONE;
        MAT_LOG("BRIDGE_NONE");
    }
}

/* ********************************************************************* */