//
// Created by 一心向丞 on 2024/5/4.
//
#include "_save.hpp"

#include <algorithm>

#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"
#include "../utils/stop_watch.hpp"
#undef LOG_TAG
#define LOG_TAG "Save"

flag_save_e flag_save = SAVE_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
#define IMG imgprocess

// 变量定义
float save_route = 0;  // 施工区编码器积分
float save_route2 = 0; // 施工区编码器积分
POINT save_near_cone;  // 较近的锥桶
StopWatch save_stop_clock;
StopWatch save_stop_clock_2;

/* ********************************************************************* */

void check_save()
{

}

/* ********************************************************************* */

void run_save()
{

}

/* ********************************************************************* */