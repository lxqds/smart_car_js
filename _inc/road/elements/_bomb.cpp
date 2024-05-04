//
// Created by 一心向丞 on 2024/5/4.
//
#include "_bomb.hpp"

#include <algorithm>

#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"
#include "../utils/stop_watch.hpp"
#undef LOG_TAG
#define LOG_TAG "Bomb"

flag_bomb_e flag_bomb = BOMB_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
#define IMG imgprocess

// 变量定义

/* ********************************************************************* */

void check_bomb()
{
    if (flag_bomb == BOMB_DETECTION)
    {
        IMG.element_identify = 1;
        flag_bomb = BOMB_FOUND;
        MAT_LOG("BOMB_FOUND");
    }

    if (flag_save == Right_DETECTION)
    {
        IMG.element_identify = 1;
        flag_bomb = Right_FOUND;
        MAT_LOG("BOMB_FOUND");
    }
}

/* ********************************************************************* */

void run_bomb()
{

}

/* ********************************************************************* */