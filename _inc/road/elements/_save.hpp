//
// Created by 一心向丞 on 2024/5/4.
//

#ifndef _SAVE_H
#define _SAVE_H

enum flag_save_e {
    SAVE_NONE = 0,
    Left_DETECTION,  // 检测到病人或伤员标志
    Left_FOUND,      // 发现病人或伤员
    Left_READY,      // 准备进入左边区域
    Left_RUN,        // 左边区域行驶
    Left_OUT,        // 左边区域驶离
    Left_STOP,       // 左边区域内停止
    Left_OUT_STOP,   // 左边区域外停止
    Left_OUT_RUN,    // 左边区域外行驶
    Left_OUT_SPEEDUP,// 左边区域外加速

    Right_DETECTION,  //检测到贼或恶人标志
    Right_FOUND,      // 发现病人或伤员
    Right_READY,      // 准备进入右边区域
    Right_RUN,        // 右边区域行驶
    Right_OUT,        // 右边区域驶离
    Right_STOP,       // 右边区域内停止
    Right_OUT_STOP,   // 右边区域外停止
    Right_OUT_RUN,    // 右边区域外行驶
    Right_OUT_SPEEDUP,// 右边区域外加速

};
extern flag_save_e flag_save;

void check_save();
void run_save();

#endif //_SAVE_H
