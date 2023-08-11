#ifndef _WORK_HPP_
#define _WORK_HPP_

enum flag_work_e
{
    WORK_NONE = 0,
    WORK_DETECTION, // 检测到标志
    WORK_FOUND,     // 准备进入车库
    WORK_READY,     // 正在进入车库
    WORK_RUN,       // 进车库
    WORK_OUT,       // 驶离
    WORK_STOP,      // 库内停止
    WORK_OUT_STOP,  // 库外停止
    WORK_OUT_RUN,   // 库外跑
    WORK_OUT_SPEEDUP,
};
extern flag_work_e flag_work;

void check_work();
void run_work();

#endif
