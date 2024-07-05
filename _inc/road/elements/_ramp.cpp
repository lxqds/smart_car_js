#include "_ramp.hpp"

#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"

#undef LOG_TAG
#define LOG_TAG "Ramp"

flag_ramp_e flag_ramp = RAMP_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
#define IMG imgprocess
extern Control_Flag control_flag;
// 变量定义
float ramp_route = 0;  // 坡道编码器积分
int ramp_x = 0;           //横坐标
int ramp_y = 0;           //纵坐标
float ramp_score;   //置信度
/* ********************************************************************* */

void check_ramp() {
    if (!IMG.element_over &&
        !control_flag.contains(bomb_element) &&    //当前没在炸弹中
        !control_flag.contains(cross_element) &&    //当前没在十字路口中
        !control_flag.contains(circle_element) &&   //当前没在环岛中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        !control_flag.contains(garage_element) &&     //当前没在入库中
        !control_flag.contains(ramp_element)      //当前没在桥中
        )
    {
        for (auto r : IMG.detection_result) {
            if (r.label == IMG.label_bridge && r.score > 0.6 && r.y > IMAGE_HEIGHT * 0.4) {
                IMG.element_identify = 1;
                control_flag.push(ramp_element);
                ramp_route = IMG.encoder.route;
                flag_ramp = RAMP_UP;
                MAT_LOG("BRIDGE_UP");
            }
        }

    }
}

/* ********************************************************************* */

void run_ramp() {
    if (control_flag.top() == ramp_element) {//当前为炸弹元素

        if (flag_ramp == RAMP_UP)
        {
            IMG.get_edge();
            IMG.flag_track = TRACK_MIDDLE;
            IMG.aim_speed = IMG._config->speed_ramp;
            IMG.aim_distance_f = 0.5;
            IMG.aim_distance_n = 0.2;
            if (IMG.encoder.route - ramp_route > IMG._config->ramp_rou)
            {
                control_flag.pop();
                flag_ramp = RAMP_NONE;
                MAT_LOG("RAMP_NONE");
            }
            IMG.rpts = IMG.rptscs;
            IMG.rpts_num = IMG.rptscs_num;
            IMG.calculate_angle();//计算角度
        }
    }
}

/* ********************************************************************* */