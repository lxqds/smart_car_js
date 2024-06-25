#include "_circle.hpp"

#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"

#undef LOG_TAG
#define LOG_TAG "Circle"

flag_circle_e flag_circle = CIRCLE_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
extern Control_Flag control_flag;
#define IMG imgprocess

// 变量定义
float circle_route = 0;  // 环岛编码器积分
int none_left_line = 0, none_right_line = 0;
int have_left_line = 0, have_right_line = 0;

/* ********************************************************************* */

void check_circle() {


    if (IMG._config->check_circle_ == 1 &&
        !IMG.element_over &&
        // !control_flag.contains(cross_element) &&    //当前没在十字路口中
        !control_flag.contains(circle_element) &&   //当前没在环岛中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        !control_flag.contains(bomb_element) &&   //当前没在炸弹中
        !control_flag.contains(garage_element) &&      //当前没在入库中
        !control_flag.contains(ramp_element)      //当前没在桥中
        )
    {
        //左圆环
        if (IMG.Lpt0_found &&   // 左角点找到
            !IMG.Lpt1_found &&  // 右角点未找到
            IMG.is_straight1 &&  // 右边长直道
            IMG.rpts0s_num < 0.8 / SAMPLE_DIST) {// 左线点比较少
            none_left_line = 0;
            have_left_line = 0;
            IMG.element_identify = 1;
            circle_route = IMG.encoder.route;
            if (control_flag.contains(cross_element)) {//如果在十字路口中则删除十字路口
                control_flag.remove(cross_element);
                flag_cross = CROSS_NONE;
            }

            control_flag.push(circle_element);
            flag_circle = CIRCLE_LEFT_BEGIN;
            MAT_LOG("CIRCLE_LEFT_BEGIN");
        }
        //右圆环
        else if (!IMG.Lpt0_found &&  // 左角点未找到
            IMG.Lpt1_found &&   // 右角点找到
            IMG.is_straight0 &&  // 左边长直道
            IMG.rpts1s_num < 0.8 / SAMPLE_DIST) {// 右线点比较少
            none_right_line = 0;
            have_right_line = 0;
            IMG.element_identify = 1;
            circle_route = IMG.encoder.route;
            if (control_flag.contains(cross_element)) {//如果在十字路口中则删除十字路口
                control_flag.remove(cross_element);
                flag_cross = CROSS_NONE;
            }
            control_flag.push(circle_element);
            flag_circle = CIRCLE_RIGHT_BEGIN;
            MAT_LOG("CIRCLE_RIGHT_BEGIN");
        }
    }
}

/* ********************************************************************* */

void run_circle() {
    if (control_flag.top() == circle_element) {//当前为圆环元素
        IMG.aim_speed = IMG._config->speed_circle;
        // 左环开始, 寻外直道右线
        if (flag_circle == CIRCLE_LEFT_BEGIN) {
            IMG.flag_track = TRACK_RIGHT;

            // 先丢左线后有线
            if (IMG.rpts0s_num < 0.2 / SAMPLE_DIST)
                none_left_line++;
            if (IMG.encoder.route - circle_route > 10 &&
                IMG.rpts0s_num > 0.8 / SAMPLE_DIST && none_left_line > 1) {
                have_left_line++;
                if (have_left_line > 1) {
                    none_left_line = 0;
                    have_left_line = 0;
                    circle_route = IMG.encoder.route;
                    flag_circle = CIRCLE_LEFT_IN;
                    MAT_LOG("CIRCLE_LEFT_IN");
                }
            }
            // 入环失败
            if (IMG.encoder.route - circle_route > 100 &&
                flag_circle != CIRCLE_LEFT_IN) {
                none_left_line = 0;
                have_left_line = 0;
                IMG.element_over = true;
                IMG.element_over_route = IMG.encoder.route;
                control_flag.pop();//移除环岛
                flag_circle = CIRCLE_NONE;
                MAT_LOG("CIRCLE_ERROR");
            }
        }
        // 入左环, 寻内圆左线
        if (flag_circle == CIRCLE_LEFT_IN) {
            IMG.flag_track = TRACK_LEFT;

            // 先丢右线后有线
            if (IMG.rpts1s_num < 0.2 / SAMPLE_DIST)
                none_right_line++;
            if (IMG.rpts1s_num > 0.8 / SAMPLE_DIST && none_right_line > 0) {
                if (IMG.encoder.route - circle_route > 60)
                    have_right_line++;
                if (have_right_line > 0) {
                    none_right_line = 0;
                    have_right_line = 0;
                    circle_route = IMG.encoder.route;
                    flag_circle = CIRCLE_LEFT_RUNNING;
                    MAT_LOG("CIRCLE_LEFT_RUNNING");
                }
            }
        }
        // 正常巡线, 寻外圆右线
        if (flag_circle == CIRCLE_LEFT_RUNNING) {
            IMG.flag_track = TRACK_RIGHT;

            // 外环存在拐点, 可再加拐点距离判据 (左L点)
            if (IMG.Lpt1_found && IMG.encoder.route - circle_route > 100) {
                flag_circle = CIRCLE_LEFT_OUT;
                MAT_LOG("CIRCLE_LEFT_OUT");
            }
        }
        // 出环, 寻内圆
        if (flag_circle == CIRCLE_LEFT_OUT) {
            IMG.flag_track = TRACK_LEFT;


            // 先丢右线后有线
            if (IMG.rpts1s_num < 0.2 / SAMPLE_DIST)
                none_right_line++;
            if (IMG.rpts1s_num > 1.0 / SAMPLE_DIST && none_right_line > 0) {
                if (have_right_line > 0 && IMG.encoder.route - circle_route > IMG._config->circle_out_rou) {
                    none_right_line = 0;
                    have_right_line = 0;
                    circle_route = IMG.encoder.route;
                    flag_circle = CIRCLE_OUT;
                    MAT_LOG("CIRCLE_OUT");
                }
                else {
                    have_right_line++;
                }
            }
        }

        /* ***************************************************************** */

        // 右环开始, 寻外直道左线
        if (flag_circle == CIRCLE_RIGHT_BEGIN) {
            IMG.flag_track = TRACK_LEFT;

            // 先丢右线后有线
            if (IMG.rpts1s_num < 0.2 / SAMPLE_DIST)
                none_right_line++;
            if (IMG.encoder.route - circle_route > 10 &&
                IMG.rpts1s_num > 0.8 / SAMPLE_DIST && none_right_line > 1) {
                have_right_line++;
                if (have_right_line > 1) {
                    none_right_line = 0;
                    have_right_line = 0;
                    circle_route = IMG.encoder.route;
                    flag_circle = CIRCLE_RIGHT_IN;
                    MAT_LOG("CIRCLE_RIGHT_IN");
                }
            }
            // 入环失败
            if (IMG.encoder.route - circle_route > 100 &&
                flag_circle != CIRCLE_RIGHT_IN) {
                none_right_line = 0;
                have_right_line = 0;
                IMG.element_over = true;
                IMG.element_over_route = IMG.encoder.route;
                control_flag.pop();//移除环岛
                flag_circle = CIRCLE_NONE;
                MAT_LOG("CIRCLE_ERROR");
            }
        }
        // 入右环, 寻内圆右线
        if (flag_circle == CIRCLE_RIGHT_IN) {
            IMG.flag_track = TRACK_RIGHT;

            // 先丢左线后有线
            if (IMG.rpts0s_num < 0.2 / SAMPLE_DIST)
                none_left_line++;
            if (IMG.rpts0s_num > 0.8 / SAMPLE_DIST && none_left_line > 0) {
                if (IMG.encoder.route - circle_route > 60)
                    have_left_line++;
                if (have_left_line > 0) {
                    none_left_line = 0;
                    have_left_line = 0;
                    circle_route = IMG.encoder.route;
                    flag_circle = CIRCLE_RIGHT_RUNNING;
                    MAT_LOG("CIRCLE_RIGHT_RUNNING");
                }
            }
        }
        // 正常巡线, 寻外圆左线
        if (flag_circle == CIRCLE_RIGHT_RUNNING) {
            IMG.flag_track = TRACK_LEFT;

            if (IMG.Lpt0_found && IMG.encoder.route - circle_route > 100) {
                flag_circle = CIRCLE_RIGHT_OUT;
                MAT_LOG("CIRCLE_RIGHT_OUT");
            }
        }
        // 出环, 寻内圆
        if (flag_circle == CIRCLE_RIGHT_OUT) {
            IMG.flag_track = TRACK_RIGHT;


            // 先丢左线后有线
            if (IMG.rpts0s_num < 0.2 / SAMPLE_DIST)
                none_left_line++;
            if (IMG.rpts0s_num > 1.0 / SAMPLE_DIST && none_left_line > 0) {
                if (have_left_line > 0 && IMG.encoder.route - circle_route > IMG._config->circle_out_rou) {
                    none_left_line = 0;
                    have_left_line = 0;
                    circle_route = IMG.encoder.route;
                    flag_circle = CIRCLE_OUT;
                    MAT_LOG("CIRCLE_OUT");
                }
                else {
                    have_left_line++;
                }
            }
        }


        if (flag_circle == CIRCLE_OUT)
        {
            if (IMG.is_straight0 || IMG.is_straight1 || IMG.encoder.route - circle_route > 100)
            {
                IMG.element_over = true;
                IMG.element_over_route = IMG.encoder.route;
                control_flag.pop();//移除环岛
                flag_circle = CIRCLE_NONE;
                MAT_LOG("CIRCLE_NONE");
            }
        }

        /* *** *** *** *** *** */
        // 正常巡线
        if (IMG.flag_track == TRACK_LEFT) {
            IMG.rpts = IMG.rptsc0;
            IMG.rpts_num = IMG.rptsc0_num;
        }
        else if (IMG.flag_track == TRACK_RIGHT) {
            IMG.rpts = IMG.rptsc1;
            IMG.rpts_num = IMG.rptsc1_num;
        }
        else {
            IMG.rpts = IMG.rptscs;
            IMG.rpts_num = IMG.rptscs_num;
        }
        IMG.calculate_angle();//计算角度
    }

}

/* ********************************************************************* */