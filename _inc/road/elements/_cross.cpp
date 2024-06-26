#include "_cross.hpp"

#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"

#undef LOG_TAG
#define LOG_TAG "Cross"

flag_cross_e flag_cross = CROSS_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
extern Control_Flag control_flag;
#define IMG imgprocess

// 变量定义
float cross_route = 0;  // 十字编码器积分
int not_have_line = 0;  // 十字丢线计次


/* ********************************************************************* */

void check_cross() {
    //是否满足角点判断条件
    bool found_Lpt = (IMG.Lpt0_found && IMG.Lpt1_found &&
        IMG.Lpt0_point[1] < 170 && IMG.Lpt1_point[1] < 170) ||//两边角点都找到
        (!control_flag.contains(circle_element) && (//不在圆环中
            (IMG.Lpt0_found && IMG.ipts0_num < 150 && IMG.ipts1_num < 40) ||//左边角点找到且右边丢线
            (IMG.Lpt1_found && IMG.ipts1_num < 150 && IMG.ipts0_num < 40)));//右边角点找到且左边丢线


    if (!IMG.element_over &&
        !control_flag.contains(cross_element) &&    //当前没在十字路口中
        !control_flag.contains(bomb_element) &&     //当前没在炸弹中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        !control_flag.contains(garage_element) &&   //当前没在入库中
        !control_flag.contains(ramp_element) &&     //当前没在桥中
        found_Lpt) {
        if (control_flag.contains(circle_element)) {//如果在十字路口中则删除十字路口
            control_flag.remove(circle_element);
            flag_circle = CIRCLE_NONE;
        }
        control_flag.push(cross_element);
        IMG.element_identify = 1;
        flag_cross = CROSS_BEGIN;
        MAT_LOG("CROSS_BEGIN");
    }
}

/* ********************************************************************* */

void run_cross() {

    if (control_flag.top() == cross_element) {// 当前为十字路口元素时
        IMG.aim_speed = IMG._config->speed_cross;
        // 检测到十字, 先按照近线走
        if (flag_cross == CROSS_BEGIN) {
            // 近角点过少, 进入远线控制
            if ((!IMG.Lpt0_found && !IMG.Lpt1_found) ||
                (IMG.ipts0_num < 20 && IMG.ipts1_num < 20)) {
                cross_route = IMG.encoder.route;
                flag_cross = CROSS_IN;
                MAT_LOG("CROSS_IN");
            }
            //补黑线
            if (IMG.far_Lpt0_found) {
                if (IMG.Lpt0_found) {
                    cv::line(IMG.mat_bin, cv::Point(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                        cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                        cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                }
                else {
                    cv::line(IMG.mat_bin, cv::Point((int)(IMAGE_WIDTH * 0.2), IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point((int)(IMAGE_WIDTH * 0.2), IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                }
            }
            else {
                if (IMG.Lpt0_found) {
                    float k = (IMG.Lpt0_point[1] - IMG.ipts0[0][1]) / (IMG.Lpt0_point[0] - IMG.ipts0[0][0]);
                    float end_y = 1;
                    float end_x = (end_y - IMG.ipts0[0][1]) / k + IMG.ipts0[0][0];
                    cv::line(IMG.mat_bin, cv::Point(IMG.ipts0[0][0], IMG.ipts0[0][1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point(IMG.ipts0[0][0], IMG.ipts0[0][1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 7);
                }
            }
            if (IMG.far_Lpt1_found) {
                if (IMG.Lpt1_found) {
                    cv::line(IMG.mat_bin, cv::Point(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                        cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                        cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                }
                else {
                    cv::line(IMG.mat_bin, cv::Point(IMAGE_WIDTH * 0.8, IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point(IMAGE_WIDTH * 0.8, IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                }
            }
            else {
                if (IMG.Lpt1_found) {
                    float k = (IMG.Lpt1_point[1] - IMG.ipts1[0][1]) / (IMG.Lpt1_point[0] - IMG.ipts1[0][0]);
                    float end_y = 1;
                    float end_x = (end_y - IMG.ipts1[0][1]) / k + IMG.ipts1[0][0];
                    cv::line(IMG.mat_bin, cv::Point(IMG.ipts1[0][0], IMG.ipts1[0][1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point(IMG.ipts1[0][0], IMG.ipts1[0][1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 7);
                }
            }

            IMG.get_edge(1);

        }

        /* ***************************************************************** */

        // 远线控制进十字
        else if (flag_cross == CROSS_IN) {
            // 出圈
            if ((IMG.ipts0_num > 50 && IMG.ipts1_num > 50 &&
                ((!IMG.Lpt0_found && !IMG.Lpt1_found) ||
                    (IMG.Lpt0_found && IMG.Lpt1_found &&
                        IMG.Lpt0_point[1] > 160 && IMG.Lpt1_point[1] > 160)))) {
                IMG.element_over = true;
                IMG.element_over_route = IMG.encoder.route;
                control_flag.pop();
                flag_cross = CROSS_NONE;
                MAT_LOG("CROSS_NONE");
            }
            //补黑线
            if (IMG.far_Lpt0_found) {
                if (IMG.Lpt0_found) {
                    double L = dis_point_point(POINT(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                        POINT(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]));
                    if (L > 10) {
                        cv::line(IMG.mat_bin, cv::Point(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                            cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                            cv::Scalar(0, 0, 0), 7);
                        cv::line(IMG.mat_result, cv::Point(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                            cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                            cv::Scalar(0, 0, 0), 7);
                    }
                    else {
                        cv::line(IMG.mat_bin, cv::Point((int)(IMAGE_WIDTH * 0.2), IMAGE_HEIGHT - 1),
                            cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                            cv::Scalar(0, 0, 0), 7);
                        cv::line(IMG.mat_result, cv::Point((int)(IMAGE_WIDTH * 0.2), IMAGE_HEIGHT - 1),
                            cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                            cv::Scalar(0, 0, 0), 7);
                    }

                }
                else {
                    cv::line(IMG.mat_bin, cv::Point((int)(IMAGE_WIDTH * 0.2), IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point((int)(IMAGE_WIDTH * 0.2), IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                }
            }
            else {
                if (IMG.Lpt0_found) {
                    float k = (IMG.Lpt0_point[1] - IMG.ipts0[0][1]) / (IMG.Lpt0_point[0] - IMG.ipts0[0][0]);
                    float end_y = 1;
                    float end_x = (end_y - IMG.ipts0[0][1]) / k + IMG.ipts0[0][0];
                    cv::line(IMG.mat_bin, cv::Point(IMG.ipts0[0][0], IMG.ipts0[0][1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point(IMG.ipts0[0][0], IMG.ipts0[0][1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 7);
                }
            }
            if (IMG.far_Lpt1_found) {

                if (IMG.Lpt1_found) {
                    double L = dis_point_point(POINT(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                        POINT(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]));
                    if (L > 10) {
                        cv::line(IMG.mat_bin, cv::Point(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                            cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                            cv::Scalar(0, 0, 0), 7);
                        cv::line(IMG.mat_result, cv::Point(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                            cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                            cv::Scalar(0, 0, 0), 7);
                    }
                    else {
                        cv::line(IMG.mat_bin, cv::Point(IMAGE_WIDTH * 0.8, IMAGE_HEIGHT - 1),
                            cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                            cv::Scalar(0, 0, 0), 7);
                        cv::line(IMG.mat_result, cv::Point(IMAGE_WIDTH * 0.8, IMAGE_HEIGHT - 1),
                            cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                            cv::Scalar(0, 0, 0), 7);
                    }

                }
                else {
                    cv::line(IMG.mat_bin, cv::Point(IMAGE_WIDTH * 0.8, IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point(IMAGE_WIDTH * 0.8, IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                        cv::Scalar(0, 0, 0), 7);
                }
            }
            else {
                if (IMG.Lpt1_found) {
                    float k = (IMG.Lpt1_point[1] - IMG.ipts1[0][1]) / (IMG.Lpt1_point[0] - IMG.ipts1[0][0]);
                    float end_y = 1;
                    float end_x = (end_y - IMG.ipts1[0][1]) / k + IMG.ipts1[0][0];
                    cv::line(IMG.mat_bin, cv::Point(IMG.ipts1[0][0], IMG.ipts1[0][1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 7);
                    cv::line(IMG.mat_result, cv::Point(IMG.ipts1[0][0], IMG.ipts1[0][1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 7);
                }
            }
            IMG.get_edge(1);
        }
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


/* ********************************************************************* */