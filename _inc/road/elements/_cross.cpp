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
    bool found_Lpt = (IMG.Lpt0_found && IMG.Lpt1_found) ||//两边角点都找到
        (!control_flag.contains(circle_element) && (//不在圆环中
            (IMG.Lpt0_found && IMG.ipts1_num < 40) ||//左边角点找到且右边丢线
            (IMG.Lpt1_found && IMG.ipts0_num < 40)));//右边角点找到且左边丢线


    if (!IMG.element_over &&
        !control_flag.contains(cross_element) &&    //当前没在十字路口中
        !control_flag.contains(bomb_element) &&     //当前没在炸弹中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        !control_flag.contains(garage_element) &&   //当前没在入库中
        !control_flag.contains(ramp_element) &&     //当前没在桥中
        found_Lpt) {

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
            //补黑线
            if (IMG.far_Lpt0_found) {
                if (IMG.Lpt0_found) {
                    cv::line(IMG.mat_bin, cv::Point(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                        cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                        cv::Scalar(0, 0, 0), 2);
                }
                else {
                    cv::line(IMG.mat_bin, cv::Point((int)(IMAGE_WIDTH * 0.2), IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                        cv::Scalar(0, 0, 0), 2);
                }
            }
            else {
                if (IMG.Lpt0_found) {
                    int dx = IMG.Lpt0_point[0] - IMG.ipts0[0][0];
                    int dy = IMG.Lpt0_point[1] - IMG.ipts0[0][1];
                    float end_y = 1;
                    float end_x = (end_y - IMG.ipts0[0][0]) / dy * dx;
                    cv::line(IMG.mat_bin, cv::Point(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 2);
                }
            }
            if (IMG.far_Lpt1_found) {
                if (IMG.Lpt1_found) {
                    cv::line(IMG.mat_bin, cv::Point(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                        cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                        cv::Scalar(0, 0, 0), 2);
                }
                else {
                    cv::line(IMG.mat_bin, cv::Point(IMAGE_WIDTH * 0.8, IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                        cv::Scalar(0, 0, 0), 2);
                }
            }
            else {
                if (IMG.Lpt1_found) {
                    int dx = IMG.Lpt1_point[0] - IMG.ipts1[0][0];
                    int dy = IMG.Lpt1_point[1] - IMG.ipts1[0][1];
                    float end_y = 1;
                    float end_x = (end_y - IMG.ipts1[0][0]) / dy * dx;
                    cv::line(IMG.mat_bin, cv::Point(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 2);
                }
            }
            int L_line_num = IMG.ipts0_num;
            int R_line_num = IMG.ipts1_num;

            IMG.get_edge();


            // 近角点过少, 进入远线控制
            if ((!IMG.Lpt0_found && !IMG.Lpt1_found) ||
                (L_line_num < 20 && R_line_num < 20)) {
                cross_route = IMG.encoder.route;
                flag_cross = CROSS_IN;
                MAT_LOG("CROSS_IN");
            }


        }

        /* ***************************************************************** */

        // 远线控制进十字
        else if (flag_cross == CROSS_IN) {
            //补黑线
            if (IMG.far_Lpt0_found) {

                if (IMG.Lpt0_found) {
                    double L = dis_point_point(POINT(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                        POINT(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]));
                    if (L > 10) {
                        cv::line(IMG.mat_bin, cv::Point(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                            cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                            cv::Scalar(0, 0, 0), 2);
                    }
                    else {
                        cv::line(IMG.mat_bin, cv::Point((int)(IMAGE_WIDTH * 0.2), IMAGE_HEIGHT - 1),
                            cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                            cv::Scalar(0, 0, 0), 2);
                    }

                }
                else {
                    cv::line(IMG.mat_bin, cv::Point((int)(IMAGE_WIDTH * 0.2), IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt0_point[0], IMG.far_Lpt0_point[1]),
                        cv::Scalar(0, 0, 0), 2);
                }
            }
            else {
                if (IMG.Lpt0_found) {
                    int dx = IMG.Lpt0_point[0] - IMG.ipts0[0][0];
                    int dy = IMG.Lpt0_point[1] - IMG.ipts0[0][1];
                    float end_y = 1;
                    float end_x = (end_y - IMG.ipts0[0][0]) / dy * dx;
                    cv::line(IMG.mat_bin, cv::Point(IMG.Lpt0_point[0], IMG.Lpt0_point[1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 2);
                }
            }
            if (IMG.far_Lpt1_found) {

                if (IMG.Lpt1_found) {
                    double L = dis_point_point(POINT(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                        POINT(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]));
                    if (L > 10) {
                        cv::line(IMG.mat_bin, cv::Point(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                            cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                            cv::Scalar(0, 0, 0), 2);
                    }
                    else {
                        cv::line(IMG.mat_bin, cv::Point(IMAGE_WIDTH * 0.8, IMAGE_HEIGHT - 1),
                            cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                            cv::Scalar(0, 0, 0), 2);
                    }

                }
                else {
                    cv::line(IMG.mat_bin, cv::Point(IMAGE_WIDTH * 0.8, IMAGE_HEIGHT - 1),
                        cv::Point(IMG.far_Lpt1_point[0], IMG.far_Lpt1_point[1]),
                        cv::Scalar(0, 0, 0), 2);
                }
            }
            else {
                if (IMG.Lpt1_found) {
                    int dx = IMG.Lpt1_point[0] - IMG.ipts1[0][0];
                    int dy = IMG.Lpt1_point[1] - IMG.ipts1[0][1];
                    float end_y = 1;
                    float end_x = (end_y - IMG.ipts1[0][0]) / dy * dx;
                    cv::line(IMG.mat_bin, cv::Point(IMG.Lpt1_point[0], IMG.Lpt1_point[1]),
                        cv::Point((int)end_x, (int)end_y),
                        cv::Scalar(0, 0, 0), 2);
                }
            }
            int L_line_num = IMG.ipts0_num;
            int R_line_num = IMG.ipts1_num;
            IMG.get_edge();
            // 出圈
            if ((L_line_num > 50 && R_line_num > 50 &&
                !IMG.Lpt0_found && !IMG.Lpt1_found)) {
                IMG.element_over = true;
                IMG.element_over_route = IMG.encoder.route;
                control_flag.pop();
                flag_cross = CROSS_NONE;
                MAT_LOG("CROSS_NONE");
            }
        }

        IMG.calculate_angle();//计算角度

    }


}

/* ********************************************************************* */


/* ********************************************************************* */