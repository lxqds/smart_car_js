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
extern Control_Flag control_flag;
#define IMG imgprocess

// 变量定义
POINT bomb_near_cone_L;  // 较近的左锥桶
POINT bomb_near_cone_R;  // 较近的右锥桶
POINT bomb_near_cone;  // 较近的锥桶
float bomb_route = 0;  // 炸弹区编码器积分
int bomb_x = 0;           //横坐标
int bomb_y = 0;           //纵坐标
int bomb_block_x = 0;           //横坐标
int bomb_block_y = 0;           //纵坐标
int first_cone = 0;        //第一个锥桶的位置
int first_cone_count = 0;    //第一个锥桶计数 

/* ********************************************************************* */

void check_bomb()
{
    if (!IMG.element_over &&
        !control_flag.contains(bomb_element) &&    //当前没在炸弹中
        !control_flag.contains(cross_element) &&    //当前没在十字路口中
        !control_flag.contains(circle_element) &&   //当前没在环岛中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        !control_flag.contains(garage_element) &&      //当前没在入库中
        !control_flag.contains(ramp_element)
        )
    {
        for (auto r : IMG.detection_result)
        {
            if (r.label == IMG.label_bomb && r.score > 0.5 && r.y > IMAGE_HEIGHT * 0.1) {
                IMG.element_identify = 1;
                control_flag.push(bomb_element);
                bomb_route = IMG.encoder.route;
                flag_bomb = BOMB_FOUND;
                MAT_LOG("BOMB_FOUND");
                break;
            }
        }
    }


}

/* ********************************************************************* */

void run_bomb()
{
    if (control_flag.top() == bomb_element) {//当前为炸弹元素
        IMG.get_edge(0);
        if (flag_bomb == BOMB_FOUND)
        {
            IMG.aim_speed = IMG._config->speed_bomb1;
            bomb_near_cone = POINT(0, 0);
            bomb_near_cone_L = POINT(0, 0);
            bomb_near_cone_R = POINT(0, 0);
            bomb_near_cone_L = search_left_near_in(IMG.edge_left, IMG.mat_lab);
            bomb_near_cone_R = search_right_near_in(IMG.edge_right, IMG.mat_lab);
            if (bomb_near_cone_L.x != 0 && bomb_near_cone_R.x != 0) {
                if (bomb_near_cone_L.x > bomb_near_cone_R.x)
                    bomb_near_cone = bomb_near_cone_L;
                else
                    bomb_near_cone = bomb_near_cone_R;

                if (bomb_near_cone.x > IMAGE_HEIGHT * 0.4) {

                    if (bomb_near_cone_L.x > bomb_near_cone_R.x) {
                        if (first_cone_count < 0)first_cone_count = 0;
                        first_cone_count++;
                    }
                    else {
                        if (first_cone_count > 0)first_cone_count = 0;
                        first_cone_count--;
                    }
                }
                MAT_LOG("near_y L=%d R=%d ", bomb_near_cone_L.x, bomb_near_cone_R.x);
            }


            if (first_cone_count >= 2) first_cone = 0;
            else if (first_cone_count <= -2) first_cone = 1;
            if (bomb_near_cone.x > IMAGE_HEIGHT * 0.1) {
                bomb_route = IMG.encoder.route;
                if (bomb_near_cone.x > IMAGE_HEIGHT * 0.4)
                    IMG._config->speed_bomb2;
            }
            // 当车辆开始靠近第一个锥桶:
            if (bomb_near_cone.x > IMAGE_HEIGHT * IMG._config->bomb_first_cone_ready &&
                (first_cone_count > 3 || first_cone_count < -3)) {
                first_cone_count = 0;
                bomb_route = IMG.encoder.route;
                flag_bomb = BOMB_FIRST_CONE;
                MAT_LOG("BOMB_FIRST_CONE");
            }
            else if (IMG.encoder.route - bomb_route > 60) {
                first_cone_count = 0;
                flag_bomb = BOMB_NONE;
                control_flag.pop();
                MAT_LOG("BOMB_ERROR1");
            }
        }
        if (flag_bomb == BOMB_FIRST_CONE) {
            IMG.aim_speed = IMG._config->speed_bomb2;
            IMG.aim_distance_f = 0.3;
            IMG.aim_distance_n = 0.1;
            IMG.aim_angle_p *= 1.5;

            bomb_near_cone = POINT(0, 0);
            bomb_near_cone_L = POINT(0, 0);
            bomb_near_cone_R = POINT(0, 0);
            bomb_near_cone_L = search_left_near_in(IMG.edge_left, IMG.mat_lab);
            bomb_near_cone_R = search_right_near_in(IMG.edge_right, IMG.mat_lab);

            if (first_cone == 0) {    //第一个锥桶在左边，循右线
                IMG.flag_track = TRACK_RIGHT;
                bomb_near_cone = bomb_near_cone_R;
                //右边线预测中线
                track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * IMG._config->bomb_first_cone_run_wide / 2);

                IMG.rptsc1_num = IMG.rpts1s_num;
                MAT_LOG(" R_near_y=%d", bomb_near_cone.x);
                if (IMG.encoder.route - bomb_route > 30 &&
                    bomb_near_cone.x > IMAGE_HEIGHT * IMG._config->bomb_second_cone_ready) {
                    bomb_route = IMG.encoder.route;
                    bomb_block_x = 0;
                    bomb_block_y = 0;
                    flag_bomb = BOMB_SECOND_CONE;
                    MAT_LOG("BOMB_SECOND_CONE");
                }
                else if (IMG.encoder.route - bomb_route > 120) {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR3");
                }

            }
            else {
                IMG.flag_track = TRACK_LEFT;
                bomb_near_cone = bomb_near_cone_L;
                //左边线预测中线
                track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * IMG._config->bomb_first_cone_run_wide / 2);
                IMG.rptsc0_num = IMG.rpts0s_num;
                MAT_LOG(" L_near_y=%d", bomb_near_cone.x);
                if (IMG.encoder.route - bomb_route > 40 &&
                    bomb_near_cone.x > IMAGE_HEIGHT * IMG._config->bomb_second_cone_ready)//最近的是左边锥桶
                {
                    bomb_route = IMG.encoder.route;
                    bomb_block_x = 0;
                    bomb_block_y = 0;
                    flag_bomb = BOMB_SECOND_CONE;
                    MAT_LOG("BOMB_SECOND_CONE");
                }
                else if (IMG.encoder.route - bomb_route > 120)
                {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR3");
                }
            }
        }
        if (flag_bomb == BOMB_SECOND_CONE)
        {
            IMG.aim_speed = IMG._config->speed_bomb2;
            IMG.aim_distance_f = 0.2;
            IMG.aim_distance_n = 0.1;
            IMG.aim_angle_p *= 1.5;


            if (first_cone == 1) {    //第一个锥桶在右边，第二个锥桶在左边，寻右线

                IMG.flag_track = TRACK_RIGHT;
                //右边线预测中线
                track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * IMG._config->bomb_second_run_wide / 2);
                IMG.rptsc1_num = IMG.rpts1s_num;
            }
            else {
                IMG.flag_track = TRACK_LEFT;
                //左边线预测中线
                track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * IMG._config->bomb_second_run_wide / 2);
                IMG.rptsc0_num = IMG.rpts0s_num;
            }
            for (auto r : IMG.detection_result)
            {
                if (r.label == IMG.label_block) {
                    bomb_block_y = r.y;
                    bomb_block_x = r.x;
                    break;
                }
            }

            if (bomb_block_y > IMAGE_HEIGHT * 0.1)
                bomb_route = IMG.encoder.route;

            if (bomb_block_y > IMAGE_HEIGHT * IMG._config->bomb_block_ready) {//靠近黑块
                bomb_route = IMG.encoder.route;
                flag_bomb = BOMB_BLOCK;
                MAT_LOG("BOMB_BLOCK");
            }
            else if (IMG.encoder.route - bomb_route > 40) {
                flag_bomb = BOMB_NONE;
                control_flag.pop();
                MAT_LOG("BOMB_ERROR3");
            }
        }
        if (flag_bomb == BOMB_BLOCK) {
            IMG.aim_speed = IMG._config->speed_bomb2;
            IMG.aim_distance_f = 0.3;
            IMG.aim_distance_n = 0.1;
            IMG.aim_angle_p *= 1.5;
            if (first_cone == 0) {    //第一个锥桶在右边，第二个锥桶在左边，寻右线
                IMG.flag_track = TRACK_RIGHT;
                //右边线预测中线
                track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * IMG._config->bomb_block_run_wide / 2);
                IMG.rptsc1_num = IMG.rpts1s_num;
                if (IMG.encoder.route - bomb_route > 60) {//靠近黑块
                    bomb_route = IMG.encoder.route;
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_NONE");
                }
                else if (IMG.encoder.route - bomb_route > 120) {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR2");
                }
                if (IMG.encoder.route - bomb_route > 30)
                    IMG.aim_speed = IMG._config->speed_bomb1;
            }
            else {
                IMG.flag_track = TRACK_LEFT;
                //左边线预测中线
                track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * IMG._config->bomb_block_run_wide / 2);
                IMG.rptsc0_num = IMG.rpts0s_num;

                if (IMG.encoder.route - bomb_route > 60) {
                    bomb_route = IMG.encoder.route;
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_NONE");
                }
                else if (IMG.encoder.route - bomb_route > 120)
                {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR2");
                }
                if (IMG.encoder.route - bomb_route > 30)
                    IMG.aim_speed = IMG._config->speed_bomb1;
            }
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

        if (IMG.rpts_num > 0.3 / SAMPLE_DIST) {
            IMG.calculate_angle();//计算角度
        }
        else if (IMG.flag_track == TRACK_RIGHT)
        {
            IMG.aim_angle = -45;
        }
        else if (IMG.flag_track == TRACK_LEFT)
        {
            IMG.aim_angle = 45;
        }

    }
}
/* ********************************************************************* */