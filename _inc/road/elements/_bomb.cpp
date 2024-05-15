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
POINT bomb_near_cone;  // 较近的锥桶
float bomb_route = 0;  // 炸弹区编码器积分
/* ********************************************************************* */

void check_bomb()
{
    if (!IMG.element_over &&
        !control_flag.contains(cross_element) &&    //当前没在炸弹中
        !control_flag.contains(cross_element) &&    //当前没在十字路口中
        !control_flag.contains(circle_element) &&   //当前没在环岛中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        !control_flag.contains(garage_element) &&      //当前没在入库中
        flag_bomb == BOMB_DETECTION                 //找到炸弹
        )
    {
        IMG.element_identify = 1;
        control_flag.push(bomb_element);
        bomb_route = IMG.encoder.route;
        flag_bomb = BOMB_FOUND;
        MAT_LOG("BOMB_FOUND");
    }


}

/* ********************************************************************* */

void run_bomb()
{
    if (control_flag.top() == bomb_element) {//当前为炸弹元素
        IMG.aim_speed = IMG._config->speed_base;
        if (flag_bomb == BOMB_FOUND)
        {
            bomb_near_cone = POINT(0, 0);
            IMG.edge_det.clear();
            IMG.edge_det = search_targets(IMG._det_result);
            bomb_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab);
            // 当车辆开始靠近第一个锥桶:
            if (bomb_near_cone.x > IMAGE_HEIGHT * 0.3)
            {
                bomb_route = IMG.encoder.route;
                flag_bomb = BOMB_FIRST_CONE;
                MAT_LOG("BOMB_FIRST_CONE");
            }
            else if (IMG.encoder.route - bomb_route > 40)
            {
                flag_bomb = BOMB_NONE;
                control_flag.pop();
                MAT_LOG("BOMB_ERROR1");
            }


        }
        if (flag_bomb == BOMB_FIRST_CONE)
        {
            IMG.flag_track = TRACK_RIGHT;
            //右边线预测的中线更靠近右边
            track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                (int)round(ANGLE_DIST / SAMPLE_DIST),
                PIXEL_PER_METER * (ROAD_WIDTH * 0.5) / 2);
            IMG.rptsc1_num = IMG.rpts1s_num;
            if (IMG.encoder.route - bomb_route > 20)  //通过一段距离后
            {
                //开始找第二个锥桶
                bomb_near_cone = POINT(0, 0);
                IMG.edge_det.clear();
                IMG.edge_det = search_targets(IMG._det_result);
                bomb_near_cone = search_right_near(IMG.edge_left, IMG.mat_lab);
                // 当车辆开始靠近第一个锥桶:
                if (bomb_near_cone.x > IMAGE_HEIGHT * 0.3)
                {
                    bomb_route = IMG.encoder.route;
                    flag_bomb = BOMB_SECOND_CONE;
                    MAT_LOG("BOMB_SECOND_CONE");
                }
                else if (IMG.encoder.route - bomb_route > 60)
                {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR2");
                }
            }


        }
        if (flag_bomb == BOMB_SECOND_CONE)
        {
            IMG.flag_track = TRACK_LEFT;
            track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
                (int)round(ANGLE_DIST / SAMPLE_DIST),
                PIXEL_PER_METER * (ROAD_WIDTH * 0.5) / 2);
            IMG.rptsc0_num = IMG.rpts0s_num;
            if (IMG.encoder.route - bomb_route > 20)  //通过一段距离后
            {
                bomb_route = IMG.encoder.route;
                flag_bomb = BOMB_BLOCK;
                MAT_LOG("BOMB_BLOCK");
            }
        }
        if (flag_bomb == BOMB_BLOCK)
        {
            IMG.flag_track = TRACK_RIGHT;
            //右边线预测的中线更靠近右边
            track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                (int)round(ANGLE_DIST / SAMPLE_DIST),
                PIXEL_PER_METER * (ROAD_WIDTH * 0.5) / 2);
            IMG.rptsc1_num = IMG.rpts1s_num;
            if (IMG.encoder.route - bomb_route > 30)  //通过一段距离后
            {
                bomb_route = IMG.encoder.route;
                control_flag.pop();//退出元素
                flag_bomb = BOMB_NONE;
                MAT_LOG("BOMB_NONE");
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
        IMG.calculate_angle();//计算角度
    }
}
/* ********************************************************************* */