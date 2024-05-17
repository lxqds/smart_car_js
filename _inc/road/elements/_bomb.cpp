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
int bomb_x = 0;           //横坐标
int bomb_y = 0;           //纵坐标
int bomb_block_x = 0;           //横坐标
int bomb_block_y = 0;           //纵坐标
/* ********************************************************************* */

void check_bomb()
{
    if (!IMG.element_over &&
        !control_flag.contains(cross_element) &&    //当前没在炸弹中
        !control_flag.contains(cross_element) &&    //当前没在十字路口中
        !control_flag.contains(circle_element) &&   //当前没在环岛中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        !control_flag.contains(garage_element) &&      //当前没在入库中
        flag_bomb == BOMB_DETECTION && bomb_y > IMAGE_HEIGHT * 0.15 //找到炸弹
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

        if (flag_bomb == BOMB_FOUND)
        {
            IMG.aim_speed = IMG._config->speed_bomb1;
            bomb_near_cone = POINT(0, 0);
            IMG.edge_det.clear();
            // IMG.edge_det = search_targets(IMG._det_result);
            bomb_near_cone = AI_search_height_near(IMG._det_result);
            // bomb_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab);
            LOG_I << "cone1 :" <<
                bomb_near_cone.x << LOG_END;
            // 当车辆开始靠近第一个锥桶:
            if (bomb_near_cone.x > IMAGE_HEIGHT * 0.1)
            {
                bomb_route = IMG.encoder.route;
                flag_bomb = BOMB_FIRST_CONE;
                MAT_LOG("BOMB_FIRST_CONE");
            }
            else if (IMG.encoder.route - bomb_route > 60)
            {
                flag_bomb = BOMB_NONE;
                control_flag.pop();
                MAT_LOG("BOMB_ERROR1");
            }
        }
        if (flag_bomb == BOMB_FIRST_CONE)
        {
            IMG.aim_speed = IMG._config->speed_bomb2;
            IMG.flag_track = TRACK_RIGHT;
            IMG.aim_distance_f = 0.5;
            IMG.aim_distance_n = 0.1;
            IMG.aim_angle_p *= 1.5;

            float rnear_cone_x;//透视后锥桶x
            int L_line_min_Number = 0;//左边线和锥桶最小距离的点序号
            int R_line_min_Number = 0;//右边线和锥桶最小距离的点序号
            float cone_line_min0 = 999.0;//左边线和锥桶最小距离
            float cone_line_min1 = 999.0;//右边线和锥桶最小距离


            bomb_near_cone = POINT(0, 0);
            IMG.edge_det.clear();
            bomb_near_cone = AI_search_height_near(IMG._det_result);

            map_perspective(bomb_near_cone.y, bomb_near_cone.x, &rnear_cone_x, 0);//锥桶透视变化
            for (int i = 0;i < IMG.rpts0s_num;i++)
            {
                float dx = IMG.rpts0s[i][0] - rnear_cone_x;
                float dy = IMG.rpts0s[i][1] - bomb_near_cone.x;
                float length = sqrt(dx * dx + dy * dy);
                if (length < cone_line_min0)
                {
                    cone_line_min0 = length;
                    L_line_min_Number = i;
                }
            }
            for (int i = 0;i < IMG.rpts1s_num;i++)
            {
                float dx = IMG.rpts1s[i][0] - rnear_cone_x;
                float dy = IMG.rpts1s[i][1] - bomb_near_cone.x;
                float length = sqrt(dx * dx + dy * dy);
                if (length < cone_line_min1)
                {
                    cone_line_min1 = length;
                    R_line_min_Number = i;
                }
            }
            if (cone_line_min0 < cone_line_min1)
            {
                IMG.flag_track = TRACK_LEFT;
                //靠近锥桶的线右移
                for (int i = 0;i < IMG.rpts0s_num;i++)
                {
                    while (true)
                    {
                        float dx = IMG.rpts0s[i][0] - IMG.rpts0s[L_line_min_Number][0];
                        float dy = IMG.rpts0s[i][1] - IMG.rpts0s[L_line_min_Number][1];
                        float length = sqrt(dx * dx + dy * dy * 2);
                        if ((length < cone_line_min0 * 1.2) && IMG.rpts0s[i][0] < IMAGE_WIDTH - 1)
                            IMG.rpts0s[i][0]++;
                        else
                            break;
                    }
                }
                //左边线预测中线
                track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.4 / 2);
                IMG.rptsc0_num = IMG.rpts0s_num;
            }
            else
            {
                IMG.flag_track = TRACK_RIGHT;
                //靠近锥桶的线左移
                for (int i = 0;i < IMG.rpts1s_num;i++)
                {
                    while (true)
                    {
                        float dx = IMG.rpts1s[i][0] - IMG.rpts1s[R_line_min_Number][0];
                        float dy = IMG.rpts1s[i][1] - IMG.rpts1s[R_line_min_Number][1];
                        float length = sqrt(dx * dx + dy * dy * 2);
                        if ((length < cone_line_min1 * 1.2) && IMG.rpts1s[i][0] > 1)
                            IMG.rpts1s[i][0]--;
                        else
                            break;
                    }
                }
                //右边线预测中线
                track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.4 / 2);
                IMG.rptsc1_num = IMG.rpts1s_num;
            }


            if (IMG.encoder.route - bomb_route > 80)  //通过一段距离后
            {
                MAT_LOG("bomb_block_y=%d", bomb_block_y);
                if (bomb_block_y > IMAGE_HEIGHT * 0.4)
                {
                    bomb_route = IMG.encoder.route;
                    flag_bomb = BOMB_BLOCK;
                    MAT_LOG("BOMB_BLOCK");
                }
                else if (IMG.encoder.route - bomb_route > 90)
                {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR3");
                }

            }
        }
        if (flag_bomb == BOMB_BLOCK)
        {
            IMG.aim_speed = IMG._config->speed_bomb2;
            IMG.flag_track = TRACK_RIGHT;
            IMG.aim_distance_f = 0.5;
            IMG.aim_distance_n = 0.2;
            // IMG.aim_angle_p *= 1.8;

            float rnear_block_x;//透视后黑块x
            int L_line_min_Number = 0;//左边线和锥桶最小距离的点序号
            int R_line_min_Number = 0;//右边线和锥桶最小距离的点序号
            float block_line_min0 = 999.0;//左边线和黑块最小距离
            float block_line_min1 = 999.0;//右边线和黑块最小距离


            bomb_near_cone = POINT(0, 0);
            IMG.edge_det.clear();
            bomb_near_cone = AI_search_height_near(IMG._det_result);

            map_perspective(bomb_near_cone.y, bomb_near_cone.x, &rnear_block_x, 0);//锥桶透视变化
            for (int i = 0;i < IMG.rpts0s_num;i++)
            {
                float dx = IMG.rpts0s[i][0] - rnear_block_x;
                float dy = IMG.rpts0s[i][1] - bomb_near_cone.x;
                float length = sqrt(dx * dx + dy * dy);
                if (length < block_line_min0)
                {
                    block_line_min0 = length;
                    L_line_min_Number = i;
                }
            }
            for (int i = 0;i < IMG.rpts1s_num;i++)
            {
                float dx = IMG.rpts1s[i][0] - rnear_block_x;
                float dy = IMG.rpts1s[i][1] - bomb_near_cone.x;
                float length = sqrt(dx * dx + dy * dy);
                if (length < block_line_min1)
                {
                    block_line_min1 = length;
                    R_line_min_Number = i;
                }
            }
            if (block_line_min0 < block_line_min1)
            {
                IMG.flag_track = TRACK_LEFT;
                //靠近锥桶的线右移
                for (int i = 0;i < IMG.rpts0s_num;i++)
                {
                    while (true)
                    {
                        float dx = IMG.rpts0s[i][0] - IMG.rpts0s[L_line_min_Number][0];
                        float dy = IMG.rpts0s[i][1] - IMG.rpts0s[L_line_min_Number][1];
                        float length = sqrt(dx * dx + dy * dy * 2);
                        if ((length < block_line_min0 * 1.8) && IMG.rpts0s[i][0] < IMAGE_WIDTH - 1)
                            IMG.rpts0s[i][0]++;
                        else
                            break;
                    }
                }
                //左边线预测中线
                track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.4 / 2);
                IMG.rptsc0_num = IMG.rpts0s_num;
            }
            else
            {
                IMG.flag_track = TRACK_RIGHT;
                //靠近黑块的线左移
                for (int i = 0;i < IMG.rpts1s_num;i++)
                {
                    while (true)
                    {
                        float dx = IMG.rpts1s[i][0] - IMG.rpts1s[R_line_min_Number][0];
                        float dy = IMG.rpts1s[i][1] - IMG.rpts1s[R_line_min_Number][1];
                        float length = sqrt(dx * dx + dy * dy * 2);
                        if ((length < block_line_min1 * 1.8) && IMG.rpts1s[i][0] > 1)
                            IMG.rpts1s[i][0]--;
                        else
                            break;
                    }
                }
                //右边线预测中线
                track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.4 / 2);
                IMG.rptsc1_num = IMG.rpts1s_num;
            }
            if (IMG.encoder.route - bomb_route > 60)  //通过一段距离后
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

        if (IMG.rpts_num > 0.3 / SAMPLE_DIST) {
            IMG.calculate_angle();//计算角度
        }
        else if (IMG.flag_track == TRACK_RIGHT)
        {
            IMG.aim_angle = 0;
        }
        else if (IMG.flag_track == TRACK_LEFT)
        {
            IMG.aim_angle = 0;
        }

    }
}
/* ********************************************************************* */