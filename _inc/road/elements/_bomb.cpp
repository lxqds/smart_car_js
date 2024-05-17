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

int first_cone = 0;        //第一个锥桶的位置
int first_cone_count = 0;    //第一个锥桶计数 
int second_cone_count = 0;   //第二个锥桶计数
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
            if (bomb_near_cone.x > IMAGE_HEIGHT * 0.1)
            {
                float rnear_cone_x;//透视后锥桶x
                float cone_line_min0 = 999.0;//左边线和锥桶最小距离
                float cone_line_min1 = 999.0;//右边线和锥桶最小距离
                map_perspective(bomb_near_cone.y, bomb_near_cone.x, &rnear_cone_x, 0);//锥桶透视变化
                for (int i = 0;i < IMG.rpts0s_num;i++)
                {
                    float dx = IMG.rpts0s[i][0] - rnear_cone_x;
                    float dy = IMG.rpts0s[i][1] - bomb_near_cone.x;
                    float length = sqrt(dx * dx + dy * dy);
                    if (length < cone_line_min0)
                    {
                        cone_line_min0 = length;
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
                    }
                }
                if (cone_line_min0 < cone_line_min1)
                    first_cone_count++;
                else
                    first_cone_count--;
                if (first_cone_count >= 3)
                {
                    first_cone = 0;
                }
                else if (first_cone_count <= -3)
                {
                    first_cone = 1;
                }
                bomb_route = IMG.encoder.route;
                if (bomb_near_cone.x > IMAGE_HEIGHT * 0.1)
                {
                    IMG._config->speed_bomb2;
                }

            }


            // 当车辆开始靠近第一个锥桶:
            if (bomb_near_cone.x > IMAGE_HEIGHT * 0.3 && (first_cone_count > 3 || first_cone_count < -3))
            {
                first_cone_count = 0;
                bomb_route = IMG.encoder.route;
                flag_bomb = BOMB_FIRST_CONE;
                MAT_LOG("BOMB_FIRST_CONE");
            }
            else if (IMG.encoder.route - bomb_route > 60)
            {
                first_cone_count = 0;
                flag_bomb = BOMB_NONE;
                control_flag.pop();
                MAT_LOG("BOMB_ERROR1");
            }
        }
        if (flag_bomb == BOMB_FIRST_CONE)
        {
            IMG.aim_speed = IMG._config->speed_bomb2;
            IMG.aim_distance_f = 0.3;
            IMG.aim_distance_n = 0.1;
            IMG.aim_angle_p *= 1.5;

            bomb_near_cone = POINT(0, 0);
            IMG.edge_det.clear();
            bomb_near_cone = AI_search_height_near(IMG._det_result);

            float rnear_cone_x;//透视后锥桶x
            float cone_line_min0 = 999.0;//左边线和锥桶最小距离
            float cone_line_min1 = 999.0;//右边线和锥桶最小距离
            map_perspective(bomb_near_cone.y, bomb_near_cone.x, &rnear_cone_x, 0);//锥桶透视变化
            for (int i = 0;i < IMG.rpts0s_num;i++)
            {
                float dx = IMG.rpts0s[i][0] - rnear_cone_x;
                float dy = IMG.rpts0s[i][1] - bomb_near_cone.x;
                float length = sqrt(dx * dx + dy * dy);
                if (length < cone_line_min0)
                {
                    cone_line_min0 = length;
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
                }
            }
            if (cone_line_min0 < cone_line_min1)
            {
                if (second_cone_count < 0)
                    second_cone_count = 0;
                second_cone_count++;
            }
            else
            {
                if (second_cone_count > 0)
                    second_cone_count = 0;
                second_cone_count--;
            }


            if (first_cone == 0)    //第一个锥桶在左边，循右线
            {
                IMG.flag_track = TRACK_RIGHT;
                //右边线预测中线
                track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.2 / 2);
                IMG.rptsc1_num = IMG.rpts1s_num;

                if (second_cone_count < -3 &&
                    IMG.encoder.route - bomb_route > 30 &&
                    bomb_near_cone.x > IMAGE_HEIGHT * 0.55)//最近的是右边锥桶
                {
                    bomb_route = IMG.encoder.route;
                    flag_bomb = BOMB_SECOND_CONE;
                    MAT_LOG("BOMB_SECOND_CONE");
                }
                else if (IMG.encoder.route - bomb_route > 120)
                {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR1");
                }

            }
            else
            {
                IMG.flag_track = TRACK_LEFT;
                //左边线预测中线
                track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.2 / 2);
                IMG.rptsc0_num = IMG.rpts0s_num;

                if (second_cone_count > 3 &&
                    IMG.encoder.route - bomb_route > 40 &&
                    bomb_near_cone.x > IMAGE_HEIGHT * 0.55)//最近的是左边锥桶
                {
                    bomb_route = IMG.encoder.route;
                    flag_bomb = BOMB_SECOND_CONE;
                    MAT_LOG("BOMB_SECOND_CONE");
                }
                else if (IMG.encoder.route - bomb_route > 120)
                {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR1");
                }
            }
        }
        if (flag_bomb == BOMB_SECOND_CONE)
        {
            IMG.aim_speed = IMG._config->speed_bomb2;
            IMG.aim_distance_f = 0.2;
            IMG.aim_distance_n = 0.1;
            IMG.aim_angle_p *= 1.5;

            bomb_near_cone = POINT(0, 0);
            IMG.edge_det.clear();
            bomb_near_cone = AI_search_height_near(IMG._det_result);

            float rnear_cone_x;//透视后锥桶x
            float cone_line_min0 = 999.0;//左边线和锥桶最小距离
            float cone_line_min1 = 999.0;//右边线和锥桶最小距离
            map_perspective(bomb_near_cone.y, bomb_near_cone.x, &rnear_cone_x, 0);//锥桶透视变化
            for (int i = 0;i < IMG.rpts0s_num;i++)
            {
                float dx = IMG.rpts0s[i][0] - rnear_cone_x;
                float dy = IMG.rpts0s[i][1] - bomb_near_cone.x;
                float length = sqrt(dx * dx + dy * dy);
                if (length < cone_line_min0)
                {
                    cone_line_min0 = length;
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
                }
            }

            if (first_cone == 1)    //第一个锥桶在右边，第二个锥桶在左边，寻右线
            {
                IMG.flag_track = TRACK_RIGHT;
                //右边线预测中线
                track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.15 / 2);
                IMG.rptsc1_num = IMG.rpts1s_num;

                if (IMG.encoder.route - bomb_route > 30 && bomb_block_y > IMAGE_HEIGHT * 0.5)//靠近黑块
                {
                    bomb_route = IMG.encoder.route;
                    flag_bomb = BOMB_BLOCK;
                    MAT_LOG("BOMB_BLOCK");
                }
                else if (IMG.encoder.route - bomb_route > 120)
                {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR2");
                }

            }
            else
            {
                IMG.flag_track = TRACK_LEFT;
                //左边线预测中线
                track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.15 / 2);
                IMG.rptsc0_num = IMG.rpts0s_num;

                if (IMG.encoder.route - bomb_route > 30 && bomb_block_y > IMAGE_HEIGHT * 0.6)//靠近黑块
                {
                    bomb_route = IMG.encoder.route;
                    flag_bomb = BOMB_BLOCK;
                    MAT_LOG("BOMB_BLOCK");
                }
                else if (IMG.encoder.route - bomb_route > 120)
                {
                    flag_bomb = BOMB_NONE;
                    control_flag.pop();
                    MAT_LOG("BOMB_ERROR2");
                }
            }
        }
        if (flag_bomb == BOMB_BLOCK)
        {
            IMG.aim_speed = IMG._config->speed_bomb2;
            IMG.aim_distance_f = 0.3;
            IMG.aim_distance_n = 0.1;
            IMG.aim_angle_p *= 1.5;

            if (first_cone == 0)    //第一个锥桶在右边，第二个锥桶在左边，寻右线
            {
                IMG.flag_track = TRACK_RIGHT;
                //右边线预测中线
                track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.2 / 2);
                IMG.rptsc1_num = IMG.rpts1s_num;

                if (IMG.encoder.route - bomb_route > 60)//靠近黑块
                {
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
                {
                    IMG.aim_speed = IMG._config->speed_bomb1;
                }


            }
            else
            {
                IMG.flag_track = TRACK_LEFT;
                //左边线预测中线
                track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * 0.2 / 2);
                IMG.rptsc0_num = IMG.rpts0s_num;

                if (IMG.encoder.route - bomb_route > 60)//靠近黑块
                {
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
                {
                    IMG.aim_speed = IMG._config->speed_bomb1;
                }
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