#include "_work.hpp"

#include <algorithm>

#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"
#include "../utils/stop_watch.hpp"
#undef LOG_TAG
#define LOG_TAG "Work"

flag_work_e flag_work = WORK_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
extern Control_Flag control_flag;
#define IMG imgprocess

// 变量定义
float work_route = 0;  // 施工区编码器积分
float work_route2 = 0; // 施工区编码器积分
POINT work_near_cone;  //色块 较近的锥桶
POINT work_near_AIcone;  //AI 较近的锥桶
StopWatch stop_clock;
StopWatch stop_clock_2;

bool direction_work = true;//true : 左工作区 false : 右工作区
/* ********************************************************************* */

void check_work()
{

    if (!IMG.element_over &&
        !control_flag.contains(circle_element) &&   //当前没在环岛中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        !control_flag.contains(garage_element) &&   //当前没在入库中
        !control_flag.contains(ramp_element) &&      //当前没在桥中
        !control_flag.contains(bomb_element))
        {
            for (auto r : IMG.detection_result)
            {
                if ((r.label == IMG.label_evil || r.label == IMG.label_thief) && r.score > 0.52 && r.y > IMAGE_HEIGHT * 0.05) {
                    IMG.element_identify = 1;
                    control_flag.push(work_element);
                    work_route = IMG.encoder.route;
                    direction_work = false;
                    IMG.flag_track = TRACK_RIGHT;
                    flag_work = WORK_FOUND_1;
                    MAT_LOG("WORK_FOUND_R");
                }
                else if ((r.label == IMG.label_patient || r.label == IMG.label_tumble) && r.score > 0.52 && r.y > IMAGE_HEIGHT * 0.05) {

                    IMG.element_identify = 1;
                    control_flag.push(work_element);
                    work_route = IMG.encoder.route;
                    direction_work = true;
                    IMG.flag_track = TRACK_LEFT;
                    flag_work = WORK_FOUND_1;
                    MAT_LOG("WORK_FOUND_L");
                }
            }
        }

}

/* ********************************************************************* */

void run_work()
{


    if (control_flag.top() == work_element) {
        IMG.get_edge(0);
        //工作区基准速度
        IMG.aim_speed = IMG._config->speed_work;

        work_near_cone = POINT(0, 0);
        work_near_AIcone = POINT(0, 0);
        IMG.edge_det.clear();
        if (flag_work == WORK_FOUND_1)
        {

            // 找锥桶
            IMG.edge_det = search_targets(IMG._det_result);

            // 进入方向
            if (true)
                work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
            else
                work_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备

          //  LOG_I << " work_near_cone :" << work_near_cone.x << " IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy :" << IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy << LOG_END;

            // 当车辆开始靠近锥桶: 准备进入 识别或色块 识别到的锥桶大于一段阈值
            if (work_near_cone.x > IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy)
            {
                flag_work = WORK_FOUND_2;
                work_route = IMG.encoder.route;
                MAT_LOG("WORK_FOUND_2");
            }

        }

        if (flag_work == WORK_FOUND_2)
        {
            // 找锥桶
            IMG.edge_det = search_targets(IMG._det_result);

            // 进入方向
            if (true)
                work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
            else
                work_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备

            if (work_near_cone.x < IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy && work_near_cone.x > IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_in && IMG.encoder.route - work_route > 10) {

                work_route = IMG.encoder.route;
                flag_work = WORK_READY;
                MAT_LOG("WORK_READY");
            }

        }

        if (flag_work == WORK_FOUND_2 || flag_work == WORK_FOUND_1) {

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

                            // 超过指定距离未进入绕行区, 元素行驶失败
            if ((IMG.encoder.route - work_route > IMG._config->work_error))
            {
                IMG.element_over = true;
                IMG.element_over_route = IMG.encoder.route;
                control_flag.pop();//移除
                flag_work = WORK_NONE;
                MAT_LOG("WORK_ERROR");
            }

        }


        if (flag_work == WORK_READY) {
            //若是大于设定停车阈值就直接停车

            //int16_t avg_y = search_mid_cone_getH(IMG.mat_lab);
            //IMG.aim_angle = 0;
            //LOG_I << "  ready_AVG_Y :" << avg_y << LOG_END;
            ////开始靠近锥桶
            //if (avg_y > IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_avg )
            //{
            //    work_route = IMG.encoder.route;
            //    flag_work = WORK_STOPPING;
            //    MAT_LOG("WORK_READY_ERROR");
            //}

            //开始调整进入姿态，若是没找到线旁的锥桶 且 小于设定调整路程则开始深入工作区
            float edgek = 0;
            float threshold_k = -1.40 ;
            // 进入方向
            if (direction_work) {
                work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
                if(IMG.edge_left.size()!=0)edgek = Calculate_K(IMG.edge_left);
            }
            else {
                work_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备
                if (IMG.edge_right.size() != 0)edgek = Calculate_K(IMG.edge_right);
                threshold_k = -threshold_k;
            }
            LOG_I << " \n edgek \n" << edgek << LOG_END;
            // 调整 进入角度
            IMG.aim_angle = IMG._config->work_angle_in_1;
            if (!direction_work) IMG.aim_angle = -IMG.aim_angle;

           // IMG.Attitude_Ring(IMG.encoder.yaw, 35, 0.8, 0.1);//(IMG.encoder.route - work_route > IMG._config->work_rou_ready ) 
            if (edgek < threshold_k  || (work_near_cone.x==0 && work_near_cone.y == 0 ))
            {

                work_route = IMG.encoder.route;
                flag_work = WORK_RUN;
                work_route2 = IMG.encoder.route;
                MAT_LOG("WORK_RUN");
            }

        }


        if (flag_work == WORK_RUN) {

            int16_t avg_y = search_mid_cone_getH(IMG.mat_lab);

            IMG.aim_angle = 0;
            LOG_I << "  get_AIorcolor_AVG_Y :" << avg_y << LOG_END;
            //开始靠近锥桶
            if (avg_y < IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_avg && IMG.encoder.route - work_route < IMG._config->work_rou_in)
            {

                IMG.aim_speed = (int16_t)(IMG._config->work_run_speed);

            }
            else {
                work_route = IMG.encoder.route;
                flag_work = WORK_STOPPING;
                MAT_LOG("WORK_STOPPING");
            }
        }

        if (flag_work == WORK_STOPPING)//刹车中
        {
            IMG.aim_speed = 0;
            if (IMG.encoder.speed < 0.1 && IMG.encoder.speed > -0.1)
            {
                flag_work = WORK_PARKING;
                MAT_LOG("WORK_STOPFINISH");//刹车完成
                stop_clock.tic();

                work_route2 = IMG.encoder.route - work_route2;//记录直线进入距离
            }
            MAT_LOG("WORK_STOPPING");
        }

        if (flag_work == WORK_PARKING)//泊车
        {
            IMG.aim_speed = 0;
            if (stop_clock.toc() > IMG._config->work_stop_time)
            {
                work_route = IMG.encoder.route;
                MAT_LOG("WORK_PARKINISH"); //泊车中
                flag_work = WORK_OUT;
            }
            else
            {
                MAT_LOG("WORK_PARKING");
            }
        }

        if (flag_work == WORK_OUT)
        {
            IMG.aim_speed = (int16_t)(IMG._config->work_recback_speed); //逐渐减速
            IMG.aim_speed = -IMG.aim_speed;//倒车         
            // POINT AI_cone_left = POINT AI_search_right_near(IMG._det_result, IMG.edge_left);
            //退出角度赋值
            if (work_route - IMG.encoder.route > work_route2 * IMG._config->work_recback_angle_route_k)//大于退出 某阶段时开始打角
            {
                IMG.aim_angle = IMG._config->work_recback_angle;

                if (!direction_work) IMG.aim_angle = -IMG.aim_angle;
            }
            //退出速度赋值
            if (work_route - IMG.encoder.route < work_route2 * IMG._config->work_recback_line_k)
            {

                IMG.aim_angle = 0;
                MAT_LOG("out");
            }

            if (IMG.rpts1s_num > 0.8 / SAMPLE_DIST && IMG.rpts0s_num > 0.8 / SAMPLE_DIST && work_route - IMG.encoder.route > work_route2)
            {
                control_flag.pop();//移除
                flag_work = WORK_NONE;
                MAT_LOG("WORK_HAVE_L");
            }

            if (work_route - IMG.encoder.route > IMG._config->work_recback_max_route)
            {
                control_flag.pop();//移除
                flag_work = WORK_NONE;
                MAT_LOG("WORK_MAX");
            }


        }

    }
}