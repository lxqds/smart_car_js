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
POINT work_near_cone1;  //AI 较近的锥桶
StopWatch stop_clock;
StopWatch stop_clock_2;
CVFind_Clone workC;
bool direction_work = true;//true : 左工作区 false : 右工作区
/* ********************************************************************* */
int FOUND_last_y;
float last_data;
uint8_t out_flage;
uint8_t in_flag;
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
                if ((r.label == IMG.label_evil || r.label == IMG.label_thief) && r.score > 0.52 && r.y > IMAGE_HEIGHT * 0.1) {
                    IMG.element_identify = 1;
                    control_flag.push(work_element);
                    work_route = IMG.encoder.route;
                    direction_work = false;
                    IMG.flag_track = TRACK_RIGHT;
                    flag_work = WORK_FOUND_1;
                    MAT_LOG("WORK_FOUND_R");
                    FOUND_last_y = 0;
                    in_flag = 0;
                }
                else if ((r.label == IMG.label_patient || r.label == IMG.label_tumble) && r.score > 0.52 && r.y > IMAGE_HEIGHT * 0.1) {

                    IMG.element_identify = 1;
                    control_flag.push(work_element);
                    work_route = IMG.encoder.route;
                    direction_work = true;
                    IMG.flag_track = TRACK_LEFT;
                    flag_work = WORK_FOUND_1;
                    MAT_LOG("WORK_FOUND_L");
                    FOUND_last_y = 0;
                    in_flag = 0;
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
        IMG.edge_det.clear();
        if (flag_work == WORK_FOUND_1)
        {
            // 找锥桶
            IMG.edge_det = search_targets(IMG._det_result);
            // 进入方向
            if (direction_work)
                work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
            else
                work_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备

          //  LOG_I << " work_near_cone :" << work_near_cone.x << " IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy :" << IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy << LOG_END;

            // 当车辆开始靠近锥桶: 准备进入 识别或色块 识别到的锥桶大于一段阈值
            //if (work_near_cone.x > IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy)
            //{
            //    flag_work = WORK_FOUND_2;
            //    work_route = IMG.encoder.route;
            //    MAT_LOG("WORK_FOUND_2");
            //}
           
            //当 第一个锥桶消失
            if (in_flag == 0 && work_near_cone.x > 0.7 * IMAGE_HEIGHT) {
                in_flag = 1;
            }
            if (FOUND_last_y > work_near_cone.x && in_flag == 1) {
                    flag_work = WORK_FOUND_2;
                    work_route = IMG.encoder.route;
                    MAT_LOG("WORK_FOUND_2");
            }

            cv::line(IMG.mat_result, cv::Point(0, (int)work_near_cone.x),
                cv::Point(IMAGE_WIDTH - 1, (int)work_near_cone.x), cv::Scalar(255, 0, 0), 1);
            drawH(0.7 * IMAGE_HEIGHT, 255, 0, 0, IMG.mat_result);
            FOUND_last_y = work_near_cone.x;
        }

        if (flag_work == WORK_FOUND_2)
        {

            IMG.edge_det = search_targets(IMG._det_result);

            // 找锥桶
            if (direction_work)
                work_near_cone = search_left_near_up(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
            else
                work_near_cone = search_right_near_up(IMG.edge_right, IMG.mat_lab);//右工作区进入准备

            int thresholdh = IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_in;

            int coneLowy = IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_in + 15;

            int err_H = IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_in+7-work_near_cone.x ;

            IMG.aim_speed = err_H;

            float speedP = err_H / 30.0;
            if (speedP > 1)speedP = 1;
            if (speedP < -1)speedP = -1;
            
            IMG.aim_speed = IMG._config->speed_work * speedP;

            int speed_err = abs(IMG.aim_speed - IMG.encoder.speed*1000);
            LOG_I << "  speed_err:" << speed_err << LOG_END;
            if (work_near_cone.x < IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy && 
                (work_near_cone.x >= thresholdh && work_near_cone.x <= coneLowy)&&
                speed_err <200 ) {

                work_route = IMG.encoder.route;
                flag_work = WORK_READY;
                MAT_LOG("WORK_READY");
            }

                drawH(thresholdh, 0, 0,255, IMG.mat_result);
                drawH(work_near_cone.x, 255, 0, 0, IMG.mat_result);
                drawH(coneLowy, 0, 0, 255, IMG.mat_result);
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
            // 调整 进入角度
            IMG.aim_angle = IMG._config->work_angle_in_1;
            if (!direction_work) IMG.aim_angle = -IMG.aim_angle;
            //开始调整进入姿态，若是没找到线旁的锥桶 且 小于设定调整路程则开始深入工作区
            float edgek = 0;
            float threshold_k = -1.20 ;
            // 进入方向
            if (direction_work) {
                work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
                if(IMG.edge_left.size()!=0)edgek = Calculate_K(IMG.edge_left);
                threshold_k = -1;
            }
            else {
                work_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备
                if (IMG.edge_right.size() != 0)edgek = Calculate_K(IMG.edge_right);
                threshold_k = -threshold_k;
            }

           // IMG.Attitude_Ring(IMG.encoder.yaw, 35, 0.8, 0.1);//(IMG.encoder.route - work_route > IMG._config->work_rou_ready )

            if ((work_near_cone.x == 0 && work_near_cone.y == 0)||(direction_work && edgek > threshold_k) || (!direction_work && edgek < threshold_k) || edgek == 0) {
                work_route = IMG.encoder.route;
                flag_work = WORK_RUN;
                work_route2 = IMG.encoder.route;
                MAT_LOG("WORK_RUN");
                workC.init();
            }
            
        }


        if (flag_work == WORK_RUN) {
            cv::Rect miny;
            //int16_t avg_y = search_mid_cone_getH(IMG.mat_lab);
            // LOG_I << "  get_AIorcolor_AVG_Y :" << avg_y << LOG_END;

            std::vector<cv::Rect> all_zt;
            all_zt = workC.search_all_cone_rect(IMG.mat_lab);

            // 最高点锥桶
            if (all_zt.size() >= 2) miny = all_zt[0];
            POINT minyCent = getCenterPoint(miny);

            IMG.aim_angle = 0;
            int err_W, err_H;
            float speedP = 0;

            err_W = IMAGE_WIDTH / 2 - minyCent.x;
            err_H = (int)IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_avg - minyCent.y;
            
            IMG.aim_angle = err_W * IMG._config->work_run_angle_p;
            speedP = err_H / 40.0;
            if (speedP > 1)speedP = 1;
            if (speedP < -1)speedP = -1;



            cv::line(IMG.mat_result, cv::Point(0, (int)IMAGE_HEIGHT* IMG._config->work_read_cone_thresholdy_avg),
                cv::Point(IMAGE_WIDTH -1, (int)IMAGE_HEIGHT* IMG._config->work_read_cone_thresholdy_avg), cv::Scalar(0, 0, 255), 1);

            cv::line(IMG.mat_result, cv::Point(IMAGE_WIDTH / 2 + 30, 0),
                cv::Point(IMAGE_WIDTH / 2 + 30, IMAGE_HEIGHT), cv::Scalar(255, 0, 0), 1);

            cv::line(IMG.mat_result, cv::Point(IMAGE_WIDTH / 2 -30, 0),
                cv::Point(IMAGE_WIDTH / 2 - 30, IMAGE_HEIGHT), cv::Scalar(255, 0, 0), 1);

            cv::rectangle(IMG.mat_result, miny, cv::Scalar(0, 255, 255), 1);
            //开始靠近锥桶
            IMG.aim_speed = (int16_t)(IMG._config->work_run_speed * speedP);
            if(err_H < IMAGE_HEIGHT * 0.1 )
            {
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
                workC.init();
                out_flage = 0;
                last_data = 0;
            }
            else
            {
                MAT_LOG("WORK_PARKING");
            }
        }

        if (flag_work == WORK_OUT)
        {
            work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
            work_near_cone1 = search_right_near(IMG.edge_right, IMG.mat_lab);  //右工作区进入准备

            //float R0 = 10.0f, R1 = 10.0f;
            //// 左线直线拟合
            //if (IMG.ipts0_num > 10)
            //    R0 = fit_line1(IMG.ipts0, IMG.ipts0_num, 60);
            //// 右线直线拟合
            //if (IMG.ipts1_num > 10)
            //    R1 = fit_line1(IMG.ipts1, IMG.ipts1_num, 60);

            //找锥桶
            cv::Rect miny;
            std::vector<cv::Rect> all_zt;
            all_zt = workC.search_all_cone_rect(IMG.mat_lab);

            // 最高点锥桶
            if(all_zt.size() >=2) miny = all_zt[0];
            POINT mcentPoint = getCenterPoint(miny);
            cv::rectangle(IMG.mat_result, miny, cv::Scalar(0, 255, 255), 1);
            drawH(mcentPoint.y, 0, 0, 255, IMG.mat_result);


            IMG.aim_speed = (int16_t)(IMG._config->work_recback_speed); //逐渐减速         

            if (out_flage == 0)//稍稍反方向打角
            {
                float clone_k = Calculate_Left_ConeK(all_zt);
                IMG.aim_speed = -IMG.aim_speed;//倒车
                IMG.aim_angle = - clone_k * 10;
                //IMG._config->work_recback_angle
                if (last_data != 0 && abs(clone_k * 100 - last_data * 100) > 2)out_flage = 1;
                last_data = clone_k;
                work_route = IMG.encoder.route;
            }

            //开始 直线退出
            if (out_flage == 1)
            {   
                float clone_k = Calculate_Left_ConeK(all_zt);
                IMG.aim_speed = -IMG.aim_speed;//倒车
                IMG.aim_angle =0;

                if (IMG.max_white_line_length > 250 && work_route - IMG.encoder.route > 20 )out_flage = 2;
                
            }

            if (out_flage == 2) {

                IMG.aim_speed = 0;
                if (abs(IMG.encoder.speed * 1000) < 200) { work_route = IMG.encoder.route;  out_flage = 3; }
            }
            //退出速度赋值
            if (out_flage == 3)
            {
                IMG.aim_speed = (IMG._config->speed_base);
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

                if ( IMG.encoder.route - work_route > 30) {
                    control_flag.pop();//移除
                    flag_work = WORK_NONE;
                    MAT_LOG("WORK_HAVE_L");
                }
            }

        }

    }
}
