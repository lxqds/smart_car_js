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

float work_yaw = 0; // 
float work_in_yaw = 0; // 施工区偏航角积分
POINT work_near_cone;  //色块 较近的锥桶
POINT work_near_cone1;  //AI 较近的锥桶

POINT work_Red;  //色块 较近的锥桶
StopWatch stop_clock;
StopWatch stop_clock_2;
CVFind_Clone workC;
bool direction_work = true;//true : 左工作区 false : 右工作区
/* ********************************************************************* */
int FOUND_last_y;
float last_data;
uint8_t out_flage;
uint8_t in_flag;
uint8_t fond_flag;
float member_K = 0;
POINT AI_find_sign = POINT(0, 0);  //标记，分数*100
//速度控制函数
void speed_fun(int speed, float brake_P, float yx_speed_err) {
    if (IMG.aim_speed == speed)return;
    float speed_err;
    if (IMG.encoder.speed * 1000 > speed)IMG.aim_speed = speed - abs(IMG.encoder.speed * 1000) * brake_P;
    else if (IMG.encoder.speed * 1000 < speed) IMG.aim_speed = speed + abs(IMG.encoder.speed * 1000) * brake_P;

    speed_err = speed - IMG.encoder.speed * 1000;
    if (abs(speed_err) < yx_speed_err) IMG.aim_speed = speed;
}

//判断俩矩阵是否香蕉
bool rectA_intersect_rectB(cv::Rect rectA, cv::Rect rectB)
{
    if (rectA.x > rectB.x + rectB.width) { return false; }
    if (rectA.y > rectB.y + rectB.height) { return false; }
    if ((rectA.x + rectA.width) < rectB.x) { return false; }
    if ((rectA.y + rectA.height) < rectB.y) { return false; }

    float colInt = min(rectA.x + rectA.width, rectB.x + rectB.width) - max(rectA.x, rectB.x);
    float rowInt = min(rectA.y + rectA.height, rectB.y + rectB.height) - max(rectA.y, rectB.y);
    float intersection = colInt * rowInt;
    float areaA = rectA.width * rectA.height;
    float areaB = rectB.width * rectB.height;
    float intersectionPercent = intersection / (areaA + areaB - intersection);

    if ((0 < intersectionPercent) && (intersectionPercent < 1) && (intersection != areaA) && (intersection != areaB)) {
        return true;
    }

    return false;
}
//判断角点锥桶
bool pt_min_rect(std::vector<cv::Rect> all_zt, cv::Rect miny) {
    miny.x -= 40;
    miny.width += 80;
    if (miny.x < 0)miny.x = 0;
    if (miny.width + miny.x > IMAGE_WIDTH)miny.width = IMAGE_WIDTH - miny.x;
    for (size_t i = 0; i < all_zt.size(); i++) {
        if (miny.y == all_zt[i].y)continue;
        LOG_I << " in_pt" << LOG_END;
       // cv::rectangle(IMG.mat_result, miny, cv::Scalar(255, 0, 0), 1);
        if (rectA_intersect_rectB(all_zt[i], miny))return true;
    }
    return false;
}

void check_work()
{

    if (!IMG.element_over &&
        (!control_flag.contains(circle_element) || flag_circle == CIRCLE_OUT) &&   //当前没在环岛中
        !control_flag.contains(yroad_element) &&    //当前没在追逐区中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        !control_flag.contains(garage_element) &&   //当前没在入库中
        !control_flag.contains(ramp_element) &&      //当前没在桥中
        !control_flag.contains(bomb_element)) {

        for (auto r : IMG.detection_result) {
            if ((r.label == IMG.label_evil || r.label == IMG.label_thief) && r.score > 0.55 && r.y > IMAGE_HEIGHT * 0.15) {
                IMG.element_identify = 1;
                IMG.set_pid(8, 1.6, 0);
                if (control_flag.contains(circle_element)) {
                    control_flag.remove(circle_element);
                    flag_circle = CIRCLE_NONE;
                }
                IMG.brake();
                control_flag.push(work_element);
                work_route = IMG.encoder.route;
                direction_work = false;
                IMG.flag_track = TRACK_RIGHT;
                flag_work = WORK_FOUND_1;
                MAT_LOG("WORK_FOUND_R");
                LOG_I << "rescore" << r.score << LOG_END;
                FOUND_last_y = 0;
                in_flag = 0;
                return;
            }
            else if ((r.label == IMG.label_patient || r.label == IMG.label_tumble) && r.score > 0.55 && r.y > IMAGE_HEIGHT * 0.15) {

                IMG.element_identify = 1;
                IMG.set_pid(8, 1.6, 0);
                if (control_flag.contains(circle_element)) {
                    control_flag.remove(circle_element);
                    flag_circle = CIRCLE_NONE;
                }
                IMG.brake();
                control_flag.push(work_element);
                work_route = IMG.encoder.route;
                direction_work = true;
                IMG.flag_track = TRACK_LEFT;
                flag_work = WORK_FOUND_1;
                MAT_LOG("WORK_FOUND_L");
                LOG_I << "rescore" << r.score << LOG_END;
                FOUND_last_y = 0;
                in_flag = 0;
                return;
            }

        }

        if (IMG._config->assist_judgment) {
            work_near_cone1 = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备
            work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab);//右工作区进入准备
            if (work_near_cone1.x == 0)work_near_cone1 = search_right_near(IMG.edge_right_far, IMG.mat_lab);//右工作区进入准备
            if (work_near_cone.x == 0)work_near_cone = search_left_near(IMG.edge_left_far, IMG.mat_lab);//右工作区进入准备

            if ((work_near_cone1.x != 0 || work_near_cone.x != 0) && (work_near_cone.x > 0.25 * IMAGE_HEIGHT || work_near_cone1.x > 0.25 * IMAGE_HEIGHT)) {
                MAT_LOG("YEOW");
                work_Red = search_right_near_Red(IMG.edge_right, IMG.mat_lab2);
                if (work_Red.x == 0)work_Red = search_right_near_Red(IMG.edge_right_far, IMG.mat_lab2);

                if (work_Red.x != 0 && work_Red.x > 0.2 * IMAGE_HEIGHT) {
                    MAT_LOG("SEE_RED");
                    IMG.element_identify = 1;

                    if (control_flag.contains(circle_element)) {
                        control_flag.remove(circle_element);
                        flag_circle = CIRCLE_NONE;
                    }
                    IMG.set_pid(8, 1.6, 0);
                    IMG.brake();
                    control_flag.push(work_element);
                    work_route = IMG.encoder.route;
                    IMG.flag_track = TRACK_RIGHT;
                    flag_work = WORK_FOUND_0;
                    speed_fun(0, 2, 500);
                    IMG.aim_speed = IMG._config->speed_work;
                    MAT_LOG("WORK_FOUND_0");
                    fond_flag = 0;
                    AI_find_sign = POINT(0, 0);
                    stop_clock.tic();
                    return;
                }
            }
        }

    }




}

/* ********************************************************************* */

void run_work()
{
    if(0)
    {
        if (control_flag.top() == work_element) {
            IMG.get_edge(0);
            //工作区基准速度
            IMG.aim_speed = IMG._config->speed_work;

            work_near_cone = POINT(0, 0);
            IMG.edge_det.clear();
            //预识别到元素，但是未识别到左右进入方向，后退回去识别
            if (flag_work == WORK_FOUND_0) {

                if (fond_flag == 0) {
                    work_near_cone1 = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备
                    work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab);//右工作区进入准备
                    if (work_near_cone1.x == 0)work_near_cone1 = search_right_near(IMG.edge_right_far, IMG.mat_lab);//右工作区进入准备
                    if (work_near_cone.x == 0)work_near_cone = search_left_near(IMG.edge_left_far, IMG.mat_lab);//右工作区进入准备
                    if (work_near_cone1.x != 0 && work_near_cone.x != 0)fond_flag = 1;
                    speed_fun(IMG._config->speed_work + 1, 2, 200); //快速减速
                }

                if (fond_flag == 1) {

                    speed_fun(IMG._config->speed_work + 1, 2, 500); //快速减速
                    for (auto r : IMG.detection_result) {  //如果有识别到元素就进入工作区
                        if ((r.label == IMG.label_evil || r.label == IMG.label_thief) && r.score > 0.7 && r.y > IMAGE_HEIGHT * 0.15) {
                            IMG.element_identify = 1;
                            work_route = IMG.encoder.route;
                            direction_work = false;
                            IMG.flag_track = TRACK_RIGHT;
                            flag_work = WORK_FOUND_1;
                            MAT_LOG("WORK_FOUND_R");
                            LOG_I << "rescore" << r.score << LOG_END;
                            FOUND_last_y = 0;
                            in_flag = 0;

                            speed_fun(IMG._config->speed_work + 1, 2, 500);

                            if (AI_find_sign.y < r.score * 100) {
                                AI_find_sign.x = 1;
                                AI_find_sign.y = r.score * 100;
                            }
                        }
                        else if ((r.label == IMG.label_patient || r.label == IMG.label_tumble) && r.score > 0.35 && r.y > IMAGE_HEIGHT * 0.15) {

                            IMG.element_identify = 1;
                            work_route = IMG.encoder.route;
                            direction_work = true;
                            IMG.flag_track = TRACK_LEFT;
                            flag_work = WORK_FOUND_1;
                            MAT_LOG("WORK_FOUND_L");
                            LOG_I << "rescore" << r.score << LOG_END;
                            FOUND_last_y = 0;
                            in_flag = 0;

                            speed_fun(IMG._config->speed_work + 1, 2, 500);

                            if (AI_find_sign.y < r.score * 100) {
                                AI_find_sign.x = 0;
                                AI_find_sign.y = r.score * 100;
                            }
                        }

                    }

                    work_Red = search_right_near_Red(IMG.edge_right, IMG.mat_lab2); //右线红色色块
                    float threshold_re = 0.5;
                    int thresholdh = IMAGE_HEIGHT * threshold_re;

                    int coneLowy = IMAGE_HEIGHT * threshold_re + 20;

                    int err_H = IMAGE_HEIGHT * threshold_re + 10 - work_Red.x;

                    IMG.aim_speed = err_H * IMG._config->work_read_cone_base_speed_k;
                    if (work_Red.x == 0)IMG.aim_speed = -IMG._config->speed_work;
                    if (IMG.aim_speed > IMG._config->speed_work)IMG.aim_speed = IMG._config->speed_work;

                    speed_fun(IMG.aim_speed + 1, 2, 500);

                    drawH(thresholdh, 0, 0, 255, IMG.mat_result);
                    drawH(work_Red.x, 255, 0, 0, IMG.mat_result);
                    drawH(coneLowy, 0, 0, 255, IMG.mat_result);
                    //LOG_I << " speed" << abs((int)(IMG.encoder.speed * 1000)) << LOG_END;


                    if (work_Red.x > thresholdh && work_Red.x < coneLowy && abs(IMG.encoder.speed * 1000) >  IMG._config->speed_work) {
                        IMG.brake();
                        IMG.aim_speed = 0;
                    }
                    if (stop_clock.toc() > 2000 && abs((int)IMG.encoder.speed * 1000) < 500) {

                        if (IMG._config->work_Identify_timeouts == 0) {  //退出工作区
                            MAT_LOG("WORK_RED_ERR");
                            flag_work = WORK_OUT;
                            out_flage = 4;
                            work_route = IMG.encoder.route;
                            return;
                        }
                        else if (IMG._config->work_Identify_timeouts == 1) {  //直接进入左侧
                            IMG.element_identify = 1;
                            work_route = IMG.encoder.route;
                            direction_work = true;
                            IMG.flag_track = TRACK_LEFT;
                            flag_work = WORK_FOUND_1;
                            MAT_LOG("WORK_ERR_L");
                            FOUND_last_y = 0;
                            in_flag = 0;
                            speed_fun(IMG._config->speed_work + 1, 2, 500);
                        }
                        else {
                            fond_flag = 2;
                            MAT_LOG("WORK_ERR");  //假死
                            stop_clock.tic();
                        }
                    }

                }

                if (fond_flag == 2) {
                    srand(IMG.encoder.speed * 1000);
                    IMG.aim_speed = 800;
                    uint8_t fx = rand() % 255;
                    if (stop_clock.toc() > 500) {
                        stop_clock.tic();
                        if (fx > 125)IMG.aim_angle = 30;
                        else IMG.aim_angle = -30;
                    }

                }

            }

            if (flag_work == WORK_FOUND_1) {
                // 找锥桶
                IMG.edge_det = search_targets(IMG._det_result);
                // 进入方向
                work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
                work_near_cone1 = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备
                IMG.aim_speed = IMG._config->speed_work;
          //  LOG_I << " work_near_cone :" << work_near_cone.x << " IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy :" << IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy << LOG_END;

            // 当车辆开始靠近锥桶: 准备进入 识别或色块 识别到的锥桶大于一段阈值
            //if (work_near_cone.x > IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy)
            //{
            //    flag_work = WORK_FOUND_2;
            //    work_route = IMG.encoder.route;
            //    MAT_LOG("WORK_FOUND_2");
            //}
                float k = 1;
                if (work_near_cone1.y != 0 && work_near_cone.y != 0)k = (work_near_cone1.x - work_near_cone.x) / (work_near_cone1.y - work_near_cone.y);

                LOG_I << " work_near_cone k:" << k * 100 << LOG_END;
                cv::line(IMG.mat_result, cv::Point(work_near_cone.y, work_near_cone.x),
                    cv::Point(work_near_cone1.y, work_near_cone1.x), cv::Scalar(0, 255, 0), 1);
            //当 第一个锥桶消失
                if (in_flag == 0 && (work_near_cone.x > IMG._config->work_read_cone_threshold * IMAGE_HEIGHT || work_near_cone1.x > IMG._config->work_read_cone_threshold * IMAGE_HEIGHT) && abs(k) < 0.1) {
                    in_flag = 1;
                    flag_work = WORK_FOUND_2;
                    work_route = IMG.encoder.route;
                    MAT_LOG("WORK_FOUND_2");
                }
                drawH(work_near_cone1.x, 0, 255, 0, IMG.mat_result);
                drawH(work_near_cone.x, 0, 255, 0, IMG.mat_result);
                drawH(IMG._config->work_read_cone_threshold * IMAGE_HEIGHT, 255, 0, 0, IMG.mat_result);
            }

            if (flag_work == WORK_FOUND_2) {

                IMG.edge_det = search_targets(IMG._det_result);
                IMG.aim_speed = IMG._config->speed_work;
                // 找锥桶
                if (direction_work)
                    work_near_cone = search_left_near_up(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
                else
                    work_near_cone = search_right_near_up(IMG.edge_right, IMG.mat_lab);//右工作区进入准备

                int thresholdh = IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_in;

                int coneLowy = IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_in + IMG._config->work_read_cone_allow_error;

                int err_H = IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_in + IMG._config->work_read_cone_allow_error / 2.0 - work_near_cone.x;

                IMG.aim_speed = err_H * IMG._config->work_read_cone_base_speed_k;
                if (IMG.aim_speed > 0 && IMG.aim_speed < 500) IMG.aim_speed = 500; //基础速度
                if (IMG.aim_speed < 0 && IMG.aim_speed > -500) IMG.aim_speed = -500;

                if (IMG.aim_speed > IMG._config->speed_work) IMG.aim_speed = IMG._config->speed_work; //限幅
                if (IMG.aim_speed < -IMG._config->speed_work) IMG.aim_speed = -IMG._config->speed_work;
              //  if (speedP < 0)speedP -= IMG._config->work_read_cone_base_speed_k;

                int speed_err = abs(IMG.aim_speed - IMG.encoder.speed * 1000);
                if (work_near_cone.x == 0)IMG.aim_speed = -IMG.aim_speed;
                // LOG_I << "  speed_err:" << speed_err << LOG_END;
                if (work_near_cone.x >= thresholdh && work_near_cone.x <= coneLowy && IMG.encoder.speed * 1000 > 1000) { IMG.brake(); speed_fun(0, 2, 500); }

                if ((work_near_cone.x >= thresholdh && work_near_cone.x <= coneLowy) &&
                    speed_err < IMG._config->work_read_cone_allow_speed_error) {
                    IMG.aim_speed = 0;
                    work_route = IMG.encoder.route;
                    flag_work = WORK_READY;
                    work_yaw = IMG.encoder.yaw;
                    work_in_yaw = IMG.encoder.yaw;
                    MAT_LOG("WORK_READY");
                }

                drawH(thresholdh, 0, 0, 255, IMG.mat_result);
                drawH(work_near_cone.x, 255, 0, 0, IMG.mat_result);
                drawH(coneLowy, 0, 0, 255, IMG.mat_result);
            }

            if ((flag_work == WORK_FOUND_2 || flag_work == WORK_FOUND_1 || flag_work == WORK_FOUND_0) && fond_flag != 2) {

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
                if (IMG.encoder.speed * 1000 < 0)IMG.aim_angle = -IMG.aim_angle;

                // 超过指定距离未进入绕行区, 元素行驶失败
                if ((IMG.encoder.route - work_route > IMG._config->work_error)) {
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
                // 
                // 调整 进入角度
                IMG.aim_speed = (IMG._config->work_ready_speed);
                IMG.aim_angle = IMG._config->work_angle_in_1;
                if (!direction_work) IMG.aim_angle = -IMG.aim_angle;

                // 进入方向
                if (direction_work) {
                    work_near_cone = search_left_near(IMG.edge_right, IMG.mat_lab); //左工作区进入准备
                }
                else {
                    work_near_cone = search_right_near(IMG.edge_left, IMG.mat_lab);//右工作区进入准备+= 360
                }

                // IMG.Attitude_Ring(IMG.encoder.yaw, 35, 0.8, 0.1);//(IMG.encoder.route - work_route > IMG._config->work_rou_ready )
                float in_yaw = 30;

                float yaw_err = IMG.encoder.yaw - work_yaw;
                if (yaw_err > 180) yaw_err -= 360;
                if (yaw_err < -180)yaw_err += 360;

                if (IMG.max_white_line_length < 300 ||
                    (abs(yaw_err) > in_yaw)) {
                    work_route = IMG.encoder.route;
                    flag_work = WORK_RUN;
                    work_route2 = IMG.encoder.route;
                    MAT_LOG("WORK_RUN");
                    workC.init();
                }

            }


            if (flag_work == WORK_RUN) {
                cv::Rect miny;

                std::vector<cv::Rect> all_zt;
                all_zt = workC.search_all_cone_rect(IMG.mat_lab);

                // 最高点锥桶 角点追踪
                //if (all_zt.size() >= 1) {
                //    miny = cv::Rect(0, IMAGE_HEIGHT, IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1);
                //    // 找最高点且在屏幕一定范围下
                //    for (size_t i = 0; i < all_zt.size(); i++) {
                //        if (!pt_min_rect(all_zt, miny) && miny.y != IMAGE_HEIGHT)continue;
                //        if (all_zt[i].y < miny.y)miny = all_zt[i];
                //    }
                //}
                //cv::rectangle(IMG.mat_result, miny, cv::Scalar(0, 0, 255), 1);
                //for (size_t i = 0; i < all_zt.size(); i++) {
                //    cv::rectangle(IMG.mat_result, all_zt[i], cv::Scalar(255, 0, 0), 1);
                //}
                POINT minyCent = getCenterPoint(miny);
                if (minyCent.x != 0 && minyCent.y != 0) {
                    if (direction_work) {
                        minyCent.x -= 10;
                    }
                    else {
                        minyCent.x += 10;
                    }

                }
                minyCent.y = IMAGE_HEIGHT;//取消角点追踪
                //若未找到合适的锥桶，则采用全屏扫描色块
                if (minyCent.y == IMAGE_HEIGHT) {
                    minyCent.x = 150;
                    minyCent.y = search_mid_cone_getH(IMG.mat_lab);
                 //   LOG_I << "AVG_Y :" << minyCent.y << LOG_END;
                }

                IMG.aim_angle = 0;

                int err_W, err_H;

                err_W = IMAGE_WIDTH / 2.0 - minyCent.x;
                err_H = (int)IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_avg - minyCent.y;

                IMG.aim_angle = err_W * IMG._config->work_run_angle_p;
                drawH(err_H, 0, 255, 0, IMG.mat_result);
                drawH(IMAGE_HEIGHT * 0.1, 255, 0, 0, IMG.mat_result);
                drawW(minyCent.x, 0, 255, 0, IMG.mat_result);
                drawW(IMAGE_WIDTH / 2, 255, 0, 0, IMG.mat_result);
                cv::rectangle(IMG.mat_result, miny, cv::Scalar(0, 255, 255), 1);


                //开始靠近锥桶
                IMG.aim_speed = (int16_t)(IMG._config->work_run_speed);

                //if (minyCent.y < IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_avg) IMG.aim_speed = -IMG.aim_speed;

                int speed_err = abs(IMG.aim_speed - IMG.encoder.speed * 1000);
                if (abs(err_H) < IMAGE_HEIGHT * 0.1) {
                    IMG.brake();
                    speed_fun(0, 2, 300);
                    work_route = IMG.encoder.route;
                    flag_work = WORK_OUT;
                    workC.init();
                    out_flage = 0;
                    if (abs(work_in_yaw - IMG.encoder.yaw) > 30)out_flage = 1;
                    last_data = 0;
                    member_K = 0;
                   // IMG.opid();
                    MAT_LOG("WORK_OUT");
                }
            }

            if (flag_work == WORK_STOPPING)//刹车中
            {
                IMG.aim_speed = 0;
                if (IMG.encoder.speed < 0.1 && IMG.encoder.speed > -0.1) {
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
                if (stop_clock.toc() > IMG._config->work_stop_time) {
                    work_route = IMG.encoder.route;
                    MAT_LOG("WORK_PARKINISH"); //泊车中
                    flag_work = WORK_OUT;
                    workC.init();
                    out_flage = 0;
                    if (abs(work_in_yaw - IMG.encoder.yaw) > 30)out_flage = 1;

                    last_data = 0;
                    member_K = 0;
                    work_yaw = IMG.encoder.yaw;
                }
                else {
                    MAT_LOG("WORK_PARKING");
                }
            }

            if (flag_work == WORK_OUT) {
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
                int16_t avg_y = search_mid_cone_getH(IMG.mat_lab);
                // 最高点锥桶
                if (all_zt.size() >= 2) miny = all_zt[0];
                POINT mcentPoint = getCenterPoint(miny);
                cv::rectangle(IMG.mat_result, miny, cv::Scalar(0, 255, 255), 1);
                drawH(mcentPoint.y, 0, 0, 255, IMG.mat_result);

                IMG.aim_speed = (int16_t)(IMG._config->work_recback_speed); //逐渐减速     

                if (member_K == 0) {
                    if (direction_work)member_K = Calculate_Right_ConeK(all_zt);  //记录后退第一帧的k值
                    else member_K = Calculate_Left_ConeK(all_zt);
                }
                if (out_flage == 0)//稍稍反方向打角
                {
                    float out_yaw = -10;
                    IMG.aim_speed = -IMG.aim_speed;//倒车
                    IMG.aim_angle = -45;
                    if (!direction_work) {
                        out_yaw = -out_yaw;
                        IMG.aim_angle = -IMG.aim_angle;
                    }

                    if (abs(IMG.encoder.yaw - work_yaw) > out_yaw) {
                        out_flage = 1;
                        MAT_LOG("out_flage1");
                    }
                    work_route = IMG.encoder.route;

                }

                //开始 退出
                if (out_flage == 1) {

                    float k = 0;
                    if (direction_work) {
                        k = Calculate_Right_ConeK(all_zt);
                        cv::line(IMG.mat_result, cv::Point(all_zt[0].x, all_zt[0].y),
                            cv::Point(IMAGE_WIDTH - 1, (int)(k * (IMAGE_WIDTH - 1 - all_zt[0].x) + all_zt[0].y)), cv::Scalar(0, 255, 0), 1);
                    }
                    else {
                        k = Calculate_Left_ConeK(all_zt);
                        cv::line(IMG.mat_result, cv::Point(0, (int)(all_zt[0].y - k * (all_zt[0].x))),
                            cv::Point(all_zt[0].x, all_zt[0].y), cv::Scalar(0, 255, 0), 1);

                    }

                    IMG.aim_angle = (member_K - k) * IMG._config->work_out_angular_P;
                    if (direction_work)IMG.aim_angle += IMG._config->work_out_angular_base_deviation;
                    else IMG.aim_angle -= IMG._config->work_out_angular_base_deviation;
                    //if (direction_work && IMG.aim_angle < IMG._config->work_out_angular_base_deviation) IMG.aim_angle = IMG._config->work_out_angular_base_deviation;
                    //else if (!direction_work && IMG.aim_angle > -IMG._config->work_out_angular_base_deviation)IMG.aim_angle = -IMG._config->work_out_angular_base_deviation;

                    IMG.aim_speed = -IMG._config->work_recback_speed;
                    speed_fun(IMG.aim_speed+1, 3, 500);
                    if(IMG.encoder.speed *1000 > 0) speed_fun(IMG.aim_speed + 1, 8, 500);

                   // LOG_I << " work_route  " << work_route  << "  enco" << IMG.encoder.route <<"  cz  " << work_route - IMG.encoder.route <<LOG_END;
                    if (IMG.max_white_line_length > 1000 && work_route - IMG.encoder.route > 30) {
                        out_flage = 2;
                        IMG.brake();
                        MAT_LOG("out_flage2");
                    }
                }
                //
                if (out_flage == 2) {
                    IMG.aim_angle = 0;
                    IMG.aim_speed = 0;
                    if (abs(IMG.encoder.speed * 1000) < 300) {
                        work_route = IMG.encoder.route;
                        out_flage = 4;
                        IMG.brake();
                        MAT_LOG("out_flage3");
                    }

                }
                //大幅度打角，将车拉回赛道
                if (out_flage == 3) {
                    IMG.aim_speed = IMG._config->speed_work;
                    IMG.aim_angle = IMG._config->work_recback_angle;

                    if (!direction_work)IMG.aim_angle = -IMG.aim_angle;

                    if (
                        (IMG.ipts0_num > 100 && IMG.ipts1_num > 100) ||
                        ((work_near_cone.x != 0 && work_near_cone.y != 0) &&
                            (work_near_cone1.x != 0 && work_near_cone1.y != 0)) ||
                        (IMG.encoder.route - work_route > 80)) {

                        out_flage = 4;
                        work_route = IMG.encoder.route;
                        MAT_LOG("out_flage4");
                    }

                }
                //退出速度赋值  正常巡线一段时间后退出工作区
                if (out_flage == 4) {
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

                    if (IMG.encoder.route - work_route > IMG._config->work_leave_route) {
                        control_flag.pop();//移除
                        flag_work = WORK_NONE;
                        MAT_LOG("WORK_HAVE_DO");
                    }
                }

            }

        }
    }
    else {
        if (control_flag.top() == work_element) {
            IMG.get_edge(0);
            //工作区基准速度
            IMG.aim_speed = IMG._config->speed_work;

            work_near_cone = POINT(0, 0);
            IMG.edge_det.clear();
            //预识别到元素，但是未识别到左右进入方向，后退回去识别
            if (flag_work == WORK_FOUND_0) {

                if (fond_flag == 0) {
                    work_near_cone1 = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备
                    work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab);//右工作区进入准备
                    if (work_near_cone1.x == 0)work_near_cone1 = search_right_near(IMG.edge_right_far, IMG.mat_lab);//右工作区进入准备
                    if (work_near_cone.x == 0)work_near_cone = search_left_near(IMG.edge_left_far, IMG.mat_lab);//右工作区进入准备
                    if (work_near_cone1.x != 0 && work_near_cone.x != 0)fond_flag = 1;
                    speed_fun(IMG._config->speed_work + 1, 2, 200); //快速减速
                }

                if (fond_flag == 1) {

                    speed_fun(IMG._config->speed_work + 1, 2, 500); //快速减速
                    for (auto r : IMG.detection_result) {  //如果有识别到元素就进入工作区
                        if ((r.label == IMG.label_evil || r.label == IMG.label_thief) && r.score > 0.7 && r.y > IMAGE_HEIGHT * 0.15) {
                            IMG.element_identify = 1;
                            work_route = IMG.encoder.route;
                            direction_work = false;
                            IMG.flag_track = TRACK_RIGHT;
                            flag_work = WORK_FOUND_1;
                            MAT_LOG("WORK_FOUND_R");
                            LOG_I << "rescore" << r.score << LOG_END;
                            FOUND_last_y = 0;
                            in_flag = 0;

                            speed_fun(IMG._config->speed_work + 1, 2, 500);

                            if (AI_find_sign.y < r.score * 100) {
                                AI_find_sign.x = 1;
                                AI_find_sign.y = r.score * 100;
                            }
                        }
                        else if ((r.label == IMG.label_patient || r.label == IMG.label_tumble) && r.score > 0.35 && r.y > IMAGE_HEIGHT * 0.15) {

                            IMG.element_identify = 1;
                            work_route = IMG.encoder.route;
                            direction_work = true;
                            IMG.flag_track = TRACK_LEFT;
                            flag_work = WORK_FOUND_1;
                            MAT_LOG("WORK_FOUND_L");
                            LOG_I << "rescore" << r.score << LOG_END;
                            FOUND_last_y = 0;
                            in_flag = 0;

                            speed_fun(IMG._config->speed_work + 1, 2, 500);

                            if (AI_find_sign.y < r.score * 100) {
                                AI_find_sign.x = 0;
                                AI_find_sign.y = r.score * 100;
                            }
                        }

                    }

                    work_Red = search_right_near_Red(IMG.edge_right, IMG.mat_lab2); //右线红色色块
                    float threshold_re = 0.5;
                    int thresholdh = IMAGE_HEIGHT * threshold_re;

                    int coneLowy = IMAGE_HEIGHT * threshold_re + 20;

                    int err_H = IMAGE_HEIGHT * threshold_re + 10 - work_Red.x;

                    IMG.aim_speed = err_H * IMG._config->work_read_cone_base_speed_k;
                    if (work_Red.x == 0)IMG.aim_speed = -IMG._config->speed_work;
                    if (IMG.aim_speed > IMG._config->speed_work)IMG.aim_speed = IMG._config->speed_work;

                    speed_fun(IMG.aim_speed + 1, 2, 500);

                    drawH(thresholdh, 0, 0, 255, IMG.mat_result);
                    drawH(work_Red.x, 255, 0, 0, IMG.mat_result);
                    drawH(coneLowy, 0, 0, 255, IMG.mat_result);
                    //LOG_I << " speed" << abs((int)(IMG.encoder.speed * 1000)) << LOG_END;


                    if (work_Red.x > thresholdh && work_Red.x < coneLowy && abs(IMG.encoder.speed * 1000) >  IMG._config->speed_work) {
                        IMG.brake();
                        IMG.aim_speed = 0;
                    }
                    if (stop_clock.toc() > 2000 && abs((int)IMG.encoder.speed * 1000) < 500) {

                        if (IMG._config->work_Identify_timeouts == 0) {  //退出工作区
                            MAT_LOG("WORK_RED_ERR");
                            flag_work = WORK_OUT;
                            out_flage = 4;
                            work_route = IMG.encoder.route;
                            return;
                        }
                        else if (IMG._config->work_Identify_timeouts == 1) {  //直接进入左侧
                            IMG.element_identify = 1;
                            work_route = IMG.encoder.route;
                            direction_work = true;
                            IMG.flag_track = TRACK_LEFT;
                            flag_work = WORK_FOUND_1;
                            MAT_LOG("WORK_ERR_L");
                            FOUND_last_y = 0;
                            in_flag = 0;
                            speed_fun(IMG._config->speed_work + 1, 2, 500);
                        }
                        else {
                            fond_flag = 2;
                            MAT_LOG("WORK_ERR");  //假死
                            stop_clock.tic();
                        }
                    }

                }

                if (fond_flag == 2) {
                    srand(IMG.encoder.speed * 1000);
                    IMG.aim_speed = 800;
                    uint8_t fx = rand() % 255;
                    if (stop_clock.toc() > 500) {
                        stop_clock.tic();
                        if (fx > 125)IMG.aim_angle = 30;
                        else IMG.aim_angle = -30;
                    }

                }

            }

            if (flag_work == WORK_FOUND_1) {
                // 找锥桶
                IMG.edge_det = search_targets(IMG._det_result);
                // 进入方向
                work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
                work_near_cone1 = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备
                IMG.aim_speed = IMG._config->speed_work;
                //  LOG_I << " work_near_cone :" << work_near_cone.x << " IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy :" << IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy << LOG_END;

                  // 当车辆开始靠近锥桶: 准备进入 识别或色块 识别到的锥桶大于一段阈值
                  //if (work_near_cone.x > IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy)
                  //{
                  //    flag_work = WORK_FOUND_2;
                  //    work_route = IMG.encoder.route;
                  //    MAT_LOG("WORK_FOUND_2");
                  //}
                float k = 1;
                if (work_near_cone1.y != 0 && work_near_cone.y != 0)k = (work_near_cone1.x - work_near_cone.x) / (work_near_cone1.y - work_near_cone.y);

                LOG_I << " conek:" << k * 100 << LOG_END;
                cv::line(IMG.mat_result, cv::Point(work_near_cone.y, work_near_cone.x),
                    cv::Point(work_near_cone1.y, work_near_cone1.x), cv::Scalar(0, 255, 0), 1);
                //当 第一个锥桶消失
                if (in_flag == 0 && (work_near_cone.x > IMG._config->work_read_cone_threshold * IMAGE_HEIGHT || work_near_cone1.x > IMG._config->work_read_cone_threshold * IMAGE_HEIGHT) && abs(k) < 0.1) {
                    in_flag = 1;
                    work_yaw = IMG.encoder.yaw;
                    flag_work = WORK_FOUND_2;
                    work_route = IMG.encoder.route;
                    work_route2 = IMG.encoder.route;
                    MAT_LOG("WORK_FOUND_2");
                }
                drawH(work_near_cone1.x, 0, 255, 0, IMG.mat_result);
                drawH(work_near_cone.x, 0, 255, 0, IMG.mat_result);
                drawH(IMG._config->work_read_cone_threshold * IMAGE_HEIGHT, 255, 0, 0, IMG.mat_result);
            }
        
            if (flag_work == WORK_FOUND_2) {

                IMG.edge_det = search_targets(IMG._det_result);
                IMG.aim_speed = IMG._config->speed_work;
                // 找锥桶
                if (direction_work)
                    work_near_cone = search_left_near_up(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
                else
                    work_near_cone = search_right_near_up(IMG.edge_right, IMG.mat_lab);//右工作区进入准备

                int thresholdh = IMAGE_HEIGHT * IMG._config->work_route_read_cone_thresholdy_in;

                if ((work_near_cone.x >= thresholdh)) {
                    in_flag = 1;
                    work_yaw = IMG.encoder.yaw;
                    flag_work = WORK_READY;
                    work_route = IMG.encoder.route;
                    work_route2 = IMG.encoder.route;
                    MAT_LOG("WORK_READY");
                }

                drawH(thresholdh, 0, 0, 255, IMG.mat_result);
                drawH(work_near_cone.x, 255, 0, 0, IMG.mat_result);
            }

            if ((flag_work == WORK_FOUND_2 || flag_work == WORK_FOUND_1 || flag_work == WORK_FOUND_0) && fond_flag != 2) {

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
                if (IMG.encoder.speed * 1000 < 0)IMG.aim_angle = -IMG.aim_angle;

                // 超过指定距离未进入绕行区, 元素行驶失败
                if ((IMG.encoder.route - work_route > IMG._config->work_error)) {
                    IMG.element_over = true;
                    IMG.element_over_route = IMG.encoder.route;
                    control_flag.pop();//移除
                    flag_work = WORK_NONE;
                    MAT_LOG("WORK_ERROR");
                }

            }

            if (flag_work == WORK_READY)
            {
                IMG.aim_speed = (IMG._config->work_route_speed);
                IMG.aim_angle = IMG._config->work_route_angle_in_1;
                if (!direction_work) IMG.aim_angle = -IMG.aim_angle;

                // 进入方向
                if (direction_work) {
                    work_near_cone = search_left_near(IMG.edge_right, IMG.mat_lab); //左工作区进入准备
                }
                else {
                    work_near_cone = search_right_near(IMG.edge_left, IMG.mat_lab);//右工作区进入准备+= 360
                }

                // IMG.Attitude_Ring(IMG.encoder.yaw, 35, 0.8, 0.1);//(IMG.encoder.route - work_route > IMG._config->work_rou_ready )
                float in_yaw = IMG._config->work_route_ready_yaw;

                float yaw_err = IMG.encoder.yaw - work_yaw;
                if (yaw_err > 180) yaw_err -= 360;
                if (yaw_err < -180)yaw_err += 360;

                if (IMG.max_white_line_length < 300 ||
                    (abs(yaw_err) > in_yaw)) {
                    work_route = IMG.encoder.route;
                    flag_work = WORK_RUN;
                    work_route2 = IMG.encoder.route - work_route2;
                    MAT_LOG("WORK_RUN");
                    workC.init();
                }

            }

            if (flag_work == WORK_RUN) {
                IMG.aim_speed = (IMG._config->work_route_speed);
                IMG.aim_angle = IMG._config->work_angle_in_1;
                if (!direction_work) IMG.aim_angle = -IMG.aim_angle;

                uint16_t coney = search_mid_cone_getH(IMG.mat_lab);
                if (IMG.encoder.route - work_route > IMG._config->work_run_route || coney > IMG._config->work_route_run_AVG * IMAGE_HEIGHT) {
                    work_route = IMG.encoder.route;
                    flag_work = WORK_OUT;
                    out_flage = 0;
                    MAT_LOG("WORK_OUT");
                }
            }

            if (flag_work == WORK_OUT)
            {
                if (out_flage == 0) {
                    IMG.aim_speed = -(IMG._config->work_recback_speed);
                    IMG.aim_angle = IMG._config->work_angle_in_1;
                    if (!direction_work) IMG.aim_angle = -IMG.aim_angle;

                    if (work_route - IMG.encoder.route > work_route2 + IMG._config->work_run_route && IMG.max_white_line_length > 1000) {
                        work_route = IMG.encoder.route;
                        out_flage=1;
                        MAT_LOG("WORK_leave");
                    }
                }

                //退出速度赋值  正常巡线一段时间后退出工作区
                if (out_flage == 1) {
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

                    if (IMG.encoder.route - work_route > IMG._config->work_leave_route) {
                        control_flag.pop();//移除
                        flag_work = WORK_NONE;
                        MAT_LOG("WORK_HAVE_DO");
                    }
                }
            }
        }
    }
}
