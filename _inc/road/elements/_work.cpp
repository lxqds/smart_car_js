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
        !control_flag.contains(bomb_element) &&    //当前没在炸弹中
        (flag_work == WORK_DETECTION || flag_work == WORK_EVEI_DETECTION))
    {
        if (flag_work == WORK_EVEI_DETECTION) {
            MAT_LOG("WORK_FOUND_R");
            direction_work = false;
            IMG.flag_track = TRACK_LEFT;
        }
        else {
            MAT_LOG("WORK_FOUND_L");
            direction_work = true;
            IMG.flag_track = TRACK_RIGHT;
        }

        IMG.element_identify = 1;
        work_route = IMG.encoder.route;
        control_flag.push(work_element);
        flag_work = WORK_FOUND_1;
    }
}

/* ********************************************************************* */

void run_work()
{


    if (control_flag.top() == work_element) {

            //工作区基准速度
            IMG.aim_speed = IMG._config->speed_work;

            work_near_cone = POINT(0, 0);
            work_near_AIcone = POINT(0, 0);
            IMG.edge_det.clear();   
            if (flag_work == WORK_FOUND_1)
            {
                // 清弯道防止加大 P 项, 提高稳定性
                IMG.is_curve0 = IMG.is_curve1 = false;

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
                // 清弯道防止加大 P 项, 提高稳定性
                IMG.is_curve0 = IMG.is_curve1 = false;

                // 找锥桶
                IMG.edge_det = search_targets(IMG._det_result);

                // 进入方向
                if (true)
                    work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
                else
                    work_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备

                if (work_near_cone.x < IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy && work_near_cone.x > IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_in  && IMG.encoder.route - work_route > 10) {

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

                // 清弯道防止加大 P 项, 提高稳定性
                IMG.is_curve0 = IMG.is_curve1 = false;
                 // 调整 进入角度
                 IMG.aim_angle = IMG._config->work_angle_in_1;
                 if (!direction_work) IMG.aim_angle = -IMG.aim_angle;
                 if (IMG.encoder.route - work_route > IMG._config->work_rou_ready)
                 {      
                
                        work_route = IMG.encoder.route;
                        flag_work =  WORK_RUN;
                        work_route2= IMG.encoder.route;
                        MAT_LOG("WORK_RUN");
                 }

            }
          

            if (flag_work == WORK_RUN) {

                int16_t avg_y = search_mid_cone_getH(IMG.mat_lab);

                IMG.aim_angle = 0;
                LOG_I << "  get_AIorcolor_AVG_Y :" << avg_y << LOG_END;
                //开始靠近锥桶
                if (avg_y < IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy_avg&& IMG.encoder.route - work_route < IMG._config->work_rou_in )
                {

                    IMG.aim_speed = (int16_t)(IMG._config->work_run_speed ) ;

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

            //"work_out_stop_time": 100,
            //    "work_out_speedup_time" : 500,
            //    "work_rou_in_total" : 80,
            //    "work_rou_run" : 1,
            //    "work_rou_out" : 20,
            //    "work_rou_none" : 10,
            //    "work_angle_in_2" : 5,
            //    "work_recback_line_route" : 27,
            //    "work_recback_line_angle" : -15,
            //    "work_recback_angle_route" : 60,

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
                if (work_route - IMG.encoder.route < work_route2 * IMG._config-> work_recback_line_k)
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
    //if (control_flag.top() == work_element) {

    //    //工作区基准速度
    //    IMG.aim_speed = IMG._config->speed_work;

    //    work_near_cone = POINT(0, 0);
    //    IMG.edge_det.clear();

    //    /* ***************************************************************** */

    //    if (flag_work == WORK_FOUND_1)
    //    {
    //        // 清弯道防止加大 P 项, 提高稳定性
    //        IMG.is_curve0 = IMG.is_curve1 = false;

    //        // 找锥桶
    //        IMG.edge_det = search_targets(IMG._det_result);

    //        // 进入方向
    //        if (true)
    //            work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab); //左工作区进入准备
    //        else
    //            work_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);//右工作区进入准备

    //        LOG_I << " work_near_cone :" << work_near_cone.x << " IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy :" << IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy << LOG_END;
    //        
    //        // 当车辆开始靠近锥桶: 准备进入
    //        if (work_near_cone.x > IMAGE_HEIGHT * IMG._config->work_read_cone_thresholdy)
    //        {
    //            work_route = IMG.encoder.route;
    //            flag_work = WORK_READY;
    //            MAT_LOG("WORK_READY");
    //        }

    //        // 超过指定距离未进入绕行区, 元素行驶失败
    //        if (flag_work != WORK_READY && (IMG.encoder.route - work_route > IMG._config->work_error))
    //        {
    //            IMG.element_over = true;
    //            IMG.element_over_route = IMG.encoder.route;
    //            control_flag.pop();//移除
    //            flag_work = WORK_NONE;
    //            MAT_LOG("WORK_ERROR");
    //        }

    //        /* *** *** ***正常巡线 *** *** */  //发现元素时正常巡线，ready状态接管控制

    //        if (IMG.flag_track == TRACK_LEFT) {
    //            IMG.rpts = IMG.rptsc0;
    //            IMG.rpts_num = IMG.rptsc0_num;
    //        }
    //        else if (IMG.flag_track == TRACK_RIGHT) {
    //            IMG.rpts = IMG.rptsc1;
    //            IMG.rpts_num = IMG.rptsc1_num;
    //        }
    //        else {
    //            IMG.rpts = IMG.rptscs;
    //            IMG.rpts_num = IMG.rptscs_num;
    //        }

    //        IMG.calculate_angle();//计算角度


    //    }

    //    /* ***************************************************************** */
    //    /* *************************** 左侧施工区 **************************** */
    //    /* ***************************************************************** */

    //    if (direction_work == true)
    //    {
    //        if (flag_work == WORK_READY)
    //        {
    //            // 清弯道防止加大 P 项, 提高稳定性
    //            IMG.is_curve0 = IMG.is_curve1 = false;

    //            // 左入口 巡右线
    //            IMG.flag_track = TRACK_RIGHT;

    //            // 找目标点
    //            IMG.edge_det = search_targets(IMG._det_result);
    //            work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab);
    //            POINT cone_RU = search_rightUp_cone(IMG.edge_det);

    //            // 补线
    //            if (IMG.encoder.route - work_route < IMG._config->work_rou_in_total)
    //            {
    //                // 第一阶段: 当赛道 右 边缘存在时
    //                if (IMG.rpts1s_num > 0.5 / SAMPLE_DIST || IMG.encoder.route - work_route < IMG._config->work_rou_ready)
    //                {
    //                    // 第一阶段打角
    //                    IMG.aim_angle = IMG._config->work_angle_in_1;
    //                    MAT_LOG("WORK_FIRST");
    //                    work_route2 = IMG.encoder.route; // 为积分第二段路程
    //                }
    //                // 第二阶段: 检查右上锥桶坐标满足巡航条件
    //                else
    //                {
    //                    if (cone_RU.x > 0 && IMG.encoder.route - work_route2 <= IMG._config->work_rou_in)
    //                    {
    //                        // 第二阶段打角
    //                        IMG.aim_angle = IMG._config->work_angle_in_2;
    //                        MAT_LOG("WORK_SECOND");
    //                    }
    //                    else {
    //                        work_route = IMG.encoder.route;
    //                        flag_work = WORK_RUN;
    //                        MAT_LOG("WORK_RUN");
    //                    }
    //                }
    //            }
    //            else
    //            {
    //                work_route = IMG.encoder.route;
    //                flag_work = WORK_RUN;
    //                MAT_LOG("WORK_RUN");
    //            }
    //        }

    //        // *****************************************************************

    //        if (flag_work == WORK_RUN)//摆正车头，现无用
    //        {
    //            // 清弯道防止加大 P 项, 提高稳定性
    //            IMG.is_curve0 = IMG.is_curve1 = false;

    //            // 左区域 巡左线
    //            IMG.flag_track = TRACK_LEFT;

    //            // 找目标点
    //            IMG.edge_det = search_targets(IMG._det_result);
    //            vector<POINT> conesLeft = search_left_cones(IMG.edge_det); // 左方锥桶
    //            POINT cone_RU = search_rightUp_cone(IMG.edge_det);         // 右上锥桶

    //            // 补线
    //            if (conesLeft.size() >= 2)
    //            {
    //                uint8_t endd = conesLeft.size() > 2 ? conesLeft.size() - 2 : conesLeft.size() - 1;
    //                double k = 0, b = 0;
    //                k = (float)(conesLeft[0].y - conesLeft[endd].y) /
    //                    (float)(conesLeft[0].x - conesLeft[endd].x);
    //                b = conesLeft[0].y - k * conesLeft[0].x;

    //                IMG.edge_left = IMG.last_edge_left;
    //                if (k != 0 && b != 0)
    //                {
    //                    POINT startPoint = POINT(-b / k, 0); // 补线起点: 左
    //                    POINT endPoint = POINT(0, b);        // 补线终点: 右
    //                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5,
    //                        (startPoint.y + endPoint.y) * 0.5); // 补线中点

    //                    vector<POINT> input = { startPoint, midPoint, endPoint };
    //                    vector<POINT> repair = bezier(0.02, input);

    //                    // 左边缘切行, 提升右拐能力
    //                    if (repair.size() > 10)
    //                    {
    //                        int index = repair.size() * 0.33f;
    //                        IMG.edge_left.clear();
    //                        for (size_t i = index; i < repair.size(); i++)
    //                        {
    //                            IMG.edge_left.push_back(repair[i]);
    //                        }
    //                        IMG.path_last_edge_left.push_back(IMG.edge_left);
    //                    }
    //                    else
    //                    {
    //                        IMG.edge_left = repair;
    //                        IMG.path_last_edge_left.push_back(IMG.edge_left);
    //                    }

    //                    IMG.last_edge_left = IMG.edge_left;
    //                }
    //            }
    //            else
    //            {
    //                IMG.edge_left = IMG.last_edge_left;
    //                IMG.path_last_edge_left.push_back(IMG.edge_left);
    //            }

    //            // 计算所有锥桶的平均高度
    //            int num = 0;
    //            // uint8_t counterRec;
    //            for (unsigned int i = 0; i < IMG.edge_det.size(); i++)
    //                num += IMG.edge_det[i].x;

    //            MAT_LOG("WORK_RUN_ING");

    //            // 积分延迟一定距离出施工区
    //            if ((cone_RU.x >= IMAGE_HEIGHT * 0.8) || (IMG.encoder.route - work_route >= IMG._config->work_rou_run))
    //            {
    //                work_route = IMG.encoder.route;
    //                flag_work = WORK_STOPPING;
    //                MAT_LOG("WORK_RUN_STOP");
    //            }
    //        }
    //        // *****************************************************************

    //        if (flag_work == WORK_STOPPING)//刹车中
    //        {
    //            IMG.aim_speed = 0;
    //            if (IMG.encoder.speed < 0.1 && IMG.encoder.speed > -0.1)
    //            {
    //                flag_work = WORK_PARKING;
    //                MAT_LOG("WORK_STOPFINISH");//刹车完成
    //                stop_clock.tic();
    //            }
    //            MAT_LOG("WORK_STOPPING");
    //        }



    //        if (flag_work == WORK_PARKING)//泊车
    //        {
    //            IMG.aim_speed = 0;
    //            if (stop_clock.toc() > IMG._config->work_stop_time)
    //            {
    //                work_route = IMG.encoder.route;
    //                MAT_LOG("WORK_PARKINISH"); //泊车中
    //                flag_work = WORK_OUT;
    //            }
    //            else
    //            {
    //                MAT_LOG("WORK_PARKING");
    //            }
    //        }

    //        if (flag_work == WORK_OUT)//倒车
    //        {
    //            //退出一阶段打角
    //            if (work_route - IMG.encoder.route <= IMG._config->work_recback_line_route)
    //            {
    //                MAT_LOG("WORK_LINE_OUT");
    //                IMG.aim_angle = IMG._config->work_recback_line_angle; //出区 角度
    //                IMG.aim_speed = -IMG._config->work_recback_speed;//出去 距离
    //            }
    //            else if (IMG.rpts1s_num > 0.8 / SAMPLE_DIST && IMG.rpts0s_num > 0.8 / SAMPLE_DIST && work_route - IMG.encoder.route > IMG._config->work_recback_line_route )
    //            {
    //                IMG.flag_track = TRACK_RIGHT;
    //                control_flag.pop();//移除
    //                flag_work = WORK_NONE;
    //                MAT_LOG("WORK_LINE");
    //            }
    //            else if (work_route - IMG.encoder.route >  IMG._config->work_recback_line_route &&  work_route - IMG.encoder.route <= IMG._config->work_recback_angle_route ) //出去路程
    //            {
    //                MAT_LOG("WORK_OUTING");
    //                IMG.aim_angle = IMG._config->work_recback_angle; //出区 角度
    //                IMG.aim_speed = -IMG._config->work_recback_speed;//出去 距离
    //            }
    //            else if(work_route - IMG.encoder.route > IMG._config->work_recback_angle_route)
    //            {
    //                control_flag.pop();//移除
    //                flag_work = WORK_NONE;
    //                MAT_LOG("WORK_NONE");
    //            }

    //        }
    //    }

    //    /* ***************************************************************** */
    //    /* *************************** 右侧施工区 **************************** */
    //    /* ***************************************************************** */
    //    /* ********************* 注意！！！ 右侧施工区有问题 ******************** */
    //    /* ************* 国赛前没调右侧施工区, 比赛的时候没有进入施工区 ************* */
    //    /* **************** 比赛完就没管了, 这个地方需要看图像调试 **************** */

    //    else
    //    {
    //        if (flag_work == WORK_READY)
    //        {
    //            // 清弯道防止加大 P 项, 提高稳定性
    //            IMG.is_curve0 = IMG.is_curve1 = false;

    //            // 右入口 巡左线
    //            IMG.flag_track = TRACK_LEFT;

    //            // 找目标点
    //            IMG.edge_det = search_targets(IMG._det_result);
    //            work_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);
    //            POINT cone_LD = search_leftDown_cone(IMG.edge_det);

    //            // 补线
    //            if (IMG.encoder.route - work_route < IMG._config->work_rou_in_total)
    //            {
    //                // 第一阶段: 当赛道边缘存在时
    //                if (IMG.rpts0s_num > 0.5 / SAMPLE_DIST )
    //                {
    //                    // 第一阶段打角
    //                    IMG.aim_angle = -IMG._config->work_angle_in_1;
    //                    //MAT_LOG("WORK_FIRST");
    //                    log_i("WORK_FIRST");
    //                    work_route2 = IMG.encoder.route;
    //                }
    //                // 第二阶段: 检查左下锥桶坐标满足巡航条件
    //                else
    //                {
    //                    if (cone_LD.x > 0&& IMG.encoder.route - work_route2 <= IMG._config->work_rou_in)
    //                    {
    //                        // 第二阶段打角
    //                        IMG.aim_angle = -IMG._config->work_angle_in_2;
    //                        log_i("WORK_SECOND");
    //                        MAT_LOG("WORK_SECOND");
    //                    }
    //                    else {
    //                        work_route = IMG.encoder.route;
    //                        flag_work = WORK_RUN;
    //                        MAT_LOG("WORK_RUN");
    //                    }
    //                }
    //            }
    //            else
    //            {
    //                work_route = IMG.encoder.route;
    //                flag_work = WORK_RUN;
    //                MAT_LOG("WORK_RUN");
    //            }
    //        }

    //        /* ***************************************************************** */

    //        if (flag_work == WORK_RUN)
    //        {
    //            // 清弯道防止加大 P 项, 提高稳定性
    //            IMG.is_curve0 = IMG.is_curve1 = false;

    //            // 右区域 巡右线
    //            IMG.flag_track = TRACK_RIGHT;

    //            // 找目标点
    //            IMG.edge_det = search_targets(IMG._det_result);
    //            vector<POINT> conesRight = search_right_cones(IMG.edge_det); // 右方锥桶
    //            POINT cone_LU = search_leftUp_cone(IMG.edge_det);//左上锥桶

    //            // 补线
    //            if (conesRight.size() >= 2)
    //            {
    //                uint8_t endd = conesRight.size() > 2 ? conesRight.size() - 2 : conesRight.size() - 1;
    //                double k = 0, b = 0;

    //                double m = conesRight[endd].y - conesRight[0].y;
    //                if (0 == m)
    //                {
    //                    k = 10000.0f;
    //                    b = conesRight[0].x - k * conesRight[0].y;
    //                }
    //                else
    //                {
    //                    k = (conesRight[endd].x - conesRight[0].x) /
    //                        (conesRight[endd].y - conesRight[0].y);
    //                    b = conesRight[0].x - k * conesRight[0].y;
    //                }
    //                int xxx = (int)(-b / k);
    //                xxx = clip(xxx, 0, IMAGE_WIDTH - 1);
    //                int yyy = int(k * (IMAGE_WIDTH - 1) + b);
    //                yyy = clip(yyy, 0, IMAGE_HEIGHT - 1);

    //                POINT startPoint = POINT(yyy, IMAGE_WIDTH - 1); // 补线起点
    //                POINT endPoint = POINT(0, xxx);                 // 补线终点
    //                POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5,
    //                    (startPoint.y + endPoint.y) * 0.5); // 补线中点

    //                vector<POINT> input = { startPoint, midPoint, endPoint };
    //                vector<POINT> repair = bezier(0.02, input);

    //                // 右边缘切行, 提升左拐能力
    //                if (repair.size() > 10)
    //                {
    //                    int index = repair.size() * 0.33f;
    //                    IMG.edge_right.clear();
    //                    for (size_t i = index; i < repair.size(); i++)
    //                    {
    //                        IMG.edge_right.push_back(repair[i]);
    //                    }
    //                }
    //                else
    //                    IMG.edge_right = repair;
    //                IMG.last_edge_right = IMG.edge_right;
    //            }
    //            else
    //            {
    //                IMG.edge_right = IMG.last_edge_right;
    //            }

    //            MAT_LOG("WORK_RUN_ING");

    //            // 积分延迟一定距离出施工区
    //            if ((cone_LU.x >= IMAGE_HEIGHT * 0.8)||(IMG.encoder.route - work_route >= IMG._config->work_rou_run))
    //            {
    //                work_route = IMG.encoder.route;
    //                flag_work = WORK_STOPPING;
    //                MAT_LOG("WORK_STOPPING");
    //            }
    //        }

    //        /* ***************************************************************** */

    //        if (flag_work == WORK_STOPPING)//刹车中
    //        {
    //            IMG.aim_speed = 0;
    //            if (IMG.encoder.speed < 0.1 && IMG.encoder.speed > -0.1)
    //            {
    //                flag_work = WORK_PARKING;
    //                MAT_LOG("WORK_STOPFINISH");//刹车完成
    //                stop_clock.tic();
    //            }
    //        }



    //        if (flag_work == WORK_PARKING)//泊车
    //        {
    //            IMG.aim_speed = 0;
    //            if (stop_clock.toc() > IMG._config->work_stop_time)
    //            {
    //                work_route = IMG.encoder.route;
    //                MAT_LOG("WORK_PARKINISH"); //泊车完成
    //                flag_work = WORK_OUT;
    //            }
    //        }

    //        if (flag_work == WORK_OUT)
    //        {
    //            MAT_LOG("WORK_OUT");
    //            //退出一阶段打角
    //            if (work_route - IMG.encoder.route <= IMG._config->work_recback_line_route)
    //            {
    //                MAT_LOG("WORK_LINE_OUT");
    //                IMG.aim_angle = -IMG._config->work_recback_line_angle; //出区 角度
    //                IMG.aim_speed = -IMG._config->work_recback_speed;//出去 距离
    //            }
    //            else if (IMG.rpts1s_num > 0.8 / SAMPLE_DIST && IMG.rpts0s_num > 0.8 / SAMPLE_DIST && work_route - IMG.encoder.route > IMG._config->work_recback_line_route)
    //            {
    //                IMG.flag_track = TRACK_RIGHT;
    //                control_flag.pop();//移除
    //                flag_work = WORK_NONE;
    //                MAT_LOG("WORK_LINE");
    //            }
    //            else if (work_route - IMG.encoder.route > IMG._config->work_recback_line_route && work_route - IMG.encoder.route <= IMG._config->work_recback_angle_route) //出去路程
    //            {
    //                MAT_LOG("WORK_OUTING");
    //                IMG.aim_angle = -IMG._config->work_recback_angle; //出区 角度
    //                IMG.aim_speed = -IMG._config->work_recback_speed;//出去 距离
    //            }
    //            else if (work_route - IMG.encoder.route > IMG._config->work_recback_angle_route)
    //            {
    //                control_flag.pop();//移除
    //                flag_work = WORK_NONE;
    //                MAT_LOG("WORK_NONE");
    //            }

    //        }
    //    }

    //    /* ***************************************************************** */
    //    /* ***************************************************************** */

    //    if (flag_work > WORK_FOUND)
    //    {
    //        IMG.det_ipts_num = 0;

    //        if (IMG.flag_track == TRACK_LEFT)
    //        {
    //            // 进行降序排序
    //            std::sort(IMG.edge_left.begin(), IMG.edge_left.end(), sort_x_down);
    //            for (size_t i = 0; i < IMG.edge_left.size(); i++)
    //            {
    //                if (i >= IMAGE_HEIGHT)
    //                    break;
    //                IMG.det_ipts[i][0] = IMG.edge_left[i].y;
    //                IMG.det_ipts[i][1] = IMG.edge_left[i].x;
    //                IMG.det_ipts_num++;
    //            }
    //        }
    //        else
    //        {
    //            // 进行降序排序
    //            std::sort(IMG.edge_right.begin(), IMG.edge_right.end(), sort_x_down);
    //            for (size_t i = 0; i < IMG.edge_right.size(); i++)
    //            {
    //                if (i >= IMAGE_HEIGHT)
    //                    break;
    //                IMG.det_ipts[i][0] = IMG.edge_right[i].y;
    //                IMG.det_ipts[i][1] = IMG.edge_right[i].x;
    //                IMG.det_ipts_num++;
    //            }
    //        }

    //        // 原图边线 -> 透视边线
    //        for (int i = 0; i < IMG.det_ipts_num; i++)
    //            map_perspective(IMG.det_ipts[i][0], IMG.det_ipts[i][1], IMG.det_rpts[i], 0);

    //        // 边线滤波
    //        blur_points(IMG.det_rpts, IMG.det_ipts_num, IMG.det_rptsb,
    //            (int)round(LINE_BLUR_KERNEL));

    //        // 边线等距采样
    //        IMG.det_rptss_num = sizeof(IMG.det_rptss) / sizeof(IMG.det_rptss[0]);
    //        resample_points(IMG.det_rptsb, IMG.det_ipts_num, IMG.det_rptss,
    //            &IMG.det_rptss_num, SAMPLE_DIST * PIXEL_PER_METER);
    //    }
    //}
}

/* ********************************************************************* */