#include "_garage.hpp"

#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"

#undef LOG_TAG
#define LOG_TAG "Garage"

flag_garage_e flag_garage = GARAGE_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
extern Control_Flag control_flag;

#define IMG imgprocess

// 变量定义
float garage_route = 0;   // 车库编码器积分
bool garage_stop = false; // 车库停车标志位
int garage_num = 0;       // 记录当前第几次车库
int none_left = 0, none_right = 0;
int have_left = 0, have_right = 0;

extern uint8_t gas_which;
CVFind_Clone garageB;
/* ********************************************************************* */

//bool pt_point_in_Line(int w) {
//
//
//}
int edge_lefty[200];
int edge_righty[200];
void check_garage()
{
    // 检测斑马线
    ///> 1、设置一个栈, 用来存储黑色元素
    ///> 2、从此行的第0个开始往左扫
    ///> 3、如果遇到黑色元素, 入栈
    ///> 4、如果遇到白色元素, 出栈, 同时统计栈中元素个数, 如果栈中元素为4—8个
    ///>    (也就是黑胶的宽度), black_blocks++, 否则不加. 然后将栈中元素清掉
    ///> 5、如果此行的black_blocks在6个左右, times++
    ///> 6、遍历18——23行, 如果times在4个左右, 则确定为起跑线。

    if (!IMG.element_over &&
        
        !control_flag.contains(work_element) &&         //当前没在工作区中
        !control_flag.contains(garage_element) &&       //当前没在出入库中
        !control_flag.contains(ramp_element) &&         //当前没在桥中
        !control_flag.contains(bomb_element) &&         //当前没在炸弹中
        !control_flag.contains(work_element) &&     //当前没在工作区中
        IMG.encoder.route > IMG._config->garage_out_route_  //出了车库之后
        ) {
        if (IMG._config->check_garage_AI == 0)      //不使用AI识别斑马线
        {
            int16_t black_blocks = 0;
            int16_t cursor = 0;
            int16_t times = 0;
            //garageB.init();
            //std::vector<cv::Rect> all_zt;
            //all_zt = garageB.search_all_cone_rect(IMG.mat_bin);

            //for (size_t i = 0; i < all_zt.size(); i++)
            //{
            //    
            //}

            sort_edge(IMG.edge_right, edge_righty, 200, 1);
            sort_edge(IMG.edge_left, edge_lefty, 200, 0);

            for (int16_t y = 40; y < 100; y++) {
                black_blocks = 0;
                cursor = 0; // 指向栈顶的游标

                if (edge_righty[y] != 0 && edge_lefty[y] != 0)
                {

                for (int16_t x = edge_lefty[y]; x < edge_righty[y]; x++) {
                    if (AT_IMAGE(IMG.mat_bin, x, y) < IMG._config->threshold) {
                        if (cursor < 20)
                            cursor++;
                    }
                    else {
                        if (cursor >= 4 && cursor <= 8)
                            black_blocks++;
                        cursor = 0;
                    }
                }
                if (black_blocks >= 5 && black_blocks <= 9)
                    times++;
                }
            }
            if (times >= 3) {
                if (control_flag.contains(cross_element)) {//如果在十字路口中则删除十字路口
                    control_flag.remove(cross_element);
                    flag_cross = CROSS_NONE;
                }
                if (control_flag.contains(circle_element)) {//如果在环岛中则删除环岛
                    control_flag.remove(circle_element);
                    flag_circle = CIRCLE_NONE;
                }
                if (control_flag.contains(work_element)) {//如果在环岛中则删除环岛
                    control_flag.remove(work_element);
                    flag_work = WORK_NONE;
                }
                control_flag.push(garage_element);
                flag_garage = GARAGE_DETECTION;
                MAT_LOG("GARAGE_DETECTION");
            }
        }
        else if (flag_garage == GARAGE_AI_DETECTION)    //如果AI识别到斑马线
        {
            if (control_flag.contains(cross_element)) {//如果在十字路口中则删除十字路口
                control_flag.remove(cross_element);
                flag_cross = CROSS_NONE;
            }
            if (control_flag.contains(circle_element)) {//如果在环岛中则删除环岛
                control_flag.remove(circle_element);
                flag_circle = CIRCLE_NONE;
            }
            if (control_flag.contains(work_element)) {//如果在环岛中则删除环岛
                control_flag.remove(work_element);
                flag_work = WORK_NONE;
            }
            control_flag.push(garage_element);
            flag_garage = GARAGE_DETECTION;
        }
    }

    /* ***************************************************************** */

    // 非出库, 找到斑马线
    if (flag_garage != GARAGE_OUT_LEFT && flag_garage != GARAGE_OUT_RIGHT &&
        flag_garage == GARAGE_DETECTION) {
        IMG.element_identify = 1;
        garage_route = IMG.encoder.route;

        // clear granary mark
        gas_which = 0;

        if (IMG._config->direction_garage == 0) { // 左库
            if (++garage_num >= IMG._config->check_garage_times_) {
                flag_garage = GARAGE_IN_LEFT; // 第二次入库
                MAT_LOG("GARAGE_IN_LEFT");
            }
            else {
                flag_garage = GARAGE_PASS_LEFT; // 第一次不入库
                MAT_LOG("GARAGE_PASS_LEFT");
            }
        }
        else if (IMG._config->direction_garage == 1) { // 右库
            if (++garage_num >= IMG._config->check_garage_times_) {
                flag_garage = GARAGE_IN_RIGHT; // 第二次入库
                MAT_LOG("GARAGE_IN_RIGHT");
            }
            else {
                flag_garage = GARAGE_PASS_RIGHT; // 第一次不入库
                MAT_LOG("GARAGE_PASS_RIGHT");
            }
        }
        else if (IMG._config->direction_garage == 2) { // 左库 循环
            flag_garage = GARAGE_PASS_LEFT;
            MAT_LOG("GARAGE_PASS_LEFT");
        }
        else if (IMG._config->direction_garage == 3) { // 右库 循环
            flag_garage = GARAGE_PASS_RIGHT;
            MAT_LOG("GARAGE_PASS_RIGHT");
        }
        else if (IMG._config->direction_garage == 4) { // 停车
            flag_garage = GARAGE_IN_MIDDLE; // 过斑马线停车
            MAT_LOG("GARAGE_IN_MIDDLE");
        }
    }
}

/* ********************************************************************* */

void run_garage()
{
    if (control_flag.top() == garage_element) {

        /* *** *** *** *** *** */
        switch (flag_garage) {
            case GARAGE_OUT_MIDDLE:
                IMG.aim_speed = IMG._config->speed_garage_out;
                if (IMG.encoder.route > IMG._config->garage_out_route_) {
                    IMG.element_over = true;
                    IMG.element_over_route = IMG.encoder.route;
                    control_flag.pop();
                    flag_garage = GARAGE_NONE;
                    MAT_LOG("GARAGE_NONE");
                }
                cv::line(IMG.mat_result, cv::Point(0, 0.5 * IMAGE_HEIGHT), cv::Point(IMAGE_WIDTH - 1, 0.5 * IMAGE_HEIGHT), cv::Scalar(255, 0, 0), 1);

                IMG.get_edge();//获取边线
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
                if (IMG.rpts_num > 0.8 / SAMPLE_DIST) {
                    IMG.calculate_angle();//计算角度
                }
                else {
                    IMG.aim_angle = IMG.aim_angle_last;
                }


                break;
            case GARAGE_IN_MIDDLE://过斑马线停车

                IMG.aim_speed = IMG._config->speed_garage_in;
                if (IMG.encoder.route - garage_route > IMG._config->garage_pass_route_) {
                    flag_garage = GARAGE_STOP;
                    MAT_LOG("GARAGE_STOP");
                }
                IMG.get_edge();//获取边线
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

                if (flag_garage == GARAGE_STOP)//停车
                {
                    IMG.aim_speed = 0;
                    IMG.aim_angle = 0;
                }
                break;
            case GARAGE_STOP://停车
                IMG.aim_speed = 0;
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
                if (IMG.rpts_num > 0.8 / SAMPLE_DIST) {
                    IMG.calculate_angle();//计算角度
                }
                else {
                    IMG.aim_angle = IMG.aim_angle_last;
                }
                if (IMG.encoder.speed < 0.1 && IMG.encoder.speed>-0.1)//停车了
                    IMG.aim_angle = 0;
                break;
            default:
                break;
        }


    }
}

/* ********************************************************************* */