#include "_yroad.hpp"

#include "../_define.hpp"
#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"

#undef LOG_TAG
#define LOG_TAG "Yroad"

flag_yroad_e flag_yroad = YROAD_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
#define IMG imgprocess

// 变量定义
float yroad_route = 0;  // 泛行区编码器积分
int yroad_out_num = 0;  // 泛行区结束计次
POINT yroad_near_cone;  // 左右较近的锥桶
uint8_t last_flag_track;
// 泛行区角点
int yroad_Lpt_id;      // 泛行区角点位置
bool yroad_Lpt_found;  // 泛行区角点检测标志

// 泛行区边线
int yroad_ipts[IMAGE_HEIGHT][2];
float yroad_rpts[IMAGE_HEIGHT][2];
float yroad_rptsb[IMAGE_HEIGHT][2];
float yroad_rptss[IMAGE_HEIGHT][2];
float yroad_rptsa[IMAGE_HEIGHT];
float yroad_rptsan[IMAGE_HEIGHT];
int yroad_ipts_num, yroad_rptss_num;

/* ********************************************************************* */

void check_yroad() {
    if (flag_yroad == YROAD_DETECTION) {
        IMG.element_identify = 1;
        IMG.element_over = false;
        yroad_route = IMG.encoder.route;
        flag_yroad = YROAD_FOUND;
        MAT_LOG("Farmland_FOUND");
    }
}

/* ********************************************************************* */

void run_yroad() {
    if (flag_yroad == YROAD_FOUND) {
        // 清弯道防止加大 P 项, 提高稳定性
        IMG.is_curve0 = IMG.is_curve1 = false;
        // 直走
        //IMG.flag_track = TRACK_MIDDLE;
        // 找锥桶
        IMG.edge_det = search_targets(IMG._det_result);

        // 找左锥桶
        yroad_near_cone = search_left_near2(IMG.edge_left, IMG.mat_lab);

        // 减少找的线
        if (IMG.edge_left.size() > 50) {
            IMG.edge_left.resize(IMG.edge_left.size() * 0.5);
        }
        if (IMG.edge_right.size() > 50) {
            IMG.edge_right.resize(IMG.edge_right.size() * 0.5);
        }

        // 进入农场的条件，锥桶靠近，或者左右线很小,或走过一段路
        if (IMG.encoder.route - yroad_route > IMG._config->farm_rou_ready) {  // 除以二为锥桶图像位置
            yroad_route = IMG.encoder.route;
            flag_yroad = YROAD_IN;
            MAT_LOG("Farmland_IN");
        }
    }

    /* ***************************************************************** */

    if (flag_yroad == YROAD_IN) {
        bool refreshed = true;
        if (IMG.encoder.route - yroad_route > IMG._config->farm_rou_in) {
            // 路程积分或识别到边线或没有识别到锥桶
            // MAT_LOG("question1");
            yroad_route = IMG.encoder.route;
            flag_yroad = YROAD_RUN;
            MAT_LOG("Farmland_RUN");
        } else {
            // MAT_LOG("question2");
            //  清弯道防止加大 P 项, 提高稳定性
            IMG.is_curve0 = IMG.is_curve1 = false;

            IMG.edge_det = search_targets(IMG._det_result);
            IMG.cones_edge_left.clear();
            IMG.cones_edge_right.clear();

            int break_left = search_break_left(IMG.edge_left, IMG.mat_lab);
            int break_right = search_break_right(IMG.edge_right, IMG.mat_lab);
            // MAT_LOG("question3");
            //  边线还在
            if (break_left > 100 && break_right > 100) {
                IMG.cones_edge_left.push_back(IMG.edge_left[break_left]);
                IMG.edge_left.resize(break_left);
                IMG.cones_edge_right.push_back(IMG.edge_right[break_right]);
                IMG.edge_left.resize(break_right);
                MAT_LOG("edge_exist1")
            } else {  // 搜索左右锥桶
                // MAT_LOG("question4");
                std::sort(IMG.edge_det.begin(), IMG.edge_det.end(), sort_x_up);  // 纵坐标升序排序

                // MAT_LOG("question5");
                /*
                // 判断是否是单边,清左右边
                if (abs(IMG.cones_edge_right[0].y - IMG.cones_edge_left[0].y) > IMAGE_WIDTH / 3 && abs(IMG.cones_edge_right[0].x - IMG.cones_edge_left[0].x) > IMAGE_HEIGHT / 3) {
                    if (IMG.cones_edge_left[0].x > IMG.cones_edge_right[0].x) {
                        // IMG.cones_edge_right.clear();
                        MAT_LOG("clear_right");
                    } else if (IMG.cones_edge_left[0].x < IMG.cones_edge_right[0].x) {
                        // IMG.cones_edge_left.clear();
                        MAT_LOG("clear_left");
                    }
                }
                */
                IMG.edge_right.clear();
                IMG.edge_left.clear();
                // 搜左右边缘，在以划分的点加左右边缘点
                std::sort(IMG.edge_det.begin(), IMG.edge_det.end(), sort_x_up);  // 纵坐标升序排序
                search_cones_edge(IMG.edge_det, &IMG.cones_edge_left, &IMG.cones_edge_right);
                if (IMG.cones_edge_left.size() >= 1 && IMG.cones_edge_left.size() < 3) {
                    POINT startPoint = POINT(IMAGE_HEIGHT - 1, 0);//图像左下角点
                    POINT endPoint = IMG.cones_edge_left[1];
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);  // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.02, input);
                    IMG.edge_left = repair;
                } else if (IMG.cones_edge_left.size() >= 3) {
                    POINT startPoint = POINT(IMAGE_HEIGHT - 1, 0);
                    POINT endPoint = IMG.cones_edge_left[2];
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);  // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.02, input);
                    IMG.edge_left = repair;
                }
                if (IMG.cones_edge_right.size() >= 1 && IMG.cones_edge_right.size() < 3) {
                    POINT startPoint = POINT(IMAGE_HEIGHT - 1, IMAGE_WIDTH-1);//图像右下角点
                    POINT endPoint = IMG.cones_edge_right[1];
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);  // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.02, input);
                    IMG.edge_right = repair;
                } else if (IMG.cones_edge_right.size() >= 3) {
                    POINT startPoint = IMG.cones_edge_right[0];
                    POINT endPoint = IMG.cones_edge_right[2];
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);  // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.02, input);
                    IMG.edge_right = repair;
                } else {
                    // IMG.cones_edge_left = IMG.last_cones_edge_left;
                    // IMG.cones_edge_right = IMG.last_cones_edge_right;
                    refreshed = false;
                    MAT_LOG("UN_FARM_REFRESHED_IN");
                }

                if (refreshed) {  // 刷新
                    MAT_LOG("FARM_REFRESHED_IN");
                }

                // IMG.last_cones_edge_left = IMG.cones_edge_left;
                // IMG.last_cones_edge_right = IMG.cones_edge_right;
            }
        }
    }

    /* ***************************************************************** */
    if (flag_yroad == YROAD_RUN) {
        if (((IMG.encoder.route - yroad_route > IMG._config->farm_rou_run && (IMG.rpts0s_num > 0.2 / SAMPLE_DIST)) || ((IMG.rpts1s_num > 0.2 / SAMPLE_DIST)) && IMG.edge_det.size() < 1)) {
            // 路程积分或识别到边线或没有识别到锥桶
            yroad_out_num = 0;
            IMG.element_over = true;
            IMG.element_over_route = IMG.encoder.route;
            flag_yroad = YROAD_OUT;
            MAT_LOG("Farmland_OUT");
        } else {
            bool refreshed = true;
            // 清弯道防止加大 P 项, 提高稳定性
            IMG.is_curve0 = IMG.is_curve1 = false;

            IMG.cones_edge_left.clear();
            IMG.cones_edge_right.clear();
            IMG.edge_det = search_targets(IMG._det_result);

            /*IMG.cones_edge_left = search_left_cones(IMG.edge_det);
            IMG.cones_edge_right = search_right_cones(IMG.edge_det);*/

            {  // 搜索左右锥桶
                // IMG.cones_edge_left[0] = (POINT(0, 50));
                // IMG.cones_edge_right[0] = (POINT(0, IMAGE_WIDTH - 50));
                std::sort(IMG.edge_det.begin(), IMG.edge_det.end(), sort_x_up);  // 纵坐标升序排序
                // for (int i = IMG.edge_det.size() - 1; i >= 0; i--)
                {
                    // 左边底部锥桶
                    /*
                    if (IMG.edge_det[i].y < IMAGE_WIDTH / 2 && IMG.edge_det[i].x > IMAGE_HEIGHT * (1 / 2))
                    {
                        if (IMG.cones_edge_left.size() == 0)
                        {
                            IMG.cones_edge_left.push_back(IMG.edge_det[i]);
                        }
                        else if (IMG.edge_det[i].x > IMG.cones_edge_left[0].x)
                        {
                            IMG.cones_edge_left[0] = IMG.edge_det[i];
                        }
                    }
                    /*
                    else
                    {
                        POINT startPoint_left = POINT(IMAGE_HEIGHT - 1, 0);
                        IMG.cones_edge_left.push_back(startPoint_left);
                    }

                    // 右边底部锥桶
                    if (IMG.edge_det[i].y > (IMAGE_WIDTH / 2) && IMG.edge_det[i].x > IMAGE_HEIGHT * (1 / 2))
                    {
                        if (IMG.cones_edge_right.size() == 0)
                        {
                            IMG.cones_edge_right.push_back(IMG.edge_det[i]);
                        }
                        else if (IMG.edge_det[i].x > IMG.cones_edge_right[0].x)
                        {
                            IMG.cones_edge_right[0] = IMG.edge_det[i];
                        }
                    }
                    /*
                    else
                    {
                        POINT startPoint_right = POINT(IMAGE_HEIGHT - 1, IMAGE_WIDTH - 1);
                        IMG.cones_edge_right.push_back(startPoint_right);
                    }
                     */
                }

                MAT_LOG("search_cones_edges");
                /*//判断是否是单边,清左右边
                if(abs(IMG.cones_edge_right[0].y-IMG.cones_edge_left[0].y )> IMAGE_WIDTH/3 && abs(IMG.cones_edge_right[0].x-IMG.cones_edge_left[0].x)>IMAGE_HEIGHT/3){
                    if(IMG.cones_edge_left[0].x > IMG.cones_edge_right[0].x){
                        //IMG.cones_edge_right.clear();
                        MAT_LOG("clear_right");
                    }else if(IMG.cones_edge_left[0].x < IMG.cones_edge_right[0].x){
                        //IMG.cones_edge_left.clear();
                        MAT_LOG("clear_left");
                    }
                }*/
                // 搜左右边缘，在以划分的点加左右边缘点
                IMG.edge_right.clear();
                IMG.edge_left.clear();

                // 搜左右边缘，在以划分的点加左右边缘点
                std::sort(IMG.edge_det.begin(), IMG.edge_det.end(), sort_x_up);  // 纵坐标升序排序
                search_cones_edge(IMG.edge_det, &IMG.cones_edge_left, &IMG.cones_edge_right);
                /*
                 if(IMG.cones_edge_left.size()==0){
                     MAT_LOG("LEFT==0");
                     IMG.cones_edge_left = search_left_cones(IMG.edge_det);
                 }
                 if(IMG.cones_edge_right.size()==0){
                     MAT_LOG("RIGHT==0");
                     IMG.cones_edge_right = search_right_cones(IMG.edge_det);
                 }
                */
                if (IMG.cones_edge_left.size() >= 1 && IMG.cones_edge_left.size() < 3) {
                    POINT startPoint = POINT(IMAGE_HEIGHT - 1, 0);
                    POINT endPoint = IMG.cones_edge_left[1];
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);  // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.02, input);
                    IMG.edge_left = repair;
                } else if (IMG.cones_edge_left.size() >= 3) {
                    POINT startPoint = POINT(IMAGE_HEIGHT - 1, 0);
                    POINT endPoint = IMG.cones_edge_left[2];
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);  // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.02, input);
                    IMG.edge_left = repair;
                }
                if (IMG.cones_edge_right.size() >= 1 && IMG.cones_edge_right.size() < 3) {
                    POINT startPoint = POINT(IMAGE_HEIGHT - 1, IMAGE_WIDTH - 1);
                    POINT endPoint = IMG.cones_edge_right[1];
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);  // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.02, input);
                    IMG.edge_right = repair;
                } else if (IMG.cones_edge_right.size() >= 3) {
                    POINT startPoint = POINT(IMAGE_HEIGHT - 1, IMAGE_WIDTH - 1);
                    POINT endPoint = IMG.cones_edge_right[2];
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);  // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.02, input);
                    IMG.edge_right = repair;
                }
                if (IMG.cones_edge_left.size() > IMG.cones_edge_right.size()) {
                    IMG.flag_track = TRACK_LEFT;
                    last_flag_track = 0;
                } else if (IMG.cones_edge_left.size() < IMG.cones_edge_right.size()) {
                    IMG.flag_track = TRACK_RIGHT;
                    last_flag_track =1;
                } else if (IMG.cones_edge_left.size() == 0 && IMG.cones_edge_right.size() == 0) {
                    /*IMG.edge_left = IMG.last_cones_edge_left;
                    IMG.edge_right = IMG.last_cones_edge_right;*/

                    refreshed = false;
                    MAT_LOG("LAST_CONES");
                } else {
                    if(last_flag_track==0){
                        IMG.flag_track = TRACK_LEFT;
                    } else{
                        IMG.flag_track = TRACK_RIGHT;
                    }//用上一次的置信
                    //IMG.flag_track = TRACK_LEFT;
                    MAT_LOG("TRACK_LEFT LEFT OR RIGHT 0");
                }
                /*if (IMG.cones_edge_left.size() >= IMG.cones_edge_right.size() && IMG.cones_edge_left.size() >= 2) {
                    //补左线
                    IMG.flag_track = TRACK_LEFT;
                    POINT startPoint = POINT(IMAGE_HEIGHT-1,0);
                    POINT endPoint = IMG.cones_edge_left[IMG.cones_edge_left.size() /2];
                    POINT midPoint = POINT((startPoint.x+endPoint.x)*0.4,(startPoint.y+endPoint.y)*0.6);      // 补线中点
                    vector <POINT> input = {startPoint, midPoint, endPoint};
                    vector <POINT> repair = bezier(0.05, input);
                    IMG.edge_left.clear();
                    IMG.edge_left = repair;
                } else if (IMG.cones_edge_left.size() < IMG.cones_edge_right.size() &&
                           IMG.cones_edge_right.size() >= 2) {
                    //补右线
                    IMG.flag_track = TRACK_RIGHT;
                    POINT startPoint = POINT(IMAGE_HEIGHT-1,IMAGE_WIDTH);
                    POINT endPoint = IMG.cones_edge_right[IMG.cones_edge_right.size() /2];
                    POINT midPoint = POINT((startPoint.x+endPoint.x)*0.4,(startPoint.y+endPoint.y)*0.6);      // 补线中点
                    vector <POINT> input = {startPoint, midPoint, endPoint};
                    vector <POINT> repair = bezier(0.05, input);
                    IMG.edge_right.clear();
                    IMG.edge_right = repair;
                } else {
                    IMG.edge_left = IMG.last_cones_edge_left;
                    IMG.edge_right = IMG.last_cones_edge_right;
                    refreshed = false;
                    MAT_LOG("UN_FARM_REFRESHED2");
                }*/

                if (refreshed) {  // 刷新
                    MAT_LOG("FARM_REFRESHED2");
                }

                IMG.last_cones_edge_left = IMG.cones_edge_left;
                IMG.last_cones_edge_right = IMG.cones_edge_right;
            }
        }
    }

    /* ***************************************************************** */

    if (flag_yroad == YROAD_OUT) {
        flag_yroad = YROAD_NONE;
        MAT_LOG("FARM_OUT_FINISH");
    }

    /* ***************************************************************** */

    if (flag_yroad > YROAD_FOUND) {
        IMG.det_ipts_num = 0;

        if (IMG.flag_track == TRACK_LEFT) {
            // 进行降序排序
            std::sort(IMG.edge_left.begin(), IMG.edge_left.end(), sort_x_down);
            for (size_t i = 0; i < IMG.edge_left.size(); i++) {
                if (i >= IMAGE_HEIGHT)
                    break;
                IMG.det_ipts[i][0] = IMG.edge_left[i].y;
                IMG.det_ipts[i][1] = IMG.edge_left[i].x;
                IMG.det_ipts_num++;
            }
        } else {
            // 进行降序排序
            std::sort(IMG.edge_right.begin(), IMG.edge_right.end(), sort_x_down);
            for (size_t i = 0; i < IMG.edge_right.size(); i++) {
                if (i >= IMAGE_HEIGHT)
                    break;
                IMG.det_ipts[i][0] = IMG.edge_right[i].y;
                IMG.det_ipts[i][1] = IMG.edge_right[i].x;
                IMG.det_ipts_num++;
            }
        }

        // 原图边线 -> 透视边线
        for (int i = 0; i < IMG.det_ipts_num; i++)
            map_perspective(IMG.det_ipts[i][0], IMG.det_ipts[i][1], IMG.det_rpts[i], 0);

        // 边线滤波
        blur_points(IMG.det_rpts, IMG.det_ipts_num, IMG.det_rptsb,
                    (int)round(LINE_BLUR_KERNEL));

        // 边线等距采样
        IMG.det_rptss_num = sizeof(IMG.det_rptss) / sizeof(IMG.det_rptss[0]);
        resample_points(IMG.det_rptsb, IMG.det_ipts_num, IMG.det_rptss,
                        &IMG.det_rptss_num, SAMPLE_DIST * PIXEL_PER_METER);
    }
}

/* ********************************************************************* */