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
        flag_yroad = YROAD_FOUND;
        MAT_LOG("Farmland_FOUND");
    }
}

/* ********************************************************************* */

void run_yroad() {
    if (flag_yroad == YROAD_FOUND) {
        // 清弯道防止加大 P 项, 提高稳定性
        IMG.is_curve0 = IMG.is_curve1 = false;
        //直走
        IMG.flag_track = TRACK_MIDDLE;
        // 找锥桶
        IMG.edge_det = search_targets(IMG._det_result);

        //找右锥桶
        yroad_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab);

        //减少找的线
        if(IMG.edge_left.size() > 50 ){
            IMG.edge_left.resize(IMG.edge_left.size() * 0.7) ;
        }
        if(IMG.edge_right.size() > 50 ){
            IMG.edge_right.resize(IMG.edge_right.size() * 0.7);
        }

        // 进入农场的条件，锥桶靠近，或者左右线很小
        if (yroad_near_cone.x > IMAGE_HEIGHT*0.5||(IMG.edge_left.size()<IMAGE_HEIGHT/2&&IMG.edge_right.size()<IMAGE_HEIGHT/2)) {
            yroad_route = IMG.encoder.route;
            flag_yroad = YROAD_IN;
            MAT_LOG("Farmland_IN");
        }
    }

    /* ***************************************************************** */

    if (flag_yroad == YROAD_IN) {

        bool refreshed = true;
        if (IMG.encoder.route - yroad_route > 50) {
            //路程积分或识别到边线或没有识别到锥桶
            yroad_route = IMG.encoder.route;
            flag_yroad = YROAD_RUN;
            MAT_LOG("Farmland_RUN");
        } else{
            // 清弯道防止加大 P 项, 提高稳定性
            IMG.is_curve0 = IMG.is_curve1 = false;

            IMG.edge_det = search_targets(IMG._det_result);
            IMG.cones_edge_left.clear();
            IMG.cones_edge_right.clear();

            int break_left = search_break_left(IMG.edge_left,IMG.mat_lab);
            int break_right = search_break_right(IMG.edge_right,IMG.mat_lab);

            //边线还在
            if(break_left > 25&&break_right > 25){
                IMG.cones_edge_left.push_back(IMG.edge_left[break_left]);
                IMG.edge_left.resize(break_left);
                IMG.cones_edge_right.push_back(IMG.edge_right[break_right]);
                IMG.edge_left.resize(break_right);
            } else {//搜索左右锥桶
                IMG.cones_edge_left.push_back(POINT(0,0));
                IMG.cones_edge_right.push_back(POINT(0,0));
                for (int i= 0;i < IMG.edge_det.size();i++){
                    //左边底部锥桶
                    if (IMG.edge_det[i].y < IMAGE_WIDTH/2&&IMG.edge_det[i].x>0){
                        IMG.cones_edge_left[0] = IMG.edge_det[i];
                    }
                    //右边底部锥桶
                    if (IMG.edge_det[i].y > IMAGE_WIDTH/2&&IMG.edge_det[i].x>0){
                        IMG.cones_edge_right[0] = IMG.edge_det[i];
                    }
                }

                //判断是否是单边,清左右边，
                if(abs(IMG.cones_edge_right[0].y-IMG.cones_edge_left[0].y )> IMAGE_WIDTH/3 && abs(IMG.cones_edge_right[0].x-IMG.cones_edge_left[0].x)>IMAGE_HEIGHT/3){
                    if(IMG.cones_edge_left[0].x > IMG.cones_edge_right[0].x){
                        IMG.cones_edge_right.clear();
                    }else if(IMG.cones_edge_left[0].x < IMG.cones_edge_right[0].x){
                        IMG.cones_edge_left.clear();
                    }
                }

                //搜左右边缘，在以划分的点加左右边缘点
                search_cones_edge(IMG.edge_det,&IMG.cones_edge_left,&IMG.cones_edge_right);

                if(IMG.cones_edge_left.size() >= IMG.cones_edge_right.size() &&IMG.cones_edge_left.size() > 2){
                    //补左线
                    POINT startPoint = IMG.cones_edge_left[0];
                    POINT midPoint = IMG.cones_edge_left[IMG.cones_edge_left.size() / 2];
                    POINT endPoint = IMG.cones_edge_left[IMG.cones_edge_left.size() - 1];
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.05, input);
                    IMG.edge_left.clear();
                    IMG.edge_left = repair;
                }else if(IMG.cones_edge_left.size() < IMG.cones_edge_right.size() && IMG.cones_edge_right.size() >= 2){
                    //补右线
                    POINT startPoint = IMG.cones_edge_right[0];
                    POINT midPoint = IMG.cones_edge_right[IMG.cones_edge_right.size() / 2];
                    POINT endPoint = IMG.cones_edge_right[IMG.cones_edge_right.size() - 1];
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = bezier(0.05, input);
                    IMG.edge_right.clear();
                    IMG.edge_right = repair;
                } else{
                    IMG.edge_left = IMG.last_cones_edge_left;
                    IMG.edge_right = IMG.last_cones_edge_right;
                    refreshed = false;
                    MAT_LOG("UN_FARM_REFRESHED");
                }

                if(refreshed){//刷新
                    MAT_LOG("FARM_REFRESHED");
                }

                IMG.last_cones_edge_left = IMG.edge_left;
                IMG.last_cones_edge_right = IMG.edge_right;
            }

        }
    }

    /* ***************************************************************** */
    if(flag_yroad ==YROAD_RUN){
        if(IMG.encoder.route - yroad_route > 100||(IMG.rpts0s_num > 0.2 / SAMPLE_DIST)||(IMG.rpts1s_num > 0.2 / SAMPLE_DIST)||IMG.edge_det.size()<1){
            //路程积分或识别到边线或没有识别到锥桶
            yroad_out_num = 0;
            IMG.element_over = true;
            IMG.element_over_route = IMG.encoder.route + 20;
            flag_yroad = YROAD_NONE;
            MAT_LOG("Farmland_OUT");
        }else {
            /*bool refreshed = true;
            // 清弯道防止加大 P 项, 提高稳定性
            IMG.is_curve0 = IMG.is_curve1 = false;

            IMG.edge_det = search_targets(IMG._det_result);
            IMG.cones_edge_left.clear();
            IMG.cones_edge_right.clear();

            int break_left = search_break_left(IMG.edge_left,IMG.mat_lab);
            int break_right = search_break_right(IMG.edge_right,IMG.mat_lab);

             {//搜索左右锥桶
                IMG.cones_edge_left.push_back(POINT(0, 0));
                IMG.cones_edge_right.push_back(POINT(0, 0));
                for (int i = 0; i < IMG.edge_det.size(); i++) {
                    //左边底部锥桶
                    if (IMG.edge_det[i].y < IMAGE_WIDTH / 2 && IMG.edge_det[i].x > 0) {
                        IMG.cones_edge_left[0] = IMG.edge_det[i];
                    }
                    //右边底部锥桶
                    if (IMG.edge_det[i].y > IMAGE_WIDTH / 2 && IMG.edge_det[i].x > 0) {
                        IMG.cones_edge_right[0] = IMG.edge_det[i];
                    }
                }

                    //判断是否是单边,清左右边，
                    if (abs(IMG.cones_edge_right[0].y - IMG.cones_edge_left[0].y) > IMAGE_WIDTH / 3 &&
                        abs(IMG.cones_edge_right[0].x - IMG.cones_edge_left[0].x) > IMAGE_HEIGHT / 3) {
                        if (IMG.cones_edge_left[0].x > IMG.cones_edge_right[0].x) {
                            IMG.cones_edge_right.clear();
                        } else if (IMG.cones_edge_left[0].x < IMG.cones_edge_right[0].x) {
                            IMG.cones_edge_left.clear();
                        }
                    }
                    //搜左右边缘，在以划分的点加左右边缘点
                    search_cones_edge(IMG.edge_det, &IMG.cones_edge_left, &IMG.cones_edge_right);

                    if (IMG.cones_edge_left.size() >= IMG.cones_edge_right.size() && IMG.cones_edge_left.size() > 2) {
                        //补左线
                        POINT startPoint = IMG.cones_edge_left[0];
                        POINT midPoint = IMG.cones_edge_left[IMG.cones_edge_left.size() / 2];
                        POINT endPoint = IMG.cones_edge_left[IMG.cones_edge_left.size() - 1];
                        vector <POINT> input = {startPoint, midPoint, endPoint};
                        vector <POINT> repair = bezier(0.05, input);
                        IMG.edge_left.clear();
                        IMG.edge_left = repair;
                    } else if (IMG.cones_edge_left.size() < IMG.cones_edge_right.size() &&
                               IMG.cones_edge_right.size() >= 2) {
                        //补右线
                        POINT startPoint = IMG.cones_edge_right[0];
                        POINT midPoint = IMG.cones_edge_right[IMG.cones_edge_right.size() / 2];
                        POINT endPoint = IMG.cones_edge_right[IMG.cones_edge_right.size() - 1];
                        vector <POINT> input = {startPoint, midPoint, endPoint};
                        vector <POINT> repair = bezier(0.05, input);
                        IMG.edge_right.clear();
                        IMG.edge_right = repair;
                    } else {
                        IMG.edge_left = IMG.last_cones_edge_left;
                        IMG.edge_right = IMG.last_cones_edge_right;
                        refreshed = false;
                        MAT_LOG("UN_FARM_REFRESHED2");
                    }

                    if (refreshed) {//刷新
                        MAT_LOG("FARM_REFRESHED2");
                    }

                    IMG.last_cones_edge_left = IMG.edge_left;
                    IMG.last_cones_edge_right = IMG.edge_right;
                }*/

            }
        }



    /* ***************************************************************** */

    if (flag_yroad == YROAD_OUT) {
        int yroad_x = 0;
        int yroad_y = BEGIN_Y;
        int line_counter = 0;

        if (IMG._config->direction_yroad == 1) {
            yroad_x = BLOCK_SIZE / 2;
            IMG.flag_track = TRACK_LEFT;
        } else {
            yroad_x = IMAGE_WIDTH - BLOCK_SIZE / 2 - 1;
            IMG.flag_track = TRACK_RIGHT;
        }

        // 向上黑白点
        bool white_found = false;
        bool black_found = false;
        for (int yyy = IMAGE_HEIGHT - 1; yyy >= 25; yyy--) {
            if (AT_IMAGE(IMG.mat_bin, yroad_x, yyy) >= IMG._config->threshold)
                white_found = true;

            if (AT_IMAGE(IMG.mat_bin, yroad_x, yyy) < IMG._config->threshold &&
                white_found) {
                black_found = true;
                yroad_y = yyy;
                break;
            }
        }

        // 搜索边界
        yroad_ipts_num = sizeof(yroad_ipts) / sizeof(yroad_ipts[0]);
        if (AT_IMAGE(IMG.mat_bin, yroad_x, yroad_y + 1) >= IMG._config->threshold &&
            black_found) {
            if (IMG._config->direction_yroad == 1) {
                findline_lefthand_adaptive(IMG.mat_bin, BLOCK_SIZE, CLIP_VALUE,
                                           yroad_x, yroad_y + 1, yroad_ipts,
                                           &yroad_ipts_num);
            } else {
                findline_righthand_adaptive(IMG.mat_bin, BLOCK_SIZE, CLIP_VALUE,
                                            yroad_x, yroad_y + 1, yroad_ipts,
                                            &yroad_ipts_num);
            }
        } else
            yroad_ipts_num = 0;

        // 原图边线 -> 透视边线
        for (int i = 0; i < yroad_ipts_num; i++) {
            if (yroad_ipts[i][1] < 25)
                break;
            map_perspective(yroad_ipts[i][0], yroad_ipts[i][1], yroad_rpts[i], 0);
            line_counter++;
        }

        // 边线滤波
        blur_points(yroad_rpts, line_counter, yroad_rptsb,
                    (int)round(LINE_BLUR_KERNEL));

        // 边线等距采样
        yroad_rptss_num = sizeof(yroad_rptss) / sizeof(yroad_rptss[0]);
        resample_points(yroad_rptsb, line_counter, yroad_rptss, &yroad_rptss_num,
                        SAMPLE_DIST * PIXEL_PER_METER);

        // 边线局部角度变化率
        local_angle_points(yroad_rptss, yroad_rptss_num, yroad_rptsa,
                           (int)round(ANGLE_DIST / SAMPLE_DIST));

        // 角度变化率非极大抑制
        nms_angle(yroad_rptsa, yroad_rptss_num, yroad_rptsan,
                  (int)round(ANGLE_DIST / SAMPLE_DIST) * 2 + 1);

        // 泛行区角点识别
        yroad_Lpt_id = 0;
        yroad_Lpt_found = false;
        for (int i = 0; i < AT_MIN(yroad_rptss_num, 180); i++) {
            if (yroad_rptsan[i] == 0)
                continue;
            int im1 = clip(i - (int)round(ANGLE_DIST / SAMPLE_DIST), 0,
                           yroad_rptss_num - 1);
            int ip1 = clip(i + (int)round(ANGLE_DIST / SAMPLE_DIST), 0,
                           yroad_rptss_num - 1);
            float conf = fabs(yroad_rptsa[i]) -
                         (fabs(yroad_rptsa[im1]) + fabs(yroad_rptsa[ip1])) / 2;

            if (20. / 180. * PI < conf && conf < 160. / 180. * PI) {
                float trans[2];
                map_perspective(yroad_rptss[i][0], yroad_rptss[i][1], trans, 1);
                if (trans[1] < 30)
                    break;

                yroad_Lpt_id = i;
                yroad_Lpt_found = true;
                break;
            }
        }

        if ((!IMG.Lpt0_found && !IMG.Lpt1_found &&
             IMG.rpts0s_num > 0.6 / SAMPLE_DIST && IMG.rpts1s_num > 0.6 / SAMPLE_DIST) ||
            (yroad_rptss_num < 0.2 / SAMPLE_DIST)) {
            yroad_out_num++;
        }

        if (yroad_out_num > 2 || IMG.encoder.route - yroad_route > 150) {
            yroad_out_num = 0;
            IMG.element_over = true;
            IMG.element_over_route = IMG.encoder.route + 20;
            flag_yroad = YROAD_NONE;
            MAT_LOG("YROAD_NONE");
        }
    }
}

/* ********************************************************************* */