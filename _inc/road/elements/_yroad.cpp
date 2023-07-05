#include "_yroad.hpp"

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
POINT yroad_ld_near_cone;  // 左下方边缘最近的锥桶
POINT yroad_rd_near_cone;  // 右下方边缘最近的锥桶
POINT yroad_lu_near_cone;  // 左上方边缘最近的锥桶
POINT yroad_ru_near_cone;  // 右上方边缘最近的锥桶

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

        // 找锥桶
        IMG.edge_det = search_targets(IMG._det_result);

        // 进入方向，以左锥桶为点或是右锥桶
        /*if (IMG._config->direction_yroad == 0)
            work_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab);
        else
            work_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);*/
        // 丢线, 进入泛行区巡线
        if (IMG.rpts0s_num < 0.2 / SAMPLE_DIST &&
            IMG.rpts1s_num < 0.2 / SAMPLE_DIST) {
            yroad_route = IMG.encoder.route;
            flag_yroad = YROAD_IN;
            MAT_LOG("Farmland_IN");
        }
    }

    /* ***************************************************************** */

    if (flag_yroad == YROAD_IN) {
        if (IMG.encoder.route - yroad_route > 180&&IMG.rpts0s_num > 0.2 / SAMPLE_DIST) {
            //路程积分和识别到双线
            yroad_route = IMG.encoder.route;
            flag_yroad = YROAD_OUT;
            MAT_LOG("Farmland_OUT");
        } else{
            // 清弯道防止加大 P 项, 提高稳定性
            IMG.is_curve0 = IMG.is_curve1 = false;

            // 正入口 巡中线
            IMG.flag_track = TRACK_MIDDLE;

            IMG.edge_det = search_targets(IMG._det_result);

            // 找目标点
            yroad_ld_near_cone = search_left_near(IMG.edge_left, IMG.mat_lab);
            yroad_rd_near_cone = search_right_near(IMG.edge_right, IMG.mat_lab);
            yroad_lu_near_cone = search_leftUp_cone(IMG.edge_det);
            yroad_ru_near_cone = search_rightDown_cone(IMG.edge_det);

            //底端看到两个锥桶
            if (yroad_ld_near_cone.x > 0 && yroad_rd_near_cone.x > 0 ){
                //补线起点,中点和终点
                POINT startPoint = POINT((yroad_ld_near_cone.x + yroad_rd_near_cone.x)/2,
                                         (yroad_ld_near_cone.y + yroad_rd_near_cone.y)/2);
                POINT middlePoint = POINT((yroad_lu_near_cone.x + yroad_ru_near_cone.x)/2,
                                         (yroad_lu_near_cone.y + yroad_ru_near_cone.y)/2);
                double k = 0;
                k = (float)(middlePoint.y - startPoint.y)/(float)(middlePoint.x-startPoint.x);
                MAT_LOG("line_k:%f",k);
                POINT endPoint;

                int x1_x2;
                int x2_x1;
                int y1_y2;
                int y2_y1;
                int x1sumx2;
                int y1sumy2;
                x1_x2 =  startPoint.x - middlePoint.x;
                x2_x1 =  middlePoint.x - startPoint.x;
                y1_y2 =  startPoint.y - middlePoint.y;
                y2_y1 =  middlePoint.y - startPoint.y;
                x1sumx2 =  startPoint.x + middlePoint.x;
                y1sumy2 =  startPoint.y + middlePoint.y;
                if(k < 0){
                    //预判点在左边，计算左边远处的点，根据两倍的角度计算
                    if((((1+sqrt(2))*x1_x2) < IMAGE_WIDTH/2) && ((2*middlePoint.y)<IMAGE_HEIGHT)){
                        //左边的点
                        endPoint = POINT((middlePoint.x - (int)((1+sqrt(2))*x1_x2)),(2*middlePoint.y));
                    }
                    else if((((1+sqrt(2))*(x1_x2)) > IMAGE_WIDTH/2) && ((y2_y1)<IMAGE_HEIGHT)){
                        //左边点在图像左边以外
                        endPoint = POINT(0,middlePoint.y+middlePoint.y);
                    }
                    else{
                        //直走
                        endPoint = POINT(IMAGE_WIDTH/2,IMAGE_HEIGHT/2);
                    }
                }
                else if(k>0){
                    //预判点在右边，计算右边远处的点，根据两倍的角度计算
                    if((((1+sqrt(2))*x2_x1) < IMAGE_WIDTH/2) && ((2*middlePoint.y)<IMAGE_HEIGHT)){
                        //右边的点
                        endPoint = POINT((middlePoint.x + (int)((1+sqrt(2))*x2_x1)),(2*middlePoint.y));
                    }
                    else if((((1+sqrt(2))*(x1_x2)) > IMAGE_WIDTH/2) && ((y2_y1)<IMAGE_HEIGHT)){
                        //左边点在图像左边以外
                        endPoint = POINT(IMAGE_WIDTH,middlePoint.y+middlePoint.y);
                    }
                    else{
                        //直走
                        endPoint = POINT(IMAGE_WIDTH/2,IMAGE_HEIGHT/2);
                    }
                } else {
                    //直走
                    endPoint = POINT(IMAGE_WIDTH/2,IMAGE_HEIGHT/2);
                }

                vector<POINT> input = {startPoint, middlePoint, endPoint};
                vector<POINT> repair = bezier(0.02, input);

                IMG.edge_right = repair;

            }

            //补线
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