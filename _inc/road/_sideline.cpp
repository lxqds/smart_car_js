#include "_sideline.hpp"
// 引用图像处理
extern ImageProcess imgprocess;
#define IMG imgprocess
/* ********************************************************************** */
/* ****************************** 透视变换 ******************************* */
/* ******************* (0:原图 -> 俯视, 1:俯视 -> 原图) ******************** */

// 原图 -> 俯视  透视矩阵
cv::Mat warpMatrix =
(cv::Mat_<float>(3, 3) << 4.830985915493104, 7.481690140845294,
    -559.9154929577662, 1.3374922084896e-14, 14.74647887323986,
    -725.1690140845323, 7.62235112648736e-17, 0.05070422535211418, 1);

// 俯视 -> 原图  透视矩阵
cv::Mat warpMatrixT =
(cv::Mat_<float>(3, 3) << 0.7231273024969306, -0.503533906399236,
    39.74299358711961, 4.254746920719163e-16, 0.06781279847182463,
    49.17574021012407, 1.303269874835107e-18, -0.003438395415472779, 1);

// 透视变换 (0:原图 -> 俯视, 1:俯视 -> 原图)
void map_perspective(float x, float y, float loc[2], uint8_t mode) {
    float xx, yy, zz;

    if (mode == 0) {
        zz = warpMatrix.at<float>(2, 0) * x +
            warpMatrix.at<float>(2, 1) * y +
            warpMatrix.at<float>(2, 2);
        xx = (warpMatrix.at<float>(0, 0) * x +
            warpMatrix.at<float>(0, 1) * y +
            warpMatrix.at<float>(0, 2)) /
            zz;
        yy = (warpMatrix.at<float>(1, 0) * x +
            warpMatrix.at<float>(1, 1) * y +
            warpMatrix.at<float>(1, 2)) /
            zz;

        loc[0] = xx;
        loc[1] = yy;
    }
    else {
        zz = warpMatrixT.at<float>(2, 0) * x +
            warpMatrixT.at<float>(2, 1) * y +
            warpMatrixT.at<float>(2, 2);
        xx = (warpMatrixT.at<float>(0, 0) * x +
            warpMatrixT.at<float>(0, 1) * y +
            warpMatrixT.at<float>(0, 2)) /
            zz;
        yy = (warpMatrixT.at<float>(1, 0) * x +
            warpMatrixT.at<float>(1, 1) * y +
            warpMatrixT.at<float>(1, 2)) /
            zz;

        loc[0] = xx;
        loc[1] = yy;
    }
}

/* ********************************************************************** */
/* ******************************* 控制类 ******************************** */
/* ********************************************************************** */

// 位置式 PID 角度外环
float pid_realize_a(float actual, float set, float _p, float _d) {
    static float last_error = 0.0f;
    static float last_out_d = 0.0f;
    // static float last_actual = 0.0f;
    // static float derivative = 0.0f;

    /* 当前误差 */
    float error = set - actual;

    /* 微分先行 */
    /*
    float temp = 0.618f * _d + _p;
    float c3 = _d / temp;
    float c2 = (_d + _p) / temp;
    float c1 = 0.618f * c3;
    derivative = c1 * derivative + c2 * actual - c3 * last_actual;
    */

    /* 不完全微分 */
    float out_d = _d * 0.8f * (error - last_error) + 0.2f * last_out_d;
    // float out_d = 0.8f * derivative + 0.2f * last_out_d;

    /* 实际输出 */
    float output = _p * error + out_d;

    /* 更新参数 */
    last_error = error;
    last_out_d = out_d;
    // last_actual = actual;

    return output;
}

// 位置式 PID 角速度内环
int pid_realize_o(int actual, int set, float _p, float _d) {
    static int last_error = 0;

    /* 当前误差 */
    int error = set - actual;

    /* 实际输出 */
    int output = (int)(_p * error + _d * (error - last_error) + 0.5f);

    /* 更新参数 */
    last_error = error;

    return output;
}

// 偏差滑动平均滤波
float filter(float value) {
    static float filter_buf[3] = { 0 };

    filter_buf[2] = filter_buf[1];
    filter_buf[1] = filter_buf[0];
    filter_buf[0] = value;

    return (filter_buf[2] + filter_buf[1] + filter_buf[0]) / 3.0f;
}

/* *********************************************************************** */
/* ******************************* 特殊处理 ******************************* */
/* *********************************************************************** */

// 赛道点集的方差计算
double sigma(float pts[][2], int num) {
    if (num < 1)
        return 0;

    double sum = 0;
    for (int i = 0; i < num; i++) sum += pts[i][0];

    double aver = (double)sum / num;
    double sigma = 0;

    for (int i = 0; i < num; i++)
        sigma += (pts[i][0] - aver) * (pts[i][0] - aver);
    sigma /= (double)num;

    return sigma;
}
double sigma(std::vector<POINT> vec) {
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (auto p : vec) sum += p.x;

    double aver = (double)sum / vec.size();
    double sigma = 0;

    for (auto p : vec)
        sigma += (p.x - aver) * (p.x - aver);
    sigma /= (double)vec.size();

    return sigma;
}

// 阶乘计算
int factorial(int x) {
    int f = 1;
    for (int i = 1; i <= x; i++) f *= i;

    return f;
}

// 点到直线的距离计算
double dis_point_line(POINT a, POINT b, POINT p) {
    double ab_distance = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    double ap_distance = sqrt((a.x - p.x) * (a.x - p.x) + (a.y - p.y) * (a.y - p.y));
    double bp_distance = sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));

    double half = (ab_distance + ap_distance + bp_distance) / 2;
    double area = sqrt(half * (half - ab_distance) * (half - ap_distance) * (half - bp_distance));

    return (2 * area / ab_distance);
}

// 两点间的距离计算
double dis_point_point(POINT a, POINT b) {
    double s = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    return s;
}

// 根据两点和第三点的y值求第三点的x值
double findThirdPointX(POINT a, POINT b, double y) {
    // 如果 y1 和 y2 相等，则两点在同一水平线上，无法确定唯一的 x3
    if (y == b.y) return 0.0f;
    double x = a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y);// 计算第三点的x值
    return x;
}

// 直线拟合 (返回平均绝对误差)
float fit_line(float pts[][2], int num, int cut_h) {
    if (num != 0) {
        std::vector<cv::Point> points;
        cv::Vec4f line_para;
        float k, b, mea = 0.0f;
        float trans[2];
        int y_counter = 0;

        for (int i = 0; i < num; i++, y_counter++) {
            map_perspective(pts[i][0], pts[i][1], trans, 1);
            if (trans[1] < cut_h)
                break;

            points.push_back(cv::Point(trans[0], trans[1]));
        }

        cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

        k = line_para[1] / line_para[0];
        b = line_para[3] - k * line_para[2];

        for (int i = 0; i < y_counter; i++)
            mea += fabs(k * points[i].x + b - points[i].y);
        return (float)(mea / y_counter);
    }

    return 100.0f;
}

// 贝塞尔曲线拟合
std::vector<POINT> bezier(double dt, std::vector<POINT> input) {
    std::vector<POINT> output;
    double t = 0;
    while (t <= 1) {
        POINT p;
        double x_sum = 0.0;
        double y_sum = 0.0;
        int i = 0;
        int n = input.size() - 1;
        while (i <= n) {
            double k = factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) *
                pow(1 - t, n - i);
            x_sum += k * input[i].x;
            y_sum += k * input[i].y;
            i++;
        }
        p.x = x_sum;
        p.y = y_sum;
        output.push_back(p);
        t += dt;
    }
    return output;
}

//计算返回斜率
float Calculate_K(std::vector<POINT> edge) {

    int n = edge.size();
    float k = 0;
    int bx, ex;
    if (edge[0].y <= edge[n - 1].y) { bx = 0; ex = n - 1; }
    else { bx = n - 1; ex = 0; }
    k = ((float)(edge[ex].x - edge[bx].x) / (float)(edge[ex].y - edge[bx].y));

    return k;

}

/* *********************************************************************** */
/* **************************** 特殊 AI 元素处理 *************************** */
/* ***************************** 加油站 & 施工区 *************************** */

bool sort_y_up(POINT a, POINT b) {  // 升序
    return (a.y < b.y);
}
bool sort_y_down(POINT a, POINT b) {  // 升序
    return (a.y > b.y);
}
bool sort_x_up(POINT a, POINT b) {  // 升序
    return (a.x < b.x);
}
bool sort_x_down(POINT a, POINT b) {  // 降序
    return (a.x > b.x);
}

// 从AI检测结果中检索锥桶坐标集合
std::vector<POINT> search_targets(std::vector<DetectionPredictResult> predict) {
    std::vector<POINT> cones;
    for (auto p : predict) {
        if (ImageProcess::DetectionLabel(p.type) == ImageProcess::label_cone) {
            cones.push_back(POINT(p.y / 2 + p.height / 4,
                p.x / 2 + p.width / 4));
        }

        // else if (ImageProcess::DetectionLabel(p.type) == ImageProcess::label_wone) {
        //     gas_which = 1;
        //     /*
        //     if (gas_which_is == false) {
        //         gas_which_is = true;
        //         if (gas_which_last == 0)
        //             gas_which = 1;
        //         else
        //             gas_which = 2;
        //     }
        //     */
        // }

        // else if (ImageProcess::DetectionLabel(p.type) == ImageProcess::label_wtwo) {
        //     gas_which = 2;
        //     /*
        //     if (gas_which_is == false) {
        //         gas_which_is = true;
        //         if (gas_which_last == 0)
        //             gas_which = 2;
        //         else
        //             gas_which = 1;
        //     }
        //     */
        // }
    }

    return cones;
}
// AI搜索距离小车最近的黄色锥桶
POINT AI_search_height_near(std::vector<DetectionPredictResult> predict) {
    POINT cones = POINT(0, 0);
    float ai_y = 0;
    for (auto p : predict) {
        if (ImageProcess::DetectionLabel(p.type) == ImageProcess::label_cone) {
            //p.y / 2 + p.height / 4,p.x / 2 + p.width / 4));
            ai_y = p.y / 2 + p.height / 4;

            if (ai_y > cones.x)cones = POINT(ai_y, p.x / 2 + p.width / 4);

        }
    }
    return cones;
}

//得到全图 锥桶 剔除异常值后得到的 y轴平均
float get_AIorcolor_AVG_Y(std::vector<DetectionPredictResult> predict) {
    vector<POINT> AI_cones = search_targets(predict);
    int num = AI_cones.size();

    float z_scores;
    std::vector<int> avers;

    int sum = 0;
    float aver = 0;
    float sigma = 0;

    for (int i = 0; i < num; i++) {
        sum += AI_cones[i].x;
    }

    aver = (float)(sum / num);

    for (int i = 0; i < num; i++)
        sigma += (AI_cones[i].x - aver) * (AI_cones[i].x - aver);
    sigma /= (float)num;

    //计算 Z-score 
    int outnum = 0;
    sum = 0;
    for (int i = 0; i < num; i++)
    {
        z_scores = ((AI_cones[i].x - aver) * (AI_cones[i].x - aver)) / sigma;// 与均值差的平方除以方差

        if (z_scores < 2.5) {
            outnum += 1;
            sum += AI_cones[i].x;
        }
    }

    //剃掉异常值后的平均值
    return (float)sum / outnum;

}

// AI搜索距离赛道左边缘最近的黄色色块 (黄色锥桶)
POINT AI_search_left_near(std::vector<DetectionPredictResult> predict, std::vector<POINT> pointsEdge) {
    vector<POINT> AI_cones = search_targets(predict);

    for (size_t i = 0; i < pointsEdge.size(); i++) {
        for (size_t a = 0; a < AI_cones.size(); a++)
        {   //锥桶中心在 任意边线点 左方宽35 高40区域内时:
            if ((pointsEdge[i].y - 35 < AI_cones[a].y && AI_cones[a].y < pointsEdge[i].y)
                && ((pointsEdge[i].x - 20 < AI_cones[a].x && AI_cones[a].x < pointsEdge[i].x)
                    || (pointsEdge[i].x + 20 > AI_cones[a].x && AI_cones[a].x > pointsEdge[i].x)))
                return POINT(pointsEdge[i].x, AI_cones[a].y);
        }
    }
    return POINT(0, 0);
}

// AI搜索距离赛道左边缘最近的黄色色块 (黄色锥桶)
POINT AI_search_right_near(std::vector<DetectionPredictResult> predict, std::vector<POINT> pointsEdge) {
    vector<POINT> AI_cones = search_targets(predict);

    for (size_t i = 0; i < pointsEdge.size(); i++) {
        for (size_t a = 0; a < AI_cones.size(); a++)
        {   //锥桶中心在 任意边线点 右方宽35 高40区域内时:
            if ((pointsEdge[i].y < AI_cones[a].y && AI_cones[a].y < pointsEdge[i].y + 35)
                && ((pointsEdge[i].x - 20 < AI_cones[a].x && AI_cones[a].x < pointsEdge[i].x)
                    || (pointsEdge[i].x + 20 > AI_cones[a].x && AI_cones[a].x > pointsEdge[i].x)))
                return POINT(pointsEdge[i].x, AI_cones[a].y);
        }
    }
    return POINT(0, 0);
}


// 搜索距离赛道左边缘最近的黄色色块 (黄色锥桶)
POINT search_left_near(std::vector<POINT> pointsEdge, cv::Mat matt) {
    for (size_t i = 0; i < pointsEdge.size(); i++)
        for (int y = pointsEdge[i].y - 25; y < pointsEdge[i].y; y++)
            if ((y >= 0) && (y <= IMAGE_WIDTH - 1))
                if (AT_IMAGE(matt, y, pointsEdge[i].x) > 0)
                    return POINT(pointsEdge[i].x, y);

    return POINT(0, 0);
}
// 搜索距离赛道左边缘上面最近的黄色色块 (黄色锥桶)
POINT search_left_near2(std::vector<POINT> pointsEdge, cv::Mat matt) {
    for (size_t i = 0; i < pointsEdge.size(); i++)
        for (int x = pointsEdge[i].x - 25; x < pointsEdge[i].x; x++)
            if ((x >= 0) && (x <= IMAGE_HEIGHT - 1))
                if (AT_IMAGE(matt, pointsEdge[i].y, x) > 0)
                    return POINT(x, pointsEdge[i].y);

    return POINT(0, 0);
}

// 搜索 屏幕中间以下 黄色锥桶 距离屏幕最下方距离
uint16_t search_mid_cone_getH(cv::Mat matt) {
    int sum = 5;
    for (size_t y = IMAGE_HEIGHT - 20; y > IMAGE_HEIGHT / 2; y--) {
        sum = 5;
        for (size_t x = IMAGE_WIDTH / 2; x < IMAGE_WIDTH / 2 + 30; x++)
        {
            if (AT_IMAGE(matt, x, y) > 0)
                sum--;
            if (sum <= 0)
                return y;
        }

        for (size_t x = IMAGE_WIDTH / 2; x > IMAGE_WIDTH / 2 - 30; x--)
        {
            if (AT_IMAGE(matt, x, y) > 0)
                sum--;
            if (sum <= 0)
                return y;
        }
    }
    return 0;
}

void CVFind_Clone::init() {
    memset(bl_map, 0, sizeof(bl_map));
}
std::vector<cv::Rect>  CVFind_Clone::search_all_cone_rect(cv::Mat matt) {
    Clone_MAP = matt.clone();
    memset(bl_map, 0, sizeof(bl_map));
    std::vector<cv::Rect> Res;
    for (int y = 10; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 5; x < IMAGE_WIDTH - 10; x++) {
            if (AT_IMAGE(Clone_MAP, x, y) > 0) {
                cv::Rect resRect(x, y, x, y);
                //printf("\n\nfind %d %d\n", x, y);
                resRect = find_Width_Begin_x(x, y, 0, 1, resRect);
                resRect.width -= resRect.x;
                resRect.height -= resRect.y;
                coverTheMat(resRect, Clone_MAP);
                if (resRect.area() < 80 || resRect.width < 0 || resRect.height < 0)continue;
                resRect.x -= 3;
                resRect.y -= 3;
                resRect.width += 3;
                resRect.height += 3;
                if (resRect.x < 0)resRect.x = 0;
                if (resRect.y < 0)resRect.y = 0;
                if (resRect.width >= IMAGE_WIDTH)resRect.width = IMAGE_WIDTH - 1;
                if (resRect.height >= IMAGE_HEIGHT)resRect.height = IMAGE_HEIGHT - 1;
                //float predit = 9.26 * resRect.y - 55;
                //if (abs(predit - resRect.area()) > 300)continue;//区域面积和y坐标线性预测值偏差大则跳过
                //cv::imshow("Clone_MAP", Clone_MAP);
                Res.push_back(resRect);
            }
        }
    }
    return Res;
}

void CVFind_Clone::coverTheMat(cv::Rect Rect, cv::Mat map) {
    int by, bx, w, h;
    by = Rect.y - 3;
    bx = Rect.x - 3;
    w = bx + Rect.width + 6;
    h = by + Rect.height + 6;
    if (by < 0)by = 0;
    if (bx < 0) bx = 0;
    if (w >= IMAGE_WIDTH)w = IMAGE_WIDTH - 1;
    if (h >= IMAGE_HEIGHT)h = IMAGE_HEIGHT - 1;

    for (; by < h; by++)
    {
        for (int x = bx; x < w; x++) {
            AT_IMAGE(map, x, by) = 0;
        }
    }
}
bool CVFind_Clone::pt_IsEdge(int x, int y) {
    //左  处于图片边界时也认定它 是边界
    for (uint8_t i = 0; i < 8; i++) {
        int frontx, fronty;
        fronty = y + cone_front[i][1];
        frontx = x + cone_front[i][0];
        if (frontx < 0 || frontx >= IMAGE_WIDTH) return true;
        if (fronty < 0 || fronty >= IMAGE_HEIGHT) return true;
        if (AT_IMAGE(Clone_MAP, frontx, fronty) <= 0)return true;
    }
    //  printf("lext not hei \n");
    return false;
}
cv::Rect CVFind_Clone::find_Width_Begin_x(int x, int y, int turncs, int nowfx, cv::Rect lastRect) {

    if (x < 0 || y < 0 || x >= IMAGE_WIDTH || y >= IMAGE_HEIGHT) return lastRect;
    if (abs(turncs) >= 8) { return lastRect; }//转动360度 结束递归 返回现在搜索到最长宽度
    cv::Rect resRect = lastRect;
    short tempfx = nowfx;
    short tempturncs = turncs;
    // cv::imshow("Clone_MAP", Clone_MAP);
     //cv::waitKey(0);
    // printf("/n/n ***************   next  ********************* \n\n x %d y %d \n\n\n",x,y);

    for (size_t i = 0; i < 8; i++)
    {
        int frontx, fronty;
        fronty = y + cone_front[i][1];
        frontx = x + cone_front[i][0];
        if (frontx < 0 || frontx >= IMAGE_WIDTH) continue;
        if (fronty < 0 || fronty >= IMAGE_HEIGHT) continue;
        //      下一格为白色                         且为 未寻找过的格子             且    下一格的八邻域为黑格（边缘格）
        if (AT_IMAGE(Clone_MAP, frontx, fronty) > 0 && bl_map[frontx][fronty] == 0 && pt_IsEdge(frontx, fronty)) {//对八邻域试探搜索
            tempfx = nowfx;
            tempturncs = turncs;
            // 要转的 减去 现在方向  大于0 右转 小于 0 左转 
            int turnfx = cone_arger[i] - nowfx;
            if (turnfx >= 4) turnfx -= 8; //将范围限制在180度以内
            if (turnfx <= -4) turnfx += 8;

            tempfx = cone_arger[i]; //更新搜索方向

            tempturncs += turnfx; //更新转弯次数
            //AT_IMAGE(Clone_MAP, x, y) = 0;
            bl_map[x][y] = 1; //禁止搜索相同路径

            if (frontx > resRect.width) {
                resRect.width = frontx;  // 更新MAX x
            }
            if (fronty > resRect.height) {
                resRect.height = fronty; // 更新MAX h
            }
            if (fronty < resRect.y) {
                resRect.y = fronty;   //更新min y
            }
            if (frontx < resRect.x) {
                resRect.x = frontx;//更新min x
            }
            resRect = find_Width_Begin_x(frontx, fronty, tempturncs, tempfx, resRect);
        }
    }
    return resRect;

}

void drawH(int h, uint8_t r, uint8_t g, uint8_t b , cv::Mat map) {
    cv::line(map, cv::Point(0, (int)h),
        cv::Point(IMAGE_WIDTH - 1, (int)h), cv::Scalar(r, g, b), 1);
}


void drawW(int w, uint8_t r, uint8_t g, uint8_t b, cv::Mat map) {
    cv::line(map, cv::Point(w, IMAGE_HEIGHT-1),
        cv::Point(w, 0, cv::Scalar(r, g, b), 1));
}
POINT getCenterPoint(cv::Rect rect){
    POINT cpt;

    cpt.x = rect.x + (int)rect.width / 2;

    cpt.y = rect.y + (int)rect.height / 2;

    return cpt;

}
//// 搜索距离赛道左边缘最近的黄色色块 (黄色锥桶) 赛道内
//POINT search_left_near_in(std::vector<POINT> pointsEdge, cv::Mat matt) {
//    int sum = 5;
//    POINT find = POINT(0, 0);
//    int resw = IMAGE_WIDTH;
//    for (size_t i = 0; i < pointsEdge.size(); i++) {
//        sum = 5;
//        for (int y = pointsEdge[i].y; y < pointsEdge[i].y + 50; y++) {
//            if ((y >= 0) && (y <= IMAGE_WIDTH - 1)) {
//                if (AT_IMAGE(matt, y, pointsEdge[i].x) > 0)
//                    sum--;
//                if (sum <= 0 && y - pointsEdge[i].y < resw)
//                {
//                    find = POINT(pointsEdge[i].x, y);
//                    resw = y - pointsEdge[i].y;
//                    break;
//                }
//            }
//        }
//    }
//    return find;
//}
//
//// 搜索距离内外赛道右边缘上面最近的黄色色块 (黄色锥桶) 赛道内
//POINT search_right_near_in(std::vector<POINT> pointsEdge, cv::Mat matt) {
//    int sum = 5;
//    POINT find = POINT(0, 0);
//    int resw = IMAGE_WIDTH;
//    for (size_t i = pointsEdge.size() - 1; i >= 0; i--) {
//        sum = 5;
//        for (int y = pointsEdge[i].y; y > pointsEdge[i].y - 50; y--) {
//            if ((y >= 0) && (y <= IMAGE_WIDTH - 1)) {
//                if (AT_IMAGE(matt, y, pointsEdge[i].x) > 0)
//                    sum--;
//                if (sum <= 0 && pointsEdge[i].y - y < resw) {
//                    resw = pointsEdge[i].y - y;
//                    find = POINT(pointsEdge[i].x, y);
//                    break;
//                }
//
//            }
//        }
//
//    }
//    return find;
//}

// 搜索距离赛道左边缘最近的黄色色块 (黄色锥桶) 赛道内
POINT search_left_near_in(std::vector<POINT> pointsEdge, cv::Mat matt) {
    int sum = 5;
    POINT find = POINT(0, 0);
    int resw = IMAGE_WIDTH;
    for (size_t i = 0; i < pointsEdge.size(); i++) {
        sum = 5;
        for (int y = pointsEdge[i].y; y < pointsEdge[i].y + 50; y++) {
            if ((y >= 0) && (y <= IMAGE_WIDTH - 1)) {
                if (AT_IMAGE(matt, y, pointsEdge[i].x) > 0) {
                    sum--;
                    cv::line(IMG.mat_result, cv::Point(pointsEdge[i].y, pointsEdge[i].x),
                        cv::Point(y, pointsEdge[i].x),
                        cv::Scalar(255, 0, 255), 7);
                }

                if (sum <= 0)
                {
                    if (resw > y - pointsEdge[i].y)
                    {
                        find = POINT(pointsEdge[i].x, y);
                        resw = y - pointsEdge[i].y;
                    }
                    break;
                }
            }
        }
    }
    return find;
}

// 搜索距离内外赛道右边缘上面最近的黄色色块 (黄色锥桶) 赛道内
POINT search_right_near_in(std::vector<POINT> pointsEdge, cv::Mat matt) {
    int sum = 5;
    POINT find = POINT(0, 0);
    int resw = IMAGE_WIDTH;
    for (size_t i = 0; i < pointsEdge.size(); i++) {
        sum = 5;
        for (int y = pointsEdge[i].y; y > pointsEdge[i].y - 50; y--) {
            if ((y >= 0) && (y <= IMAGE_WIDTH - 1)) {

                if (AT_IMAGE(matt, y, pointsEdge[i].x) > 0)
                {
                    sum--;
                    cv::line(IMG.mat_result, cv::Point(pointsEdge[i].y, pointsEdge[i].x),
                        cv::Point(y, pointsEdge[i].x),
                        cv::Scalar(255, 0, 255), 7);
                }
                if (sum <= 0)
                {
                    if (resw > pointsEdge[i].y - y)
                    {
                        find = POINT(pointsEdge[i].x, y);
                        resw = pointsEdge[i].y - y;
                    }
                    break;
                }

            }
        }

    }

    return find;
}

// 找左边线离锥桶最近的边线点的位次,x为行，第几行
uint16_t search_break_left(std::vector<POINT> pointsEdge, cv::Mat matt) {
    for (size_t i = 0; i < pointsEdge.size(); i++)
        for (int x = pointsEdge[i].x - 25; x < pointsEdge[i].x; x++)
            if ((x >= 0) && (x <= IMAGE_HEIGHT - 1))
                if (AT_IMAGE(matt, pointsEdge[i].y, x) > 0)
                    return i;
    return 0;
}


// 找右边线离锥桶最近的边线点的位次,x为行，第几行
uint16_t search_break_right(std::vector<POINT> pointsEdge, cv::Mat matt) {
    for (size_t i = 0; i < pointsEdge.size(); i++)
        for (int x = pointsEdge[i].x - 25; x < pointsEdge[i].x; x++)
            if ((x >= 0) && (x <= IMAGE_HEIGHT - 1))
                if (AT_IMAGE(matt, pointsEdge[i].y, x) > 0)
                    return i;
    return 0;
}
// 搜索距离赛道左边缘上15最近的黄色色块 (黄色锥桶)
POINT search_left_near15(std::vector<POINT> pointsEdge, cv::Mat matt) {
    for (size_t i = 0; i < pointsEdge.size(); i++)
        for (int y = pointsEdge[i].y - 15; y < pointsEdge[i].y; y++)
            if ((y >= 0) && (y <= IMAGE_WIDTH - 1))
                if (AT_IMAGE(matt, y, pointsEdge[i].x) > 0)
                    return POINT(pointsEdge[i].x, y);

    return POINT(0, 0);
}
// 搜索距离赛道左边缘上35最近的黄色色块 (黄色锥桶)
POINT search_left_near35(std::vector<POINT> pointsEdge, cv::Mat matt) {
    for (size_t i = 0; i < pointsEdge.size(); i++)
        for (int y = pointsEdge[i].y - 35; y < pointsEdge[i].y; y++)
            if ((y >= 0) && (y <= IMAGE_WIDTH - 1))
                if (AT_IMAGE(matt, y, pointsEdge[i].x) > 0)
                    return POINT(pointsEdge[i].x, y);

    return POINT(0, 0);
}

// 搜索距离赛道右边缘最近的黄色色块 (黄色锥桶)
POINT search_right_near(std::vector<POINT> pointsEdge, cv::Mat matt) {
    for (size_t i = 0; i < pointsEdge.size(); i++)
        for (int y = pointsEdge[i].y + 25; y > pointsEdge[i].y; y--)
            if ((y >= 0) && (y <= IMAGE_WIDTH - 1))
                if (AT_IMAGE(matt, y, pointsEdge[i].x) > 0)
                    return POINT(pointsEdge[i].x, y);

    return POINT(0, 0);
}
// 两点距离公式
float distance_for_points(POINT a, POINT b) {
    return (float)(sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
}
// 搜索左右边缘的锥桶
void search_cones_edge(std::vector<POINT> cones, std::vector<POINT>* left_cones, std::vector<POINT>* right_cones) {
    std::vector<POINT> points_cones = cones;
    // if (left_cones->size() == 0 && right_cones->size() == 0) {
    //    return;
    //}
    float Left_Last_distance = 0.0f;
    double Left_Last_angle = 0.0f;
    float Right_Last_distance = 0.0f;
    double Right_Last_angle = 0.0f;
    int left_cnt = 0;
    int right_cnt = 0;
    // MAT_LOG("qpoint1");

    /*
        for (int i = points_cones.size() - 1; i >= 0; i--) {
            // 左边底部锥桶
            // if (IMG.edge_det[i].y < IMAGE_WIDTH / 3 && IMG.edge_det[i].x > IMAGE_HEIGHT / 3)
            if (left_cones->size() > 0) {
                if (!left_cnt) {
                    Left_Last_distance = distance_for_points(points_cones[i], left_cones->at(0));
                    left_cones->push_back(points_cones[i]);

                    left_cnt++;
                    log_i("ld:%f", Left_Last_distance);
                    log_i("l_cnt++");
                } else if (distance_for_points(points_cones[i], left_cones->at(0)) < Left_Last_distance) {
                    Left_Last_distance = distance_for_points(points_cones[i], left_cones->at(0));

                    left_cones->at(1) = points_cones[i];

                    log_i("ld:%f", Left_Last_distance);
                    log_i("leftconere2");
                }
            }

            if (right_cones->size() > 0) {
                if (!right_cnt) {
                    Right_Last_distance = distance_for_points(points_cones[i], right_cones->at(0));
                    right_cones->push_back(points_cones[i]);
                    right_cnt++;
                    log_i("rd:%f", Right_Last_distance);
                    log_i("r_cnt++");
                } else if (distance_for_points(points_cones[i], right_cones->at(0)) < Right_Last_distance) {
                    Right_Last_distance = distance_for_points(points_cones[i], right_cones->at(0));
                    right_cones->at(1) = points_cones[i];

                    log_i("rd:%f", Right_Last_distance);
                    log_i("rightconere2");
                }
            }
        }
    */
    left_cones->push_back(POINT(IMAGE_HEIGHT - 1, 0));
    right_cones->push_back(POINT(IMAGE_HEIGHT - 1, IMAGE_WIDTH - 1));
    for (int i = points_cones.size() - 1; i >= 0; i--) {
        // 左边底部锥桶
        // if (IMG.edge_det[i].y < IMAGE_WIDTH / 3 && IMG.edge_det[i].x > IMAGE_HEIGHT / 3)
        if (points_cones[i].y < (IMAGE_WIDTH / 2) && dis_point_point(points_cones[i], POINT(IMAGE_HEIGHT - 1, 0)) < IMG._config->farm_left_distance) {
            if (left_cones->size() == 1) {
                Left_Last_distance = distance_for_points(left_cones->at(left_cones->size() - 1), points_cones[i]);
                Left_Last_angle = atan((double)(points_cones[i].y - left_cones->at(left_cones->size() - 1).y) * 1.0 / (double)(left_cones->at(left_cones->size() - 1).x - points_cones[i].x));
                left_cones->push_back(points_cones[i]);

            }
            else if (points_cones[i].x > left_cones->at(1).x) {
                Left_Last_distance = distance_for_points(left_cones->at(left_cones->size() - 1), points_cones[i]);
                Left_Last_angle = atan((double)(points_cones[i].y - left_cones->at(left_cones->size() - 1).y) * 1.0 / (double)(left_cones->at(left_cones->size() - 1).x - points_cones[i].x));
                left_cones->at(1) = points_cones[i];
            }
        }
        // 右边底部锥桶
        // if (IMG.edge_det[i].y > (IMAGE_WIDTH * 2 / 3) && IMG.edge_det[i].x > IMAGE_HEIGHT / 3)
        if (points_cones[i].y > (IMAGE_WIDTH / 2) && dis_point_point(points_cones[i], POINT(IMAGE_HEIGHT - 1, IMAGE_WIDTH - 1)) < IMG._config->farm_right_distance) {
            if (right_cones->size() == 1) {
                Right_Last_distance = distance_for_points(right_cones->at(right_cones->size() - 1), points_cones[i]);
                Right_Last_angle = atan((double)(points_cones[i].y - right_cones->at(right_cones->size() - 1).y) * 1.0 / (double)(right_cones->at(right_cones->size() - 1).x - points_cones[i].x));
                right_cones->push_back(points_cones[i]);
            }
            else if (points_cones[i].x > right_cones->at(1).x) {
                Right_Last_angle = atan((double)(points_cones[i].y - right_cones->at(right_cones->size() - 1).y) * 1.0 / (double)(right_cones->at(right_cones->size() - 1).x - points_cones[i].x));
                Right_Last_distance = distance_for_points(right_cones->at(right_cones->size() - 1), points_cones[i]);
                right_cones->at(1) = points_cones[i];
            }
        }
    }

    for (int j = 0; j < (int)(points_cones.size() * points_cones.size()); j++) {
        int index = -1;

        // 搜索左边缘
        for (int i = 0; i < (int)points_cones.size(); i++) {
            if (left_cones->size() > 0) {
                // 先前左边缘只到中间，拓展左边缘
                // log_i("in1");
                if (points_cones[i].x < left_cones->at(left_cones->size() - 1).x) {
                    if ((distance_for_points(left_cones->at(left_cones->size() - 1), points_cones[i]) / Left_Last_distance < 3) && (distance_for_points(left_cones->at(left_cones->size() - 1), points_cones[i]) / Left_Last_distance > 0.3)) {
                        if (fabs(atan((points_cones[i].y - left_cones->at(left_cones->size() - 1).y) * 1.0 / (left_cones->at(left_cones->size() - 1).x - points_cones[i].x)) - Left_Last_angle) < IMG._config->farm_left_angle) {
                            index = i;
                            log_i("leftconeidx2");
                        }
                    }
                }
            }
        }
        if (index >= 0) {
            Left_Last_distance = distance_for_points(left_cones->at(left_cones->size() - 1), points_cones[index]);
            log_i("ild:%f", Left_Last_distance);
            Left_Last_angle = atan((double)(points_cones[index].y - left_cones->at(left_cones->size() - 1).y) * 1.0 / (double)(left_cones->at(left_cones->size() - 1).x - points_cones[index].x));
            left_cnt++;
            left_cones->push_back(points_cones[index]);
            swap(points_cones[index], points_cones[points_cones.size() - 1]);
            points_cones.pop_back();
            index = -1;
        }
        index = -1;
        // 搜索右边缘
        for (int i = 0; i < (int)points_cones.size(); i++) {
            if (right_cones->size() > 0) {
                // 先前左边缘只到中间，拓展左边缘
                // log_i("in1");
                if (points_cones[i].x < right_cones->at(right_cones->size() - 1).x) {
                    // log_i("in1-1");
                    if ((distance_for_points(right_cones->at(right_cones->size() - 1), points_cones[i]) * 1.0 / Right_Last_distance < 3) && (distance_for_points(right_cones->at(right_cones->size() - 1), points_cones[i]) / Right_Last_distance > 0.3)) {
                        if (fabs(atan((double)(points_cones[i].y - right_cones->at(right_cones->size() - 1).y) * 1.0 / (double)(right_cones->at(right_cones->size() - 1).x - points_cones[i].x)) - Right_Last_angle) < IMG._config->farm_right_angle) {
                            index = i;
                            log_i("rightconeidx2");
                        }
                    }
                }
            }
        }
        if (index >= 0) {
            Right_Last_distance = distance_for_points(right_cones->at(right_cones->size() - 1), points_cones[index]);
            log_i("ird:%f", Right_Last_distance);
            Right_Last_angle = atan((double)(points_cones[index].y - right_cones->at(right_cones->size() - 1).y) * 1.0 / (double)(right_cones->at(right_cones->size() - 1).x - points_cones[index].x));
            right_cnt++;
            right_cones->push_back(points_cones[index]);
            swap(points_cones[index], points_cones[points_cones.size() - 1]);
            points_cones.pop_back();
            index = -1;
        }
        // MAT_LOG("qpoint2");
    }
}
// 搜索左方的锥桶集合 ( IMAGE_WIDTH / 3 )
std::vector<POINT> search_left_cones(std::vector<POINT> pointsCone) {
    std::vector<POINT> points;

    for (auto p : pointsCone)
        if ((p.y < IMAGE_WIDTH / 3) && (p.x > 10) && (p.x < IMAGE_HEIGHT))
            points.push_back(p);

    if (points.size() >= 2)
        std::sort(points.begin(), points.end(), sort_y_up);  // 纵坐标升序排序

    return points;
}

// 搜索右方的锥桶集合 ( IMAGE_WIDTH * 2 / 3 )
std::vector<POINT> search_right_cones(std::vector<POINT> pointsCone) {
    std::vector<POINT> points;

    for (auto p : pointsCone)
        if ((p.y > IMAGE_WIDTH * 2 / 3) && (p.x > 10) && (p.x < IMAGE_HEIGHT))
            points.push_back(p);

    if (points.size() >= 2)
        std::sort(points.begin(), points.end(), sort_y_down);  // 纵坐标降序排序

    return points;
}

// 搜索左上方的锥桶集合 (除了右下角)
std::vector<POINT> search_leftUp_cones(std::vector<POINT> pointsCone) {
    std::vector<POINT> points;

    for (auto p : pointsCone) {
        if (p.x > 30) {
            if (p.x > IMAGE_HEIGHT / 2 && p.y > IMAGE_WIDTH / 2)
                continue;

            points.push_back(p);
        }
    }

    if (points.size() >= 2)
        std::sort(points.begin(), points.end(), sort_y_up);  // 纵坐标升序排序

    return points;
}

// 搜索右上方的锥桶集合 (除了左下角)
std::vector<POINT> search_rightUp_cones(std::vector<POINT> pointsCone) {
    std::vector<POINT> points;

    for (auto p : pointsCone) {
        if (p.x > 30) {
            if (p.x > IMAGE_HEIGHT / 2 && p.y < IMAGE_WIDTH / 2)
                continue;

            points.push_back(p);
        }
    }

    if (points.size() >= 2)
        std::sort(points.begin(), points.end(), sort_y_down);  // 纵坐标升序排序

    return points;
}

// 搜索右上方的锥桶
POINT search_rightUp_cone(std::vector<POINT> pointsCone) {
    POINT point(0, 0);
    POINT max_point(IMAGE_HEIGHT - 1, 0);
    double min_dis = 99999.0f;

    for (auto p : pointsCone) {
        if (p.x < 30)
            continue;

        if (p.x < max_point.x)
            max_point.x = p.x;
        if (p.y > max_point.y)
            max_point.y = p.y;
    }

    for (auto p : pointsCone) {
        if (abs(max_point.x - p.x) >= 40 || abs(max_point.y - p.y) >= 40)
            continue;

        double dis = dis_point_point(p, max_point);
        if (dis < min_dis) {
            min_dis = dis;
            point = p;
        }
    }

    if (point.x == 0 && point.y == 0 && pointsCone.size() > 0)
        point = max_point;

    return point;
}

// 搜索左上方的锥桶
POINT search_leftUp_cone(std::vector<POINT> pointsCone) {
    POINT point(0, 0);
    POINT max_point(IMAGE_HEIGHT - 1, IMAGE_WIDTH - 1);
    double min_dis = 99999.0f;

    for (auto p : pointsCone) {
        if (p.x < 30)
            continue;

        if (p.x < max_point.x)
            max_point.x = p.x;
        if (p.y < max_point.y)
            max_point.y = p.y;
    }

    for (auto p : pointsCone) {
        if (abs(max_point.x - p.x) >= 40 || abs(max_point.y - p.y) >= 40)
            continue;

        double dis = dis_point_point(p, max_point);
        if (dis < min_dis) {
            min_dis = dis;
            point = p;
        }
    }

    if (point.x == 0 && point.y == 0 && pointsCone.size() > 0)
        point = max_point;

    return point;
}

// 搜索右下方的锥桶
POINT search_rightDown_cone(std::vector<POINT> pointsCone) {
    POINT point(0, 0);

    for (auto p : pointsCone)
        if (p.y > (IMAGE_WIDTH / 2 - 10) && p.x > point.x)
            point = p;

    return point;
}

// 搜索左下方的锥桶
POINT search_leftDown_cone(std::vector<POINT> pointsCone) {
    POINT point(0, 0);

    for (auto p : pointsCone)
        if (p.y < (IMAGE_WIDTH / 2 + 10) && p.x > point.x)
            point = p;

    return point;
}

/* *********************************************************************** */
/* ******************************* 边线处理 ******************************* */
/* *********************************************************************** */

// 限制范围
int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low
        : x;
}

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
const int dir_front[4][2] = { {0, -1}, {1, 0}, {0, 1}, {-1, 0} };
const int dir_frontleft[4][2] = { {-1, -1}, {1, -1}, {1, 1}, {-1, 1} };
const int dir_frontright[4][2] = { {1, -1}, {1, 1}, {-1, 1}, {-1, -1} };

// 左手迷宫巡线
void findline_lefthand_adaptive(cv::Mat img, int block_size, int clip_value,
    int x, int y, int pts[][2], int* num) {
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < *num) && (half <= x) && (x <= img.cols - half - 1) &&
        (half <= y) && (y <= img.rows - half - 1) && (turn < 4)) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += AT_IMAGE(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value =
            AT_IMAGE(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontleft_value =
            AT_IMAGE(img, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        }
        else if (frontleft_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

// 右手迷宫巡线
void findline_righthand_adaptive(cv::Mat img, int block_size, int clip_value,
    int x, int y, int pts[][2], int* num) {
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < *num) && (half <= x) && (x <= img.cols - half - 1) &&
        (half <= y) && (y <= img.rows - half - 1) && (turn < 4)) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += AT_IMAGE(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value =
            AT_IMAGE(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontright_value =
            AT_IMAGE(img, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}


/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
const int far_dir_front[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };
const int far_dir_frontleft[4][2] = { {-1, 1}, {1, 1}, {1, -1}, {-1, -1} };
const int far_dir_frontright[4][2] = { {1, 1}, {1, -1}, {-1, -1}, {-1, 1} };

// 左手迷宫巡线（向下）
void far_findline_lefthand_adaptive(cv::Mat img, int block_size, int clip_value,
    int x, int y, int pts[][2], int* num) {
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < *num) && (half <= x) && (x <= img.cols - half - 1) &&
        (half <= y) && (y <= img.rows - half - 1) && (turn < 100)) {
        //求自适应阈值
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += AT_IMAGE(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value =
            AT_IMAGE(img, x + far_dir_front[dir][0], y + far_dir_front[dir][1]);
        int frontleft_value =
            AT_IMAGE(img, x + far_dir_frontleft[dir][0], y + far_dir_frontleft[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        }
        else if (frontleft_value < local_thres) {
            x += far_dir_front[dir][0];
            y += far_dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else {
            x += far_dir_frontleft[dir][0];
            y += far_dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

// 右手迷宫巡线（向下）
void far_findline_righthand_adaptive(cv::Mat img, int block_size, int clip_value,
    int x, int y, int pts[][2], int* num) {
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < *num) && (half <= x) && (x <= img.cols - half - 1) &&
        (half <= y) && (y <= img.rows - half - 1) && (turn < 4)) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += AT_IMAGE(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value =
            AT_IMAGE(img, x + far_dir_front[dir][0], y + far_dir_front[dir][1]);
        int frontright_value =
            AT_IMAGE(img, x + far_dir_frontright[dir][0], y + far_dir_frontright[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (frontright_value < local_thres) {
            x += far_dir_front[dir][0];
            y += far_dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else {
            x += far_dir_frontright[dir][0];
            y += far_dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}
//找左线（向上）
void find_left_line(cv::Mat mat_bin, int x, int y, int ipts[][2], int* ipts_num)
{
    int x1 = x, y1 = y;
    // 向左寻找白点
    if (AT_IMAGE(mat_bin, x1, y1) < 127)
        for (x1--; x1 > 0; x1--)
            if (AT_IMAGE(mat_bin, x1, y1) >= 127)
                break;
    // 向左寻找黑点
    for (; x1 > 0; x1--)
        if (AT_IMAGE(mat_bin, x1 - 1, y1) < 127)
            break;
    // 向上寻找黑点
    if (x1 < BLOCK_SIZE / 2) {
        x1 = BLOCK_SIZE / 2;
        for (; y1 > IMAGE_HEIGHT * 2 / 3; y1--)
            if (AT_IMAGE(mat_bin, x1, y1 - 1) < 127)
                break;
    }
    if (AT_IMAGE(mat_bin, x1, y1) >= 127)
        findline_lefthand_adaptive(mat_bin, BLOCK_SIZE, CLIP_VALUE,
            x1, y1, ipts, ipts_num);
    else *ipts_num = 0;
}
//找右线（向上）
void find_right_line(cv::Mat mat_bin, int x, int y, int ipts[][2], int* ipts_num)
{
    int x2 = x, y2 = y;
    // 向右寻找白点
    if (AT_IMAGE(mat_bin, x2, y2) < 127)
        for (x2++; x2 < IMAGE_WIDTH - 1; x2++)
            if (AT_IMAGE(mat_bin, x2, y2) >= 127)
                break;
    // 向右寻找黑点
    for (; x2 < IMAGE_WIDTH - 1; x2++)
        if (AT_IMAGE(mat_bin, x2 + 1, y2) < 127)
            break;
    // 向上寻找黑点
    if (x2 > IMAGE_WIDTH - BLOCK_SIZE / 2 - 1) {
        x2 = IMAGE_WIDTH - BLOCK_SIZE / 2 - 1;
        for (; y2 > IMAGE_HEIGHT * 2 / 3; y2--)
            if (AT_IMAGE(mat_bin, x2, y2 - 1) < 127)
                break;
    }
    if (AT_IMAGE(mat_bin, x2, y2) >= 127)
        findline_righthand_adaptive(mat_bin, BLOCK_SIZE, CLIP_VALUE,
            x2, y2, ipts, ipts_num);
    else *ipts_num = 0;
}
//找左线（向下）
void find_far_left_line(cv::Mat mat_bin, int x, int y, int ipts[][2], int* ipts_num)
{
    int x1 = x, y1 = y;
    // 向左寻找白点
    if (AT_IMAGE(mat_bin, x1, y1) < 127)
        for (x1--; x1 > 0; x1--)
            if (AT_IMAGE(mat_bin, x1, y1) >= 127)
                break;
    // 向左寻找黑点
    for (; x1 > 0; x1--)
        if (AT_IMAGE(mat_bin, x1 - 1, y1) < 127)
            break;
    // 向下寻找黑点
    if (x1 < BLOCK_SIZE / 2) {
        x1 = BLOCK_SIZE / 2;
        for (; y1 < IMAGE_HEIGHT * 2 / 3; y1++)
            if (AT_IMAGE(mat_bin, x1, y1 + 1) < 127)
                break;
    }
    if (AT_IMAGE(mat_bin, x1, y1) >= 127)
        far_findline_lefthand_adaptive(mat_bin, BLOCK_SIZE, CLIP_VALUE,
            x1, y1, ipts, ipts_num);
    else *ipts_num = 0;
}
//找右线（向下）
void find_far_right_line(cv::Mat mat_bin, int x, int y, int ipts[][2], int* ipts_num)
{
    int x2 = x, y2 = y;
    // 向右寻找白点
    if (AT_IMAGE(mat_bin, x2, y2) < 127)
        for (x2++; x2 < IMAGE_WIDTH - 1; x2++)
            if (AT_IMAGE(mat_bin, x2, y2) >= 127)
                break;
    // 向右寻找黑点
    for (; x2 < IMAGE_WIDTH - 1; x2++)
        if (AT_IMAGE(mat_bin, x2 + 1, y2) < 127)
            break;
    // 向下寻找黑点
    if (x2 > IMAGE_WIDTH - BLOCK_SIZE / 2 - 1) {
        x2 = IMAGE_WIDTH - BLOCK_SIZE / 2 - 1;
        for (; y2 < IMAGE_HEIGHT * 2 / 3; y2++)
            if (AT_IMAGE(mat_bin, x2, y2 + 1) < 127)
                break;
    }
    if (AT_IMAGE(mat_bin, x2, y2) >= 127)
        far_findline_righthand_adaptive(mat_bin, BLOCK_SIZE, CLIP_VALUE,
            x2, y2, ipts, ipts_num);
    else *ipts_num = 0;
}

//根据顺序点集找角点
void Find_corner(float rpts[][2], int rpts_num, int corner_point[2], float* angle)
{
    *angle = 180;
    float angle_xy[2]{};
    if (rpts_num > 30) {
        float min_angle = 180;
        float min_angle_x = 0;
        float min_angle_y = 0;
        int i = 0;
        float k[3][2];
        while (true) {
            k[0][0] = rpts[i][0];
            k[0][1] = rpts[i][1];
            k[1][0] = rpts[i + 15][0];
            k[1][1] = rpts[i + 15][1];
            k[2][0] = rpts[i + 30][0];
            k[2][1] = rpts[i + 30][1];
            float BAx = (float)(k[0][0] - k[1][0]);
            float BAy = (float)(k[0][1] - k[1][1]);
            float BCx = (float)(k[2][0] - k[1][0]);
            float BCy = (float)(k[2][1] - k[1][1]);
            // 计算点积
            float dotProduct = (BAx * BCx) + (BAy * BCy);
            // 计算向量的模长
            float magnitudeBA = sqrt(BAx * BAx + BAy * BAy);
            float magnitudeBC = sqrt(BCx * BCx + BCy * BCy);
            // 计算夹角的余弦值
            float cosTheta = dotProduct / (magnitudeBA * magnitudeBC);
            // 计算夹角（弧度）
            float angleRad = acos(cosTheta);
            // 转换为角度
            float angleDeg = angleRad * (180.0f / PI);
            if (angleDeg < min_angle) {
                min_angle = angleDeg;
                min_angle_x = (float)k[1][0];
                min_angle_y = (float)k[1][1];
            }
            i += 5;
            if (i + 30 >= rpts_num)break;
        }
        *angle = min_angle;
        map_perspective(min_angle_x, min_angle_y, angle_xy, 1);
        corner_point[0] = (int)angle_xy[0];
        corner_point[1] = (int)angle_xy[1];
    }
}

// 点集三角滤波
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) {
    assert(kernel % 2 == 1);

    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}

// 点集等距采样  使走过的采样前折线段的距离为`dist`
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int* num2,
    float dist) {
    int remain = 0, len = 0;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i + 1][0] - x0;
        float dy = pts_in[i + 1][1] - y0;
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;

        while (remain < dn && len < *num2) {
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;

            len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}

// 点集局部角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[],
    int dist) {
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

// 角度变化率非极大抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel) {
    assert(kernel % 2 == 1);

    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;
                break;
            }
        }
    }
}

// 左边线跟踪中线
void track_leftline(float pts_in[][2], int num, float pts_out[][2],
    int approx_num, float dist) {
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] -
            pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] -
            pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);

        dx /= dn;
        dy /= dn;

        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;
    }
}

// 右边线跟踪中线
void track_rightline(float pts_in[][2], int num, float pts_out[][2],
    int approx_num, float dist) {
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] -
            pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] -
            pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);

        dx /= dn;
        dy /= dn;

        pts_out[i][0] = pts_in[i][0] + dy * dist;
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}

/* *********************************************************************** */
/* *********************************************************************** */
/* *********************************************************************** */