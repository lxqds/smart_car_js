#ifndef _SIDELINE_HPP_
#define _SIDELINE_HPP_

#include <assert.h>

#include <cstdint>
#include <opencv2/opencv.hpp>

#include "imgprocess.hpp"
#include "predictor_api.h"

/* ********************************************************************* */

#define AT_IMAGE(img, x, y) (img.at<uint8_t>(y, x))
#define AT_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define AT_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define AT_MINMAX(input, low, upper) MIN(MAX(input, low), upper)

/* ********************************************************************* */

/* 窗口绘制日志 */
#define MAT_LOG(...)                      \
    sprintf(IMG.string_log, __VA_ARGS__); \
    log_i(__VA_ARGS__);

/* 窗口绘制信息 */
#define MAT_INFO(mat, point, size, ...)                                        \
    sprintf(string_buf, __VA_ARGS__);                                          \
    cv::putText(mat, (std::string)string_buf, point, cv::FONT_HERSHEY_SIMPLEX, \
                size, cv::Scalar(0, 0, 255));

/* ********************************************************************* */

// 透视变换 (0:原图->俯视, 1:俯视->原图)
void map_perspective(float x, float y, float loc[2], uint8_t mode);

/* ********************************************************************* */

// 位置式 PID 角度外环
float pid_realize_a(float actual, float set, float _p, float _d);

// 位置式 PID 角速度内环
int pid_realize_o(int actual, int set, float _p, float _d);

// 滑动平均滤波
float filter(float value);

/* ********************************************************************* */

// 赛道点集的方差计算
double sigma(float pts[][2], int num);
double sigma(std::vector<POINT> vec);

// 点到直线的距离计算
double dis_point_line(POINT a, POINT b, POINT p);

// 两点间的距离计算
double dis_point_point(POINT a, POINT b);

// 根据两点和第三点的y值求第三点的x值
double findThirdPointX(POINT a, POINT b, double y);

// 直线拟合 (返回平均绝对误差)
float fit_line(float pts[][2], int num, int cut_h);
//计算返回斜率
float Calculate_K(std::vector<POINT> edge);
// 贝塞尔曲线拟合
std::vector<POINT> bezier(double dt, std::vector<POINT> input);

/* ********************************************************************* */

bool sort_y_up(POINT a, POINT b);    // 升序
bool sort_y_down(POINT a, POINT b);  // 降序
bool sort_x_up(POINT a, POINT b);    // 升序
bool sort_x_down(POINT a, POINT b);  // 降序

//两点距离公式
float distance_for_points(POINT a, POINT b);
//搜索左右边缘的锥桶
void search_cones_edge(std::vector<POINT> cones, std::vector<POINT>* left_cones, std::vector<POINT>* right_cones);

// 从AI检测结果中检索锥桶坐标集合
std::vector<POINT> search_targets(std::vector<DetectionPredictResult> predict);

// 从AI检测结果中检索锥桶y坐标 平均值
float get_AIorcolor_AVG_Y(std::vector<DetectionPredictResult> predict);

//在赛道内找黄色锥桶
POINT search_right_near_in(std::vector<POINT> pointsEdge, cv::Mat matt);
//在赛道内找黄色锥桶
POINT search_left_near_in(std::vector<POINT> pointsEdge, cv::Mat matt);
// 搜索 屏幕中间以下 黄色锥桶 距离屏幕最下方距离
uint16_t search_mid_cone_getH(cv::Mat matt);

// AI搜索距离赛道左边缘最近的黄色色块 (黄色锥桶)
POINT AI_search_left_near(std::vector<DetectionPredictResult> predict, std::vector<POINT> pointsEdge);
// AI搜索距离赛道右边缘最近的黄色色块 (黄色锥桶)
POINT AI_search_right_near(std::vector<DetectionPredictResult> predict, std::vector<POINT> pointsEdge);


//AI搜索距离小车高度最近锥桶
POINT AI_search_height_near(std::vector<DetectionPredictResult> predict);
// 搜索距离赛道左边缘最近的红色色块 (红色锥桶)
POINT search_left_near(std::vector<POINT> pointsEdge, cv::Mat matt);

// 搜索距离赛道左边缘上面最近的黄色色块 (黄色锥桶)
POINT search_left_near2(std::vector<POINT> pointsEdge, cv::Mat matt);

// 搜索距离赛道右边缘最近的红色色块 (红色锥桶)
POINT search_right_near(std::vector<POINT> pointsEdge, cv::Mat matt);

//找左边线离锥桶最近的边线点的位次,x为行，第几行
uint16_t search_break_left(std::vector<POINT> pointsEdge, cv::Mat matt);

//找右边线离锥桶最近的边线点的位次,x为行，第几行
uint16_t search_break_right(std::vector<POINT> pointsEdge, cv::Mat matt);

// 搜索距离赛道左边缘上15最近的黄色色块 (黄色锥桶)
POINT search_left_near15(std::vector<POINT> pointsEdge, cv::Mat matt);
// 搜索左方的锥桶集合 ( IMAGE_WIDTH / 3 )
std::vector<POINT> search_left_cones(std::vector<POINT> pointsCone);

// 搜索右方的锥桶集合 ( IMAGE_WIDTH * 2 / 3 )
std::vector<POINT> search_right_cones(std::vector<POINT> pointsCone);

// 搜索左上方的锥桶集合 (除了右下角)
std::vector<POINT> search_leftUp_cones(std::vector<POINT> pointsCone);

// 搜索右上方的锥桶集合 (除了左下角)
std::vector<POINT> search_rightUp_cones(std::vector<POINT> pointsCone);

// 搜索右上方的锥桶
POINT search_rightUp_cone(std::vector<POINT> pointsCone);

// 搜索左上方的锥桶
POINT search_leftUp_cone(std::vector<POINT> pointsCone);

// 搜索右下方的锥桶
POINT search_rightDown_cone(std::vector<POINT> pointsCone);

// 搜索左下方的锥桶
POINT search_leftDown_cone(std::vector<POINT> pointsCone);

/* ********************************************************************* */

class CVFind_Clone
{
public:
    void init();
    std::vector<cv::Rect> search_all_cone_rect(cv::Mat matt);
private:
    const int cone_front[8][2] = { {1, 0},{0, -1}, {0, 1}, {-1, 0},{-1, -1},{1, 1},{-1, 1},{1, -1} };
    const int cone_arger[8] = { 2,0,4,6,7,3,5,1 };
    cv::Mat Clone_MAP;
    bool bl_map[IMAGE_WIDTH][IMAGE_HEIGHT]; //搜索地图
    void coverTheMat(cv::Rect Rect, cv::Mat map);
    bool pt_IsEdge(int x, int y);
    cv::Rect find_Width_Begin_x(int x, int y, int turncs, int nowfx, cv::Rect lastRect);

};

/* ********************************************************************* */
// 限制范围
int clip(int x, int low, int up);

// // 左手迷宫巡线
// void findline_lefthand_adaptive(cv::Mat img, int block_size, int clip_value,
//     int x, int y, int pts[][2], int* num);

// // 右手迷宫巡线
// void findline_righthand_adaptive(cv::Mat img, int block_size, int clip_value,
//     int x, int y, int pts[][2], int* num);

//向上找左线
void find_left_line(cv::Mat mat_bin, int x, int y, int ipts[][2], int* ipts_num);
//向上找右线
void find_right_line(cv::Mat mat_bin, int x, int y, int ipts[][2], int* ipts_num);
//向下找左线
void find_far_left_line(cv::Mat mat_bin, int x, int y, int ipts[][2], int* ipts_num);
//向下找右线
void find_far_right_line(cv::Mat mat_bin, int x, int y, int ipts[][2], int* ipts_num);
//根据顺序点集找角点
void Find_corner(float rpts[][2], int rpts_num, int corner_point[2], float* angle);

// 点集三角滤波
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);

// 点集等距采样  使走过的采样前折线段的距离为`dist`
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int* num2,
    float dist);

// 点集局部角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[],
    int dist);

// 角度变化率非极大抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);

// 左边线跟踪中线
void track_leftline(float pts_in[][2], int num, float pts_out[][2],
    int approx_num, float dist);

// 右边线跟踪中线
void track_rightline(float pts_in[][2], int num, float pts_out[][2],
    int approx_num, float dist);

/* ********************************************************************* */

#endif
