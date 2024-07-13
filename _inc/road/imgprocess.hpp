#pragma once
#include <onnxruntime_cxx_api.h>

#include <cstdint>
#include <opencv2/opencv.hpp>

#include "../common/model_config.hpp"
#include "../core/detection.hpp"
#include "../utils/logger_helper.hpp"
#include "../utils/serial_packets.hpp"
#include "_define.hpp"
#include "_extern.hpp"
#include "_sideline.hpp"
#include "elements/_circle.hpp"
#include "elements/_cross.hpp"
#include "elements/_garage.hpp"
#include "elements/_gas.hpp"
#include "elements/_ramp.hpp"
#include "elements/_work.hpp"
#include "elements/_bomb.hpp"
#include "elements/_yroad.hpp"
#include "elements/_slowzone.hpp"
#include "elements/_save.hpp"
#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <chrono>
#include <queue>
#include <thread>
#include <future>
#include <functional>



class ImageProcess {
public:
    ImageProcess() {};
    ~ImageProcess() {};


    int init(std::string config_path, bool is_result);
    void process(cv::Mat& mat_origin, cv::Mat& mat_result, Payload_t& payload);
    void calculate_angle(void);
    void get_edge(int mode = 0);
    void Attitude_Ring(float nowYaw, float expYaw, float P, float D); //姿态环
private:
    bool _is_result = false;  // 是否生成处理后的图像

    float cx = IMAGE_WIDTH / 2.0f;  // 车轮对应点 (纯跟踪起始点)
    float cy = IMAGE_HEIGHT * 0.99f;

    int aim_idx_far = 0;  // 远预锚点位置
    int aim_idx_near = 0;  // 近预锚点位置

    float mea_0 = 0.0f;  // 左直线拟合平均绝对误差
    float mea_1 = 0.0f;  // 右直线拟合平均绝对误差

    std::vector<POINT> bezier_line;  // 中线贝塞尔曲线拟合
public:
    float aim_angle_p_k;
    float aim_angle_p;
    float aim_angle_d;

    uint16_t speed_diff = 0;  // 减速率

public:
    std::shared_ptr<ControlConfig> _config;           // 跑车配置文件
    std::vector<DetectionPredictResult> _det_result;  // 目标检测结果

    cv::Mat mat_result;
    cv::Mat mat_bin;  // 原图转二值化图
    cv::Mat mat_lab;  // 原图转 LAB 图
    cv::Mat mat_lab2;  // 原图转 LAB 图
    flag_track_e flag_track = TRACK_MIDDLE;  // 巡线选择

    int begin_x_l = BEGIN_X;                // 巡线横坐标起始点 左
    int begin_x_r = IMAGE_WIDTH - BEGIN_X;  // 巡线横坐标起始点 右
    int begin_y_t = BEGIN_Y;                // 巡线纵坐标起始点

    int16_t aim_speed = 0;        // 速度量
    int16_t aim_speed_shift = 0;  // 变速计数器
    float aim_angle = 0.0f;       // 偏差量
    float aim_angle_last = 0.0f;  // 偏差量 上一帧
    float aim_sigma = 0.0f;       // 偏差方差
    float aim_distance_f = 0.0f;  // 远锚点
    float aim_distance_n = 0.0f;  // 近锚点

    int element_begin_id = 0;         // 特殊元素中线起始点
    float element_over_route = 0.0f;  // 元素结束路程
    bool element_over = true;         // 元素结束标志
    int element_identify = 0;         // 识别元素

    char string_buf[64];               // 窗口信息缓存
    char string_log[32] = "yyds_hardware car";  // 窗口日志缓存

    bool special_w_c = false;  // 施工区到十字慢速处理
    bool special_w = false;    // 施工区结束特殊处理
    bool special_g = false;    // 加油站结束特殊处理

    /* *********************************************************************** */
    /* ****************************** 下位机数据 ****************************** */
    /* *********************************************************************** */

public:
    Encoder_t encoder = { 0 };

    /* *********************************************************************** */
    /* ***************************** 目标检测数据 ***************************** */
    /* *********************************************************************** */

public:
    bool detection_new = false;

    string detection_labels[20] = {
            "bomb"
            "cone"
            "block"
            "bridge"
            "crosswalk"
            "thief"
            "evil"
            "patient"
            "tumble"
            "prop"
            "safety"
            "safety"
            "danger"
    };

    enum DetectionLabel {
        label_bomb = 0, //炸弹

        label_bridge,   //桥

        label_safety,   //普通车辆

        label_cone,     //锥桶

        label_crosswalk,//斑马线

        label_danger,   //危险车辆

        label_evil,     //恶人
        label_block,    //黑块

        label_patient,  //病人

        label_prop,     //道具小车

        label_spy,      //嫌疑车辆

        label_thief,    //贼

        label_tumble,   //跌倒

    };
    // 重构坐标类
    struct DETECTION {
        int label;
        int x;           //横坐标
        int y;           //纵坐标
        int w;           //宽
        int h;           //高
        float score;    //相似度
    };
    std::vector<DETECTION> detection_result;  // 目标检测结果

    /* *********************************************************************** */
    /* ******************************* 角点数据 ******************************* */
    /* *********************************************************************** */
public:
    // L 角点
    bool Lpt0_found, Lpt1_found;
    float Lpt0_angle, Lpt1_angle;
    int Lpt0_point[2], Lpt1_point[2];//左右角点坐标
    bool far_Lpt0_found, far_Lpt1_found;
    float far_Lpt0_angle, far_Lpt1_angle;
    int far_Lpt0_point[2], far_Lpt1_point[2];//远处左右角点坐标
    // 长直道
    bool is_straight0, is_straight1;
    //主赛道预测线长度
    float max_white_line_length;
    //横向扫描得到的横线数据集
    std::vector<POINT> white_row_start_point[40];//白行起始点
    std::vector<POINT> white_row_end_point[40];//白行结束点

    /* *********************************************************************** */
    /* ******************************* 边线数据 ******************************* */
    /* *********************************************************************** */

public:
    int begin_x, begin_y;//找线开始坐标


    // 原图左右中线数据定义
    int ipts0[IMAGE_HEIGHT][2];  // 左: 0
    int ipts1[IMAGE_HEIGHT][2];  // 右: 1
    int iptsc[IMAGE_HEIGHT];     // 中: c
    int ipts0_num, ipts1_num, rptsc_num;

    int iptsc0[IMAGE_HEIGHT] = { 0 };
    int iptsc1[IMAGE_HEIGHT] = { 0 };


    //远线左右线定义
    int far_ipts0[IMAGE_HEIGHT][2];
    int far_ipts1[IMAGE_HEIGHT][2];
    int far_ipts0_num, far_ipts1_num;

    // 透视变换后左右中线  滤波: b
    float rpts0b[IMAGE_HEIGHT][2];
    float rpts1b[IMAGE_HEIGHT][2];
    float rptscb[IMAGE_HEIGHT][2];

    // 透视变换后左右中线  等距采样: s
    float rpts0s[IMAGE_HEIGHT][2];
    float rpts1s[IMAGE_HEIGHT][2];
    float rptscs[IMAGE_HEIGHT][2];
    int rpts0s_num, rpts1s_num, rptscs_num;
    // 左右边线局部角度变化率: a
    float rpts0a[IMAGE_HEIGHT];
    float rpts1a[IMAGE_HEIGHT];
    // 左右边线局部角度变化率非极大抑制: an
    float rpts0an[IMAGE_HEIGHT];
    float rpts1an[IMAGE_HEIGHT];
    // 左右边线偏移中线: c
    float rptsc0[IMAGE_HEIGHT][2];
    float rptsc1[IMAGE_HEIGHT][2];
    int rptsc0_num, rptsc1_num;

public:
    std::vector<POINT> edge_det;        // AI元素检测边缘点集
    std::vector<POINT> edge_left;       // 赛道左边缘点集   注意: 此点集 x 与 y 位置相反 !!!
    std::vector<POINT> edge_right;      // 赛道右边缘点集
    std::vector<POINT> last_edge_left;  // 记录上一场边缘点集 (丢失边)
    std::vector<POINT> last_edge_right;
    std::vector<vector<POINT>> path_last_edge_left; //记录左路径
    std::vector<vector<POINT>> path_last_edge_right; //记录右路径
    std::vector<POINT> cones_edge_left;     //左边锥桶
    std::vector<POINT> cones_edge_right;    //右边锥桶
    std::vector<POINT> last_cones_edge_left; //记录前一场左边缘锥桶点集
    std::vector<POINT> last_cones_edge_right; //记录前一场右边缘锥桶点集

public:
    int det_ipts[IMAGE_HEIGHT][2];     // 加油站 & 施工区 边线原图
    float det_rpts[IMAGE_HEIGHT][2];   // 加油站 & 施工区 边线透视
    float det_rptsb[IMAGE_HEIGHT][2];  // 加油站 & 施工区 边线滤波
    float det_rptss[IMAGE_HEIGHT][2];  // 加油站 & 施工区 边线等距采样
    int det_ipts_num, det_rptss_num;

public:
    // 中线
    float(*rpts)[2];
    int rpts_num;
    // 归一化中线
    float rptsn[IMAGE_HEIGHT][2];
    int rptsn_num;
    bool flag_rpts = false;
    // 透视变换临时变量
    float trans[2];

    /* *********************************************************************** */
    /* *********************************************************************** */
    /* *********************************************************************** */
};

enum Control_element {
    none_element,   //无元素    
    cross_element,  //十字
    circle_element, //环岛
    work_element,   //工作区
    garage_element, //车库
    bomb_element,   //炸弹
    ramp_element    //桥
};

class Control_Flag {
private:
    std::vector<int> data;

public:
    void push(int value); //入栈
    void pop(); //出栈
    int top(); //获取栈顶元素
    bool contains(int value); //判断栈中是否包含元素
    void remove(int value); //删除栈中指定元素
};

