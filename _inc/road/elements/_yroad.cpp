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

}

/* ********************************************************************* */

void run_yroad() {

}

/* ********************************************************************* */