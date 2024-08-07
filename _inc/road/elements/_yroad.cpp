#include "_yroad.hpp"

#include "../_define.hpp"
#include "../imgprocess.hpp"
#include "../utils/logger_helper.hpp"

#undef LOG_TAG
#define LOG_TAG "Yroad"

flag_yroad_e flag_yroad = YROAD_NONE;

// 引用图像处理
extern ImageProcess imgprocess;
extern Control_Flag control_flag;
#define IMG imgprocess

// 变量定义
float yroad_route = 0;  // 追逐区编码器积分
uint8_t yroad_position = 0;  // 道具小车位置
POINT yroad_near_cone;  // 左右较近的锥桶
uint8_t last_flag_track;
// 追逐区角点
int yroad_Lpt_id;      // 追逐区角点位置
bool yroad_Lpt_found;  // 追逐区角点检测标志


/* ********************************************************************* */

void check_yroad() {
	if (!IMG.element_over &&
		!control_flag.contains(yroad_element) &&    //当前没在追逐区中
		!control_flag.contains(bomb_element) &&    //当前没在炸弹中
		!control_flag.contains(cross_element) &&    //当前没在十字路口中
		!control_flag.contains(circle_element) &&   //当前没在环岛中
		!control_flag.contains(work_element) &&     //当前没在工作区中
		!control_flag.contains(garage_element) &&      //当前没在入库中
		!control_flag.contains(ramp_element)
		) {
		for (auto r : IMG.detection_result) {
			switch (r.label) {
				case IMG.label_safety://普通车辆
				{
					if (r.score > 0.7 && r.y > IMAGE_HEIGHT * 0.1 && IMG.ipts0_num > 180 && IMG.ipts1_num > 180) {
						double L0 =
							dis_point_line(
								POINT(IMG.ipts0[0][0], IMG.ipts0[0][1]),
								POINT(IMG.ipts0[IMG.ipts0_num - 1][0], IMG.ipts0[IMG.ipts0_num - 1][1]),
								POINT(r.x, r.y));
						double L1 =
							dis_point_line(
								POINT(IMG.ipts1[0][0], IMG.ipts1[0][1]),
								POINT(IMG.ipts1[IMG.ipts1_num - 1][0], IMG.ipts1[IMG.ipts1_num - 1][1]),
								POINT(r.x, r.y));
						if (L0 < L1) yroad_position = 0;
						else yroad_position = 1;

						IMG.element_identify = 1;
						control_flag.push(yroad_element);
						yroad_route = IMG.encoder.route;
						flag_yroad = YROAD_FOUND_1;
						MAT_LOG("YROAD_FOUND_1");
						return;
					}
				}
				return;
				case IMG.label_spy://嫌疑车辆
				{
					if (r.score > 0.7 && r.y > IMAGE_HEIGHT * 0.1 && IMG.ipts0_num > 180 && IMG.ipts1_num > 180) {
						double L0 =
							dis_point_line(
								POINT(IMG.ipts0[0][0], IMG.ipts0[0][1]),
								POINT(IMG.ipts0[IMG.ipts0_num - 1][0], IMG.ipts0[IMG.ipts0_num - 1][1]),
								POINT(r.x, r.y));
						double L1 =
							dis_point_line(
								POINT(IMG.ipts1[0][0], IMG.ipts1[0][1]),
								POINT(IMG.ipts1[IMG.ipts1_num - 1][0], IMG.ipts1[IMG.ipts1_num - 1][1]),
								POINT(r.x, r.y));
						if (L0 < L1) yroad_position = 0;
						else yroad_position = 1;
						IMG.element_identify = 1;
						control_flag.push(yroad_element);
						yroad_route = IMG.encoder.route;
						flag_yroad = YROAD_FOUND_2;
						MAT_LOG("YROAD_FOUND_2");
						return;
					}
				}
				return;
				case IMG.label_danger://危险车辆
				{
					if (r.score > 0.7 && r.y > IMAGE_HEIGHT * 0.1 && IMG.ipts0_num > 180 && IMG.ipts1_num > 180) {
						double L0 =
							dis_point_line(
								POINT(IMG.ipts0[0][0], IMG.ipts0[0][1]),
								POINT(IMG.ipts0[IMG.ipts0_num - 1][0], IMG.ipts0[IMG.ipts0_num - 1][1]),
								POINT(r.x, r.y));
						double L1 =
							dis_point_line(
								POINT(IMG.ipts1[0][0], IMG.ipts1[0][1]),
								POINT(IMG.ipts1[IMG.ipts1_num - 1][0], IMG.ipts1[IMG.ipts1_num - 1][1]),
								POINT(r.x, r.y));
						if (L0 < L1) yroad_position = 0;
						else yroad_position = 1;
						IMG.element_identify = 1;
						control_flag.push(yroad_element);
						yroad_route = IMG.encoder.route;
						flag_yroad = YROAD_FOUND_3;
						MAT_LOG("YROAD_FOUND_3");
						return;
					}
				}
				return;
			}
		}
		//道具小车在左边
		if (IMG.ipts1_num > 180 &&// 右边长道
			IMG.ipts0_num > 180 // 左边长道
			) {
			if ((!IMG.Lpt1_found &&  // 右角点未找到
				!IMG.far_Lpt1_found &&  // 右远角点未找到
				IMG.Lpt0_found &&//左角点找到
				IMG.far_Lpt0_found) //左远角点找到
				||
				(IMG.Lpt1_found &&  // 右角点找到
					IMG.far_Lpt1_found &&  // 右远角点找到
					!IMG.Lpt0_found &&//左角点未找到
					!IMG.far_Lpt0_found) //左远角点未找到
				) {
				IMG.element_identify = 1;
				control_flag.push(yroad_element);
				yroad_route = IMG.encoder.route;
				flag_yroad = YROAD_FOUND_0;
				MAT_LOG("YROAD_FOUND_0");
				return;
			}
		}
	}
}

/* ********************************************************************* */

void run_yroad() {
	if (control_flag.top() == circle_element) {//当前为圆环元素

		if (YROAD_FOUND_0) {
			IMG.aim_speed = 100;

			for (auto r : IMG.detection_result) {
				switch (r.label) {
					case IMG.label_safety://普通车辆
					{
						if (r.score > 0.7 && IMG.ipts0_num > 180 && IMG.ipts1_num > 180) {
							double L0 =
								dis_point_line(
									POINT(IMG.ipts0[0][0], IMG.ipts0[0][1]),
									POINT(IMG.ipts0[IMG.ipts0_num - 1][0], IMG.ipts0[IMG.ipts0_num - 1][1]),
									POINT(r.x, r.y));
							double L1 =
								dis_point_line(
									POINT(IMG.ipts1[0][0], IMG.ipts1[0][1]),
									POINT(IMG.ipts1[IMG.ipts1_num - 1][0], IMG.ipts1[IMG.ipts1_num - 1][1]),
									POINT(r.x, r.y));
							if (L0 < L1) yroad_position = 0;
							else yroad_position = 1;

							IMG.element_identify = 1;
							yroad_route = IMG.encoder.route;
							flag_yroad = YROAD_FOUND_1;
							MAT_LOG("YROAD_FOUND_1");
							return;
						}
					}
					return;
					case IMG.label_spy://嫌疑车辆
					{
						if (r.score > 0.7 && IMG.ipts0_num > 180 && IMG.ipts1_num > 180) {
							double L0 =
								dis_point_line(
									POINT(IMG.ipts0[0][0], IMG.ipts0[0][1]),
									POINT(IMG.ipts0[IMG.ipts0_num - 1][0], IMG.ipts0[IMG.ipts0_num - 1][1]),
									POINT(r.x, r.y));
							double L1 =
								dis_point_line(
									POINT(IMG.ipts1[0][0], IMG.ipts1[0][1]),
									POINT(IMG.ipts1[IMG.ipts1_num - 1][0], IMG.ipts1[IMG.ipts1_num - 1][1]),
									POINT(r.x, r.y));
							if (L0 < L1) yroad_position = 0;
							else yroad_position = 1;
							IMG.element_identify = 1;
							yroad_route = IMG.encoder.route;
							flag_yroad = YROAD_FOUND_2;
							MAT_LOG("YROAD_FOUND_2");
							return;
						}
					}
					return;
					case IMG.label_danger://危险车辆
					{
						if (r.score > 0.7 && IMG.ipts0_num > 180 && IMG.ipts1_num > 180) {
							double L0 =
								dis_point_line(
									POINT(IMG.ipts0[0][0], IMG.ipts0[0][1]),
									POINT(IMG.ipts0[IMG.ipts0_num - 1][0], IMG.ipts0[IMG.ipts0_num - 1][1]),
									POINT(r.x, r.y));
							double L1 =
								dis_point_line(
									POINT(IMG.ipts1[0][0], IMG.ipts1[0][1]),
									POINT(IMG.ipts1[IMG.ipts1_num - 1][0], IMG.ipts1[IMG.ipts1_num - 1][1]),
									POINT(r.x, r.y));
							if (L0 < L1) yroad_position = 0;
							else yroad_position = 1;
							IMG.element_identify = 1;
							yroad_route = IMG.encoder.route;
							flag_yroad = YROAD_FOUND_3;
							MAT_LOG("YROAD_FOUND_3");
							return;
						}
					}
					return;
				}
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

		}
		else if (YROAD_FOUND_1) {
			IMG.aim_speed = 100;

			if (IMG.encoder.route - yroad_route > 200) {
				IMG.element_over = true;
				IMG.element_over_route = IMG.encoder.route;
				control_flag.pop();//移除环岛
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
		}
		else if (YROAD_FOUND_2) {
			IMG.aim_speed = 100;

			if (IMG.encoder.route - yroad_route > 200) {
				IMG.element_over = true;
				IMG.element_over_route = IMG.encoder.route;
				control_flag.pop();//移除环岛
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
		}
		else if (YROAD_FOUND_3) {
			IMG.aim_speed = 100;

			if (IMG.encoder.route - yroad_route > 200) {
				IMG.element_over = true;
				IMG.element_over_route = IMG.encoder.route;
				control_flag.pop();//移除环岛
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
		}
	}
}

/* ********************************************************************* */