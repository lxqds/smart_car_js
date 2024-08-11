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
StopWatch yroad_stop_clock;//停车计时
POINT Lpt0_point_first;
POINT Lpt1_point_first;



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
						imgprocess.brake();
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
						imgprocess.brake();
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
						imgprocess.brake();
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
		if (IMG.ipts1_num > 180 &&// 右边长道
			IMG.ipts0_num > 180 // 左边长道
			) {
			float R0 = 10.0f, R1 = 10.0f;
			// 左线直线拟合
			if (IMG.ipts0_num > 10)
				R0 = fit_line1(IMG.ipts0, IMG.ipts0_num, 60);
			// 右线直线拟合
			if (IMG.ipts1_num > 10)
				R1 = fit_line1(IMG.ipts1, IMG.ipts1_num, 60);

			if (R1 < 1.0f && !IMG.Lpt1_found &&  // 右角点未找到
				!IMG.far_Lpt1_found &&  // 右远角点未找到
				IMG.Lpt0_found &&//左角点找到
				IMG.far_Lpt0_found //左远角点找到
				) {

				for (size_t direction_i = 0; direction_i < IMG.ipts0_direction.size(); ++direction_i) {
					if (IMG.ipts0_direction[direction_i] == 4) {
						int sign = 0;
						int count = 0;
						for (size_t direction_j = direction_i; direction_j < IMG.ipts0_direction.size(); ++direction_j) {
							if (sign == 0) {
								if (IMG.ipts0_direction[direction_j] == 1) sign = 1;
								else if (IMG.ipts0_direction[direction_j] != 4)break;
							}
							else if (sign == 1) {
								if (IMG.ipts0_direction[direction_j] == 2 && count > 2) {
									IMG.element_identify = 1;
									yroad_position = 0;
									imgprocess.brake();
									control_flag.push(yroad_element);
									yroad_route = IMG.encoder.route;
									flag_yroad = YROAD_FOUND_0;
									MAT_LOG("YROAD_FOUND_0");
									return;
								}
								else if (IMG.ipts0_direction[direction_j] == 1) {
									if (++count > 8)break;
								}
								else break;
							}
						}
					}
				}

				// float x0 = IMG.ipts0[0][0];
				// float y0 = IMG.ipts0[0][1];
				// float x1 = IMG.ipts0[IMG.ipts0_num - 1][0];
				// float y1 = IMG.ipts0[IMG.ipts0_num - 1][1];
				// float r = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
				// float a = (x1 - x0) / r;
				// float b = (y1 - y0) / r;
				// int flag = 0;
				// for (int i = x0 + 2;i < x0 + 15; i++) {
				// 	int sign = 0;
				// 	int count = 0;
				// 	for (float L = 1; ; L++) {
				// 		int x = (int)(i + L * a);
				// 		int y = (int)(y0 + L * b);
				// 		if (x < 0 || x >= IMAGE_WIDTH || y < 0 || y >= IMAGE_HEIGHT || y < y1)break;
				// 		if (sign == 0) {
				// 			if (AT_IMAGE(IMG.mat_bin, (int)x, (int)y) < 127)count++;
				// 			else count = 0;
				// 			if (count > 10) {
				// 				L -= 10;
				// 				sign = 1;
				// 				count = 0;
				// 				cv::line(IMG.mat_result, cv::Point(x, y),
				// 					cv::Point(x, y),
				// 					cv::Scalar(255, 0, 255), 1);
				// 			}
				// 		}
				// 		else if (sign == 1) {
				// 			if (AT_IMAGE(IMG.mat_bin, (int)x, (int)y) > 127)count++;
				// 			else count = 0;
				// 			if (count > 5) {
				// 				flag++;
				// 				count = 0;
				// 				break;
				// 			}
				// 		}
				// 	}
				// 	if (flag >= 5) {

				// 	}
				// }
			}
			else if (R0 < 1.0f && IMG.Lpt1_found &&  // 右角点找到
				IMG.far_Lpt1_found &&  // 右远角点找到
				!IMG.Lpt0_found &&//左角点未找到
				!IMG.far_Lpt0_found //左远角点未找到 
				) {
				for (size_t direction_i = 0; direction_i < IMG.ipts1_direction.size(); ++direction_i) {
					if (IMG.ipts1_direction[direction_i] == 2) {
						int sign = 0;
						int count = 0;
						for (size_t direction_j = direction_i; direction_j < IMG.ipts1_direction.size(); ++direction_j) {
							if (sign == 0) {
								if (IMG.ipts1_direction[direction_j] == 1) sign = 1;
								else if (IMG.ipts1_direction[direction_j] != 2)break;
							}
							else if (sign == 1) {
								if (IMG.ipts1_direction[direction_j] == 4) {
									IMG.element_identify = 1;
									yroad_position = 0;
									imgprocess.brake();
									control_flag.push(yroad_element);
									yroad_route = IMG.encoder.route;
									flag_yroad = YROAD_FOUND_0;
									MAT_LOG("YROAD_FOUND_0");
									return;
								}
								else if (IMG.ipts1_direction[direction_j] == 1) {
									if (++count > 8)break;
								}
								else break;
							}
						}
					}
				}
				// float x0 = IMG.ipts1[0][0] - IMAGE_WIDTH / 2;
				// float y0 = IMAGE_HEIGHT - IMG.ipts1[0][1];

				// float x1 = IMG.ipts1[IMG.ipts1_num - 1][0] - IMAGE_WIDTH / 2;
				// float y1 = IMAGE_HEIGHT - IMG.ipts1[IMG.ipts1_num - 1][1];

				// float x0 = IMG.ipts1[0][0];
				// float y0 = IMG.ipts1[0][1];
				// float x1 = IMG.ipts1[IMG.ipts1_num - 1][0];
				// float y1 = IMG.ipts1[IMG.ipts1_num - 1][1];
				// float r = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
				// float a = (x1 - x0) / r;
				// float b = (y1 - y0) / r;
				// int flag = 0;
				// for (int i = x0 - 2;i > x0 - 15; i--) {
				// 	int sign = 0;
				// 	int count = 0;
				// 	for (float L = 1; ; L++) {
				// 		int x = (int)(i + L * a);
				// 		int y = (int)(y0 + L * b);
				// 		if (x < 0 || x >= IMAGE_WIDTH || y < 0 || y >= IMAGE_HEIGHT || y < y1)break;
				// 		if (sign == 0) {
				// 			if (AT_IMAGE(IMG.mat_bin, (int)x, (int)y) < 127)count++;
				// 			else count = 0;
				// 			if (count > 10) {
				// 				L -= 10;
				// 				sign = 1;
				// 				count = 0;
				// 				cv::line(IMG.mat_result, cv::Point(x, y),
				// 					cv::Point(x, y),
				// 					cv::Scalar(255, 0, 255), 1);
				// 			}
				// 		}
				// 		else if (sign == 1) {
				// 			if (AT_IMAGE(IMG.mat_bin, (int)x, (int)y) > 127)count++;
				// 			else count = 0;
				// 			if (count > 5) {
				// 				flag++;
				// 				count = 0;
				// 				break;
				// 			}
				// 		}
				// 	}
				// 	if (flag >= 5) {

				// 	}
				// }
			}
		}
	}
}
				/* ********************************************************************* */

void run_yroad() {
	if (control_flag.top() == yroad_element) {//当前为圆环元素

		if (flag_yroad == YROAD_FOUND_0) {



			IMG.aim_distance_f = 0.6;
			IMG.aim_distance_n = 0.3;
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
							MAT_LOG("YROAD_FOUND_1_%d", yroad_position);
							return;
						}
					}
					return;
					case IMG.label_spy://嫌疑车辆
					{
						if (r.score > 0.5 && IMG.ipts0_num > 180 && IMG.ipts1_num > 180) {
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
							Lpt0_point_first = POINT(0, 0);
							Lpt1_point_first = POINT(0, 0);

							IMG.element_identify = 1;
							yroad_route = IMG.encoder.route;
							flag_yroad = YROAD_FOUND_2;
							MAT_LOG("YROAD_FOUND_2_%d", yroad_position);
							return;
						}
					}
					return;
					case IMG.label_danger://危险车辆
					{
						if (r.score > 0.8 && IMG.ipts0_num > 180 && IMG.ipts1_num > 180) {
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
							MAT_LOG("YROAD_FOUND_3_%d", yroad_position);
							return;
						}
					}
					return;
				}
			}
			int Lpt0_point[2] = { 0 };
			int Lpt1_point[2] = { 0 };
			if (Find_corner_first(IMG.rpts0, IMG.ipts0_num, Lpt0_point, 110))Lpt0_point_first = POINT(Lpt0_point[0], Lpt0_point[1]);
			else Lpt0_point_first = POINT(0, 0);
			if (Find_corner_first(IMG.rpts1, IMG.ipts1_num, Lpt1_point, 110)) Lpt1_point_first = POINT(Lpt1_point[0], Lpt1_point[1]);
			else Lpt1_point_first = POINT(0, 0);




			IMG.get_edge();//获取边线
			if (yroad_position == 0) {
				IMG.flag_track = TRACK_LEFT;
				if (IMG.encoder.speed * 1000 > 1500)IMG.aim_speed = -3000;
				else {
					if (Lpt0_point_first.y > 0.6 * IMAGE_HEIGHT)IMG.aim_speed = -100;
					else if (Lpt0_point_first.y == 0) IMG.aim_speed = -500;
					else IMG.aim_speed = 400;
				}
				if (Lpt0_point_first.y > 0.5 * IMAGE_HEIGHT || Lpt0_point_first.y == 0) {
					IMG.flag_track = TRACK_RIGHT;
					//右边线预测中线
					track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
						(int)round(ANGLE_DIST / SAMPLE_DIST),
						PIXEL_PER_METER * 0.2 / 2);
					IMG.rptsc1_num = IMG.rpts1s_num;
				}
			}
			else {
				IMG.flag_track = TRACK_RIGHT;
				if (IMG.encoder.speed * 1000 > 1500)IMG.aim_speed = -3000;
				else {
					if (Lpt1_point_first.y > 0.6 * IMAGE_HEIGHT)IMG.aim_speed = -100;
					else if (Lpt1_point_first.y == 0) IMG.aim_speed = -500;
					else IMG.aim_speed = 400;

					if (Lpt1_point_first.y > 0.5 * IMAGE_HEIGHT || Lpt1_point_first.y == 0) {
						IMG.flag_track = TRACK_LEFT;
						//左边线预测中线
						track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
							(int)round(ANGLE_DIST / SAMPLE_DIST),
							PIXEL_PER_METER * 0.2 / 2);
						IMG.rptsc0_num = IMG.rpts0s_num;
					}

				}


			}


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

			if (IMG.rpts_num > 0.3 / SAMPLE_DIST) {
				IMG.calculate_angle();//计算角度
			}
			else if (IMG.flag_track == TRACK_RIGHT) {
				IMG.aim_angle = -45;
			}
			else if (IMG.flag_track == TRACK_LEFT) {
				IMG.aim_angle = 45;
			}
			if (IMG.encoder.speed * 1000 < 0)IMG.aim_angle = -IMG.aim_angle;
		}
		else if (flag_yroad == YROAD_FOUND_1) {
			IMG.aim_speed = IMG._config->speed_base;;
			IMG.aim_distance_f = 0.6;
			IMG.aim_distance_n = 0.2;
			IMG.aim_angle_p *= 2;
			if (IMG.encoder.route - yroad_route > 100) {
				IMG.element_over = true;
				IMG.element_over_route = IMG.encoder.route;
				flag_yroad = YROAD_NONE;
				control_flag.pop();//移除环岛
			}

			IMG.get_edge();//获取边线
			if (yroad_position == 0) {
				IMG.flag_track = TRACK_RIGHT;
				//右边线预测中线
				track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.2 / 2);
				IMG.rptsc1_num = IMG.rpts1s_num;
			}
			else {
				IMG.flag_track = TRACK_LEFT;
				//左边线预测中线
				track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.2 / 2);
				IMG.rptsc0_num = IMG.rpts0s_num;
			}




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

			if (IMG.rpts_num > 0.3 / SAMPLE_DIST) {
				IMG.calculate_angle();//计算角度
			}
			else if (IMG.flag_track == TRACK_RIGHT) {
				IMG.aim_angle = -45;
			}
			else if (IMG.flag_track == TRACK_LEFT) {
				IMG.aim_angle = 45;
			}
			if (IMG.encoder.speed * 1000 < 0)IMG.aim_angle = -IMG.aim_angle;
		}
		else if (flag_yroad == YROAD_FOUND_2) {
			if (IMG.encoder.speed * 1000 > 1500)IMG.aim_speed = -1000;
			else IMG.aim_speed = 700;

			int Lpt0_point[2] = { 0 };
			int Lpt1_point[2] = { 0 };
			if (Find_corner_first(IMG.rpts0, IMG.ipts0_num, Lpt0_point, 110))Lpt0_point_first = POINT(Lpt0_point[0], Lpt0_point[1]);
			else Lpt0_point_first = POINT(0, 0);
			if (Find_corner_first(IMG.rpts1, IMG.ipts1_num, Lpt1_point, 110)) Lpt1_point_first = POINT(Lpt1_point[0], Lpt1_point[1]);
			else Lpt1_point_first = POINT(0, 0);
			IMG.get_edge();//获取边线
			if (yroad_position == 0) {
				IMG.flag_track = TRACK_RIGHT;
				drawH(Lpt0_point_first.y, 0, 0, 255, IMG.mat_result);
				if (Lpt0_point_first.y > 0.7 * IMAGE_HEIGHT) {
					yroad_route = IMG.encoder.route;
					flag_yroad = YROAD_FOUND_2_1;
					MAT_LOG("YROAD_FOUND_2_1_%d", yroad_position);
				}

				//右边线预测中线
				track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.2 / 2);
				IMG.rptsc1_num = IMG.rpts1s_num;
			}
			else {
				IMG.flag_track = TRACK_LEFT;
				drawH(Lpt1_point_first.y, 0, 0, 255, IMG.mat_result);
				if (Lpt1_point_first.y > 0.7 * IMAGE_HEIGHT) {
					yroad_route = IMG.encoder.route;
					flag_yroad = YROAD_FOUND_2_1;
					MAT_LOG("YROAD_FOUND_2_1_%d", yroad_position);
				}
				//左边线预测中线
				track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.2 / 2);
				IMG.rptsc0_num = IMG.rpts0s_num;
			}


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
		else if (flag_yroad == YROAD_FOUND_2_1) {//超车
			if (IMG.encoder.speed * 1000 > 1500)IMG.aim_speed = -1000;
			else IMG.aim_speed = 700;
			IMG.get_edge();//获取边线

			if (yroad_position == 0) {
				IMG.flag_track = TRACK_RIGHT;

				//右边线预测中线
				track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.2 / 2);
				IMG.rptsc1_num = IMG.rpts1s_num;
			}
			else {
				IMG.flag_track = TRACK_LEFT;

				//左边线预测中线
				track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.2 / 2);
				IMG.rptsc0_num = IMG.rpts0s_num;
			}
			if (IMG.encoder.route - yroad_route > 50) {
				yroad_route = IMG.encoder.route;
				flag_yroad = YROAD_FOUND_2_2;
				MAT_LOG("YROAD_FOUND_2_2_%d", yroad_position);
			}

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
		else if (flag_yroad == YROAD_FOUND_2_2) {//开始别车
			IMG.aim_distance_f = 0.6;
			IMG.aim_distance_n = 0.3;
			IMG.aim_angle_p *= 2;
			if (IMG.encoder.speed * 1000 > 1500)IMG.aim_speed = -1000;
			else IMG.aim_speed = 700;
			IMG.get_edge();//获取边线
			if (yroad_position == 0) {
				IMG.flag_track = TRACK_LEFT;
				//左边线预测中线
				track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.1 / 2);
				IMG.rptsc0_num = IMG.rpts0s_num;
			}
			else {
				IMG.flag_track = TRACK_RIGHT;

				//右边线预测中线
				track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.1 / 2);
				IMG.rptsc1_num = IMG.rpts1s_num;

			}
			if (IMG.encoder.route - yroad_route > 50) {
				yroad_route = IMG.encoder.route;
				yroad_stop_clock.tic();
				flag_yroad = YROAD_FOUND_2_3;
				MAT_LOG("YROAD_FOUND_2_3_%d", yroad_position);
			}

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
		else if (flag_yroad == YROAD_FOUND_2_3) {//停车
			IMG.aim_distance_f = 0.3;
			IMG.aim_distance_n = 0.1;
			IMG.aim_angle_p *= 2;
			IMG.aim_speed = 0;
			IMG.get_edge();//获取边线
			if (yroad_position == 0) {
				IMG.flag_track = TRACK_LEFT;
				//左边线预测中线
				track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.2 / 2);
				IMG.rptsc0_num = IMG.rpts0s_num;
			}
			else {
				IMG.flag_track = TRACK_RIGHT;

				//右边线预测中线
				track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.2 / 2);
				IMG.rptsc1_num = IMG.rpts1s_num;

			}
			if (yroad_stop_clock.toc() > 3000) {
				IMG.element_over = true;
				IMG.element_over_route = IMG.encoder.route;
				flag_yroad = YROAD_NONE;
				control_flag.pop();//移除环岛
			}

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
		else if (flag_yroad == YROAD_FOUND_3) {
			IMG.aim_speed = IMG._config->speed_base;
			IMG.aim_distance_f = 0.3;
			IMG.aim_distance_n = 0.1;
			IMG.aim_angle_p *= 1.5;
			if (IMG.encoder.route - yroad_route > 100) {
				IMG.element_over = true;
				IMG.element_over_route = IMG.encoder.route;
				flag_yroad = YROAD_NONE;
				control_flag.pop();//移除环岛
			}

			IMG.get_edge();//获取边线
			if (yroad_position == 0) {
				IMG.flag_track = TRACK_RIGHT;
				//右边线预测中线
				track_rightline(IMG.rpts1s, IMG.rpts1s_num, IMG.rptsc1,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.5 / 2);
				IMG.rptsc1_num = IMG.rpts1s_num;
			}
			else {
				IMG.flag_track = TRACK_LEFT;
				//左边线预测中线
				track_leftline(IMG.rpts0s, IMG.rpts0s_num, IMG.rptsc0,
					(int)round(ANGLE_DIST / SAMPLE_DIST),
					PIXEL_PER_METER * 0.5 / 2);
				IMG.rptsc0_num = IMG.rpts0s_num;
			}




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

			if (IMG.rpts_num > 0.3 / SAMPLE_DIST) {
				IMG.calculate_angle();//计算角度
			}
			else if (IMG.flag_track == TRACK_RIGHT) {
				IMG.aim_angle = -45;
			}
			else if (IMG.flag_track == TRACK_LEFT) {
				IMG.aim_angle = 45;
			}
		}
	}
}

/* ********************************************************************* */