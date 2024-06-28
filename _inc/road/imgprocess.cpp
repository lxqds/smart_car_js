#include "imgprocess.hpp"

#include "serial.hpp"

#undef LOG_TAG
#define LOG_TAG "Run"

extern std::shared_ptr<SerialDriver> serial;
Control_Flag control_flag;


//线程池
class _MyThreadPool {
private:

public:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
    int num_thread;
    int num_thread_free;
    _MyThreadPool(int num_threads) : stop(false) {
        num_thread = num_threads;
        num_thread_free = num_threads;
        for (int i = 0; i < num_threads; ++i) {
            workers.emplace_back(std::thread([this]() {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock, [this] { return !tasks.empty() || stop; });
                        if (stop && tasks.empty()) return;
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    task(); // 执行任务
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        num_thread_free++;
                    }

                    //std::cout << _i << "线程  ";
                }
                }));
        }
    }
    template<typename Func, typename... Args>
    bool addTask(Func&& func, Args&&... args) {
        {
            if (getNumIdleThreads() == 0) {
                return false;
            }
            std::lock_guard<std::mutex> lock(queue_mutex);
            auto task = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);
            tasks.push(task);
            num_thread_free--;
        }
        condition.notify_one();

        return true;
    }
    // 判断线程池是否空闲
    void waitThreadPoolFree() {
        while (true) {
            int num = getNumIdleThreads();
            // MAT_LOG("NumIdleThreads=%d", num);
            if (num == num_thread)break;
        }
    }
    // 获取空闲线程的数量
    int getNumIdleThreads() {
        std::lock_guard<std::mutex> lock(queue_mutex);
        return num_thread_free;
    }
    ~_MyThreadPool() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }
}MyThreadPool(0);


/* 初始化 */
int ImageProcess::init(std::string config_path, bool is_result) {
    _config = make_shared<ControlConfig>(config_path);
    _is_result = is_result;

    // 初始化出库方向
    if (_config->direction_garage == 0)
    {
        control_flag.push(garage_element);
        flag_garage = GARAGE_OUT_LEFT;
    }

    else if (_config->direction_garage == 1)
    {
        control_flag.push(garage_element);
        flag_garage = GARAGE_OUT_RIGHT;
    }
    else if (_config->direction_garage == 4)
    {
        control_flag.push(garage_element);
        flag_garage = GARAGE_OUT_MIDDLE;
        MAT_LOG("GARAGE_OUT_MIDDLE");
    }
    else
        flag_garage = GARAGE_NONE;

    // 初始化参数
    aim_distance_f = _config->aim_distance_far;
    aim_distance_n = _config->aim_distance_near;
    element_over = false;
    speed_diff = (_config->speed_up - _config->speed_base) / 10;

    return 0;
}

/* 图像处理 */
void ImageProcess::process(cv::Mat& mat_origin, cv::Mat& _mat_result,
    Payload_t& payload) {
    /* ***************************************************************** */
    /* ***************************** 初始化 ***************************** */
    /* ***************************************************************** */
    mat_result = _mat_result;
    // 图像预处理: 缩放 -> 灰度 -> 二值化
    cv::resize(mat_origin, mat_result, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 0, 0, cv::INTER_AREA);
    cv::cvtColor(mat_result, mat_bin, cv::COLOR_BGR2GRAY);
    cv::threshold(mat_bin, mat_bin, 0, 255, cv::THRESH_OTSU);

    // 获取 LAB 图片 B 通道 (黄) -> 提取锥桶
    cv::cvtColor(mat_result, mat_lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> channels_lab;
    cv::split(mat_lab, channels_lab);
    cv::threshold(channels_lab[2], mat_lab, 150, 255, cv::THRESH_BINARY);

    // 绘制处理结果图, 灰度格式转BGR格式

    if (_config->check_binary_ || _config->check_lab_) {
        std::vector<cv::Mat> channels;
        if (_config->check_binary_) {
            for (int i = 0; i < 3; i++)
                channels.push_back(mat_bin);
        }
        else {
            for (int i = 0; i < 3; i++)
                channels.push_back(mat_lab);
        }
        cv::merge(channels, mat_result);
    }

    /* ***************************************************************** */
    /* **************************** 边线处理 **************************** */
    /* ***************************************************************** */


    {
        //扇形扫描找最远的白线
        int max_white_line_end[2]{};
        float max_white_line_length = 0;
        int white_line_end[120][2]{};
        float white_line_length[120]{};
        {
            auto f = [&white_line_length, &white_line_end, this](int t) {
                float L = 5;
                int count = 0;
                while (true) {
                    float _x = (float)(L * cos((t + 30) / 180.0f * PI));
                    float _y = (float)(L * sin((t + 30) / 180.0f * PI) + 5);
                    float x = _x + IMAGE_WIDTH / 2;
                    float y = IMAGE_HEIGHT - _y;
                    bool is_range = x > 5 && x < IMAGE_WIDTH - 5 && y >5;
                    if (is_range) {
                        if (AT_IMAGE(this->mat_bin, (int)x, (int)y) < 127)count++;
                        else count = 0;
                    }
                    if (count == 2 || !is_range) {
                        float a[2]{};
                        map_perspective(x, y, a, 0);
                        float l = (float)dis_point_point(POINT((int)a[0], (int)a[1]), POINT(IMAGE_WIDTH / 2, IMAGE_HEIGHT - 1));
                        white_line_end[t][0] = (int)x;
                        white_line_end[t][1] = (int)y;
                        white_line_length[t] = l;
                        break;
                    }
                    L += 1.0f;
                }
                };
            for (int t = 0; t < 120; t++) {
                if (!MyThreadPool.addTask(f, t))
                {
                    f(t);
                }
            }
            MyThreadPool.waitThreadPoolFree();
        }
        for (int t = 0; t < 120; t += 1) {
            if (white_line_length[t] > max_white_line_length) {
                max_white_line_end[0] = white_line_end[t][0];
                max_white_line_end[1] = white_line_end[t][1];
                max_white_line_length = white_line_length[t];
            }
            // cv::line(mat_result, cv::Point((int)white_line_end[t][0], (int)white_line_end[t][1]),
            //     cv::Point(IMAGE_WIDTH / 2, IMAGE_HEIGHT - 1), cv::Scalar(0, 255, 0), 1);
        }
        cv::line(mat_result, cv::Point((int)max_white_line_end[0], (int)max_white_line_end[1]),
            cv::Point(IMAGE_WIDTH / 2, IMAGE_HEIGHT - 1), cv::Scalar(0, 0, 255), 1);



        //横向扫描,得到横线数据集
        std::vector<POINT> white_row_start_point[40];//白行起始点
        std::vector<POINT> white_row_end_point[40];//白行结束点
        std::vector<int> white_row_length[40];//透视变换白行长度
        //寻找最符合跑道的白行
        {
            auto f = [&white_row_start_point, &white_row_end_point, &white_row_length, this](int i) {
                int start = 0, sign = 0, count = 0;
                for (int x = 0; x < IMAGE_WIDTH; x += 2) {
                    if (sign == 0) {//向右开始找白点
                        if (AT_IMAGE(this->mat_bin, x, i * 5) >= 127)count++;
                        else count = 0;
                        if (count == 2) {
                            start = x - count + 1;
                            sign = 1;
                        }
                    }
                    if (sign == 1) {//向右开始找黑点
                        if (AT_IMAGE(this->mat_bin, x, i * 5) < 127)count++;
                        else count = 0;
                        if (count == 2 || x >= IMAGE_WIDTH - 2) {
                            float a[2]{}, b[2]{};
                            map_perspective((float)start, (float)i * 5, a, 0);
                            map_perspective((float)x - count + 1, (float)i * 5, b, 0);
                            float L = b[0] - a[0];
                            if (L > 50) {
                                white_row_start_point[i].push_back(POINT(start, i * 5));
                                white_row_length[i].push_back((int)L);
                                white_row_end_point[i].push_back(POINT(x - count + 1, i * 5));
                            }
                            count = 0;
                            sign = 0;
                        }
                    }
                }
                };
            for (int i = 39; i > 0; i--) {
                if (!MyThreadPool.addTask(f, i))
                {
                    f(i);
                }
            }
            MyThreadPool.waitThreadPoolFree();
        }

        //处理横线数据集,得到最贴合主赛道的横线数据集
        for (int i = 39, start_sign = 0, end_sign = 0; i > 0; i--) {
            float mindistance = 0;
            if (end_sign == 0) {
                for (uint32_t j = 0; j < white_row_start_point[i].size();) {
                    float x = (white_row_start_point[i][j].x + white_row_end_point[i][j].x) / 2.0f;
                    float y = (float)white_row_start_point[i][j].y;
                    float _x = (float)findThirdPointX(POINT(max_white_line_end[0], max_white_line_end[1]),
                        POINT(IMAGE_WIDTH / 2, IMAGE_HEIGHT - 1), y);
                    float L = abs(_x - x);
                    if (j == 0) {
                        mindistance = L;
                        j++;
                    }
                    else {
                        if (L < mindistance) {
                            white_row_start_point[i].erase(white_row_start_point[i].begin());
                            white_row_end_point[i].erase(white_row_end_point[i].begin());
                            white_row_length[i].erase(white_row_length[i].begin());
                            mindistance = L;
                        }
                        else {
                            white_row_start_point[i].erase(white_row_start_point[i].begin() + 1);
                            white_row_end_point[i].erase(white_row_end_point[i].begin() + 1);
                            white_row_length[i].erase(white_row_length[i].begin() + 1);
                        }
                    }
                }
                if (start_sign == 1 && (
                    white_row_start_point[i].empty() ||
                    white_row_start_point[i].back().x > white_row_end_point[i + 1].back().x ||
                    white_row_end_point[i].back().x < white_row_start_point[i + 1].back().x ||
                    white_row_start_point[i].back().y < max_white_line_end[1])) {
                    end_sign = 1;
                }
                if (!white_row_start_point[i].empty()) {
                    start_sign = 1;
                    // cv::line(mat_result, cv::Point(white_row_start_point[i].back().x, white_row_start_point[i].back().y),
                    //     cv::Point(white_row_end_point[i].back().x, white_row_end_point[i].back().y),
                    //     cv::Scalar(0, 255, 0), 1);
                }
            }
            else {
                white_row_start_point[i].clear();
                white_row_end_point[i].clear();
                white_row_length[i].clear();
            }
        }
        //找近处巡线开始点
        for (int i = 39; i > 28; i--) {
            if (!white_row_start_point[i].empty()) {
                begin_x = (white_row_start_point[i].back().x + white_row_end_point[i].back().x) / 2;
                begin_y = white_row_start_point[i].back().y;
                break;
            }
        }
        {
            ipts0_num = 180; ipts1_num = 180;
            //双线程巡线
            if (!MyThreadPool.addTask(find_left_line, mat_bin, begin_x, begin_y, ipts0, &ipts0_num))
                find_left_line(mat_bin, begin_x, begin_y, ipts0, &ipts0_num);
            find_right_line(mat_bin, begin_x, begin_y, ipts1, &ipts1_num);
            MyThreadPool.waitThreadPoolFree();
        }




        //找远处巡线开始点
        int far_begin_x = 0, far_begin_y = 0;//远线向下找线的开始位置
        int count = 0;
        far_ipts0_num = 0; far_ipts1_num = 0;
        for (int i = 5; i < 20; i++) {
            if (!white_row_start_point[i].empty()) {
                if (white_row_length[i].back() < 220) count++;
                if (count == 3)
                {
                    far_begin_x = (white_row_start_point[i].back().x + white_row_end_point[i].back().x) / 2;
                    far_begin_y = white_row_start_point[i].back().y;
                    far_ipts0_num = 120; far_ipts1_num = 120;
                    break;
                }
            }
        }
        {
            //双线程巡线
            if (!MyThreadPool.addTask(find_far_left_line, mat_bin, far_begin_x, far_begin_y, far_ipts0, &far_ipts0_num))
                find_far_left_line(mat_bin, far_begin_x, far_begin_y, far_ipts0, &far_ipts0_num);// 原图找左边线
            find_far_right_line(mat_bin, far_begin_x, far_begin_y, far_ipts1, &far_ipts1_num);// 原图找右边线
            MyThreadPool.waitThreadPoolFree();
        }


    }
    // 变换后左右线
    float rpts0[IMAGE_HEIGHT][2] = { 0 };
    float rpts1[IMAGE_HEIGHT][2] = { 0 };
    float far_rpts0[IMAGE_HEIGHT][2] = { 0 };
    float far_rpts1[IMAGE_HEIGHT][2] = { 0 };

    //透视变换得到的线
    for (int i = 0; i < ipts0_num; i++)
        map_perspective((float)ipts0[i][0], (float)ipts0[i][1], rpts0[i], 0);
    for (int i = 0; i < ipts1_num; i++)
        map_perspective((float)ipts1[i][0], (float)ipts1[i][1], rpts1[i], 0);
    for (int i = 0; i < far_ipts0_num; i++)
        map_perspective((float)far_ipts0[i][0], (float)far_ipts0[i][1], far_rpts0[i], 0);
    for (int i = 0; i < far_ipts1_num; i++)
        map_perspective((float)far_ipts1[i][0], (float)far_ipts1[i][1], far_rpts1[i], 0);

    Find_corner(rpts0, ipts0_num, Lpt0_point, &Lpt0_angle);
    Find_corner(rpts1, ipts1_num, Lpt1_point, &Lpt1_angle);
    Find_corner(far_rpts0, far_ipts0_num, far_Lpt0_point, &far_Lpt0_angle);
    Find_corner(far_rpts1, far_ipts1_num, far_Lpt1_point, &far_Lpt1_angle);

    Lpt0_found = IMG.Lpt0_angle > 50 && IMG.Lpt0_angle < 120;
    Lpt1_found = IMG.Lpt1_angle > 50 && IMG.Lpt1_angle < 120;
    far_Lpt0_found = IMG.far_Lpt0_angle > 50 && IMG.far_Lpt0_angle < 120;
    far_Lpt1_found = IMG.far_Lpt1_angle > 50 && IMG.far_Lpt1_angle < 120;


    /* ***************************************************************** */
    /* *************************** 弯直道检测 *************************** */
    /* ***************************************************************** */






    /* ***************************************************************** */
    /* *************************** 目标检测结果 ************************** */
    /* ***************************************************************** */

    // 检测完成判断 (加油站 & 施工区 跳过 防止重复获取检测结果)
    if ((flag_work <= WORK_DETECTION) &&
        detection->_is_predicted) {
        _det_result = detection->getResult()->result;

        // 遍历检测结果
        for (size_t i = 0; i < _det_result.size(); i++) {
            DetectionPredictResult r = _det_result[i];

            // 检测结果筛选
            if (r.score < _config->threshold_detection)
                continue;
            if (DetectionLabel(r.type) == label_cone) {
                // continue;
            }

            // 转换到处理图像坐标
            int deX = r.x / 2, deY = r.y / 2;
            int deH = r.height / 2, deW = r.width / 2;

            log_i("Detection GET %d ,%f", r.type, r.score);
            // 尺寸位置筛选
            // if ((deX < 10 || deX + deW > 300) || deY < 20 || (deH < 25 || deW < 25))
                //continue;

            // 禁止在元素内判断其他元素


            switch (DetectionLabel(r.type)) {
                case label_bomb:      // 炸弹
                {
                    if (!control_flag.contains(circle_element) &&   //当前没在环岛中
                        !control_flag.contains(work_element) &&     //当前没在工作区中
                        !control_flag.contains(garage_element) &&      //当前没在入库中
                        !control_flag.contains(ramp_element) &&      //当前没在桥中
                        !control_flag.contains(bomb_element)      //当前没在炸弹中
                        ) {
                        flag_bomb = BOMB_DETECTION;

                        sprintf(string_log, "label_bomb_DETECTION");
                        log_i("label_bomb_DETECTION");
                    }
                    bomb_y = deY + deH / 2;
                    bomb_x = deX + deW / 2;
                }break;
                case label_thief:      // 贼 和 恶人处理方式一样 不 break
                case label_evil:      //恶人
                {
                    if (!control_flag.contains(circle_element) &&   //当前没在环岛中
                        !control_flag.contains(work_element) &&     //当前没在工作区中
                        !control_flag.contains(garage_element) &&      //当前没在入库中
                        !control_flag.contains(ramp_element) &&      //当前没在桥中
                        !control_flag.contains(bomb_element) &&     //当前没在炸弹中
                        r.score > 0.6) {
                        if (flag_work == WORK_NONE && _config->check_work_ == 1) {
                            flag_work = WORK_EVEI_DETECTION;
                            sprintf(string_log, "WORK_EVEI_DETECTION");
                        }

                        if (flag_work != WORK_NONE) sprintf(string_log, "flag_work != WORK_NONE");

                        sprintf(string_log, "label_evil_thief");
                        log_i("label_evil_DETECTION or label_thief");
                    }
                }break;
                case label_tumble:      // 跌倒
                case label_patient:     // 病人
                    if (!control_flag.contains(circle_element) &&   //当前没在环岛中
                        !control_flag.contains(work_element) &&     //当前没在工作区中
                        !control_flag.contains(garage_element) &&      //当前没在入库中
                        !control_flag.contains(ramp_element) &&      //当前没在桥中
                        !control_flag.contains(bomb_element) &&     //当前没在炸弹中
                        r.score > 0.6
                        ) {
                        if (flag_work == WORK_NONE && _config->check_work_ == 1) {

                            flag_work = WORK_DETECTION;
                            sprintf(string_log, "PATIENT_DETECTION");
                            log_i("PATIENT_DETECTION");
                        }
                    }

                    break;
                case label_prop:      // 道具小车
                {
                    sprintf(string_log, "label_prop_DETECTION");
                    log_i("label_prop_DETECTION");
                }break;
                case label_safety:      // 普通小车
                {
                    sprintf(string_log, "label_safety_DETECTION");
                    log_i("label_safety_DETECTION");
                }break;
                case label_spy:      // 嫌犯小车
                {
                    sprintf(string_log, "label_spy_DETECTION");
                    log_i("label_spy_DETECTION");
                }break;
                case label_danger:      // 危险小车
                {
                    sprintf(string_log, "label_danger_DETECTION");
                    log_i("label_danger_DETECTION");
                }break;

                case label_bridge:  // 桥
                    if (!control_flag.contains(circle_element) &&   //当前没在环岛中
                        !control_flag.contains(work_element) &&     //当前没在工作区中
                        !control_flag.contains(garage_element) &&      //当前没在入库中
                        !control_flag.contains(ramp_element) &&      //当前没在桥中
                        !control_flag.contains(bomb_element) &&     //当前没在炸弹中
                        r.score > 0.75
                        ) {
                        flag_ramp = RAMP_DETECTION;
                        sprintf(string_log, "label_bomb_DETECTION");
                        log_i("BRIDGE_DETECTION");
                    }
                    ramp_y = deY + deH / 2;
                    ramp_x = deX + deW / 2;
                    ramp_score = r.score;
                    break;
                case label_block:      // 黑块
                {
                    bomb_block_y = deY + deH / 2;
                    bomb_block_x = deX + deW / 2;
                    // sprintf(string_log, "label_block_DETECTION");
                    // log_i("label_block_DETECTION");
                }break;
                case label_crosswalk:  // 斑马线
                    if (flag_garage == GARAGE_NONE && _config->check_garage_AI == 1) {
                        flag_garage = GARAGE_AI_DETECTION;
                        sprintf(string_log, "GARAGE_AI_DETECTION");
                        log_i("GARAGE_AI_DETECTION");
                    }
                    break;
                default:
                    break;
            }

            if (r.type > 0 && r.type < 16) {
                // 计算偏移
                if (deY < BEGIN_Y + 30 && deY + deH > BEGIN_Y - 30 &&
                    deX < IMAGE_WIDTH - BEGIN_X + 20 && deX + deW > BEGIN_X - 20) {
                    if (deX < begin_x_l)
                        begin_x_l = deX;
                    if (deX + deW > begin_x_r)
                        begin_x_r = deX + deW;
                }

                // 绘制检测结果
                if (_is_result) {
                    MAT_INFO(mat_result, cv::Point(deX, deY), 0.4, "%s - %.2f",
                        detection_labels[r.type].c_str(), r.score);
                    cv::Rect rect(deX, deY, deW, deH);
                    cv::rectangle(mat_result, rect, cv::Scalar(0, 0, 224), 2);
                }
            }
        }
    }

    /* ***************************************************************** */
    /* **************************** 元素检测 **************************** */
    /* ***************************************************************** */

    // 重置识别元素开始标志
    element_identify = 0;

    // 完成一个元素后, 过一段路程再开始检测(识别)新的元素
    if (element_over && (encoder.route - element_over_route > _config->over_route_)) {
        element_over = false;
        element_over_route = 0.0f;
        element_begin_id = 0;
        special_w = false;
        special_g = false;
    }

    // 识别元素
    check_garage();  // 斑马线 - 识别特殊特征
    check_work();  // 施工区 - 识别标志
    check_cross();  // 十字路 - 识别角点特征
    check_circle();  // 环岛路 - 识别角点特征
    check_bomb();   //炸弹
    check_ramp();   //桥

    aim_distance_f = _config->aim_distance_far;
    aim_distance_n = _config->aim_distance_near;
    aim_angle_p_k = _config->steering_p_k;
    aim_angle_p = _config->steering_p;
    aim_angle_d = _config->steering_d;
    run_garage();
    run_work();
    run_cross();
    run_circle();
    run_bomb();
    run_ramp();
    if (element_identify == 1) {
        serial->buzzerSound(1);
    }

    /* ***************************************************************** */
    /* **************************** 中线处理 **************************** */
    /* ***************************************************************** */



    /* ***************************************************************** */
    /* **************************** 偏差计算 **************************** */
    /* ***************************************************************** */


    /* ***************************************************************** */
    /* **************************** 速度判定 **************************** */
    /* ***************************************************************** */

    if (control_flag.top() == none_element)
    {
        int mode = 0;
        if (Lpt0_found && ipts0[0][1] - Lpt0_point[1] < 10)
        {
            cv::line(mat_bin, cv::Point((int)(IMAGE_WIDTH * 0.2f), (int)(IMAGE_HEIGHT - 1)),
                cv::Point(Lpt0_point[0], Lpt0_point[1]),
                cv::Scalar(0, 0, 0), 7);
            cv::line(mat_result, cv::Point((int)(IMAGE_WIDTH * 0.2f), (int)(IMAGE_HEIGHT - 1)),
                cv::Point(Lpt0_point[0], Lpt0_point[1]),
                cv::Scalar(0, 0, 0), 7);
            mode = 1;
        }
        if (Lpt1_found && ipts1[0][1] - Lpt1_point[1] < 10)
        {
            cv::line(IMG.mat_bin, cv::Point((int)(IMAGE_WIDTH * 0.8f), (int)(IMAGE_HEIGHT - 1)),
                cv::Point(Lpt1_point[0], Lpt0_point[1]),
                cv::Scalar(0, 0, 0), 7);
            cv::line(IMG.mat_result, cv::Point((int)(IMAGE_WIDTH * 0.8f), (int)(IMAGE_HEIGHT - 1)),
                cv::Point(Lpt1_point[0], Lpt0_point[1]),
                cv::Scalar(0, 0, 0), 7);
            mode = 1;
        }


        get_edge(mode);
        aim_speed = _config->speed_base;
        if (is_straight0 && is_straight1)
        {
            aim_speed_shift = _config->speed_base +
                (1 - clip(abs(aim_sigma), 0, 30) / 30.0) * (_config->speed_up - _config->speed_base);
            // 速度限幅
            aim_speed_shift = clip(aim_speed_shift, _config->speed_base, _config->speed_up);
            aim_speed = aim_speed_shift;
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



    /* ***************************************************************** */
    /* **************************** 运行控制 **************************** */
    /* ***************************************************************** */

    // 偏差限幅
    aim_angle = aim_angle > 500.0f ? 500.0f : aim_angle < -500.0f ? -500.0f
        : aim_angle;

    // 偏差滤波
    float aim_angle_filter = filter(aim_angle);
    aim_angle_last = aim_angle_filter;

    // 计算舵机 PID
    int aim_angle_pwm = 0;
    aim_angle_pwm = (int)(pid_realize_a(aim_angle_filter, 0.0f, aim_angle_p, aim_angle_d) + 0.5f);
    aim_angle_pwm = 0 - clip(aim_angle_pwm, -500, 500);

    // 赋值数据包
    payload.tSpeed = aim_speed;
    payload.tAngle = (int16_t)aim_angle_pwm;
    payload.element = element_identify;

    /* ***************************************************************** */
    /* **************************** 绘制结果 **************************** */
    /* ***************************************************************** */

    if (_is_result) {
        std::string track_label;
        if (flag_track == TRACK_LEFT)
            track_label = "track: LEFT";
        else if (flag_track == TRACK_RIGHT)
            track_label = "track: RIGHT";
        else
            track_label = "track: MIDDLE";

        MAT_INFO(mat_result, cv::Point(5, 25), 0.4, "%s", track_label.c_str());
        MAT_INFO(mat_result, cv::Point(5, 40), 0.4, "speed: %d", aim_speed);
        MAT_INFO(mat_result, cv::Point(5, 55), 0.4, "angle: %.2f,pwm:%d", aim_angle_filter, aim_angle_pwm);
        MAT_INFO(mat_result, cv::Point(5, 70), 0.4, "stg_p: %.4f", aim_angle_p);
        MAT_INFO(mat_result, cv::Point(5, 90), 0.4, "sigma: %.2f", aim_sigma);
        MAT_INFO(mat_result, cv::Point(5, 105), 0.4, "mea-0: %.2f", mea_0);
        MAT_INFO(mat_result, cv::Point(5, 125), 0.4, "mea-1: %.2f", mea_1);
        MAT_INFO(mat_result, cv::Point(5, 150), 0.4, "speed: %.2fm/s", encoder.speed);
        MAT_INFO(mat_result, cv::Point(5, 165), 0.4, "route: %.0fcm", encoder.route);
        MAT_INFO(mat_result, cv::Point(5, 180), 0.4, "YAW: %.2f", encoder.yaw);
        MAT_INFO(mat_result, cv::Point(5, 195), 0.4, "log: %-20s", string_log);

        // 边线
        for (int a = 0; a < rpts0s_num; a++) {  // 边线等距采样 左
            map_perspective(rpts0s[a][0], rpts0s[a][1], trans, 1);
            if ((int)trans[1] >= 0 && (int)trans[1] < mat_result.rows &&
                (int)trans[0] >= 0 && (int)trans[0] < mat_result.cols) {
                mat_result.at<cv::Vec3b>((int)trans[1], (int)trans[0])[0] = 0;
                mat_result.at<cv::Vec3b>((int)trans[1], (int)trans[0])[1] = 0;
                mat_result.at<cv::Vec3b>((int)trans[1], (int)trans[0])[2] = 255;
            }
        }
        for (int a = 0; a < rpts1s_num; a++) {  // 边线等距采样 右
            map_perspective(rpts1s[a][0], rpts1s[a][1], trans, 1);
            if ((int)trans[1] >= 0 && (int)trans[1] < mat_result.rows &&
                (int)trans[0] >= 0 && (int)trans[0] < mat_result.cols) {
                mat_result.at<cv::Vec3b>((int)trans[1], (int)trans[0])[0] = 255;
                mat_result.at<cv::Vec3b>((int)trans[1], (int)trans[0])[1] = 0;
                mat_result.at<cv::Vec3b>((int)trans[1], (int)trans[0])[2] = 0;
            }
        }


        // 中线
        for (int a = 0; a < rptscs_num; a++) {  // 原图中线
            if ((int)rptscs[a][1] >= 0 && (int)rptscs[a][1] < mat_result.rows &&
                (int)rptscs[a][0] >= 0 && (int)rptscs[a][0] < mat_result.cols) {
                mat_result.at<cv::Vec3b>((int)rptscs[a][1], (int)rptscs[a][0])[0] = 0;
                mat_result.at<cv::Vec3b>((int)rptscs[a][1], (int)rptscs[a][0])[1] = 0;
                mat_result.at<cv::Vec3b>((int)rptscs[a][1], (int)rptscs[a][0])[2] = 0;
            }
        }
        for (int a = 0; a < rptsc0_num; a++) {  // 左中线
            if ((int)rptsc0[a][1] >= 0 && (int)rptsc0[a][1] < mat_result.rows &&
                (int)rptsc0[a][0] >= 0 && (int)rptsc0[a][0] < mat_result.cols) {
                mat_result.at<cv::Vec3b>((int)rptsc0[a][1], (int)rptsc0[a][0])[0] = 0;
                mat_result.at<cv::Vec3b>((int)rptsc0[a][1], (int)rptsc0[a][0])[1] = 0;
                mat_result.at<cv::Vec3b>((int)rptsc0[a][1], (int)rptsc0[a][0])[2] = 200;
            }
        }
        for (int a = 0; a < rptsc1_num; a++) {  // 右中线
            if ((int)rptsc1[a][1] >= 0 && (int)rptsc1[a][1] < mat_result.rows &&
                (int)rptsc1[a][0] >= 0 && (int)rptsc1[a][0] < mat_result.cols) {
                mat_result.at<cv::Vec3b>((int)rptsc1[a][1], (int)rptsc1[a][0])[0] = 200;
                mat_result.at<cv::Vec3b>((int)rptsc1[a][1], (int)rptsc1[a][0])[1] = 0;
                mat_result.at<cv::Vec3b>((int)rptsc1[a][1], (int)rptsc1[a][0])[2] = 0;
            }
        }
        //远线绘制
        if (far_ipts0_num != 0)
        {
            // 左线
            for (int a = 0; a < far_ipts0_num; a++) {  // 原图中线
                if ((int)far_ipts0[a][1] >= 0 && (int)far_ipts0[a][1] < mat_result.rows &&
                    (int)far_ipts0[a][0] >= 0 && (int)far_ipts0[a][0] < mat_result.cols) {
                    mat_result.at<cv::Vec3b>((int)far_ipts0[a][1], (int)far_ipts0[a][0])[0] = 0;
                    mat_result.at<cv::Vec3b>((int)far_ipts0[a][1], (int)far_ipts0[a][0])[1] = 255;
                    mat_result.at<cv::Vec3b>((int)far_ipts0[a][1], (int)far_ipts0[a][0])[2] = 255;
                }
            }
            // 右线
            for (int a = 0; a < far_ipts1_num; a++) {  // 原图中线
                if ((int)far_ipts1[a][1] >= 0 && (int)far_ipts1[a][1] < mat_result.rows &&
                    (int)far_ipts1[a][0] >= 0 && (int)far_ipts1[a][0] < mat_result.cols) {
                    mat_result.at<cv::Vec3b>((int)far_ipts1[a][1], (int)far_ipts1[a][0])[0] = 255;
                    mat_result.at<cv::Vec3b>((int)far_ipts1[a][1], (int)far_ipts1[a][0])[1] = 255;
                    mat_result.at<cv::Vec3b>((int)far_ipts1[a][1], (int)far_ipts1[a][0])[2] = 0;
                }
            }
        }
        if (Lpt0_found)
            cv::circle(mat_result, cv::Point(Lpt0_point[0], Lpt0_point[1]), 3, cv::Scalar(0, 0, 255), 2, 8);
        if (Lpt1_found)
            cv::circle(mat_result, cv::Point(Lpt1_point[0], Lpt1_point[1]), 3, cv::Scalar(255, 0, 0), 2, 8);
        if (far_Lpt0_found)
            cv::circle(mat_result, cv::Point(far_Lpt0_point[0], far_Lpt0_point[1]), 3, cv::Scalar(0, 255, 255), 2, 8);
        if (far_Lpt1_found)
            cv::circle(mat_result, cv::Point(far_Lpt1_point[0], far_Lpt1_point[1]), 3, cv::Scalar(255, 255, 0), 2, 8);

        cv::circle(mat_result, cv::Point(begin_x, begin_y), 3, cv::Scalar(255, 0, 255), 2, 8);
        if (flag_rpts) {
            // 贝塞尔曲线
            for (auto p : bezier_line)
                circle(mat_result, cv::Point(p.x, p.y), 1, cv::Scalar(0, 0, 255), -1);

            // 归一化中线
            for (int a = 0; a < rptsn_num; a++) {
                if ((int)rptsn[a][1] >= 0 && (int)rptsn[a][1] < mat_result.rows &&
                    (int)rptsn[a][0] >= 0 && (int)rptsn[a][0] < mat_result.cols) {
                    mat_result.at<cv::Vec3b>((int)rptsn[a][1], (int)rptsn[a][0])[0] = 0;
                    mat_result.at<cv::Vec3b>((int)rptsn[a][1], (int)rptsn[a][0])[1] = 0;
                    mat_result.at<cv::Vec3b>((int)rptsn[a][1], (int)rptsn[a][0])[2] = 255;
                }
            }

        }

        // for (int i = 1; i < 40; i++)
        // {
        //     cv::line(mat_result, cv::Point(White_row_start_point[i][0], White_row_start_point[i][1]),
        //         cv::Point(White_row_end_point[i][0], White_row_end_point[i][1]), cv::Scalar(0, 255, 0), 1);
        //     MAT_INFO(mat_result, cv::Point(IMAGE_WIDTH / 2, White_row_end_point[i][1]), 0.4, "%d", White_row_length[i]);
        // }
    }
    _mat_result = mat_result;
}

/* ******************************************************************* */
/* ******************************************************************* */
/* ******************************************************************* */

void ImageProcess::get_edge(int mode)
{
    if (mode == 1)
    {
        ipts0_num = 180; ipts1_num = 180;
        // //双线程巡线
        if (!MyThreadPool.addTask(find_left_line, mat_bin, begin_x, begin_y, ipts0, &ipts0_num))
            find_left_line(mat_bin, begin_x, begin_y, ipts0, &ipts0_num);
        find_right_line(mat_bin, begin_x, begin_y, ipts1, &ipts1_num);
        MyThreadPool.waitThreadPoolFree();
    }
    // 变换后左右中线
    float rpts0[IMAGE_HEIGHT][2] = { 0 };
    float rpts1[IMAGE_HEIGHT][2] = { 0 };
    float rptsc[IMAGE_HEIGHT][2] = { 0 };
    // 原图边线转换数据定义
    int iptsc0[IMAGE_HEIGHT] = { 0 };
    int iptsc1[IMAGE_HEIGHT] = { 0 };

    // 原图边线 -> 透视边线 左
    rptsc_num = 0;
    for (int i = 0; i < ipts0_num; i++, rptsc_num++) {
        if (ipts0[i][1] < 10)
            break;
        map_perspective(ipts0[i][0], ipts0[i][1], rpts0[i], 0);
        iptsc0[ipts0[i][1]] = ipts0[i][0];
    }
    ipts0_num = rptsc_num;
    // 原图边线 -> 透视边线 右
    rptsc_num = 0;
    edge_right.clear();
    for (int i = 0; i < ipts1_num; i++, rptsc_num++) {
        if (ipts1[i][1] < 10)
            break;
        map_perspective(ipts1[i][0], ipts1[i][1], rpts1[i], 0);
        iptsc1[ipts1[i][1]] = ipts1[i][0];
    };
    ipts1_num = rptsc_num;
    // 中线获取 图像顶部10个像素丢弃
    rptsc_num = 0;
    for (int ccy = IMAGE_HEIGHT - 1; ccy >= 10; ccy--) {
        iptsc[ccy] = iptsc0[ccy] + iptsc1[ccy];
        if (iptsc[ccy] != 0) {
            // 如果右边线丢失
            if (iptsc1[ccy] == 0)
                iptsc[ccy] = (int)((IMAGE_WIDTH - iptsc[ccy]) / 2 + iptsc[ccy]);
            else  // 左边线丢失
                iptsc[ccy] = (int)(iptsc[ccy] / 2);
            // 原图中线 -> 透视中线
            map_perspective(iptsc[ccy], ccy, rptsc[rptsc_num++], 0);
        }
    }

    // 滤波
    blur_points(rpts0, ipts0_num, rpts0b, (int)round(LINE_BLUR_KERNEL));
    blur_points(rpts1, ipts1_num, rpts1b, (int)round(LINE_BLUR_KERNEL));
    blur_points(rptsc, rptsc_num, rptscb, (int)round(LINE_BLUR_KERNEL));

    // 边线等距采样
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, ipts0_num, rpts0s, &rpts0s_num, SAMPLE_DIST * PIXEL_PER_METER);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, ipts1_num, rpts1s, &rpts1s_num, SAMPLE_DIST * PIXEL_PER_METER);
    rptscs_num = sizeof(rptscs) / sizeof(rptscs[0]);
    resample_points(rptscb, rptsc_num, rptscs, &rptscs_num, SAMPLE_DIST * PIXEL_PER_METER);

    // 左右中线跟踪
    track_leftline(rpts0s, rpts0s_num, rptsc0,
        (int)round(ANGLE_DIST / SAMPLE_DIST),
        PIXEL_PER_METER * ROAD_WIDTH / 2);
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1,
        (int)round(ANGLE_DIST / SAMPLE_DIST),
        PIXEL_PER_METER * ROAD_WIDTH / 2);
    rptsc1_num = rpts1s_num;

    // 透视中线 -> 原图中线
    for (int i = 0; i < rptsc0_num; i++)
        map_perspective(rptsc0[i][0], rptsc0[i][1], rptsc0[i], 1);
    for (int i = 0; i < rptsc1_num; i++)
        map_perspective(rptsc1[i][0], rptsc1[i][1], rptsc1[i], 1);
    for (int i = 0; i < rptscs_num; i++)
        map_perspective(rptscs[i][0], rptscs[i][1], rptscs[i], 1);

    // 标志位重置
    is_straight0 = is_straight1 = false;
    mea_0 = mea_1 = 10.0f;
    // 左线直线拟合
    if (rpts0s_num > 10) {
        mea_0 = fit_line(rpts0s, rpts0s_num, 60);
        if (mea_0 < 2.5f && rpts0s_num > 60)
            is_straight0 = true;
    }
    // 右线直线拟合
    if (rpts1s_num > 10) {
        mea_1 = fit_line(rpts1s, rpts1s_num, 60);
        if (mea_1 < 2.5f && rpts1s_num > 60)
            is_straight1 = true;
    }

    /* ***************************************************************** */
    /* **************************** 选定中线 **************************** */
    /* *********************** 单侧线少, 切换巡线方向 ********************** */
    if (is_straight0 && is_straight1)
        flag_track = TRACK_MIDDLE;
    else if (is_straight0)
        flag_track = TRACK_LEFT;
    else if (is_straight1)
        flag_track = TRACK_RIGHT;
    else if (rpts0s_num == 0 && rpts1s_num != 0)
        flag_track = TRACK_RIGHT;
    else if (rpts0s_num != 0 && rpts1s_num == 0)
        flag_track = TRACK_LEFT;
    else if (rpts0s_num < rpts1s_num / 2)
        flag_track = TRACK_RIGHT;
    else if (rpts0s_num / 2 > rpts1s_num)
        flag_track = TRACK_LEFT;
    else if (rpts0s_num < 10 && rpts0s_num < rpts1s_num)
        flag_track = TRACK_RIGHT;
    else if (rpts1s_num < 10 && rpts0s_num > rpts1s_num)
        flag_track = TRACK_LEFT;
    else
        flag_track = TRACK_MIDDLE;


}

void ImageProcess::calculate_angle(void)
{

    // 找最近点(起始点中线归一化)
    float min_dist = 1e10;
    int begin_id = -1;
    flag_rpts = false;
    for (int i = 0; i < rpts_num; i++) {
        float dx = rpts[i][0] - cx;
        float dy = rpts[i][1] - cy;
        float dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            begin_id = i;
        }
    }
    begin_id = begin_id + element_begin_id >= rpts_num ? begin_id
        : begin_id + element_begin_id;

    // 特殊模式下, 不找最近点 (由于边线会绕一圈回来, 导致最近点为边线最后一个点, 从而中线无法正常生成)
    if (flag_garage == GARAGE_IN_LEFT || flag_garage == GARAGE_IN_RIGHT || flag_cross == CROSS_IN)
        begin_id = 0;

    // int bseqx_y1 = 0, bseqx_y2 = 0, bseqx_y3 = 0;
    // 中线有点, 同时最近点不是最后几个点
    if (begin_id >= 0 && rpts_num - begin_id >= 3) {
        // 找到中线
        flag_rpts = true;
        // 归一化中线
        rpts[begin_id][0] = cx;
        rpts[begin_id][1] = cy;
        rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
        resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num,
            SAMPLE_DIST * PIXEL_PER_METER);
        aim_idx_far = clip(round(aim_distance_f / SAMPLE_DIST), 0, rptsn_num - 1);
        aim_idx_near = clip(round(aim_distance_n / SAMPLE_DIST), 0, rptsn_num - 1);
        std::vector<POINT> v_center(4);  // 三阶贝塞尔曲线
        v_center[0] = { (int)cx, (int)cy };
        v_center[1] = { (int)rptsn[aim_idx_near][0], (int)rptsn[aim_idx_near][1] };
        v_center[2] = { (int)rptsn[(int)((aim_idx_far + aim_idx_near) / 2)][0], (int)rptsn[(int)((aim_idx_far + aim_idx_near) / 2)][1] };
        v_center[3] = { (int)rptsn[aim_idx_far][0], (int)rptsn[aim_idx_far][1] };
        bezier_line = bezier(0.03, v_center);

        // 计算远锚点偏差值
        float dx = bezier_line[bezier_line.size() - 1].x - cx;  // rptsn[aim_idx_far][0] - cx;
        float dy = cy - bezier_line[bezier_line.size() - 1].y;  // cy - rptsn[aim_idx_far][1];
        float error_far = (-atan2f(dx, dy) * 180 / PI);
        assert(!isnan(error_far));

        // 计算近锚点偏差值
        float dx_near = bezier_line[bezier_line.size() / 2].x - cx;  // rptsn[aim_idx_near][0] - cx;
        float dy_near = cy - bezier_line[bezier_line.size() / 2].y;  // cy - rptsn[aim_idx_near][1];
        float error_near = (-atan2f(dx_near, dy_near) * 180 / PI);
        assert(!isnan(error_near));

        aim_angle = error_far * 0.9 + error_near * 0.1;
        aim_sigma = sigma(rptsn + aim_idx_near, aim_idx_far - aim_idx_near);
        // aim_sigma = sigma(v_center);
    }
    else {
        // 中线点过少
        flag_rpts = false;
        aim_angle = aim_angle_last;
        aim_sigma = 100.0f;

        // // 施工区丢线
        // if (flag_work != WORK_NONE)
        //     aim_angle = aim_angle_last;
        // // 环岛内部丢线
        if (control_flag.top() == circle_element)
            aim_angle = aim_angle_last * 1.39f;
        // // 十字丢线
        // if (control_flag.top() == cross_element)
        //     aim_angle = aim_angle_last;
        // // 斑马线丢线
        // if (flag_garage >= GARAGE_PASS_LEFT)
        //     aim_angle = aim_angle_last;
        // else if (flag_garage >= GARAGE_IN_LEFT)
        //     aim_angle = aim_angle_last * 1.35f;
    }
}

void Control_Flag::push(int value) {
    data.push_back(value);
}

void Control_Flag::pop() {
    if (!data.empty()) {
        data.pop_back();
    }
}

int Control_Flag::top() {
    if (!data.empty()) {
        return data.back();
    }
    return none_element;
}

bool Control_Flag::contains(int value) {
    for (size_t i = 0; i < data.size(); i++) {
        if (data[i] == value) {
            return true;
        }
    }
    return false;
}

void Control_Flag::remove(int value) {
    for (size_t i = 0; i < data.size(); i++) {
        if (data[i] == value) {
            data.erase(data.begin() + i);
            break;
        }
    }
}