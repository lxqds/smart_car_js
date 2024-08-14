#include "yyds.hpp"

#undef LOG_TAG
#define LOG_TAG "Main"

ImageProcess imgprocess; // 图像处理
logger_helper logger;    // 日志

std::shared_ptr<CaptureInterface> capture = nullptr; // 线程 -> 摄像头
std::shared_ptr<Detection> detection = nullptr;      // 线程 -> 目标检测
std::shared_ptr<Display> display = nullptr;          // 线程 -> 图像显示及视频录制
std::shared_ptr<SerialDriver> serial = nullptr;      // 线程 -> 串口接收及发送

extern uint8_t gas_which;
uint8_t serial_echo;

/* 主函数 */
int main(int argc, char const* argv[])
{
    /* 数据定义 */
    bool _windows, _video;  // 命令行参数
    int ret = 0;            // 初始化返回
    Payload_t payload;      // 数据包 (EB -> TC212)
    Payload_t payload_exit; // 数据包 (EB -> TC212)
    payload_exit.tAngle = 0;
    payload_exit.tSpeed = 0;
    payload_exit.element = 0;

    char fps_buffer[16]; // 帧率图像显示缓存
    double fps_time = 0; // 帧率计时变量
    StopWatch fps_sw;    // 帧率计时器
    cv::Mat frame_origin; // 摄像头原始图像
    cv::Mat frame_imgpro; // 图像处理输出

    // StopWatch echo_sw;   // 应答计时器
    /* 命令行参数 */
    auto args = util::argparser("yyds car command line parameters explained.");
    args.set_program_name("yyds")
        .add_help_option()
        .add_option("-l", "--launch", "launch!!!")
        .add_option("-w", "--windows", "show windows")
        .add_option("-v", "--video", "save video")
        .parse(argc, argv);
    _windows = args.has_option("--windows");
    _video = args.has_option("--video");
    /* 日志初始化 */
    logger.init_logger();

    /* 程序退出信号 */
    signal(SIGINT, callbackSignal);

    /* 图像处理 */
    ret = imgprocess.init("../configs/control.json", (_windows || _video));
    CHECK(0 == ret) << "../configs/control.json can not found !!" << LOG_END;
    LOG_I << "exposure=" << imgprocess._config->exposure << LOG_END;
    /* 摄像头 */
    capture = std::make_shared<CaptureInterface>();
    ret = capture->init(imgprocess._config->exposure);
    CHECK(0 == ret) << "device video can not open !!" << LOG_END;

    /* 目标检测 */
    detection = std::make_shared<Detection>();
    ret = detection->init("../configs/usbcamera.json");
    CHECK(0 == ret) << "../configs/usbcamera.json can not found !!" << LOG_END;

    /* 窗口 */
    display = std::make_shared<Display>(_windows, _video);
    ret = display->init();
    CHECK(0 == ret) << "windows or video can not create !!" << LOG_END;

    /* 串口 */
    serial = std::make_shared<SerialDriver>("/dev/ttyUSB0", BaudRate::BAUD_115200);
    ret = serial->open();
    CHECK(0 == ret) << "/dev/ttyUSB0 can not open !!" << LOG_END;

    /* 启动多线程 */
    capture->start();
    detection->start();
    display->start();

    /* 初始化PID */
    float speedPID[3] = {
        imgprocess._config->speed_kp,
        imgprocess._config->speed_ki,
        imgprocess._config->speed_kd };
    memcpy(imgprocess.speedPID_BUFF, speedPID, 12);
    LOG_I << "kp=" << speedPID[0] << LOG_END;
    LOG_I << "ki=" << speedPID[1] << LOG_END;
    LOG_I << "kd=" << speedPID[2] << LOG_END;
    /* 开跑 */

    imgprocess.init_slave_flag = 1;
    // serial->buzzerSound(3);
    LOG_I << "PID Initing\n" << speedPID[0] << LOG_END;
    imgprocess.brake_flag = 1;
    while (capture->_loop) {

        // 开始计时
        fps_sw.tic();
        if (imgprocess.init_slave_flag) {

            serial->transmit_packet(0x09, imgprocess.speedPID_BUFF, 12);

            if (serial_echo == 0xA1) {
                serial_echo = 0;
                LOG_I << "pid set" << "..." << serial_echo << LOG_END;
                imgprocess.init_slave_flag = 0;//如果有响应
            }

        }




        if (imgprocess.brake_flag)serial->transmit_packet(0x10, (uint8_t*)"1", 1);
        if (serial_echo == 0xA2) { 
            imgprocess.brake_flag = 0;   
            serial_echo = 0;//如果有响应
            LOG_I << "shou!" << "..." << serial_echo << LOG_END;
         }
        // 获取图像
        capture->get(frame_origin);

        // 目标检测
       detection->run(frame_origin);IMG.edge_

        {
            // 图像处理 正常帧率
            imgprocess.process(frame_origin, frame_imgpro, payload);

            ret = 1;
        }

        if (ret != 0) {
            ret = 0;

            // 发送指令
            if (capture->_launch) {
                serial->sendPack(payload);
            }
            static double _time = fps_time;
            _time = _time * 0.95 + fps_time * 0.05;
            // 显示图像
            sprintf(fps_buffer, "FPS: %.2f", 1000 / _time);
            cv::putText(frame_imgpro, (std::string)fps_buffer, cv::Point(5, 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255));


            if (imgprocess._config->rec_video_format == 0) {
                display->put(frame_origin.clone());
            }
            else {
                display->put(frame_imgpro.clone());
            }
            // display->put(IMG.mat_lab.clone());
        }

        // 停止计时
        fps_time = fps_sw.toc();
    }

    /* 结束程序 */
    serial->sendPack(payload_exit);
    serial->sendPack(payload_exit);
    serial->sendPack(payload_exit);
    serial->sendPack(payload_exit);
    serial->sendPack(payload_exit);
    callbackSignal(0);
    exit(EXIT_SUCCESS);
}

#undef LOG_TAG
#define LOG_TAG "System"

/* 系统信号回调函数: 系统退出 (Ctrl+C) */
void callbackSignal(int signum)
{
    /* 防止多次退出系统 */
    static uint8_t is_exit = 0;
    is_exit += 1;

    if (is_exit == 1) {
        /* 结束线程 */
        capture->stop();
        log_i("capture stopped.");
        detection->stop();
        log_i("detection stopped.");
        display->stop();
        log_i("display stopped.");
        serial->close();
        log_i("serial stopped.");

        /* 结束日志 */
        log_i("Exit Code: %d", signum);
        log_i("Finish !!!");
    }
}

