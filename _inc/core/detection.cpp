#include "detection.hpp"

#undef LOG_TAG
#define LOG_TAG "DetPredictor"

void DetectionPredictor::_boundaryCorrection(DetectionPredictResult &r,
                                             int width_range,
                                             int height_range) {
#define MARGIN_PIXELS (2)
    r.width = (r.width > (width_range - r.x - MARGIN_PIXELS))
                  ? (width_range - r.x - MARGIN_PIXELS)
                  : r.width;
    r.height = (r.height > (height_range - r.y - MARGIN_PIXELS))
                   ? (height_range - r.y - MARGIN_PIXELS)
                   : r.height;

    r.x = (r.x < MARGIN_PIXELS) ? MARGIN_PIXELS : r.x;
    r.y = (r.y < MARGIN_PIXELS) ? MARGIN_PIXELS : r.y;
}

// 初始化
int DetectionPredictor::init() {
   /* 
    _model_config = std::make_shared<ModelConfig>(_config_path);

    CHECK(_model_config != nullptr) << "Create Model config failed, config path: "
                                    << _config_path << LOG_END;

    std::vector<Place> valid_places({
        Place{TARGET(kFPGA), PRECISION(kFP16), DATALAYOUT(kNHWC)},
        Place{TARGET(kHost), PRECISION(kFloat)},
        Place{TARGET(kARM), PRECISION(kFloat)},
    });

    paddle::lite_api::CxxConfig config;

    if (_model_config->is_combined_model) {
        config.set_model_file(_model_config->model_file);
        config.set_param_file(_model_config->params_file);
    } else {
        config.set_model_dir(_model_config->model_params_dir);
    }

    config.set_valid_places(valid_places);

    _predictor = paddle::lite_api::CreatePaddlePredictor(config);

    if (!_predictor) {
        log_e("Create Paddle Predictor Failed.");
        return -1;
    }
    log_i("Init Success.");
    */

    /* NEW PPNC PREDICTION CODES BELOW*/

    // Initialize model
    LOG_I << "[PPNC] Initialize model." << LOG_END;
    this->predictor_nna_ = std::make_shared<PPNCPredictor>("../configs/config_ppncnna.json");//如何构建 NNA? 
    this->predictor_nms_ = std::make_shared<PPNCPredictor>("../configs/config_ppncnms.json");//判断重合框，与部署无关
    this->onnx_env_ = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "test");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(8);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    std::string onnx_model = _config_path + "/post.onnx";
    this->predictor_onnx_ = std::make_shared<Ort::Session>(this->onnx_env_, onnx_model.c_str(), session_options);
    LOG_I << "Done." << LOG_END;

    // Load ONNX model
    LOG_I << "[PPNC] Load ONNX model." << LOG_END;
    this->onnx_input_names_.first.push_back("im_shape");
    this->onnx_input_names_.first.push_back("scale_factor");
    std::string io_paddle = _config_path + "/io_paddle.json";
    LOG_I << "Done." << LOG_END;

    // Read io_paddle
    LOG_I << "[PPNC] Read io_paddle." << LOG_END;
    std::ifstream ifs(io_paddle);
    LOG_I << "Done." << LOG_END;

    // Read json file
    LOG_I << "[PPNC] Read JSON file." << LOG_END;
    nlohmann::json j;
    ifs >> j;
    ifs.close();
    LOG_I << "Done." << LOG_END;

    // Compile .so file
    LOG_I << "[PPNC] Compile NMS .so file." << LOG_END;
    for (size_t i = 0; i < j.size(); ++i) {
        if (j[i]["type"] == "OUTPUT") {
            assert(j[i]["shape"].size() == 4);
            this->onnx_input_names_.first.push_back(j[i]["name"]);
        }
        if (j[i]["type"] == "post_out") {
            this->onnx_out_names_.first.push_back(j[i]["name"]);
        }
    }

    for (auto &s : this->onnx_input_names_.first) {
        this->onnx_input_names_.second.push_back(s.c_str());
    }

    for (auto &s : this->onnx_out_names_.first) {
        this->onnx_out_names_.second.push_back(s.c_str());
    }
    build_nms(_config_path);
    LOG_I << "Done." << LOG_END;

    // Load predictor
    LOG_I << "[PPNC] Load Predictor." << LOG_END;
    this->predictor_nna_->load();
    this->predictor_nms_->load();
    LOG_I << "Done." << LOG_END;

    // Load labels
    LOG_I << "[PPNC] Load labels." << LOG_END;
    std::string pathLabels = _config_path + "/label_list.txt";
    labels.clear();
    std::ifstream file(pathLabels);
    if (file.is_open()) {
        std::string line;
        while (getline(file, line)) {
            labels.push_back(line);
        }
        file.close();

        for (uint8_t i = 0; i < labels.size(); i++) {
            LOG_I << "[Label " << (i+0) << "] " << labels.at(i) << LOG_END;
        }

        LOG_I << "Done." << LOG_END;

        return 0;
    } else {
        LOG_I << "Failed." << LOG_END;
    }
    return -1;
}

void DetectionPredictor::build_nms(const std::string &model_dir) {
    std::string model_file = model_dir + "/nms.tar";
    std::string untar_cmd = "tar -xf " + model_file + " -C . --no-same-owner";
    std::string final_file = model_file + ".so";
    std::string cc_cmd = "g++ -shared -fPIC -o " + final_file + " lib0.o devc.o";
    int sys_status = 0;

    sys_status = system(untar_cmd.c_str());
    if (sys_status) {
        LOG_I << "ERROR: cannot untar file " << model_file << LOG_END;
        exit(-1);
    }

    // create shared
    sys_status = system(cc_cmd.c_str());
    if (sys_status) {
        LOG_I << "ERROR: compile for " << model_file << LOG_END;
        exit(-1);
    }
    LOG_I << "Compile done." << LOG_END;
}

std::vector<DetectionPredictResult> DetectionPredictor::predict(cv::Mat input_frame) {
    /*
    std::vector<DetectionPredictResult> predict_ret;

    auto input = _predictor->GetInput(0);
    input->Resize({1, 3, _model_config->input_height, _model_config->input_width});

    // auto *in_data = input->mutable_data<uint16_t>();
    input->mutable_data<uint16_t>();
    fpga_preprocess(input_frame, _model_config, input);

    if (_model_config->is_yolo) {
        auto img_shape = _predictor->GetInput(1);
        img_shape->Resize({1, 2});
        auto *img_shape_data = img_shape->mutable_data<int32_t>();
        img_shape_data[0] = input_frame.rows;
        img_shape_data[1] = input_frame.cols;
    }

    _predictor->Run();

    auto output = _predictor->GetOutput(0);
    float *result_data = output->mutable_data<float>();
    int size = output->shape()[0];

    for (int i = 0; i < size; i++) {
        float *data = result_data + i * 6;
        float score = data[1];
        if (score < _model_config->threshold) {
            continue;
        }
        DetectionPredictResult r;
        r.type = (int)data[0];
        r.score = score;
        if (_model_config->is_yolo) {
            r.x = data[2];
            r.y = data[3];
            r.width = data[4] - r.x;
            r.height = data[5] - r.y;
        } else {
            r.x = data[2] * input_frame.cols;
            r.y = data[3] * input_frame.rows;
            r.width = data[4] * input_frame.cols - r.x;
            r.height = data[5] * input_frame.rows - r.y;
        }
        predict_ret.push_back(r);
    }

    return predict_ret;
    */

    std::vector<DetectionPredictResult> predict_ret;

    auto feeds = preprocess(input_frame, {320, 320});
    auto &image = feeds->at("image");

    this->predictor_nna_->set_inputs({{"image", image}});

    // ppnc_nna run
    this->predictor_nna_->run();

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    std::vector<Ort::Value> input_tensors;
    NDTensor im_shape = feeds->at("im_shape");
    NDTensor scale_factor = feeds->at("scale_factor");

    int numel = std::accumulate(im_shape.shape.begin(), im_shape.shape.end(), 1, std::multiplies<>());
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        memory_info,
        im_shape.value(),
        numel,
        im_shape.shape.data(),
        im_shape.shape.size()));
    numel = std::accumulate(scale_factor.shape.begin(), scale_factor.shape.end(), 1, std::multiplies<>());
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        memory_info,
        scale_factor.value(),
        numel,
        scale_factor.shape.data(),
        scale_factor.shape.size()));

    for (size_t i = 2; i < this->onnx_input_names_.second.size(); ++i) {
        const NDTensor &t = this->predictor_nna_->get_output(i - 2);
        numel = std::accumulate(t.shape.begin(), t.shape.end(), 1, std::multiplies<>());
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, t.value(), numel, t.shape.data(), t.shape.size()));
    }

    Ort::RunOptions run_options;
    auto onnx_out = this->predictor_onnx_->Run(
        run_options,
        this->onnx_input_names_.second.data(),
        input_tensors.data(),
        input_tensors.size(),
        this->onnx_out_names_.second.data(),
        this->onnx_out_names_.second.size());

    auto *data0 = onnx_out[0].GetTensorData<float>();
    auto *data1 = onnx_out[1].GetTensorData<float>();
    auto output_shape0 = onnx_out[0].GetTensorTypeAndShapeInfo().GetShape();
    auto output_shape1 = onnx_out[1].GetTensorTypeAndShapeInfo().GetShape();

    int32_t cls_num, num, box_point;

    // bbox
    num = output_shape0[1];
    box_point = output_shape0[2];
    cls_num = output_shape1[1];

    NDTensor bboxes({1, num, box_point});
    NDTensor scores({1, cls_num, num});

    std::memcpy(bboxes.value(), data0, num * box_point * sizeof(float));
    std::memcpy(scores.value(), data1, cls_num * num * sizeof(float));

    // ppnc_nms run
    this->predictor_nms_->set_inputs({{"bboxes", bboxes}, {"scores", scores}});
    this->predictor_nms_->run();

    // Get result and return
    NDTensor res = this->predictor_nms_->get_output(0);
    auto data = res.value();
    auto prod = std::accumulate(res.shape.begin(), res.shape.end(), 1, std::multiplies<int64_t>());

    predict_ret.clear();
    DetectionPredictResult result;
    for (int i = 0; i < prod; i += 6) {
        result.type = data[i];
        result.score = data[i + 1];

        if (result.score < 0.5) {
            continue;
        }
        result.x = data[i + 2];
        result.y = data[i + 3];
        result.width = data[i + 4] - data[i + 2];
        result.height = data[i + 5] - data[i + 3];

         //LOG_I << "result.type = " << result.type << "\r\n"
         //      << "result.score = " << result.score << LOG_END;
        // LOG_I << "result.x = " << result.x << "\r\n"
        //       << "result.y = " << result.y << "\r\n"
        //       << "result.width = " << result.width << "\r\n"
        //       << "result.height = " << result.score << LOG_END;
        //
        //printResults(result);
        drawResults(input_frame,result);
        predict_ret.push_back(result);
    }

    return predict_ret;
}

std::shared_ptr<std::unordered_map<std::string, NDTensor>> DetectionPredictor::preprocess(cv::Mat frame, const std::vector<int64_t> &input_size) {
    cv::Mat x;
    NDTensor scale_factor({1, 2}), img({1, 3, input_size[0], input_size[1]});
    scale_factor.value()[0] = static_cast<float>(input_size[0]) / frame.size[0];
    scale_factor.value()[1] = static_cast<float>(input_size[1]) / frame.size[1];
    NDTensor im_shape({1, 2});
    im_shape.value()[0] = input_size[0];
    im_shape.value()[1] = input_size[1];
    cv::cvtColor(frame, x, cv::COLOR_BGR2RGB);
    cv::resize(x, x, cv::Size(input_size[0], input_size[1]), 0, 0, 2);
    x.convertTo(x, CV_32FC3);
    x *= 1 / 255.0;
    cv::subtract(x, cv::Scalar(0.485, 0.456, 0.406), x);
    cv::multiply(x, cv::Scalar(1 / 0.229, 1 / 0.224, 1 / 0.225), x);
    TransposeAndCopyToTensor(x, img);

    std::unordered_map<std::string, NDTensor> ret = {
        {"image", img}, {"im_shape", im_shape}, {"scale_factor", scale_factor}};

    return std::make_shared<std::unordered_map<std::string, NDTensor>>(ret);
}

void DetectionPredictor::TransposeAndCopyToTensor(const cv::Mat &src, NDTensor &dst) {
    cv::Mat channels[3];
    split(src, channels);
    int offset = src.rows * src.cols;
    auto start = dst.value();
    for (int i = 0; i < 3; ++i) {
        std::memcpy(start + i * offset, channels[i].data,
                    offset * sizeof(float));
    }
}

std::string DetectionPredictor::getLabel(uint16_t type) {
    if ((type < 0) || (type > this->labels.size())) {
        LOG_I << "Predictor Label [" << type << "] is error." << LOG_END;
        return "BackGround";
    }

    return this->labels[type];
}


void DetectionPredictor::printResults(
    DetectionPredictResult &results) {

        DetectionPredictResult r = results;
        if (r.type >= 0 && (uint16_t)r.type < this->labels.size()) {
            LOG_I << "label:" << this->labels[r.type] << LOG_END;
            LOG_I << "\t\tindex:" << r.type << ", score:" << r.score
                  << ", loc:" << r.x << ", " << r.y << ", " << r.width << ", "
                  << r.height << LOG_END;
        }
    
}


// 针对检测结果, 进行画框操作
void DetectionPredictor::drawResults(
    cv::Mat &input_frame, DetectionPredictResult &results) {

        DetectionPredictResult r = results;
        _boundaryCorrection(r, input_frame.cols, input_frame.rows);
        if (r.type >= 0 && (uint16_t)r.type < this->labels.size()) {
            cv::Point origin(r.x, r.y);
            std::string label_name = this->labels[r.type];
            cv::putText(input_frame, label_name, origin, cv::FONT_HERSHEY_PLAIN, 1,
                        cv::Scalar(255, 0, 0, 255), 2);
        }
        cv::Rect rect(r.x, r.y, r.width, r.height);
        cv::rectangle(input_frame, rect, cv::Scalar(0, 0, 224), 2);
}

