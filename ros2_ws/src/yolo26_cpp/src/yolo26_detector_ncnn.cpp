#include "yolo26_cpp/yolo26_detector.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace yolo26_cpp
{

const std::array<std::string, 80> Yolo26Detector::COCO_CLASSES = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
  "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
  "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
  "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
  "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
  "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
  "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
  "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
  "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
  "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
  "toothbrush"
};

Yolo26Detector::Yolo26Detector(const Config& config)
  : config_(config), ratio_(1.0f), pad_w_(0.0f), pad_h_(0.0f)
{
  // Configure NCNN
  net_.opt.use_vulkan_compute = false;
  net_.opt.num_threads = config_.num_threads;
  net_.opt.lightmode = true;
  net_.opt.use_packing_layout = true;
  
  // Load model
  std::string param_path = config_.model_path + "/model.ncnn.param";
  std::string bin_path = config_.model_path + "/model.ncnn.bin";
  
  int ret_param = net_.load_param(param_path.c_str());
  int ret_bin = net_.load_model(bin_path.c_str());
  
  if (ret_param != 0 || ret_bin != 0) {
    throw std::runtime_error("Failed to load NCNN model: " + config_.model_path);
  }
}

void Yolo26Detector::warmup()
{
  cv::Mat dummy(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
  std::vector<Detection> dets;
  detect(dummy, dets);
  detect(dummy, dets);
}

const std::string& Yolo26Detector::getClassName(int class_id)
{
  static const std::string unknown = "unknown";
  if (class_id >= 0 && class_id < static_cast<int>(COCO_CLASSES.size())) {
    return COCO_CLASSES[class_id];
  }
  return unknown;
}

void Yolo26Detector::preprocess(const cv::Mat& image, ncnn::Mat& in)
{
  const int input_size = config_.input_size;
  const int img_h = image.rows;
  const int img_w = image.cols;

  // Letterbox
  ratio_ = std::min(static_cast<float>(input_size) / img_h,
                    static_cast<float>(input_size) / img_w);
  
  const int new_w = static_cast<int>(std::round(img_w * ratio_));
  const int new_h = static_cast<int>(std::round(img_h * ratio_));
  
  pad_w_ = (input_size - new_w) / 2.0f;
  pad_h_ = (input_size - new_h) / 2.0f;

  // Create letterboxed image
  cv::Mat letterboxed(input_size, input_size, CV_8UC3, cv::Scalar(114, 114, 114));
  
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  
  const int top = static_cast<int>(std::round(pad_h_ - 0.1f));
  const int left = static_cast<int>(std::round(pad_w_ - 0.1f));
  resized.copyTo(letterboxed(cv::Rect(left, top, new_w, new_h)));

  // Convert to NCNN Mat (BGR -> RGB, normalize)
  in = ncnn::Mat::from_pixels(letterboxed.data, ncnn::Mat::PIXEL_BGR2RGB, 
                               input_size, input_size);
  
  const float norm_vals[3] = {1/255.0f, 1/255.0f, 1/255.0f};
  in.substract_mean_normalize(0, norm_vals);
}

void Yolo26Detector::postprocess(const ncnn::Mat& out, const cv::Size& img_size,
                                  std::vector<Detection>& detections)
{
  detections.clear();
  
  const int img_w = img_size.width;
  const int img_h = img_size.height;
  const float conf_threshold = config_.confidence_threshold;

  // YOLO26 output: (84, N) where 84 = 4 + 80
  // Need to transpose mentally: iterate over columns
  const int num_features = out.h;  // 84
  const int num_anchors = out.w;   // N
  
  detections.reserve(100);

  for (int i = 0; i < num_anchors; ++i) {
    // Get class scores
    int best_class = 0;
    float best_score = out.row(4)[i];
    
    for (int c = 1; c < 80; ++c) {
      float score = out.row(4 + c)[i];
      if (score > best_score) {
        best_score = score;
        best_class = c;
      }
    }

    if (best_score < conf_threshold) {
      continue;
    }

    if (!config_.class_filter.empty()) {
      if (std::find(config_.class_filter.begin(), config_.class_filter.end(),
                    best_class) == config_.class_filter.end()) {
        continue;
      }
    }

    // Get bbox
    float cx = out.row(0)[i];
    float cy = out.row(1)[i];
    float w = out.row(2)[i];
    float h = out.row(3)[i];

    // Convert to original image coordinates
    float x1 = (cx - w / 2.0f - pad_w_) / ratio_;
    float y1 = (cy - h / 2.0f - pad_h_) / ratio_;
    float x2 = (cx + w / 2.0f - pad_w_) / ratio_;
    float y2 = (cy + h / 2.0f - pad_h_) / ratio_;

    // Clip
    x1 = std::max(0.0f, std::min(x1, static_cast<float>(img_w)));
    y1 = std::max(0.0f, std::min(y1, static_cast<float>(img_h)));
    x2 = std::max(0.0f, std::min(x2, static_cast<float>(img_w)));
    y2 = std::max(0.0f, std::min(y2, static_cast<float>(img_h)));

    if (x2 <= x1 || y2 <= y1) {
      continue;
    }

    Detection det;
    det.class_id = best_class;
    det.class_name = COCO_CLASSES[best_class];
    det.confidence = best_score;
    det.bbox = cv::Rect(static_cast<int>(x1), static_cast<int>(y1),
                        static_cast<int>(x2 - x1), static_cast<int>(y2 - y1));
    det.bbox_norm = cv::Rect2f(x1 / img_w, y1 / img_h,
                               (x2 - x1) / img_w, (y2 - y1) / img_h);
    
    detections.push_back(det);
  }

  std::sort(detections.begin(), detections.end(),
            [](const Detection& a, const Detection& b) {
              return a.confidence > b.confidence;
            });
}

double Yolo26Detector::detect(const cv::Mat& image, std::vector<Detection>& detections)
{
  ncnn::Mat in;
  preprocess(image, in);

  auto start = std::chrono::high_resolution_clock::now();
  
  ncnn::Extractor ex = net_.create_extractor();
  ex.input("in0", in);
  
  ncnn::Mat out;
  ex.extract("out0", out);
  
  auto end = std::chrono::high_resolution_clock::now();
  double inference_ms = std::chrono::duration<double, std::milli>(end - start).count();

  postprocess(out, image.size(), detections);

  return inference_ms;
}

}  // namespace yolo26_cpp