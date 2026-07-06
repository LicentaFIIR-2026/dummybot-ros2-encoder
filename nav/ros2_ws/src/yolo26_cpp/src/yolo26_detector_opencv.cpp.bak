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
  // Load ONNX model with OpenCV DNN
  std::string onnx_path = config_.model_path;
  
  // Check if path ends with _ncnn_model, convert to onnx path
  if (onnx_path.find("_ncnn_model") != std::string::npos) {
    // Try to find .onnx file in parent directory
    size_t pos = onnx_path.find("_ncnn_model");
    onnx_path = onnx_path.substr(0, pos) + ".onnx";
  }
  
  net_ = cv::dnn::readNetFromONNX(onnx_path);
  
  if (net_.empty()) {
    throw std::runtime_error("Failed to load model: " + onnx_path);
  }

  // Optimize for CPU
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  
  // Set number of threads
  cv::setNumThreads(config_.num_threads);

  // Pre-allocate letterbox buffer
  letterboxed_ = cv::Mat(config_.input_size, config_.input_size, CV_8UC3, cv::Scalar(114, 114, 114));
}

void Yolo26Detector::warmup()
{
  cv::Mat dummy(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
  std::vector<Detection> dummy_det;
  detect(dummy, dummy_det);
  detect(dummy, dummy_det);  // Second run for stable timing
}

const std::string& Yolo26Detector::getClassName(int class_id)
{
  static const std::string unknown = "unknown";
  if (class_id >= 0 && class_id < static_cast<int>(COCO_CLASSES.size())) {
    return COCO_CLASSES[class_id];
  }
  return unknown;
}

void Yolo26Detector::preprocess(const cv::Mat& image, cv::Mat& blob)
{
  const int input_size = config_.input_size;
  const int img_h = image.rows;
  const int img_w = image.cols;

  // Calculate letterbox parameters
  ratio_ = std::min(static_cast<float>(input_size) / img_h,
                    static_cast<float>(input_size) / img_w);
  
  const int new_w = static_cast<int>(std::round(img_w * ratio_));
  const int new_h = static_cast<int>(std::round(img_h * ratio_));
  
  pad_w_ = (input_size - new_w) / 2.0f;
  pad_h_ = (input_size - new_h) / 2.0f;

  // Reset letterbox to gray
  letterboxed_.setTo(cv::Scalar(114, 114, 114));

  // Resize and place in center
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  
  const int top = static_cast<int>(std::round(pad_h_ - 0.1f));
  const int left = static_cast<int>(std::round(pad_w_ - 0.1f));
  
  resized.copyTo(letterboxed_(cv::Rect(left, top, new_w, new_h)));

  // Convert to blob (NCHW format, normalized 0-1)
  cv::dnn::blobFromImage(letterboxed_, blob, 1.0/255.0, 
                         cv::Size(input_size, input_size),
                         cv::Scalar(0, 0, 0), true, false, CV_32F);
}

void Yolo26Detector::postprocess(const cv::Mat& output, const cv::Size& img_size,
                                  std::vector<Detection>& detections)
{
  detections.clear();
  
  const int img_w = img_size.width;
  const int img_h = img_size.height;
  const float conf_threshold = config_.confidence_threshold;

  // YOLO26 output shape: [1, 84, N] where 84 = 4 (bbox) + 80 (classes)
  // We need to transpose to [N, 84]
  
  cv::Mat output_mat;
  if (output.dims == 3) {
    // Shape: [1, 84, N] -> reshape to [84, N] then transpose to [N, 84]
    const int num_features = output.size[1];  // 84
    const int num_anchors = output.size[2];   // N
    
    output_mat = output.reshape(1, num_features);  // [84, N]
    cv::transpose(output_mat, output_mat);          // [N, 84]
  } else if (output.dims == 2) {
    if (output.rows == 84) {
      cv::transpose(output, output_mat);  // [N, 84]
    } else {
      output_mat = output;  // Already [N, 84]
    }
  } else {
    return;  // Unexpected format
  }

  const int num_detections = output_mat.rows;
  const int num_classes = 80;
  
  // Pre-reserve space
  detections.reserve(100);

  // Process each detection
  const float* data = reinterpret_cast<const float*>(output_mat.data);
  const int row_size = output_mat.cols;

  for (int i = 0; i < num_detections; ++i) {
    const float* row = data + i * row_size;
    
    // Find best class (indices 4-83)
    int best_class = 0;
    float best_score = row[4];
    
    for (int c = 1; c < num_classes; ++c) {
      if (row[4 + c] > best_score) {
        best_score = row[4 + c];
        best_class = c;
      }
    }

    // Skip low confidence
    if (best_score < conf_threshold) {
      continue;
    }

    // Class filter
    if (!config_.class_filter.empty()) {
      if (std::find(config_.class_filter.begin(), config_.class_filter.end(), 
                    best_class) == config_.class_filter.end()) {
        continue;
      }
    }

    // Get bbox (cx, cy, w, h) in input coordinates
    const float cx = row[0];
    const float cy = row[1];
    const float w = row[2];
    const float h = row[3];

    // Convert to x1, y1, x2, y2 and scale to original image
    float x1 = (cx - w / 2.0f - pad_w_) / ratio_;
    float y1 = (cy - h / 2.0f - pad_h_) / ratio_;
    float x2 = (cx + w / 2.0f - pad_w_) / ratio_;
    float y2 = (cy + h / 2.0f - pad_h_) / ratio_;

    // Clip to image bounds
    x1 = std::max(0.0f, std::min(x1, static_cast<float>(img_w)));
    y1 = std::max(0.0f, std::min(y1, static_cast<float>(img_h)));
    x2 = std::max(0.0f, std::min(x2, static_cast<float>(img_w)));
    y2 = std::max(0.0f, std::min(y2, static_cast<float>(img_h)));

    // Skip invalid boxes
    if (x2 <= x1 || y2 <= y1) {
      continue;
    }

    Detection det;
    det.class_id = best_class;
    det.class_name = COCO_CLASSES[best_class];
    det.confidence = best_score;
    det.bbox = cv::Rect(
      static_cast<int>(x1),
      static_cast<int>(y1),
      static_cast<int>(x2 - x1),
      static_cast<int>(y2 - y1)
    );
    det.bbox_norm = cv::Rect2f(
      x1 / img_w,
      y1 / img_h,
      (x2 - x1) / img_w,
      (y2 - y1) / img_h
    );

    detections.push_back(det);
  }

  // Sort by confidence (highest first)
  std::sort(detections.begin(), detections.end(),
            [](const Detection& a, const Detection& b) {
              return a.confidence > b.confidence;
            });
}

double Yolo26Detector::detect(const cv::Mat& image, std::vector<Detection>& detections)
{
  auto start = std::chrono::high_resolution_clock::now();

  // Preprocess
  preprocess(image, blob_);

  // Forward pass
  net_.setInput(blob_);
  net_.forward(outputs_, net_.getUnconnectedOutLayersNames());

  auto end_inference = std::chrono::high_resolution_clock::now();

  // Postprocess
  if (!outputs_.empty()) {
    postprocess(outputs_[0], image.size(), detections);
  }

  auto end = std::chrono::high_resolution_clock::now();
  
  double inference_ms = std::chrono::duration<double, std::milli>(end_inference - start).count();
  
  return inference_ms;
}

}  // namespace yolo26_cpp