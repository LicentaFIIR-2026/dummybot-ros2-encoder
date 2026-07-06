#ifndef YOLO26_CPP__YOLO26_DETECTOR_HPP_
#define YOLO26_CPP__YOLO26_DETECTOR_HPP_

#include <ncnn/net.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <array>
#include <memory>

namespace yolo26_cpp
{

struct Detection
{
  int class_id;
  std::string class_name;
  float confidence;
  cv::Rect bbox;
  cv::Rect2f bbox_norm;
};

class Yolo26Detector
{
public:
  struct Config
  {
    std::string model_path;
    int input_size = 416;
    float confidence_threshold = 0.5f;
    int num_threads = 4;
    std::vector<int> class_filter;
  };

  explicit Yolo26Detector(const Config& config);
  ~Yolo26Detector() = default;

  Yolo26Detector(const Yolo26Detector&) = delete;
  Yolo26Detector& operator=(const Yolo26Detector&) = delete;

  double detect(const cv::Mat& image, std::vector<Detection>& detections);
  void warmup();
  static const std::string& getClassName(int class_id);

private:
  void preprocess(const cv::Mat& image, ncnn::Mat& in);
  void postprocess(const ncnn::Mat& out, const cv::Size& img_size,
                   std::vector<Detection>& detections);

  Config config_;
  ncnn::Net net_;
  
  float ratio_;
  float pad_w_;
  float pad_h_;

  static const std::array<std::string, 80> COCO_CLASSES;
};

}  // namespace yolo26_cpp

#endif
