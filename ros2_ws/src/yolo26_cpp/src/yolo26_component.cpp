#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <cv_bridge/cv_bridge.hpp>

#include "yolo26_cpp/yolo26_detector.hpp"

namespace yolo26_cpp
{

class Yolo26Component : public rclcpp::Node
{
public:
  explicit Yolo26Component(const rclcpp::NodeOptions& options)
    : Node("yolo26_component", options)
  {
    declare_parameter("model_path", "/home/pi/yolo26/yolo26n_ncnn_model");
    declare_parameter("input_size", 416);
    declare_parameter("confidence_threshold", 0.5);
    declare_parameter("num_threads", 4);
    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("detections_topic", "/yolo26/detections");
    declare_parameter("max_detection_rate", 15.0);

    Yolo26Detector::Config config;
    config.model_path = get_parameter("model_path").as_string();
    config.input_size = get_parameter("input_size").as_int();
    config.confidence_threshold = static_cast<float>(get_parameter("confidence_threshold").as_double());
    config.num_threads = get_parameter("num_threads").as_int();

    detector_ = std::make_unique<Yolo26Detector>(config);
    detector_->warmup();

    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      get_parameter("detections_topic").as_string(), 10);

    rclcpp::QoS qos(1);
    qos.best_effort();
    
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("image_topic").as_string(), qos,
      std::bind(&Yolo26Component::imageCallback, this, std::placeholders::_1));

    max_rate_ = get_parameter("max_detection_rate").as_double();
    min_interval_ = (max_rate_ > 0) ? 1.0 / max_rate_ : 0.0;

    RCLCPP_INFO(get_logger(), "YOLO26 Component initialized");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    auto current_time = now();
    if (min_interval_ > 0) {
      if ((current_time - last_time_).seconds() < min_interval_) return;
    }
    last_time_ = current_time;

    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      
      std::vector<Detection> detections;
      detector_->detect(cv_ptr->image, detections);

      auto det_msg = std::make_unique<vision_msgs::msg::Detection2DArray>();
      det_msg->header = msg->header;

      for (const auto& det : detections) {
        vision_msgs::msg::Detection2D d;
        d.bbox.center.position.x = det.bbox.x + det.bbox.width / 2.0;
        d.bbox.center.position.y = det.bbox.y + det.bbox.height / 2.0;
        d.bbox.size_x = det.bbox.width;
        d.bbox.size_y = det.bbox.height;

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = std::to_string(det.class_id);
        hyp.hypothesis.score = det.confidence;
        d.results.push_back(hyp);
        d.id = det.class_name;

        det_msg->detections.push_back(d);
      }

      detections_pub_->publish(std::move(det_msg));
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Error: %s", e.what());
    }
  }

  std::unique_ptr<Yolo26Detector> detector_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  
  double max_rate_;
  double min_interval_;
  rclcpp::Time last_time_;
};

}  // namespace yolo26_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(yolo26_cpp::Yolo26Component)