#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include "image_transport/image_transport.hpp"

#include "yolo26_cpp/yolo26_detector.hpp"

namespace yolo26_cpp
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Yolo26Node : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Yolo26Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp_lifecycle::LifecycleNode("yolo26_detector", options)
  {
    declare_parameter("model_path", "/home/pi/yolo26/yolo26n_ncnn_model");
    declare_parameter("input_size", 416);
    declare_parameter("confidence_threshold", 0.5);
    declare_parameter("num_threads", 4);
    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("detections_topic", "/yolo26/detections");
    declare_parameter("max_detection_rate", 15.0);
    declare_parameter("class_filter", std::vector<int64_t>{});

    RCLCPP_INFO(get_logger(), "YOLO26 C++ Node created (Unconfigured)");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(get_logger(), "Configuring...");

    try {
      Yolo26Detector::Config config;
      config.model_path = get_parameter("model_path").as_string();
      config.input_size = get_parameter("input_size").as_int();
      config.confidence_threshold = static_cast<float>(get_parameter("confidence_threshold").as_double());
      config.num_threads = get_parameter("num_threads").as_int();
      
      auto filter = get_parameter("class_filter").as_integer_array();
      for (auto& c : filter) {
        config.class_filter.push_back(static_cast<int>(c));
      }

      detector_ = std::make_unique<Yolo26Detector>(config);
      
      RCLCPP_INFO(get_logger(), "Warming up...");
      detector_->warmup();

      detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        get_parameter("detections_topic").as_string(), 10);

      max_rate_ = get_parameter("max_detection_rate").as_double();
      min_interval_ = (max_rate_ > 0) ? 1.0 / max_rate_ : 0.0;

      RCLCPP_INFO(get_logger(), "Model: %s", config.model_path.c_str());
      RCLCPP_INFO(get_logger(), "Input: %dx%d, Threads: %d", 
                  config.input_size, config.input_size, config.num_threads);
      RCLCPP_INFO(get_logger(), "Configuration complete");

      return CallbackReturn::SUCCESS;
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(get_logger(), "Activating...");

    rclcpp::QoS qos(1);
    qos.best_effort();
    qos.durability_volatile();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("image_topic").as_string(), qos,
      std::bind(&Yolo26Node::imageCallback, this, std::placeholders::_1));

    frame_count_ = 0;
    total_inference_time_ = 0.0;
    last_log_time_ = now();
    last_detection_time_ = now();

    RCLCPP_INFO(get_logger(), "YOLO26 C++ Node active");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    image_sub_.reset();

    if (frame_count_ > 0) {
      double avg_ms = total_inference_time_ / frame_count_;
      RCLCPP_INFO(get_logger(), "Stats: %ld frames, avg %.1f ms", frame_count_, avg_ms);
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(get_logger(), "Cleanup...");
    detector_.reset();
    detections_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(get_logger(), "Shutdown...");
    return CallbackReturn::SUCCESS;
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    if (!detector_) return;

    auto current_time = now();
    if (min_interval_ > 0) {
      if ((current_time - last_detection_time_).seconds() < min_interval_) return;
    }
    last_detection_time_ = current_time;

    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

      std::vector<Detection> detections;
      double inference_ms = detector_->detect(cv_ptr->image, detections);

      frame_count_++;
      total_inference_time_ += inference_ms;

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

      double log_elapsed = (current_time - last_log_time_).seconds();
      if (log_elapsed >= 5.0) {
        double fps = frame_count_ / log_elapsed;
        double avg_ms = total_inference_time_ / frame_count_;
        RCLCPP_INFO(get_logger(), "FPS: %.1f, Inference: %.1f ms, Detections: %zu",
                    fps, avg_ms, detections.size());
        frame_count_ = 0;
        total_inference_time_ = 0.0;
        last_log_time_ = current_time;
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
    }
  }

  std::unique_ptr<Yolo26Detector> detector_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;

  double max_rate_;
  double min_interval_;
  rclcpp::Time last_detection_time_;

  int64_t frame_count_ = 0;
  double total_inference_time_ = 0.0;
  rclcpp::Time last_log_time_;
};

}  // namespace yolo26_cpp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<yolo26_cpp::Yolo26Node>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}