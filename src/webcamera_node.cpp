#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace camera
{
class CameraNode : public rclcpp::Node
{
public:
   CameraNode(const rclcpp::NodeOptions & options) : Node("camera", options), stream_opened_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Starting CameraNode!");

    // 声明参数
    this->declare_parameter<std::string>("stream_url", "");
    this->declare_parameter<bool>("use_sensor_data_qos", true);
    this->declare_parameter<std::string>("camera_name", "narrow_stereo");
    this->declare_parameter<std::string>("camera_info_url", "package://webcamera_node/config/camera_info.yaml");
    this->declare_parameter<bool>("debug", false);

    // 获取参数
    std::string stream_url = this->get_parameter("stream_url").as_string();
    bool use_sensor_data_qos = this->get_parameter("use_sensor_data_qos").as_bool();
    std::string camera_name = this->get_parameter("camera_name").as_string();
    std::string camera_info_url = this->get_parameter("camera_info_url").as_string();
    debug = this->get_parameter("debug").as_bool();
    if(debug)
    {
    RCLCPP_INFO(this->get_logger(), "Stream URL: %s", stream_url.c_str());
    RCLCPP_INFO(this->get_logger(), "Use sensor data QoS: %s", use_sensor_data_qos ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Camera Name: %s", camera_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera Info URL: %s", camera_info_url.c_str());
    }

    // 尝试打开视频流
    try {
      cap_.open(stream_url, cv::CAP_FFMPEG);
      if (!cap_.isOpened()) {
        throw std::runtime_error("Failed to open video stream: " + stream_url);
      }
      stream_opened_ = true;
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Error details: %s", e.what());
    }

    // 设置 QoS
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    // 加载相机信息
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name);
    try {
      if (camera_info_manager_->validateURL(camera_info_url)) {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
        RCLCPP_INFO(this->get_logger(), "Camera info loaded successfully.");
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load camera info: %s", e.what());
    }

    // 创建采集线程
    capture_thread_ = std::thread{[this]() -> void {
      cv::Mat frame;
      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "bgr8";

      RCLCPP_INFO(this->get_logger(), "Publishing video stream!");

      while (rclcpp::ok()&&debug) {
        if (!stream_opened_) {
          RCLCPP_WARN(this->get_logger(), "Video stream not opened. Skipping frame capture.");
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          continue;
        }

        cap_ >> frame;

        if (frame.empty()&&debug) {
          RCLCPP_WARN(this->get_logger(), "Failed to capture frame from stream!");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
        }

        // 填充 ROS 图像消息
        image_msg_.header.stamp = this->now();
        image_msg_.height = frame.rows;
        image_msg_.width = frame.cols;
        image_msg_.step = frame.cols * frame.elemSize();
        image_msg_.data.assign(frame.datastart, frame.dataend);

        camera_info_msg_.header = image_msg_.header;
        camera_pub_.publish(image_msg_, camera_info_msg_);
      }
    }};
  }

  ~CameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (cap_.isOpened()) {
      cap_.release();
    }
    RCLCPP_INFO(this->get_logger(), "CameraNode destroyed!");
  }

private:
  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  cv::VideoCapture cap_;
  std::thread capture_thread_;
  bool stream_opened_;
  bool debug;
};
}  

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<camera::CameraNode>(rclcpp::NodeOptions());
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)