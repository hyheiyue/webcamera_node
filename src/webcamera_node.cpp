#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <sched.h>

namespace camera
{
class CameraNode : public rclcpp::Node
{
public:
  CameraNode(const rclcpp::NodeOptions & options) 
  : Node("camera", options), 
    is_stream_opened_(false),
    debug_mode_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing CameraNode...");

    // 参数声明与获取
    this->declare_parameter<std::string>("stream_url", "");
    this->declare_parameter<bool>("use_sensor_data_qos", true);
    this->declare_parameter<std::string>("camera_name", "narrow_stereo");
    this->declare_parameter<std::string>("camera_info_url", "");
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<std::string>("encoding", "bgr8"); // 支持 bgr8 和 jpeg

    const auto stream_url = this->get_parameter("stream_url").as_string();
    const auto use_sensor_qos = this->get_parameter("use_sensor_data_qos").as_bool();
    const auto camera_name = this->get_parameter("camera_name").as_string();
    const auto camera_info_url = this->get_parameter("camera_info_url").as_string();
    const auto encoding = this->get_parameter("encoding").as_string();
    debug_mode_ = this->get_parameter("debug").as_bool();

    if(stream_url.empty()){
      RCLCPP_FATAL(this->get_logger(), "Stream URL parameter is empty!");
      rclcpp::shutdown();
      return;
    }

    // 视频流初始化
    try {
      // 打开HTTP流
      video_cap_.open(stream_url, cv::CAP_FFMPEG);
      if (video_cap_.isOpened()) {
        // 设置缓冲区大小
        if (!video_cap_.set(cv::CAP_PROP_BUFFERSIZE, 1)) {
          RCLCPP_WARN(this->get_logger(), "Failed to set buffer size");
        }

        // 获取帧属性
        frame_width_ = video_cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        frame_height_ = video_cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        RCLCPP_INFO(this->get_logger(), "Stream opened with resolution: %dx%d", 
                   frame_width_, frame_height_);
        is_stream_opened_ = true;
      } else {
        throw std::runtime_error("Failed to open video stream");
      }
    } 
    catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Stream init error: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // ROS接口初始化（优化QoS设置）
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_profile.depth = 1;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_profile.deadline = {0, 100000000}; // 100ms
    const auto qos = use_sensor_qos ? qos_profile : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    // 相机标定信息管理
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name);
    if(!camera_info_url.empty()){
      try {
        if(camera_info_manager_->validateURL(camera_info_url) && 
           camera_info_manager_->loadCameraInfo(camera_info_url)){
          camera_info_msg_ = camera_info_manager_->getCameraInfo();
          RCLCPP_INFO(this->get_logger(), "Loaded camera info from: %s", camera_info_url.c_str());
        }
      } 
      catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Camera info load failed: %s", e.what());
      }
    }

    // 采集线程
    capture_thread_ = std::thread([this, stream_url, encoding](){
      // 设置实时线程优先级
      struct sched_param param;
      param.sched_priority = sched_get_priority_max(SCHED_FIFO);
      if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to set thread priority");
      }

      cv::Mat current_frame;
      auto last_pub_time = std::chrono::steady_clock::now();

      while (rclcpp::ok()) {
        // 自动重连机制
        if (!is_stream_opened_) {
          RCLCPP_WARN(this->get_logger(), "Attempting to reconnect...");
          if (video_cap_.open(stream_url, cv::CAP_FFMPEG)) {
            is_stream_opened_ = true;
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
          }
        }

        // 帧捕获
        if (!video_cap_.read(current_frame) || current_frame.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Frame capture failed!");
          is_stream_opened_ = false;
          video_cap_.release();
          continue;
        }

        // 准备ROS消息
        auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
        img_msg->header.stamp = this->now();
        img_msg->header.frame_id = "camera_optical_frame";
        img_msg->height = current_frame.rows;
        img_msg->width = current_frame.cols;
        img_msg->step = current_frame.step;

        if (encoding == "bgr8") {
          // bgr8 模式：直接使用原始图像数据
          img_msg->encoding = "bgr8";
          img_msg->data.assign(current_frame.data, current_frame.data + current_frame.total() * current_frame.elemSize());
        } else if (encoding == "jpeg") {
          // jpeg 模式：压缩图像
          std::vector<uchar> compressed_buffer;
          cv::imencode(".jpg", current_frame, compressed_buffer);
          img_msg->encoding = "jpeg";
          img_msg->data = compressed_buffer;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", encoding.c_str());
          continue;
        }

        // 将camera_info_msg_转换为ConstSharedPtr
        auto camera_info_msg_ptr = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_msg_);
        camera_info_msg_ptr->header.stamp = img_msg->header.stamp;
        camera_info_msg_ptr->header.frame_id = img_msg->header.frame_id;

        // 发布消息
        camera_pub_.publish(img_msg, camera_info_msg_ptr);

        // 调试信息
        if (debug_mode_) {
          auto now = std::chrono::steady_clock::now();
          auto pub_duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pub_time).count();
          RCLCPP_INFO(this->get_logger(), "Publish interval: %ld ms", pub_duration);
          last_pub_time = now;
        }
      }
    });
  }

  ~CameraNode() override
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down");
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (video_cap_.isOpened()) {
      video_cap_.release();
    }
  }

private:
  // 成员变量
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  cv::VideoCapture video_cap_;
  std::atomic<bool> is_stream_opened_;
  unsigned int frame_width_ = 0;
  unsigned int frame_height_ = 0;
  std::thread capture_thread_;
  std::atomic<bool> debug_mode_;
};

} // namespace camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)
