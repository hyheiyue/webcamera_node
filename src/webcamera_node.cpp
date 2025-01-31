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

    // **************************
    // * 参数声明与获取
    // **************************
    /* 参数说明:
     * - stream_url: 视频流地址 (e.g. "rtsp://ip:port/stream")
     * - use_sensor_data_qos: 是否使用传感器数据QoS配置
     * - camera_name: 相机命名空间 (用于话题和相机信息管理)
     * - camera_info_url: 相机标定文件路径 
     * - debug: 调试模式开关
     */
    this->declare_parameter<std::string>("stream_url", "");
    this->declare_parameter<bool>("use_sensor_data_qos", true);
    this->declare_parameter<std::string>("camera_name", "narrow_stereo");
    this->declare_parameter<std::string>("camera_info_url", "");
    this->declare_parameter<bool>("debug", false);

    const auto stream_url = this->get_parameter("stream_url").as_string();
    const auto use_sensor_qos = this->get_parameter("use_sensor_data_qos").as_bool();
    const auto camera_name = this->get_parameter("camera_name").as_string();
    const auto camera_info_url = this->get_parameter("camera_info_url").as_string();
    debug_mode_ = this->get_parameter("debug").as_bool();

    // 参数有效性检查
    if(stream_url.empty()){
      RCLCPP_FATAL(this->get_logger(), "Stream URL parameter is empty!");
      rclcpp::shutdown();
      return;
    }

    // **************************
    // * 视频流初始化
    // **************************
    /* 使用OpenCV的FFmpeg后端打开视频流
     * 支持协议包括: rtsp, http, 本地文件等
     * 注意: 需要OpenCV编译时包含FFmpeg支持
     */
    RCLCPP_INFO(this->get_logger(), "Attempting to open stream: %s", stream_url.c_str());
    try {
      video_cap_.open(stream_url, cv::CAP_FFMPEG);
      if (!video_cap_.isOpened()) {
        throw std::runtime_error("Failed to open video stream");
      }
      is_stream_opened_ = true;
      RCLCPP_INFO(this->get_logger(), "Stream opened successfully");
      
      // 获取初始帧用于确定分辨率
      cv::Mat test_frame;
      if(video_cap_.read(test_frame)){
        frame_width_ = test_frame.cols;
        frame_height_ = test_frame.rows;
      }
    } 
    catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Stream init error: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // **************************
    // * ROS 接口初始化
    // **************************
    // QoS配置 (传感器数据模式使用更低的延迟配置)
    const auto qos = use_sensor_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    // 相机标定信息管理
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name);
    try {
      if(camera_info_manager_->validateURL(camera_info_url)){
        if(camera_info_manager_->loadCameraInfo(camera_info_url)){
          camera_info_msg_ = camera_info_manager_->getCameraInfo();
          RCLCPP_INFO(this->get_logger(), "Loaded camera info from: %s", camera_info_url.c_str());
          
          // 检查标定信息与视频分辨率是否匹配
          if(camera_info_msg_.width != static_cast<unsigned int>(frame_width_) || 
             camera_info_msg_.height != static_cast<unsigned int>(frame_height_)){
            RCLCPP_WARN(this->get_logger(), 
              "Camera info resolution (%dx%d) doesn't match video stream (%dx%d)!", 
              camera_info_msg_.width, camera_info_msg_.height, frame_width_, frame_height_);
          }
        }
      }
    } 
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Camera info load failed: %s", e.what());
    }

    // **************************
    // * 采集线程启动
    // **************************
    /* 独立线程处理视频采集与发布:
     * 1. 避免阻塞主线程
     * 2. 实现稳定的帧率控制
     * 3. 包含自动重连机制
     */
    capture_thread_ = std::thread([this, stream_url](){
      RCLCPP_INFO(this->get_logger(), "Capture thread started");
      
      cv::Mat current_frame;
      sensor_msgs::msg::Image img_msg;
      img_msg.encoding = "bgr8";
      img_msg.header.frame_id = "camera_optical_frame";

      // 根据摄像头帧率计算采集间隔 (默认30fps)
      const auto frame_interval = std::chrono::milliseconds(33); 
      
      while(rclcpp::ok()){
        // 自动重连机制
        if(!is_stream_opened_){
          RCLCPP_WARN(this->get_logger(), "Attempting to reconnect to stream...");
          if(video_cap_.open(stream_url, cv::CAP_FFMPEG)){
            is_stream_opened_ = true;
            RCLCPP_INFO(this->get_logger(), "Reconnected to stream successfully");
          } else {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
          }
        }

        // 帧捕获
        if(!video_cap_.read(current_frame) || current_frame.empty()){
          RCLCPP_ERROR(this->get_logger(), "Frame capture failed! Stream disconnected?");
          is_stream_opened_ = false;
          video_cap_.release();
          continue;
        }

        // 准备ROS消息
        const auto timestamp = this->now();
        img_msg.header.stamp = timestamp;
        img_msg.height = current_frame.rows;
        img_msg.width = current_frame.cols;
        img_msg.step = current_frame.step;
        img_msg.data.assign(current_frame.datastart, current_frame.dataend);

        // 更新并发布相机信息
        camera_info_msg_.header.stamp = timestamp;
        camera_info_msg_.header.frame_id = img_msg.header.frame_id;
        
        // 发布消息
        camera_pub_.publish(img_msg, camera_info_msg_);

        // 帧率控制
        std::this_thread::sleep_for(frame_interval);
      }
    });
  }

  ~CameraNode() override
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down CameraNode");
    
    // 终止采集线程
    if(capture_thread_.joinable()){
      capture_thread_.join();
    }
    
    // 释放视频资源
    if(video_cap_.isOpened()){
      video_cap_.release();
    }
  }

private:
  // ROS 接口
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  // 视频采集
  cv::VideoCapture video_cap_;
  std::atomic<bool> is_stream_opened_;
  unsigned int frame_width_ = 0;  
  unsigned int frame_height_ = 0; 

  // 线程管理
  std::thread capture_thread_;
  std::atomic<bool> debug_mode_;
};

} // namespace camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)
