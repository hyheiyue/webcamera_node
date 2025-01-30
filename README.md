# webcamera_node
This package is a ROS node that uses OpenCV to capture images from a webcam and publish them as ROS images.
# environment
* ROS2 Humble

# build
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install
```
# run
```
ros2 launch webcamera_node webcamera_node.launch.py
```
# parameters
* config/camera_params.yaml
```yaml
webcamera_node:
  ros__parameters:
    stream_url: "http://IP:PORT/video"
    use_sensor_data_qos: true
    camera_name: "narrow_stereo"
    camera_info_url: "package://webcamera_node/config/camera_info.yaml"
    debug: true
```
* config/camera_info.yaml    **A series of parameters for your camera，you can demarcate by**
```
  ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.015 image:=/image_raw camera:=/camera
``` 
**and then use the new camera_info instead of the default one**



