#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <camera_info_manager/camera_info_manager.h>

using namespace cv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher left_pub =
    it.advertiseCamera("camera/left/image_raw", 1);
  image_transport::CameraPublisher right_pub =
    it.advertiseCamera("camera/right/image_raw", 1);
  camera_info_manager::CameraInfoManager left_cam_info(nh, "left_camera");
  camera_info_manager::CameraInfoManager
      right_cam_info(nh, "right_camera");
  left_cam_info.loadCameraInfo(
      "package://zed_camera_ros/params/left_camera_calibration.yaml");
  right_cam_info.loadCameraInfo(
      "package://zed_camera_ros/params/right_camera_calibration.yaml");
 
  if (!left_cam_info.isCalibrated()) {
    ROS_ERROR("Cannot load left camera calibration");
    return 1; 
  }
  if (!right_cam_info.isCalibrated()) {
    ROS_ERROR("Cannot load right camera calibration");
    return 1;
  }
  sensor_msgs::CameraInfoPtr cil(
      new sensor_msgs::CameraInfo(left_cam_info.getCameraInfo()));
  sensor_msgs::CameraInfoPtr cir(
      new sensor_msgs::CameraInfo(right_cam_info.getCameraInfo()));

  ros::NodeHandle nh_priv("~");
  int camera_number;
  nh_priv.param<int>("zed_camera_number", camera_number, 0);
  cv::VideoCapture cap(camera_number);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) {
    ROS_ERROR("Cannot open camera");
    return 1;
  }

  cv::Mat frame, left_frame, right_frame;
  sensor_msgs::ImagePtr left_msg, right_msg;
  // Setup frames
  cil->header.frame_id = "/camera_frame";
  cir->header.frame_id = "/camera_frame";
  while (nh.ok()) {
    cap >> frame;
    //cv::resize(frame, frame, cv::Size(1280, 480));
    if (!frame.empty()) {
      left_frame = frame(Rect(0, 0, frame.cols / 2, frame.rows));
      right_frame = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
      left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_frame)
          .toImageMsg();
      right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_frame)
          .toImageMsg();
      left_msg->header.frame_id = "/camera_frame";
      right_msg->header.frame_id = "/camera_frame";
      left_pub.publish(left_msg, cil);
      right_pub.publish(right_msg, cir);
    }
    ros::spinOnce();
  }
}
