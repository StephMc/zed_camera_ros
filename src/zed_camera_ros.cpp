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

image_transport::CameraPublisher left_pub;
image_transport::CameraPublisher right_pub;
sensor_msgs::CameraInfoPtr cil;
sensor_msgs::CameraInfoPtr cir;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat frame, left_frame, right_frame;
  sensor_msgs::ImagePtr left_msg, right_msg;
  //cv::resize(frame, frame, cv::Size(1280, 480));

  left_frame = cv_ptr->image(Rect(0, 0, cv_ptr->image.cols / 2, cv_ptr->image.rows));
  right_frame = cv_ptr->image(Rect(cv_ptr->image.cols / 2, 0, cv_ptr->image.cols / 2, cv_ptr->image.rows));
  left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_frame).toImageMsg();
  right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_frame).toImageMsg();
  left_msg->header.frame_id = "/camera_frame";
  right_msg->header.frame_id = "/camera_frame";
  left_pub.publish(left_msg, cil);
  right_pub.publish(right_msg, cir);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  left_pub = it.advertiseCamera("camera/left/image_raw", 1);
  right_pub = it.advertiseCamera("camera/right/image_raw", 1);

  camera_info_manager::CameraInfoManager left_cam_info(nh, "left_camera");
  camera_info_manager::CameraInfoManager right_cam_info(nh, "right_camera");

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
  cil.reset(new sensor_msgs::CameraInfo(left_cam_info.getCameraInfo()));
  cir.reset(new sensor_msgs::CameraInfo(right_cam_info.getCameraInfo()));
  cil->header.frame_id = "/camera_frame";
  cir->header.frame_id = "/camera_frame";

  image_transport::Subscriber image_sub =
    it.subscribe("/zed/usb_cam/image_raw", 1, imageCb);

  ros::spin();
  return 0;
}
