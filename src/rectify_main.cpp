#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat rmap[2][2];
cv::Mat r_img1, r_img2;
image_transport::Publisher rectified_l_publisher, rectified_r_publisher;
std::string stereo_parameters;
int IMG_HEIGHT_R;
int IMG_WIDTH_R;

void readCalibrationParameters(cv::Mat &K1, cv::Mat &K2, cv::Mat &R,
                               cv::Mat &R1, cv::Mat &R2, cv::Mat &P1,
                               cv::Mat &P2, cv::Vec3d &T,
                               cv::Vec4d &D1, cv::Vec4d &D2, cv::Size &img_size)
{
  cv::FileStorage fs(stereo_parameters,cv::FileStorage::READ);
  if (!fs.isOpened()) {
    ROS_ERROR("Failed to open calibration parameter file.");
    exit(1);
  }
  fs["K1"] >> K1;
  fs["K2"] >> K2;
  fs["R"] >> R;
  fs["R1"] >> R1;
  fs["R2"] >> R2;
  fs["P1"] >> P1;
  fs["P2"] >> P2;
  fs["T"] >> T;
  fs["D1"] >> D1;
  fs["D2"] >> D2;
  img_size.height = IMG_HEIGHT_R;
  img_size.width = IMG_WIDTH_R;
}

void applyRectificationMaps(cv::Mat img_left, cv::Mat img_right)
{
  cv::remap(img_left, r_img1, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
  cv::remap(img_right, r_img2, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
  if(r_img1.channels() > 1)
    cv::cvtColor(r_img1,r_img1,CV_RGB2GRAY);
  if(r_img2.channels() > 1)
    cv::cvtColor(r_img2,r_img2,CV_RGB2GRAY);
}

void publishRectifiedImages(cv::Mat &left_rect, cv::Mat &right_rect, cv_bridge::CvImagePtr cv_ptr)
{
  cv_bridge::CvImage out_rect_l;
  cv_bridge::CvImage out_rect_r;
  out_rect_l.header   = cv_ptr->header;
  out_rect_r.header = cv_ptr->header;
  out_rect_l.encoding   = sensor_msgs::image_encodings::MONO8;
  out_rect_r.encoding   = sensor_msgs::image_encodings::MONO8;
  cv::Mat r_l_u8, r_r_u8;
  left_rect.convertTo(r_l_u8, CV_8UC1);
  right_rect.convertTo(r_r_u8, CV_8UC1);
  out_rect_l.image = r_l_u8;
  out_rect_r.image = r_r_u8;
  rectified_l_publisher.publish(out_rect_l.toImageMsg());
  rectified_r_publisher.publish(out_rect_r.toImageMsg());
}

void rectifyCallback(const sensor_msgs::ImageConstPtr& cam_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img_1, img_2;
  try
  {
  cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::MONO8);
  cv::Rect roi_left;
  roi_left.x = 0;
  roi_left.y = 0;
  roi_left.width = 1280;
  roi_left.height = 1024;
  cv::Rect roi_right;
  roi_right.x = 0;
  roi_right.y = 1024;
  roi_right.width = 1280;
  roi_right.height = 1024;
  cv::Mat img1 = cv_ptr->image(roi_left);
  cv::Mat img2 = cv_ptr->image(roi_right);
  applyRectificationMaps(img1, img2);
  publishRectifiedImages(r_img1, r_img2, cv_ptr);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not execute callback function for stereo rectify");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_rectify");
  ros::NodeHandle nh("~");

  nh.param("calib_file", stereo_parameters, std::string(" "));
  ROS_INFO("Calibration file location (Stereo Fisheye Rectify): %s", stereo_parameters.c_str());
  nh.param("img_width", IMG_WIDTH_R, 1280);
  nh.param("img_height", IMG_HEIGHT_R, 1024);

  cv::Vec3d T;
  cv::Vec4d D1,D2;
  cv::Mat R1, R2, P1, P2, K1, K2, R;
  cv::Size img_size;

  readCalibrationParameters(K1, K2, R, R1, R2, P1, P2, T, D1, D2, img_size);

  cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_16SC2, rmap[0][0], rmap[0][1]);
  cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_16SC2, rmap[1][0], rmap[1][1]);

  image_transport::ImageTransport it(nh);

  ros::Subscriber image_sub = nh.subscribe("image", 1, &rectifyCallback);

  rectified_l_publisher = it.advertise("left/rectified",1);
  rectified_r_publisher   = it.advertise("right/rectified",1);
  ros::spin();
}
