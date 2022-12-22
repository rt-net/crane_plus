#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include <iostream>
#include <iomanip>
using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node
{
  public:
    ImageSubscriber()
    : Node("color_detection")
    {
      image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, std::bind(&ImageSubscriber::image_callback, this, _1));

      camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", 10, std::bind(&ImageSubscriber::camera_info_callback, this, _1));

      tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    int lowH = 150;//170
    int highH = 200;//179
    int lowS = 100;//150
    int highS = 255;//255
    int lowV = 30;//60
    int highV = 255;//255

    // 1ピクセルあたりの距離（メートル）
    // 適当にメジャーで計測
    double mppX = 0.60 / 640;
    double mppY = 0.44 / 480;

    int posX = -1;
    int posY = -1;
    double X = 0;
    double Y = 0;
    double Z = 0.46; // 決め打ち

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
      cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2HSV);
      cv::Mat imageCopy;
      cv_img->image.copyTo(imageCopy);

      cv::Mat imageT;
      cv::inRange(imageCopy, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), imageT);

      erode(imageT, imageT, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      dilate(imageT, imageT, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

      dilate(imageT, imageT, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      erode(imageT, imageT, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

      cv::Moments oMoments = moments(imageT);

      double dM01 = oMoments.m01;
      double dM10 = oMoments.m10;
      double dArea = oMoments.m00;

      if(dArea > 10000)
      {
        posX = dM10 / dArea;
        posY = dM01 / dArea;
        posX -= imageT.cols/2;
        posY -= imageT.rows/2;
        X = posX * mppX;
        Y = posY * mppY;

        std::cout << std::fixed;
        std::cout << "X : ";
        std::cout << std::setprecision(3) << X;
        std::cout << ", Y : ";
        std::cout << std::setprecision(3) << Y;
        std::cout << ", Z : ";
        std::cout << std::setprecision(3) << Z;
        std::cout << std::endl;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "camera_color_optical_frame";
        t.child_frame_id = "target";
        t.transform.translation.x = X;
        t.transform.translation.y = Y;
        t.transform.translation.z = Z;
        tf_broadcaster_->sendTransform(t);
      }

      cv::imshow("out", imageT);
      cv::waitKey(1);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
      camera_info = msg;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
