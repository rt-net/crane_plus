// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Reference:
// https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html

#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
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

  // 環境に合わせて調整
  int lowH = 150;
  int highH = 200;
  int lowS = 100;
  int highS = 255;
  int lowV = 30;
  int highV = 255;

  // 1ピクセルあたりの距離（メートル）
  // 適当にメジャーで計測
  double mppX = 0.60 / 640;
  double mppY = 0.44 / 480;

  // 画像中の把持対象物の位置
  int posX = 0;
  int posY = 0;

  // カメラ座標系における把持対象物の位置
  double X = 0;
  double Y = 0;
  double Z = 0.46;  // 決め打ち

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // OpenCVによる色検出
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

    if (dArea > 10000) {
      // 把持対象物の位置を算出
      posX = dM10 / dArea;
      posY = dM01 / dArea;
      posX -= imageT.cols / 2;  // 原点の調整
      posY -= imageT.rows / 2;  // 原点の調整
      X = posX * mppX;
      Y = posY * mppY;

      // デバッグ用の出力
      std::cout << std::fixed;
      std::cout << "X : ";
      std::cout << std::setprecision(3) << X;
      std::cout << ", Y : ";
      std::cout << std::setprecision(3) << Y;
      std::cout << ", Z : ";
      std::cout << std::setprecision(3) << Z;
      std::cout << std::endl;

      // 把持対象物の位置をTFに配信
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
