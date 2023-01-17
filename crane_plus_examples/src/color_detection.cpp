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
#include "image_geometry/pinhole_camera_model.h"
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
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 環境に合わせて調整
  int low_h = 150, high_h = 200;
  int low_s = 100, high_s = 255;
  int low_v = 30, high_v = 255;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // OpenCVによる色検出
    auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
    cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2HSV);
    cv::Mat img_thresholded;
    cv::inRange(
      cv_img->image,
      cv::Scalar(low_h, low_s, low_v),
      cv::Scalar(high_h, high_s, high_v),
      img_thresholded);
    erode(
      img_thresholded,
      img_thresholded,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    dilate(
      img_thresholded,
      img_thresholded,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    dilate(
      img_thresholded,
      img_thresholded,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    erode(
      img_thresholded,
      img_thresholded,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::Moments moment = moments(img_thresholded);
    double d_m01 = moment.m01;
    double d_m10 = moment.m10;
    double d_area = moment.m00;

    if (d_area > 10000) {
      // カメラモデル作成
      image_geometry::PinholeCameraModel camera_model;

      // カメラのパラメータを設定
      camera_model.fromCameraInfo(camera_info_);

      // 画像中の把持対象物の位置（2D、ピクセル）
      const double pixel_x = d_m10 / d_area;
      const double pixel_y = d_m01 / d_area;
      const cv::Point2d point(pixel_x, pixel_y);

      // 補正後のカメラ画像における位置を取得（2D）
      const cv::Point2d rect_point = camera_model.rectifyPoint(point);

      // 画像における位置（2D）をカメラ座標系における位置（3D）に変換
      const cv::Point3d ray = camera_model.projectPixelTo3dRay(rect_point);

      // 高さ1.0[m]を高さcamera_height[m]として計算
      const double camera_height = 0.44;
      cv::Point3d ray_after(ray.x * camera_height, ray.y * camera_height, ray.z * camera_height);

      // 把持対象物の位置をTFに配信
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "camera_color_optical_frame";
      t.child_frame_id = "target";
      t.transform.translation.x = ray_after.x;
      t.transform.translation.y = ray_after.y;
      t.transform.translation.z = ray_after.z;
      tf_broadcaster_->sendTransform(t);
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_ = msg;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
