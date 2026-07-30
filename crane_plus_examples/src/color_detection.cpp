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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"

using std::placeholders::_1;
using std::placeholders::_2;

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("color_detection")
  {
    color_sub_.subscribe(this, "image_raw", "raw");
    info_sub_.subscribe(this, "camera_info");

    sync_ = std::make_unique<message_filters::Synchronizer<ExactPolicy>>(
      ExactPolicy(10), color_sub_, info_sub_);
    sync_->registerCallback(std::bind(&ImageSubscriber::camera_callback, this, _1, _2));

    image_thresholded_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_thresholded", 10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  using ExactPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  image_transport::SubscriberFilter color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::unique_ptr<message_filters::Synchronizer<ExactPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_thresholded_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void camera_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    // 赤い物体を検出するようにHSVの範囲を設定
    // 周囲の明るさ等の動作環境に合わせて調整
    const int LOW_H_1 = 0, HIGH_H_1 = 10;
    const int LOW_H_2 = 170, HIGH_H_2 = 179;
    const int LOW_S = 100, HIGH_S = 255;
    const int LOW_V = 100, HIGH_V = 255;

    // カメラ画像を受け取る
    auto cv_img = cv_bridge::toCvShare(img_msg, img_msg->encoding);

    // 画像をRGBからHSVに変換
    cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2HSV);

    // 画像処理用の変数を用意
    cv::Mat img_mask_1;
    cv::Mat img_mask_2;
    cv::Mat img_thresholded;

    // 画像の二値化
    cv::inRange(
      cv_img->image,
      cv::Scalar(LOW_H_1, LOW_S, LOW_V),
      cv::Scalar(HIGH_H_1, HIGH_S, HIGH_V),
      img_mask_1);
    cv::inRange(
      cv_img->image,
      cv::Scalar(LOW_H_2, LOW_S, LOW_V),
      cv::Scalar(HIGH_H_2, HIGH_S, HIGH_V),
      img_mask_2);

    // マスク画像の合成
    img_thresholded = img_mask_1 | img_mask_2;

    // ノイズ除去の処理
    cv::morphologyEx(
      img_thresholded,
      img_thresholded,
      cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // 穴埋めの処理
    cv::morphologyEx(
      img_thresholded,
      img_thresholded,
      cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // 画像の検出領域におけるモーメントを計算
    cv::Moments moment = moments(img_thresholded);
    double d_m01 = moment.m01;
    double d_m10 = moment.m10;
    double d_area = moment.m00;

    // 閾値による二値化画像を配信
    sensor_msgs::msg::Image::SharedPtr img_thresholded_msg =
      cv_bridge::CvImage(img_msg->header, "mono8", img_thresholded).toImageMsg();
    image_thresholded_publisher_->publish(*img_thresholded_msg);

    // 検出した領域のピクセル数が10000より大きい場合に把持位置を配信
    if (d_area <= 10000) {
      return;
    }

    // カメラモデル作成
    image_geometry::PinholeCameraModel camera_model;

    // カメラのパラメータを設定
    camera_model.fromCameraInfo(*info_msg);

    // 画像座標系における把持対象物の位置（2D）
    const double pixel_x = d_m10 / d_area;
    const double pixel_y = d_m01 / d_area;
    const cv::Point2d point(pixel_x, pixel_y);

    // 補正後の画像座標系における把持対象物の位置を取得（2D）
    const cv::Point2d rect_point = camera_model.rectifyPoint(point);

    // カメラ座標系から見た把持対象物の方向（Ray）を取得する
    const cv::Point3d ray = camera_model.projectPixelTo3dRay(rect_point);

    // カメラの高さを0.46[m]として把持対象物の位置を計算
    const double CAMERA_HEIGHT = 0.46;
    cv::Point3d object_position(
      ray.x * CAMERA_HEIGHT,
      ray.y * CAMERA_HEIGHT,
      ray.z * CAMERA_HEIGHT);

    // 把持対象物の位置をTFに配信
    geometry_msgs::msg::TransformStamped t;
    t.header = img_msg->header;
    t.child_frame_id = "target_0";
    t.transform.translation.x = object_position.x;
    t.transform.translation.y = object_position.y;
    t.transform.translation.z = object_position.z;
    tf_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
