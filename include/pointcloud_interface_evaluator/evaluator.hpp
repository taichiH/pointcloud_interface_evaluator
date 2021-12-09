#ifndef POINTCLOUD_INTERFACE_EVALUATOR_HPP_
#define POINTCLOUD_INTERFACE_EVALUATOR_HPP_

#include "sensor_msgs/msg/point_field.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pointcloud_interface_evaluator {

struct PointXYZI
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  friend bool operator==(
    const PointXYZI & p1,
    const PointXYZI & p2) noexcept
  {
    return true;
  }
};

class Evaluator : public rclcpp::Node {
public:
  explicit Evaluator(const rclcpp::NodeOptions &options);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  void onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  bool use_pcl_;
  bool use_wrapper_;
  bool use_nothing_;
  bool use_only_itr_;
};
} // namespace pointcloud_interface_evaluator

#endif // POINTCLOUD_INTERFACE_EVALUATOR_HPP_