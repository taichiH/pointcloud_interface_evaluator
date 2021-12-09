#include "pointcloud_interface_evaluator/evaluator.hpp"

namespace pointcloud_interface_evaluator {

Evaluator::Evaluator(const rclcpp::NodeOptions &options)
    : Node("Evaluator", options) {

  use_pcl_ = declare_parameter("use_pcl", false);
  use_wrapper_ = declare_parameter("use_wrapper", false);
  use_nothing_ = declare_parameter("use_nothing", false);
  use_only_itr_ = declare_parameter("use_only_itr", false);

  points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input", rclcpp::SensorDataQoS(),
      std::bind(&Evaluator::onPoints, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/output", rclcpp::SensorDataQoS());
}

void Evaluator::onPoints(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {

  if (use_nothing_) {
    return;
  }

  if (use_pcl_) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    sensor_msgs::msg::PointCloud2::SharedPtr output(
        new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*cloud, *output);
    auto publish_output =
        std::make_unique<sensor_msgs::msg::PointCloud2>(*output);
    pub_->publish(std::move(publish_output));
  }

  if (use_wrapper_) {
    sensor_msgs::msg::PointCloud2::SharedPtr tmp_msg =
        std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
    sensor_msgs::msg::PointCloud2 output;
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> msg_modifier{
        output, tmp_msg->header.frame_id};

    std::size_t point_step = tmp_msg->point_step;
    msg_modifier.reserve(tmp_msg->data.size());
    for (std::size_t idx = 0U; idx < tmp_msg->data.size(); idx += point_step) {
      PointXYZI *pt;
      pt = reinterpret_cast<PointXYZI *>(&tmp_msg->data[idx]);
      msg_modifier.push_back(PointXYZI{pt->x, pt->y, pt->z, pt->intensity});
    }
    output.header = tmp_msg->header;
    auto publish_output =
        std::make_unique<sensor_msgs::msg::PointCloud2>(output);
    pub_->publish(std::move(publish_output));
  }

  if (use_only_itr_) {
    sensor_msgs::msg::PointCloud2::SharedPtr tmp_msg =
        std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
    sensor_msgs::msg::PointCloud2 output;
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> msg_modifier{
        output, tmp_msg->header.frame_id};

    std::size_t point_step = msg->point_step;
    msg_modifier.reserve(tmp_msg->data.size());
    for (std::size_t idx = 0U; idx < tmp_msg->data.size(); idx += point_step) {
      PointXYZI *pt;
      pt = reinterpret_cast<PointXYZI *>(&tmp_msg->data[idx]);
    }
  }
}

} // namespace pointcloud_interface_evaluator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_interface_evaluator::Evaluator)