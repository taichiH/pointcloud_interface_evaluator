#include "sensor_msgs/msg/point_field.hpp"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <iostream>

namespace custom_type {
struct Point {
  float x;
  float y;
  float z;
  float intensity;
  uint16_t ring;
  float azimuth;
  float distance;
  uint8_t return_type;
  double time_stamp;
  friend bool operator==(const custom_type::Point &p1,
                         const custom_type::Point &p2) noexcept {
    return true;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZI {
  PCL_ADD_POINT4D;
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace custom_type

POINT_CLOUD_REGISTER_POINT_STRUCT(
    custom_type::PointXYZI,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    custom_type::Point,
    (float, x,
     x)(float, y,
        y)(float, z,
           z)(float, intensity,
              intensity)(uint16_t, ring,
                         ring)(float, azimuth,
                               azimuth)(float, distance,
                                        distance)(uint8_t, return_type,
                                                  return_type)(double,
                                                               time_stamp,
                                                               time_stamp))

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(azimuth);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(distance);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(return_type);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(time_stamp);

bool deserialize(std::vector<custom_type::Point> &pts,
                 const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {

  if (msg->data.size() != msg->row_step) {
    printf("invariant data\n");
    return false;
  }
  for (int idx = 0; idx < msg->row_step; idx += msg->point_step) {
    // offsetの作られ方が統一されるなら構造体のタイプにキャストしてしまっても良い
    unsigned char *addr = &msg->data[idx];
    float x = *reinterpret_cast<float *>(addr + msg->fields.at(0).offset);
    float y = *reinterpret_cast<float *>(addr + msg->fields.at(1).offset);
    float z = *reinterpret_cast<float *>(addr + msg->fields.at(2).offset);
    float intensity =
        *reinterpret_cast<float *>(addr + msg->fields.at(3).offset);
    uint16_t ring =
        *reinterpret_cast<uint16_t *>(addr + msg->fields.at(4).offset);
    float azimuth = *reinterpret_cast<float *>(addr + msg->fields.at(5).offset);
    float distance =
        *reinterpret_cast<float *>(addr + msg->fields.at(6).offset);
    uint8_t return_type =
        *reinterpret_cast<uint8_t *>(addr + msg->fields.at(7).offset);
    double time_stamp =
        *reinterpret_cast<double *>(addr + msg->fields.at(8).offset);
    pts.push_back(custom_type::Point{x, y, z, intensity, ring, azimuth,
                                     distance, return_type, time_stamp});
  }

  return true;
}

int main() {

  printf(" --- ELIGN16付きの構造体からXYZIを作る\n");
  {
    pcl::PointCloud<custom_type::PointXYZI>::Ptr pcl_cloud_4d(
        new pcl::PointCloud<custom_type::PointXYZI>);
    pcl_cloud_4d->width = 1;
    pcl_cloud_4d->height = 1;
    pcl_cloud_4d->points.push_back(custom_type::PointXYZI{1, 2, 3, 4});

    sensor_msgs::msg::PointCloud2::SharedPtr from_pcl_msg_4d(
        new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*pcl_cloud_4d, *from_pcl_msg_4d);
    printf("ELIGN16付きの構造体から作られたField\n");
    for (int i = 0; i < from_pcl_msg_4d->fields.size(); i++) {
      printf(" - offset %s: %ld,\n", from_pcl_msg_4d->fields.at(i).name.c_str(),
             from_pcl_msg_4d->fields.at(i).offset);
    }
    printf(" - point_step: %d", from_pcl_msg_4d->point_step);
  }
  printf("\n");

  /////////////////////////////////////////////////////
  printf(" --- ELIGN16付きの構造体からXYZIRADRTを作る\n");

  pcl::PointCloud<custom_type::Point>::Ptr pcl_cloud(
      new pcl::PointCloud<custom_type::Point>);
  pcl_cloud->width = 1;
  pcl_cloud->height = 1;
  pcl_cloud->points.push_back(custom_type::Point{1, 2, 3, 4, 5, 6, 7, 8, 9});

  sensor_msgs::msg::PointCloud2::SharedPtr from_pcl_msg(
      new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*pcl_cloud, *from_pcl_msg);

  printf("toROSMsgによって作られたField\n");
  for (int i = 0; i < from_pcl_msg->fields.size(); i++) {
    printf(" - offset %s: %ld,\n", from_pcl_msg->fields.at(i).name.c_str(),
           from_pcl_msg->fields.at(i).offset);
  }
  printf(" - point_step: %d",
         from_pcl_msg
             ->point_step); // ELIGN16の効果で48確保されるサイズ的には40で足りる
  printf("\n");

  printf("生のシリアライズデータ\n");
  for (int i = 0; i < from_pcl_msg->data.size(); i++) {
    uint8_t *decimal_ptr = reinterpret_cast<uint8_t *>(&from_pcl_msg->data[i]);
    printf("%d,", *decimal_ptr);
    if ((i + 1) % 4 == 0)
      printf(" | ");
  }
  printf("\n\n");
  std::vector<custom_type::Point> pts;
  deserialize(pts, from_pcl_msg);
  std::cout << "キャストしたデータ" << std::endl;
  std::cout << pts[0].x << ", ";
  std::cout << pts[0].y << ", ";
  std::cout << pts[0].z << ", ";
  std::cout << pts[0].intensity << ", ";
  std::cout << pts[0].ring << ", ";
  std::cout << pts[0].azimuth << ", ";
  std::cout << pts[0].distance << ", ";
  std::cout << static_cast<int>(pts[0].return_type) << ", ";
  std::cout << pts[0].time_stamp << ", ";
  std::cout << std::endl;

  // wrapper
  using CustomGenerator =
      std::tuple<point_cloud_msg_wrapper::field_x_generator,
                 point_cloud_msg_wrapper::field_y_generator,
                 point_cloud_msg_wrapper::field_z_generator,
                 point_cloud_msg_wrapper::field_intensity_generator,
                 point_cloud_msg_wrapper::field_ring_generator,
                 field_azimuth_generator, field_distance_generator,
                 field_return_type_generator, field_time_stamp_generator>;
  sensor_msgs::msg::PointCloud2 from_wrapper_msg;
  point_cloud_msg_wrapper::PointCloud2Modifier<custom_type::Point,
                                               CustomGenerator>
      msg_modifier{from_wrapper_msg, from_pcl_msg->header.frame_id};
  printf("wrapperによって作られたField\n");
  for (int i = 0; i < from_wrapper_msg.fields.size(); i++) {
    printf(" - offset %s: %ld,\n", from_wrapper_msg.fields.at(i).name.c_str(),
           from_wrapper_msg.fields.at(i).offset);
  }
  printf(" - point_step: %d", from_wrapper_msg.point_step);
  printf("\n\n");



  printf("wrapperのpush_backの性能を試す\n");
  std::chrono::system_clock::time_point start;
  start = std::chrono::system_clock::now();

  int vlp128_num = 225628;
  int num = std::pow(10, 7);
  std::cerr << "point num: " << num << std::endl;

  msg_modifier.clear();
  msg_modifier.reserve(num * from_wrapper_msg.point_step);

  for (uint32_t i = 0; i < num; i++) {
    custom_type::Point pt{10, 11, 12, 13, 14, 15, 16, 17, 18};
    msg_modifier.push_back(pt);
  }
  double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now() - start)
                       .count();
  std::cerr << "elapsed time (copy): " << elapsed << " (millisec)" << std::endl;

  msg_modifier.clear();

  start = std::chrono::system_clock::now();
  for (uint32_t i = 0; i < num; i++) {
    custom_type::Point pt{10, 11, 12, 13, 14, 15, 16, 17, 18};
    msg_modifier.push_back(std::move(pt));
  }
  elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - start)
                .count();
  std::cerr << "elapsed time (copy): " << elapsed << " (millisec)" << std::endl;

  start = std::chrono::system_clock::now();
  for (uint32_t i = 0; i < num; i++) {
    if (i % 2 == 0) {
      continue;
    }
  }
  elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - start)
                .count();
  std::cerr << "elapsed time (copy): " << elapsed << " (millisec)" << std::endl;
}
