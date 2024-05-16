#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals; // enable the usage of 1s, 1ms, 1us, etc

struct ImuData
{
  double acc_x = -1;
  double acc_y = -1;
  double acc_z = -1;

  double gyro_x = -1;
  double gyro_y = -1;
  double gyro_z = -1;

  double mag_x = -1;
  double mag_y = -1;
  double mag_z = -1;

  double quat_x = -1;
  double quat_y = -1;
  double quat_z = -1;
  double quat_w = -1;

  double grav_x = -1;
  double grav_y = -1;
  double grav_z = -1;

  double lin_acc_x = -1;
  double lin_acc_y = -1;
  double lin_acc_z = -1;
};


class PayloadControl : public rclcpp::Node
{
public:
  PayloadControl()
  : Node("payload_control")
  {
    // Subscribers
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10, std::bind(&PayloadControl::imu_callback, this, std::placeholders::_1));

    mag_sub = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "imu/mag", 10, std::bind(&PayloadControl::mag_callback, this, std::placeholders::_1));

    quat_sub = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "imu/quat", 10, std::bind(&PayloadControl::quat_callback, this, std::placeholders::_1));

    acc_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "imu/acc", 10, std::bind(&PayloadControl::acc_callback, this, std::placeholders::_1));

    grav_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "imu/grav", 10, std::bind(&PayloadControl::grav_callback, this, std::placeholders::_1));

    // Timers
    timer_ =
      this->create_wall_timer(0.01s, std::bind(&PayloadControl::timer_callback, this));
  }

private:

  void timer_callback()
  {

    // log the IMU data struct
    RCLCPP_INFO(this->get_logger(), "IMU Data: \n"
      "acc_x: %f, acc_y: %f, acc_z: %f\n"
      "gyro_x: %f, gyro_y: %f, gyro_z: %f\n"
      "mag_x: %f, mag_y: %f, mag_z: %f\n"
      "quat_x: %f, quat_y: %f, quat_z: %f, quat_w: %f\n"
      "grav_x: %f, grav_y: %f, grav_z: %f\n"
      "lin_acc_x: %f, lin_acc_y: %f, lin_acc_z: %f\n",
      imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
      imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
      imu_data.mag_x, imu_data.mag_y, imu_data.mag_z,
      imu_data.quat_x, imu_data.quat_y, imu_data.quat_z, imu_data.quat_w,
      imu_data.grav_x, imu_data.grav_y, imu_data.grav_z,
      imu_data.lin_acc_x, imu_data.lin_acc_y, imu_data.lin_acc_z);

  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_data.gyro_x = msg->angular_velocity.x;
    imu_data.gyro_y = msg->angular_velocity.y;
    imu_data.gyro_z = msg->angular_velocity.z;

    imu_data.lin_acc_x = msg->linear_acceleration.x;
    imu_data.lin_acc_y = msg->linear_acceleration.y;
    imu_data.lin_acc_z = msg->linear_acceleration.z;
  }

  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    imu_data.mag_x = msg->magnetic_field.x;
    imu_data.mag_y = msg->magnetic_field.y;
    imu_data.mag_z = msg->magnetic_field.z;
  }

  void quat_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    imu_data.quat_x = msg->quaternion.x;
    imu_data.quat_y = msg->quaternion.y;
    imu_data.quat_z = msg->quaternion.z;
    imu_data.quat_w = msg->quaternion.w;
  }

  void acc_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    imu_data.acc_x = msg->point.x;
    imu_data.acc_y = msg->point.y;
    imu_data.acc_z = msg->point.z;
  }

  void grav_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    imu_data.grav_x = msg->point.x;
    imu_data.grav_y = msg->point.y;
    imu_data.grav_z = msg->point.z;
  }

  // Declare timers
  rclcpp::TimerBase::SharedPtr timer_;

  // Declare subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr quat_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr acc_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr grav_sub;

  // Variables
  ImuData imu_data;

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadControl>());
  rclcpp::shutdown();
  return 0;
}