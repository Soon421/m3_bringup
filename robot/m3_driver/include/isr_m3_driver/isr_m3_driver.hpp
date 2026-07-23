#pragma once

// C++ Standard Libraries
#include "mutex"

// ROS2 Libraries
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// External Libraries
#include "boost/array.hpp"
#include "boost/asio.hpp"

// Macro Constants
#define COMMAND_INITIALIZE 0
#define COMMAND_MOTOR_ENABLE 1
#define COMMAND_MOTOR_RUN 2
#define COMMAND_MOTOR_STOP 3
#define COMMAND_MOTOR_ACTUAL_SPEED_READ 4
#define COMMAND_MOTOR_ACTUAL_SPEED_READ_RE 5 // for return message
#define COMMAND_ENCODER_READ 11
#define COMMAND_ENCODER_READ_RE 12 // for return message
#define COMMAND_ENCODER_RESET 13
#define COMMAND_STATUS 21
#define COMMAND_STATUS_RE 22 // for return message
#define WHEEL_RADIUS_M 0.12 // Unit:m
#define WHEEL_BASE_M 0.43   // Unit:m
#define WHEEL_WIDTH_M 0.07  // Unit:m
#define ENCODER_PPR 6400.0   // Unit: pulse/rev
#define GEAR_RATIO                                                             \
  31.778 // Gearhead reduction ratio: 26 (26:1), Spurgear reduction ratio: 1.22
         // (44:36)
#define MPS2RPM 61.608 // same as (60 /(2 * M_PI * WHEEL_RADIUS_M))
#define MAX_RPM 4650.0

class ISR_M3 : public rclcpp::Node {
public:
  struct Position {
    double x;
    double y;
    double theta;
  } position; // Robot pose calculated by dead-reckoning (Unit: m, m, rad)

  struct Velocity {
    double v;
    double w;
  };

  // ROS2 Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

  // ROS2 Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

  // ROS2 TF Broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster;

  // ROS2 TF Buffer/Listener (for map->base_link lookup)
  std::shared_ptr<tf2_ros::Buffer> base_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> base_tf_listener;

  // ROS2 Timer
  rclcpp::TimerBase::SharedPtr M3_timer;

  // Character Variables
  std::string port;
  std::string odom_frame;
  std::string base_frame;

  // Numeric Variables
  long left_encoder;       // A encoder value of left wheel (Unit: pulse)
  long right_encoder;      // A encoder value of right wheel (Unit: pulse)
  double del_dist_left_m;  // left_encoder - prev_leftEncoder
  double del_dist_right_m; // right_encoder - prev_rightEncoder
  double left_wheel_vel_endoer_mps;
  double right_wheel_vel_encoder_mps;
  uint32_t baudrate;

  // Mutex
  std::mutex serialio_mut;

  // ROS2 Variables
  rclcpp::Time prev_encoder_time; // ms
  rclcpp::Time cur_encoder_time;  // ms

  // External Library Variables
  boost::asio::io_service io;
  boost::asio::serial_port serial;

  // Custom Variables
  Velocity velocity; // Robot velocity calculated by dead-reckoning (Unit: m/s,
                     // rad/s)
  Velocity cmd_vel;  // Robot velocity from cmd_vel (Unit: m/s, rad/s)

  // ROS2 Publisher/Subscriber Functions
  void set_pubs_subs();
  void get_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publish_odom_msg();
  void publish_pose_msg();

  // Helper Functions
  bool send_data(uint8_t command, uint8_t numparam, uint8_t *params);
  std::vector<uint8_t> receive_data(uint8_t &command);
  int32_t get_long(uint8_t *data);
  geometry_msgs::msg::Quaternion get_quaternion(double yaw);

  // Initialization Functions
  void set_params();
  bool connect_robot(const std::string &port, const uint32_t baudrate);
  bool initialize();

  // Update Functions
  bool read_encoder();
  bool set_velocity(double linearVel_MPS, double angularVel_RPS); // m/s, rad/s
  void dead_reckoning(long dl, long dr);

  // Constructors
  ISR_M3();

  // Loops
  void M3_loop();
};
