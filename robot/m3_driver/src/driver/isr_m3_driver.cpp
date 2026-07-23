#include "isr_m3_driver/isr_m3_driver.hpp"

void ISR_M3::set_pubs_subs() {
  cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&ISR_M3::get_cmd_vel, this, std::placeholders::_1));

  odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
      "pose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  // TF Broadcaster for odom->base_link
  odom_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // TF Buffer/Listener for map->base_link lookup
  base_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  base_tf_listener = std::make_shared<tf2_ros::TransformListener>(
      *base_tf_buffer, this, false);
}

void ISR_M3::get_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel.v = msg->linear.x;
  cmd_vel.w = msg->angular.z;
}

void ISR_M3::publish_odom_msg() {
  // Publish odometry message
  geometry_msgs::msg::Quaternion odom_quat = get_quaternion(position.theta);
  nav_msgs::msg::Odometry odom;

  odom.header.stamp = cur_encoder_time;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_frame;
  odom.pose.pose.position.x = position.x;
  odom.pose.pose.position.y = position.y;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance.fill(0);
  odom.twist.twist.linear.x = velocity.v;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = velocity.w;
  odom.twist.covariance.fill(0);

  odom_pub->publish(odom);

  // NOTE: TF publishing enabled for AMCL mode
  // When using lidar_localization, disable this to avoid conflict

  // Publish odom -> base_link TF
  // geometry_msgs::msg::TransformStamped odom_tf;
  // odom_tf.header.stamp = cur_encoder_time;
  // odom_tf.header.frame_id = odom_frame;
  // odom_tf.child_frame_id = base_frame;
  // odom_tf.transform.translation.x = position.x;
  // odom_tf.transform.translation.y = position.y;
  // odom_tf.transform.translation.z = 0.0;
  // odom_tf.transform.rotation = odom_quat;
  // odom_tf_broadcaster->sendTransform(odom_tf);
}

void ISR_M3::publish_pose_msg() {
  geometry_msgs::msg::TransformStamped map_to_base_tf;
  try {
    map_to_base_tf =
        base_tf_buffer->lookupTransform("map", base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header = map_to_base_tf.header;
  pose.pose.position.x = map_to_base_tf.transform.translation.x;
  pose.pose.position.y = map_to_base_tf.transform.translation.y;
  pose.pose.position.z = map_to_base_tf.transform.translation.z;
  pose.pose.orientation.x = map_to_base_tf.transform.rotation.x;
  pose.pose.orientation.y = map_to_base_tf.transform.rotation.y;
  pose.pose.orientation.z = map_to_base_tf.transform.rotation.z;
  pose.pose.orientation.w = map_to_base_tf.transform.rotation.w;

  pose_pub->publish(pose);
}

bool ISR_M3::send_data(uint8_t command, uint8_t numparam, uint8_t *params) {
  uint8_t CRC = numparam + 1;
  boost::array<uint8_t, 16> data_;

  data_[0] = 0x06;
  data_[1] = 0x85;
  data_[2] = numparam + 1; // command(1) + data.size(n)
  CRC ^= command;
  data_[3] = command;

  for (int i = 0; i < numparam; i++) {
    CRC ^= params[i];
    data_[4 + i] = params[i];
  }

  data_[4 + numparam] = CRC;

  size_t sentdata_size =
      boost::asio::write(serial, boost::asio::buffer(data_, 5 + numparam));

  if (sentdata_size == (size_t)0)
    return false;

  return true;
}

std::vector<uint8_t> ISR_M3::receive_data(uint8_t &command) {
  uint8_t start_count = 0;
  bool got_data = false;
  boost::array<uint8_t, 32> raw_bytes;
  std::vector<uint8_t> data;

  while (!got_data) {
    // Wait until first data sync of frame: 0x06, 0x85
    boost::asio::read(serial, boost::asio::buffer(&raw_bytes[start_count], 1));

    if (start_count == 0) {
      if (raw_bytes[start_count] == 0x06) // raw_bytes[0]
      {
        start_count = 1;
      }
    } else if (start_count == 1) {
      if (raw_bytes[start_count] == 0x85) // raw_bytes[1]
      {
        start_count = 2;
      } else {
        start_count = 0;
      }
    } else if (start_count == 2) {
      if (raw_bytes[start_count] !=
          0) // raw_bytes[2]: rx_len (command(1) + numdata(n))
      {
        uint8_t rx_len = raw_bytes[2];
        uint8_t numdata_ = rx_len - 1;

        // Now that entire start sequence has been found, read in the rest of
        // the message
        got_data = true;
        boost::asio::read(serial,
                          boost::asio::buffer(&raw_bytes[3], rx_len + 1));

        // seem to have got whole message
        // last uint8_t is CS
        uint8_t checkCS = rx_len;

        for (int i = 0; i < rx_len; i++) {
          checkCS ^= raw_bytes[3 + i];
        }

        if (checkCS == raw_bytes[3 + rx_len]) {
          // CS good
          command = raw_bytes[3];

          for (int i = 0; i < numdata_; i++) {
            data.push_back(raw_bytes[4 + i]);
          }

          return data;
        } else {
          // Failed to check checksum. Return empty vector.
          return std::vector<uint8_t>();
        }
      }

      start_count = 0;
    }
  }

  return std::vector<uint8_t>(); // Return empty vector.
}

int32_t ISR_M3::get_long(uint8_t *data) {
  int dat_1 = data[0] << 24;
  int dat_2 = data[1] << 16;
  int dat_3 = data[2] << 8;
  int dat_4 = data[3];
  int dat = dat_1 | dat_2 | dat_3 | dat_4;

  return dat;
}

geometry_msgs::msg::Quaternion ISR_M3::get_quaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void ISR_M3::set_params() {
  // Declare and acquire serial port parameter
  declare_parameter("port", "/dev/ttyACM0");
  declare_parameter("baudrate", 115200);
  declare_parameter("odom_frame", "odom");
  declare_parameter("base_frame", "base_link");

  get_parameter_or("port", port, std::string("/dev/ttyACM0"));
  get_parameter_or("baudrate", baudrate, uint32_t(115200));
  get_parameter_or("odom_frame", odom_frame, std::string("odom"));
  get_parameter_or("base_frame", base_frame, std::string("base_link"));

  std::cout << "Serial port: " << port << std::endl;
  std::cout << "Baudrate: " << baudrate << std::endl;
  std::cout << "Odom frame: " << odom_frame << std::endl;
  std::cout << "Base frame: " << base_frame << std::endl;
}

bool ISR_M3::connect_robot(const std::string &port, const uint32_t baudrate) {
  try {
    serial.open(port);
  } catch (boost::system::system_error &e) {
    std::cout << "Cannot open port " << port << std::endl;
    return false;
  }

  try {
    serial.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
  } catch (boost::system::system_error &e) {
    std::cout << "Unknown baudrate " << baudrate << std::endl;
    return false;
  }

  serial.set_option(boost::asio::serial_port_base::character_size(8));
  serial.set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none));
  serial.set_option(boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one));
  serial.set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "Serial port " << port << " opened at baudrate " << baudrate
            << std::endl;

  if (!initialize())
    return false;

  M3_timer = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&ISR_M3::M3_loop, this));
  return true;
}

bool ISR_M3::initialize() {
  if (!send_data(COMMAND_INITIALIZE, 0, NULL))
    return false;

  prev_encoder_time = cur_encoder_time = now();
  left_encoder = right_encoder = 0;
  del_dist_left_m = del_dist_right_m = 0.0;
  left_wheel_vel_endoer_mps = right_wheel_vel_encoder_mps = 0.0;
  position.x = position.y = position.theta = 0.0;

  return true;
}

bool ISR_M3::read_encoder() {
  if (!send_data(COMMAND_ENCODER_READ, 0, NULL))
    return false;

  uint8_t command;
  std::vector<uint8_t> data = receive_data(command);

  if (data.size() == 0)
    return false;

  if (command == COMMAND_ENCODER_READ_RE && data.size() == 8) {
    prev_encoder_time = cur_encoder_time;
    cur_encoder_time = get_clock()->now();

    long _leftEncoderPrev = left_encoder;
    long _rightEncoderPrev = right_encoder;

    std::vector<uint8_t>::iterator it;
    uint8_t byteToConvert[4];
    int position = 0;

    for (it = data.begin(); it < data.end() - 4; it++) {
      byteToConvert[position++] = *it;
    }

    left_encoder = get_long(byteToConvert) / 4;
    position = 0;

    for (it = data.begin() + 4; it < data.end(); it++) {
      byteToConvert[position++] = *it;
    }

    right_encoder = get_long(byteToConvert) / 4;

    left_encoder = -left_encoder;
    right_encoder = -right_encoder;

    // Calculates odometry when calls read_encoder func
    long dl = (left_encoder - _leftEncoderPrev);
    long dr = (right_encoder - _rightEncoderPrev);
    dead_reckoning(dl, dr);

    return true;
  } else {
    return false;
  }

  return false;
}

bool ISR_M3::set_velocity(double linearVel_MPS,
                          double angularVel_RPS) // m/s, rad/s
{
  double leftWheelVel_MPS =
      linearVel_MPS - (WHEEL_BASE_M * angularVel_RPS / 2.);
  int leftMotorVel_RPM = (int)(leftWheelVel_MPS * MPS2RPM * GEAR_RATIO);

  double rightWheelVel_MPS =
      linearVel_MPS + (WHEEL_BASE_M * angularVel_RPS / 2.);
  int rightMotorVel_RPM = (int)(rightWheelVel_MPS * MPS2RPM * GEAR_RATIO);

  // Exceed maximum motor rpm.
  if (leftMotorVel_RPM > MAX_RPM || rightMotorVel_RPM > MAX_RPM ||
      leftMotorVel_RPM < -MAX_RPM || rightMotorVel_RPM < -MAX_RPM)
    return true;

  uint8_t data[4];
  data[0] = leftMotorVel_RPM >> 8;
  data[1] = leftMotorVel_RPM;
  data[2] = rightMotorVel_RPM >> 8;
  data[3] = rightMotorVel_RPM;

  if (!send_data(COMMAND_MOTOR_RUN, 4, data))
    return false;

  return true;
}

void ISR_M3::dead_reckoning(long dl, long dr) {
  double r = 2.0 * M_PI * WHEEL_RADIUS_M / ENCODER_PPR / GEAR_RATIO;
  double del_dist_left_m = r * dl;
  double del_dist_right_m = r * dr;

  auto diff = (cur_encoder_time - prev_encoder_time)
                  .to_chrono<std::chrono::milliseconds>()
                  .count() *
              1e-3;

  left_wheel_vel_endoer_mps = del_dist_left_m / diff;    // m/s
  right_wheel_vel_encoder_mps = del_dist_right_m / diff; // m/s

  velocity.v = (left_wheel_vel_endoer_mps + right_wheel_vel_encoder_mps) / 2.0;
  velocity.w =
      (right_wheel_vel_encoder_mps - left_wheel_vel_endoer_mps) / WHEEL_BASE_M;

  const auto &v = velocity.v;
  const auto &w = velocity.w;

  if (-0.001 > w || w > 0.001) {
    position.x +=
        v / w * (sin(position.theta + w * diff) - sin(position.theta));
    position.y -=
        v / w * (cos(position.theta + w * diff) - cos(position.theta));
  } else {
    double ds = (del_dist_left_m + del_dist_right_m) / 2.0;
    position.x += ds * cos(position.theta);
    position.y += ds * sin(position.theta);
  }

  position.theta += w * diff;
}

ISR_M3::ISR_M3() : Node("isr_m3_driver_node"), serial(io) {
  set_params();
  set_pubs_subs();

  if (!connect_robot(port, baudrate)) {
    std::cout << "Failed to connect to robot." << std::endl;
    rclcpp::shutdown();
    return;
  }
}

void ISR_M3::M3_loop() {
  serialio_mut.lock();
  set_velocity(cmd_vel.v, cmd_vel.w);
  read_encoder();
  publish_odom_msg();
  publish_pose_msg();
  serialio_mut.unlock();
}
