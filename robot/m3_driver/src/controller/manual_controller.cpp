#include "manual_controller/manual_controller.hpp"

ManualController::ManualController()
    : Node("manual_controller"), lin_vel(0.0), ang_vel(0.0) {
  // 초기화 순서: 파라미터 → 플러그인 → 통신
  set_params();
  initialize_sdl();
  initialize_input_devices();
  set_pubs_subs();
  print_help();
}

ManualController::~ManualController() {
  // 플러그인 관련 리소스는 각 플러그인에서 처리
  // SDL 관련 정리 작업은 keyboard 플러그인으로 이동
  if (keyboard_plugin) {
    keyboard_plugin->cleanup_sdl(&window, &renderer, &surface);
  }

  // 조이스틱 관련 정리는 joy 플러그인에서 처리

  SDL_Quit();
}

void ManualController::set_pubs_subs() {
  cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  emergency_stop_pub = create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);

  controller_timer =
      create_wall_timer(std::chrono::milliseconds(20),
                        std::bind(&ManualController::controller_loop, this));
}

void ManualController::publish_cmd_vel() { cmd_vel_pub->publish(cmd_vel); }

void ManualController::set_params() {
  // 파라미터 선언
  declare_parameter("delta_v", 0.1);
  declare_parameter("delta_w", 0.05);
  declare_parameter("joy_axis_linear", 1);
  declare_parameter("joy_axis_angular", 0);
  declare_parameter("joy_button_move", 0);
  declare_parameter("joy_button_help", 6);
  declare_parameter("joy_button_toggle", 3);
  declare_parameter("joy_button_emergency_stop", 4);
  declare_parameter("joy_button_autonomous_toggle", 1);
  declare_parameter("joy_button_status_report", 7);
  declare_parameter("joy_scale_linear", 0.7);
  declare_parameter("joy_scale_angular", 0.7);
  declare_parameter("joy_deadzone", 0.05);
  declare_parameter("joy_device_index", 2);

  // 파라미터 로드
  get_parameter_or("delta_v", delta_v, 0.1);
  get_parameter_or("delta_w", delta_w, 0.05);
  get_parameter_or("joy_axis_linear", joy_axis_linear, 1);
  get_parameter_or("joy_axis_angular", joy_axis_angular, 0);
  get_parameter_or("joy_button_move", joy_button_move, 0);
  get_parameter_or("joy_button_help", joy_button_help, 6);
  get_parameter_or("joy_button_toggle", joy_button_toggle, 3);
  get_parameter_or("joy_button_emergency_stop", joy_button_emergency_stop, 4);
  get_parameter_or("joy_button_autonomous_toggle", joy_button_autonomous_toggle, 1);
  get_parameter_or("joy_button_status_report", joy_button_status_report, 7);
  get_parameter_or("joy_scale_linear", joy_scale_linear, 0.7);
  get_parameter_or("joy_scale_angular", joy_scale_angular, 0.7);
  get_parameter_or("joy_deadzone", joy_deadzone, 0.05);
  get_parameter_or("joy_device_index", joy_device_index, 2);
}

void ManualController::initialize_sdl() {
  // Keyboard 플러그인에 SDL 초기화 위임
  keyboard_plugin = std::make_unique<Keyboard>();
  if (!keyboard_plugin->initialize_sdl(&window, &renderer, &surface,
                                       "Manual Controller")) {
    RCLCPP_ERROR(get_logger(), "SDL initialization failed");
    throw std::runtime_error("SDL initialization failed");
  }
}

void ManualController::initialize_input_devices() {
  // 조이스틱 플러그인 초기화
  joy_plugin = std::make_unique<Joy>();

  // 조이스틱 초기화 시도
  if (joy_plugin->initialize_joystick(joy_device_index)) {
    RCLCPP_INFO(get_logger(), "Joystick initialized successfully");
    joystick = joy_plugin->get_joystick();
  } else {
    RCLCPP_WARN(get_logger(),
                "Joystick initialization failed, using keyboard only");
    active_input_source = KEYBOARD;
  }
}

void ManualController::process_inputs() {
  SDL_Event event;
  bool quit_requested = false;
  bool help_requested = false;
  bool toggle_requested = false;

  // 조이스틱 버튼 상태 확인 (도움말, 토글, 비상정지)
  if (joystick) {
    if (joy_plugin->check_help_button(joystick, joy_button_help,
                                      prev_help_button_state)) {
      print_help();
    }

    if (joy_plugin->check_toggle_button(joystick, joy_button_toggle,
                                        prev_toggle_button_state)) {
      handle_input_source_toggle(true, false);
    }

    if (joy_plugin->check_button_state(joystick, joy_button_emergency_stop,
                                      prev_emergency_button_state)) {
      emergency_stop_active = !emergency_stop_active;

      std_msgs::msg::Bool stop_msg;
      stop_msg.data = emergency_stop_active;
      emergency_stop_pub->publish(stop_msg);

      if (emergency_stop_active) {
        RCLCPP_WARN(get_logger(), "비상정지 버튼(Y) 누름: 정지 상태 진입");
        active_input_source = JOYSTICK;
        autonomous_mode = false;
      } else {
        RCLCPP_INFO(get_logger(), "비상정지(Y) 해제됨 (입력 소스: 조이스틱)");
        active_input_source = JOYSTICK;
      }

      lin_vel = 0.0;
      ang_vel = 0.0;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }

    if (joy_plugin->check_button_state(joystick, joy_button_autonomous_toggle,
                                   prev_autonomous_button_state)) {
      autonomous_mode = !autonomous_mode;

      if (autonomous_mode) {
        RCLCPP_INFO(get_logger(), "자율주행(B) 모드로 전환됨: cmd_vel 송신 중지");
      } else {
        RCLCPP_INFO(get_logger(), "수동제어(B) 모드로 전환됨: cmd_vel 송신 재개");
        active_input_source = JOYSTICK;
      }

      lin_vel = 0.0;
      ang_vel = 0.0;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }
    if (joy_plugin->check_button_state(joystick, joy_button_status_report,
                                   prev_status_report_button_state)) {
      RCLCPP_INFO(get_logger(), "===== 현재 상태 요약 =====");
      RCLCPP_INFO(get_logger(), "비상정지 상태: %s", emergency_stop_active ? "ON" : "OFF");
      RCLCPP_INFO(get_logger(), "주행 모드: %s", autonomous_mode ? "자율주행" : "수동주행");
      RCLCPP_INFO(get_logger(), "입력 소스: %s", active_input_source == JOYSTICK ? "조이스틱" : "키보드");
      RCLCPP_INFO(get_logger(), "=====================================");
    }
  }

  // 키보드 이벤트 처리
  while (keyboard_plugin->poll_events(event, quit_requested, help_requested,
                                      toggle_requested)) {
    if (quit_requested) {
      rclcpp::shutdown();
      return;
    }

    if (help_requested) {
      print_help();
    }

    if (toggle_requested) {
      handle_input_source_toggle(false, true);
    }

    // 키보드 모드일 때 키보드 입력 처리
    if (active_input_source == KEYBOARD && event.type == SDL_KEYDOWN) {
      keyboard_plugin->process_keyboard_event(event, lin_vel, ang_vel, delta_v,
                                              delta_w);
      cmd_vel.linear.x = lin_vel;
      cmd_vel.angular.z = ang_vel;
      last_input_source = KEYBOARD;
    }
  }

  // 조이스틱 모드일 때 조이스틱 입력 처리
  if (joystick && active_input_source == JOYSTICK && !emergency_stop_active && !autonomous_mode) {
    joy_plugin->process_joystick_input(
        joystick, lin_vel, ang_vel, joy_axis_linear, joy_axis_angular,
        joy_scale_linear, joy_scale_angular, joy_deadzone, joy_move_allowed,
        joy_button_move);
    cmd_vel.linear.x = lin_vel;
    cmd_vel.angular.z = ang_vel;
    last_input_source = JOYSTICK;
  } else if (!joy_move_allowed && active_input_source == JOYSTICK) {
    // 조이스틱 모드지만 이동 버튼이 눌리지 않은 경우
    lin_vel = 0.0;
    ang_vel = 0.0;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }
}

void ManualController::handle_input_source_toggle(bool joy_toggle_pressed,
                                                  bool key_x_pressed) {
  if (joy_toggle_pressed || key_x_pressed) {
    if (active_input_source == KEYBOARD) {
      if (joystick) {
        active_input_source = JOYSTICK;
        RCLCPP_INFO(get_logger(), "입력 소스 전환: 키보드 -> 조이스틱");
      } else {
        RCLCPP_WARN(get_logger(), "조이스틱이 연결되어 있지 않습니다.");
      }
    } else {
      active_input_source = KEYBOARD;
      RCLCPP_INFO(get_logger(), "입력 소스 전환: 조이스틱 -> 키보드");
    }
  }
}

void ManualController::print_help() {
  RCLCPP_INFO(get_logger(), "\n==========================");
  RCLCPP_INFO(get_logger(), "매뉴얼 컨트롤러 시작됨");
  RCLCPP_INFO(get_logger(), "---------------------------");

  // 각 플러그인의 도움말 출력 기능 활용
  keyboard_plugin->print_keyboard_help();

  if (joystick) {
    joy_plugin->print_joystick_help();
  }
}

void ManualController::controller_loop() {
  process_inputs();
  
  std_msgs::msg::Bool stop_msg;
  if (emergency_stop_active) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    stop_msg.data = true;
  } else {
    stop_msg.data = false;
  }
  emergency_stop_pub->publish(stop_msg);

  if (!autonomous_mode) {
    publish_cmd_vel();
  } 
}
