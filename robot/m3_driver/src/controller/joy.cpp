#include "manual_controller/plugins/joy.hpp"
#include <cmath>
#include <iostream>

Joy::Joy() : joystick(nullptr), joystick_connected(false) {
  // Constructor implementation
}

Joy::~Joy() { close_joystick(); }

bool Joy::initialize_joystick(int device_index) {
  SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
  SDL_JoystickEventState(SDL_ENABLE);
  // Ensure SDL has joystick subsystem initialized
  if (!(SDL_WasInit(SDL_INIT_JOYSTICK))) {
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) < 0) {
      std::cerr << "Could not initialize SDL Joystick subsystem: "
                << SDL_GetError() << std::endl;
      return false;
    }
  }

  // Check if any joysticks are available
  if (SDL_NumJoysticks() < 1) {
    std::cout << "No joysticks connected!" << std::endl;
    return false;
  }

  // List available joysticks
  list_available_joysticks();

  // Try to open joystick
  joystick = SDL_JoystickOpen(device_index);
  if (joystick == nullptr) {
    std::cerr << "Could not open joystick " << device_index << ": "
              << SDL_GetError() << std::endl;
    return false;
  }

  joystick_connected = true;

  std::cout << "Joystic Connected: " << SDL_JoystickName(joystick) << std::endl;
  std::cout << "Axes Number: " << SDL_JoystickNumAxes(joystick) << std::endl;
  std::cout << "Button Number: " << SDL_JoystickNumButtons(joystick)
            << std::endl;

  return true;
}

void Joy::close_joystick() {
  if (joystick != nullptr) {
    SDL_JoystickClose(joystick);
    joystick = nullptr;
    joystick_connected = false;
  }
}

void Joy::list_available_joysticks() {
  std::cout << "--- 연결된 조이스틱 장치 목록 ---" << std::endl;
  for (int i = 0; i < SDL_NumJoysticks(); i++) {
    const char *joystick_name = SDL_JoystickNameForIndex(i);
    std::cout << "장치 " << i << ": "
              << (joystick_name ? joystick_name : "알 수 없음") << std::endl;
  }
  std::cout << "-------------------------" << std::endl;
}

// New buttons state checking methods
bool Joy::check_button_state(SDL_Joystick *joy, int button_index,
                             bool &prev_state) {
  if (!joy || button_index < 0 || button_index >= SDL_JoystickNumButtons(joy))
    return false;

  bool current_state = SDL_JoystickGetButton(joy, button_index);
  bool button_pressed = current_state && !prev_state;
  prev_state = current_state;

  return button_pressed;
}

bool Joy::check_toggle_button(SDL_Joystick *joy, int toggle_button,
                              bool &prev_state) {
  return check_button_state(joy, toggle_button, prev_state);
}

bool Joy::check_help_button(SDL_Joystick *joy, int help_button,
                            bool &prev_state) {
  return check_button_state(joy, help_button, prev_state);
}

void Joy::process_joystick_input(SDL_Joystick *joy, double &lin_vel,
                                 double &ang_vel, int axis_linear,
                                 int axis_angular, double scale_linear,
                                 double scale_angular, double deadzone,
                                 bool &move_allowed, int button_move) {
  if (!joy)
    return;

  SDL_JoystickUpdate();

  bool changed = false;

  // Process linear axis if available
  if (SDL_JoystickNumAxes(joy) > axis_linear && move_allowed) {
    float raw_value = SDL_JoystickGetAxis(joy, axis_linear) / 32767.0f;
    raw_value = -raw_value; // Invert if needed

    // Apply deadzone
    if (std::abs(raw_value) < deadzone) {
      raw_value = 0.0f;
    }

    double new_lin_vel = raw_value * scale_linear;
    if (std::abs(new_lin_vel - lin_vel) > 0.01) {
      lin_vel = new_lin_vel;
      changed = true;
      std::cout << "조이스틱 선형 속도: " << lin_vel << " m/s" << std::endl;
    }
  }

  // Process angular axis if available
  if (SDL_JoystickNumAxes(joy) > axis_angular && move_allowed) {
    float raw_value = SDL_JoystickGetAxis(joy, axis_angular) / 32767.0f;

    // Apply deadzone
    if (std::abs(raw_value) < deadzone) {
      raw_value = 0.0f;
    }

    double new_ang_vel = -raw_value * scale_angular;
    if (std::abs(new_ang_vel - ang_vel) > 0.01) {
      ang_vel = new_ang_vel;
      changed = true;
      std::cout << "조이스틱 각속도: " << ang_vel << " rad/s" << std::endl;
    }
  }

  // Check if move button is pressed
  bool old_move_allowed = move_allowed;
  if (SDL_JoystickNumButtons(joy) > button_move) {
    move_allowed = SDL_JoystickGetButton(joy, button_move);

    if (old_move_allowed && !move_allowed) {
      std::cout << "이동 불가, 조이스틱 버튼 A를 누르면서 로봇을 이동하세요"
                << std::endl;
      lin_vel = 0.0;
      ang_vel = 0.0;
      changed = true;
    }
  } else {
    move_allowed = true;
  }
}

void Joy::print_joystick_help() {
  std::cout << "\n==========================\n";
  std::cout << "조이스틱 컨트롤러 사용법\n";
  std::cout << "---------------------------\n";
  std::cout << "조이스틱 A 버튼: 조이스틱 이동 허용\n";
  std::cout << "조이스틱 왼쪽 아날로그 스틱(↑↓): linear 값 +/-\n";
  std::cout << "조이스틱 왼쪽 아날로그 스틱(←→): angular 값 +/-\n";
  std::cout << "조이스틱 X 버튼: 키보드/조이스틱 전환\n";
  std::cout << "조이스틱 B 버튼: 자율주행/수동주행 전환 (cmd_vel publish 여부)\n";
  std::cout << "조이스틱 Y 버튼: 비상정지 토글\n";
  std::cout << "조이스틱 L 버튼: 도움말\n";
  std::cout << "조이스틱 R 버튼: 현재 상태 출력\n";
  std::cout << "!!조이스틱 A 버튼을 누르면서 로봇을 이동하세요!!\n";
  std::cout << "==========================" << std::endl;
}