#pragma once

// C++ Standard Libraries
#include <memory>
#include <string>

// External Libraries
#include "SDL2/SDL.h"

class Joy {
public:
  // Initialization
  Joy();
  ~Joy();

  // Joystick management functions
  bool initialize_joystick(int device_index);
  void close_joystick();
  bool is_connected() const { return joystick_connected; }
  SDL_Joystick *get_joystick() { return joystick; }

  // Input processing
  void process_joystick_input(SDL_Joystick *joy, double &lin_vel,
                              double &ang_vel, int axis_linear,
                              int axis_angular, double scale_linear,
                              double scale_angular, double deadzone,
                              bool &move_allowed, int button_move);

  // Enhanced button handling
  bool check_button_state(SDL_Joystick *joy, int button_index,
                          bool &prev_state);
  bool check_toggle_button(SDL_Joystick *joy, int toggle_button,
                           bool &prev_state);
  bool check_help_button(SDL_Joystick *joy, int help_button, bool &prev_state);

  // Utility functions
  void print_joystick_help();
  void list_available_joysticks();

private:
  // Joystick related members
  SDL_Joystick *joystick = nullptr;
  bool joystick_connected = false;
};
