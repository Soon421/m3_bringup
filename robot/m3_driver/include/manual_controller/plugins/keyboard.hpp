#pragma once

// C++ Standard Libraries
#include <memory>
#include <string>

// External Libraries
#include "SDL2/SDL.h"

class Keyboard {
public:
  // Initialization
  Keyboard();
  ~Keyboard();

  // Key processing functions
  void process_keyboard_event(SDL_Event &event, double &lin_vel,
                              double &ang_vel, double delta_v, double delta_w);

  // Enhanced event handling
  bool check_keyboard_events(SDL_Event &event, bool &help_requested,
                             bool &toggle_requested);
  bool poll_events(SDL_Event &event, bool &quit_requested, bool &help_requested,
                   bool &toggle_requested);

  // Utility functions
  bool initialize_sdl(SDL_Window **window, SDL_Renderer **renderer,
                      SDL_Surface **surface,
                      const char *title = "Manual Controller");
  void cleanup_sdl(SDL_Window **window, SDL_Renderer **renderer,
                   SDL_Surface **surface);
  void print_keyboard_help();

private:
  // SDL related members
  bool sdl_initialized = false;
};
