#include "manual_controller/plugins/keyboard.hpp"
#include <iostream>

Keyboard::Keyboard() {
  // Constructor implementation
}

Keyboard::~Keyboard() {
  // Clean up if needed
}

bool Keyboard::initialize_sdl(SDL_Window **window, SDL_Renderer **renderer,
                              SDL_Surface **surface, const char *title) {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0) {
    std::cerr << "Could not initialize SDL: " << SDL_GetError() << std::endl;
    return false;
  }

  *window =
      SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                       100, 100, SDL_WINDOW_SHOWN);
  if (!*window) {
    std::cerr << "Could not create window: " << SDL_GetError() << std::endl;
    return false;
  }

  *renderer = SDL_CreateRenderer(*window, -1, 0);
  if (!*renderer) {
    std::cerr << "Could not create renderer: " << SDL_GetError() << std::endl;
    return false;
  }

  *surface = SDL_CreateRGBSurface(0, 100, 100, 32, 0, 0, 0, 0);
  if (!*surface) {
    std::cerr << "Could not create surface: " << SDL_GetError() << std::endl;
    return false;
  }

  sdl_initialized = true;
  return true;
}

void Keyboard::cleanup_sdl(SDL_Window **window, SDL_Renderer **renderer,
                           SDL_Surface **surface) {
  if (*surface) {
    SDL_FreeSurface(*surface);
    *surface = nullptr;
  }

  if (*renderer) {
    SDL_DestroyRenderer(*renderer);
    *renderer = nullptr;
  }

  if (*window) {
    SDL_DestroyWindow(*window);
    *window = nullptr;
  }
}

bool Keyboard::poll_events(SDL_Event &event, bool &quit_requested,
                           bool &help_requested, bool &toggle_requested) {
  quit_requested = false;
  help_requested = false;
  toggle_requested = false;

  if (!SDL_PollEvent(&event))
    return false;

  if (event.type == SDL_QUIT) {
    quit_requested = true;
    return true;
  }

  if (check_keyboard_events(event, help_requested, toggle_requested))
    return true;

  return true;
}

bool Keyboard::check_keyboard_events(SDL_Event &event, bool &help_requested,
                                     bool &toggle_requested) {
  if (event.type != SDL_KEYDOWN)
    return false;

  switch (event.key.keysym.sym) {
  case SDLK_x:
    toggle_requested = true;
    return true;

  case SDLK_b:
    help_requested = true;
    return true;

  default:
    return false;
  }
}

void Keyboard::process_keyboard_event(SDL_Event &event, double &lin_vel,
                                      double &ang_vel, double delta_v,
                                      double delta_w) {
  if (event.type != SDL_KEYDOWN)
    return;

  switch (event.key.keysym.sym) {
  case SDLK_UP:
    lin_vel += delta_v;
    std::cout << "키보드 속도 증가: " << lin_vel << " m/s" << std::endl;
    break;

  case SDLK_DOWN:
    lin_vel -= delta_v;
    std::cout << "키보드 속도 감소: " << lin_vel << " m/s" << std::endl;
    break;

  case SDLK_LEFT:
    ang_vel += delta_w;
    std::cout << "키보드 회전 증가: " << ang_vel << " rad/s" << std::endl;
    break;

  case SDLK_RIGHT:
    ang_vel -= delta_w;
    std::cout << "키보드 회전 감소: " << ang_vel << " rad/s" << std::endl;
    break;

  case SDLK_SPACE:
    lin_vel = 0.0;
    ang_vel = 0.0;
    std::cout << "키보드 정지" << std::endl;
    break;
  }
}

void Keyboard::print_keyboard_help() {
  std::cout << "\n==========================\n";
  std::cout << "키보드 컨트롤러 사용법\n";
  std::cout << "---------------------------\n";
  std::cout << "키보드 방향키(↑↓): linear 값 +/- 0.1\n";
  std::cout << "키보드 방향키(←→): angular 값 +/- 0.05\n";
  std::cout << "키보드 스페이스바: 정지\n";
  std::cout << "키보드 X 버튼: 키보드/조이스틱 전환\n";
  std::cout << "키보드 B 버튼: 도움말\n";
  std::cout << "==========================" << std::endl;
}