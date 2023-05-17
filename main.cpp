// #include <opencv2/opencv.hpp>
// #include <stdio.h>
// using namespace cv;
// int main(int argc, char** argv)
// {
//     if (argc != 2) {
//         printf("usage: DisplayImage.out <Image_Path>\n");
//         return -1;
//     }
//     Mat image;
//     image = imread(argv[1], 1);
//     if (!image.data) {
//         printf("No image data \n");
//         return -1;
//     }
//     namedWindow("Display Image", WINDOW_AUTOSIZE);  
//     imshow("Display Image", image);
//     waitKey(0);
//     return 0;
// }
// Window.h
#include <SDL.h>
#include <iostream>
#include "include/Window.h"

int main() {
  Window AppWindow;
  auto KeyboardState = SDL_GetKeyboardState(nullptr);
  SDL_Event Event;
  while(true) {
    while (SDL_PollEvent(&Event)) {
      // System
      if (Event.type == SDL_QUIT) [[unlikely]] {
        SDL_Quit();
        return 0;
      }

      // Mouse Input
      else if (Event.type == SDL_MOUSEBUTTONDOWN) {
        if (Event.button.button == SDL_BUTTON_LEFT) {
          SDL_ShowCursor(SDL_ENABLE);
          AppWindow.GrabMouse();
        } else if (Event.button.button == SDL_BUTTON_RIGHT) {
          if (KeyboardState[SDL_SCANCODE_SPACE]) {
            std::cout << "Spacebar is pressed\n";
          } else {
            std::cout << "Spacebar is not pressed\n";
          }
        }
      } else if (Event.type == SDL_MOUSEBUTTONUP) {
        if (Event.button.button == SDL_BUTTON_LEFT) {
          SDL_ShowCursor(SDL_ENABLE);
          AppWindow.FreeMouse();
        }
      } else if (Event.type == SDL_MOUSEWHEEL) {
        AppWindow.ChangeWindowSize(Event.wheel.y * 3);
        AppWindow.SetTitle("Focused");
      } else if (Event.type == SDL_MOUSEMOTION) [[likely]] {
        AppWindow.SetBackgroundColor(
          Event.motion.x * 255 / AppWindow.GetWindowWidth(),
          Event.motion.y * 255 / AppWindow.GetWindowHeight(),
          0
        );
      } else if (Event.type == SDL_WINDOWEVENT) {
        if (
          Event.window.event == SDL_WINDOWEVENT_ENTER
        ) {
          AppWindow.SetTitle("Focused");
        } else if (
          Event.window.event == SDL_WINDOWEVENT_LEAVE
        ) {
          AppWindow.SetTitle("Unfocused");
        }
      }

      // Keyboard Input
      else if (Event.type == SDL_KEYDOWN) {
        if (Event.key.keysym.sym == SDLK_UP) {
          AppWindow.MoveRelative(0, -10);
        } else if (Event.key.keysym.sym == SDLK_DOWN) {
          AppWindow.MoveRelative(0, 10);
        } else if (Event.key.keysym.sym == SDLK_LEFT) {
          AppWindow.MoveRelative(-10, 0);
        } else if (Event.key.keysym.sym == SDLK_RIGHT) {
          AppWindow.MoveRelative(10, 0);
        } else if (Event.key.keysym.sym == SDLK_RETURN) {
          int x, y;
          Uint32 Buttons { SDL_GetMouseState(&x, &y) };

          std::cout << "Mouse is at " << x << ", " << y;
          if ((Buttons & SDL_BUTTON_LMASK)) {
            std::cout << " - Left Button is pressed";
          }
          if ((Buttons & SDL_BUTTON_RMASK)) {
            std::cout << " - Right Button is pressed";
          }
          std::cout << "\n";
        } else {
          std::cout
            << "Key Pressed! Key Code: "
            << Event.key.keysym.sym
            << ", Key Name: "
            << SDL_GetKeyName(Event.key.keysym.sym)
            << '\n';
        }
      }
    }
    AppWindow.Update();
    AppWindow.RenderFrame();
  }
}