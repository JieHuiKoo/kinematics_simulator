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
#include <SDL.h>
#include <iostream>

int main() {
  auto KeyboardState = SDL_GetKeyboardState(nullptr);
  SDL_Event Event;
  while(true) {
    while (SDL_PollEvent(&Event)) {
      // System
      if (Event.type == SDL_QUIT) [[unlikely]] {
        SDL_Quit();
        return 0;
      }

      // Keyboard Input
      else if (Event.type == SDL_KEYDOWN) {
        if (Event.key.keysym.sym == SDLK_UP) 
        {
          // Key Press Up
        } 
        
        else if (Event.key.keysym.sym == SDLK_DOWN) 
        {
          // Key Press Down
        } 
        
        else if (Event.key.keysym.sym == SDLK_LEFT) 
        {
          // Key Press Left
        } 
        
        else if (Event.key.keysym.sym == SDLK_RIGHT) 
        {
          // Key Press Right
        }
        else {
          std::cout
            << "Key Pressed! Key Code: "
            << Event.key.keysym.sym
            << ", Key Name: "
            << SDL_GetKeyName(Event.key.keysym.sym)
            << '\n';
        }
      }
    }
  }
}