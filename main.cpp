// #include <opencv2/opencv.hpp>
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
#include <iostream>
#include "include/Window.h"
#include "include/Vehicle.h"

std::vector<bool> GetKeyStateArray(const unsigned char* KeyboardState)
{
  // An array that stores the bool state of (1) UP, (2) Down, (3) Left, (4) Right keys
  std::vector<bool> KeyStateArray(4, false);

  bool pressed = false;

  // If Up key pressed
  if (KeyboardState[SDL_SCANCODE_UP])
  {
    KeyStateArray[0] = true;
    pressed = true;
    
    std::cout<<"Up";
  }
  
  // If Down key pressed
  if (KeyboardState[SDL_SCANCODE_DOWN])
  {
    KeyStateArray[1] = true;
    pressed = true;

    std::cout<< "Down";
  }
  
  // If Left/Right key pressed
  if (!(KeyboardState[SDL_SCANCODE_LEFT] && KeyboardState[SDL_SCANCODE_RIGHT]))
  {
    if (KeyboardState[SDL_SCANCODE_LEFT])
    {
      KeyStateArray[2] = true;
      pressed = true;

      std::cout<< "Left";  
    }
    if (KeyboardState[SDL_SCANCODE_RIGHT])
    {
      KeyStateArray[3] = true;
      pressed = true;

      std::cout<< "Right";
    }
  }

  if (pressed) std::cout << "\n === \n";
  
  return KeyStateArray;
}

int main() {
  Window AppWindow;
  SDL_Event Event;
  cv::namedWindow("Kinematics Simulator", cv::WINDOW_AUTOSIZE);

  // ====== Declare Variables =====

  // Declare time step on each CPU while loop tick
  double time_step = 0.0001; // Time step

  // Declare Map
  cv::Mat map = cv::Mat::zeros(cv::Size (1000, 1000), CV_8UC3); // Declare map of size 1000x1000 pixels
  // |============== MAP ================|  /|\ +ve y       
  // | X(0,0)                            |   |         
  // |                                   | 
  // |                                   | 
  // |                                   |    
  // |                                   |   
  // |                                   |      
  // |===================================|               /|\
  // |--> +ve x                             ----> 0 deg   |  90deg
  // | Note: Opencv Y axis is inverted

  // Annotate the centre of the map
  cv::circle(map, cv::Point(round(map.cols/2), round(map.rows/2)), 4, cv::Scalar(255, 255, 255), 2);

  // Define a vehicle model
  PoseFrame car_model_intiial_pose (map.cols/2, -map.rows/2, 0);
  Vehicle car_model(10, 3, car_model_intiial_pose);

  auto KeyboardState = SDL_GetKeyboardState(nullptr);

  while(true) {

    // Get the Key Presses and store in array
    GetKeyStateArray(KeyboardState);
    car_model.DrawPosition(&map, 20);

    while (SDL_PollEvent(&Event)) {
      // System
      if (Event.type == SDL_QUIT) [[unlikely]] {
        SDL_Quit();
        return 0;
      }
    }
    AppWindow.Update();
    AppWindow.RenderFrame();
    cv::imshow("Kinematics Simulator", map);
    cv::waitKey(1);
  }
}