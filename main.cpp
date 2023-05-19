#include <iostream>
#include "include/Window.h"
#include "include/Vehicle.h"
#include <boost/numeric/odeint.hpp> 

std::vector<bool> GetKeyStateArray(const unsigned char* KeyboardState)
{
  // An array that stores the bool state of (1) UP, (2) Down, (3) Left, (4) Right keys
  std::vector<bool> KeyStateArray(4, false);

  bool pressed = false;

  // If Up key pressed
  if (!(KeyboardState[SDL_SCANCODE_UP] && KeyboardState[SDL_SCANCODE_DOWN]))
  {
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

  // Declare Map
  cv::Size map_size = cv::Size (1000, 1000);
  cv::Mat map = cv::Mat::zeros(cv::Size (1000, 1000), CV_8UC3); // Declare map of size 1000x1000 pixels
  // |============== MAP ================|  /|\ +ve y, 90 deg       
  // | X(0,0)                            |   |         
  // |                                   | 
  // |                                   | 
  // |                                   |    
  // |                                   |   
  // |                                   |      
  // |===================================|          
  // |--> +ve x, 0deg                           
  // | Note: Opencv Y axis is inverted

  // Declare Walls of map
  std::vector<std::array <cv::Point, 2>> obstacles;
  obstacles.push_back({cv::Point(100, -100), cv::Point(900, -100)});
  obstacles.push_back({cv::Point(900, -100), cv::Point(900, -900)});
  obstacles.push_back({cv::Point(900, -900), cv::Point(100, -900)});
  obstacles.push_back({cv::Point(100, -900), cv::Point(100, -100)});

  // Define a vehicle model
  PoseFrame CarModel_initial_pose (map.cols/2, -map.rows/2, 0);
  Vehicle CarModel(10, 3, CarModel_initial_pose, 20);

  auto KeyboardState = SDL_GetKeyboardState(nullptr);

  while(true) 
  {
    // Create car_drawing
    cv::Mat car_drawing = cv::Mat::zeros(map_size, CV_8UC3);

    // Get the Key Presses and store in array
    std::vector<bool> movement_state_array = GetKeyStateArray(KeyboardState);

    CarModel.UpdateMovementState(movement_state_array);
    CarModel.UpdatePosition();
    CarModel.DrawPosition(&car_drawing);
    CarModel.AnnotateSensorReading(obstacles, &car_drawing, &map);


    while (SDL_PollEvent(&Event)) {
      // System
      if (Event.type == SDL_QUIT) [[unlikely]] {
        SDL_Quit();
        return 0;
      }
    }
    AppWindow.Update();
    AppWindow.RenderFrame();

    //Overlay car onto map
    cv::Mat output;
    cv::bitwise_or(map, car_drawing, output);

    cv::imshow("Kinematics Simulator", output);
    cv::waitKey(1);
  }
}