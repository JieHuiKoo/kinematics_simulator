#include <iostream>
#include "include/Window.h"
#include "include/Vehicle.h"
#include <boost/numeric/odeint.hpp> 
#include "include/Map.h"

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
  Map base_environment(cv::Size(1000, 1000));
  base_environment.AddObstacle({cv::Point(100, -100), cv::Point(900, -100)});
  base_environment.AddObstacle({cv::Point(900, -100), cv::Point(900, -900)});
  base_environment.AddObstacle({cv::Point(900, -900), cv::Point(100, -900)});
  base_environment.AddObstacle({cv::Point(100, -900), cv::Point(100, -100)});

  // Define a vehicle model
  PoseFrame CarModel_initial_pose (base_environment.map_size.width/2, -base_environment.map_size.height/2, 0);
  Vehicle CarModel(10, 3, CarModel_initial_pose, 20, 3, base_environment.map_size);

  // Define waypoints
  CarModel.AddPathWaypoints(cv::Point(500, -450), base_environment.map_size);
  CarModel.AddPathWaypoints(cv::Point(750, -450), base_environment.map_size);
  CarModel.AddPathWaypoints(cv::Point(800, -400), base_environment.map_size);
  CarModel.AddPathWaypoints(cv::Point(800, -250), base_environment.map_size);
  CarModel.AddPathWaypoints(cv::Point(750, -200), base_environment.map_size);
  CarModel.AddPathWaypoints(cv::Point(200, -200), base_environment.map_size);
  CarModel.AddPathWaypoints(cv::Point(200, -800), base_environment.map_size);
  CarModel.AddPathWaypoints(cv::Point(800, -800), base_environment.map_size);
  CarModel.AddPathWaypoints(cv::Point(800, -400), base_environment.map_size);




  auto KeyboardState = SDL_GetKeyboardState(nullptr);

  while(true) 
  {
    // Create car_drawing
    Map car_environment(cv::Size(1000, 1000));

    // Get the Key Presses and store in array
    std::vector<bool> movement_state_array = GetKeyStateArray(KeyboardState);

    CarModel.UpdateMovementState(movement_state_array);

    CarModel.CalculateSensorReading(base_environment.obstacles);
    CarModel.DrawSensorReading(&car_environment.map_layer);

    CarModel.CalculatePathPursuit();
    CarModel.AnnotatePathPursuit(&car_environment.map_layer);

    CarModel.UpdatePosition();
    CarModel.DrawPosition(&car_environment.map_layer);


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
    cv::bitwise_or(base_environment.map_layer, car_environment.map_layer, output);

    cv::imshow("Kinematics Simulator", output);
    cv::waitKey(1);
  }
}