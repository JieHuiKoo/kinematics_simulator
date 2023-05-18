#include <opencv2/opencv.hpp>
#include "PoseFrame.h"
#include <math.h> 

class Vehicle {
public:
    void DrawPosition(cv::Mat *map, float scale) {
        
        cv::Point pt1 = cv::Point(round(this->veh_pose_in_globalMap.x), round(this->veh_pose_in_globalMap.y-this->veh_width/2));
        cv::Point pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale), round(this->veh_pose_in_globalMap.y+this->veh_width/2*scale));

        cv::rectangle(*map,pt1,pt2,cv::Scalar(0, 255, 0));
    }
    
    void UpdatePosition() {

    }
    void SetMovementState() {

    }

    Vehicle() : veh_length(10), veh_width(10) { }
    Vehicle(float veh_length, float veh_width, PoseFrame veh_pose_in_globalMap) 
    {
        this->veh_length = veh_length; 
        this->veh_width = veh_width; 
        this->veh_pose_in_globalMap = veh_pose_in_globalMap;
    } 

private:
  //             <---------Length------->
  //
  //
  //                                    
  //             +++                  +++
  // /|\ (0,0) ->X=======================                          y
  //  |          -                      -                          ^
  //  |          -                      -                          |
  //  |          -X <-Veh Frame     D   -     ---------->          |-->x
  // Width       -                      -
  //  |          -                      -
  // \|/         =======================X <- (Length, Width)      D = Sensor facing right
  //             +++                  +++

  
  float veh_length;
  float veh_width;
  PoseFrame veh_pose_in_globalMap;

  // === Frames ===
  PoseFrame veh_poseframe;

  // Frames with respect to veh_frame
  PoseFrame TOFsensor1_frame;
  
  // TODO:
  float wheel_diam;

  PoseFrame leftFrontWheel_frame;
  PoseFrame rightFrontWheel_frame;
  PoseFrame leftBackWheel_frame;
  PoseFrame rightBackWheel_frame;
  };