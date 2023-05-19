#include <opencv2/opencv.hpp>
#include "PoseFrame.h"
#include <math.h> 

class Vehicle {
public:
    void DrawPosition(cv::Mat *map, float scale) {
        
        cv::Point pt1, pt2;

        // Draw Car Body
        pt1 = cv::Point(round(this->veh_pose_in_globalMap.x), -round(this->veh_pose_in_globalMap.y-this->veh_width/2*scale));
        pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale), -round(this->veh_pose_in_globalMap.y+this->veh_width/2*scale));
        cv::rectangle(*map,pt1,pt2,cv::Scalar(0, 255, 0), -1);

        // Draw Forward Back Indicators
        if (this->velocity_MetersPerSec == 0)
        {   
            // Forward Indicator (Dark)
            pt1 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.9), -round(this->veh_pose_in_globalMap.y-this->veh_width/2*scale));
            pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale), -round(this->veh_pose_in_globalMap.y+this->veh_width/2*scale));
            cv::rectangle(*map,pt1,pt2,cv::Scalar(100, 0, 0), -1);

            // Backward Indicator (Dark)
            pt1 = cv::Point(round(this->veh_pose_in_globalMap.x), -round(this->veh_pose_in_globalMap.y-this->veh_width/2*scale));
            pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.1), -round(this->veh_pose_in_globalMap.y+this->veh_width/2*scale));
            cv::rectangle(*map,pt1,pt2,cv::Scalar(0, 0, 100), -1);

        }
        else if (this->velocity_MetersPerSec > 0)
        {
            // Forward Indicator (Light)
            pt1 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.9), -round(this->veh_pose_in_globalMap.y-this->veh_width/2*scale));
            pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale), -round(this->veh_pose_in_globalMap.y+this->veh_width/2*scale));
            cv::rectangle(*map,pt1,pt2,cv::Scalar(255, 0, 0), -1);

            // Backward Indicator (Dark)
            pt1 = cv::Point(round(this->veh_pose_in_globalMap.x), -round(this->veh_pose_in_globalMap.y-this->veh_width/2*scale));
            pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.1), -round(this->veh_pose_in_globalMap.y+this->veh_width/2*scale));
            cv::rectangle(*map,pt1,pt2,cv::Scalar(0, 0, 100), -1);
        }
        else
        {
            // Forward Indicator (Dark)
            pt1 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.9), -round(this->veh_pose_in_globalMap.y-this->veh_width/2*scale));
            pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale), -round(this->veh_pose_in_globalMap.y+this->veh_width/2*scale));
            cv::rectangle(*map,pt1,pt2,cv::Scalar(100, 0, 0), -1);

            // Backward Indicator (Light)
            pt1 = cv::Point(round(this->veh_pose_in_globalMap.x), -round(this->veh_pose_in_globalMap.y-this->veh_width/2*scale));
            pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.1), -round(this->veh_pose_in_globalMap.y+this->veh_width/2*scale));
            cv::rectangle(*map,pt1,pt2,cv::Scalar(0, 0, 255), -1);
        }

        // Draw Turning indicators
        if (this->turn_amt_Radians > 0) // Left
        {
            pt1 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.8), -round(this->veh_pose_in_globalMap.y));
            pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.85), -round(this->veh_pose_in_globalMap.y+this->veh_width/2*scale));
            cv::rectangle(*map,pt1,pt2,cv::Scalar(0, 69, 255), -1);
        }
        else if (this->turn_amt_Radians < 0) // Right
        {
            pt1 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.8), -round(this->veh_pose_in_globalMap.y-this->veh_width/2*scale));
            pt2 = cv::Point(round(this->veh_pose_in_globalMap.x + this->veh_length*scale*0.85), -round(this->veh_pose_in_globalMap.y));
            cv::rectangle(*map,pt1,pt2,cv::Scalar(0, 69, 255), -1);
        }

    }
    
    void UpdatePosition() {

    }
    void UpdateMovementState(const std::vector<bool>& movement_state_vector) {
        
        // Update velocity
        if (movement_state_vector[0]) this->velocity_MetersPerSec = 1;
        else if (movement_state_vector[1]) this->velocity_MetersPerSec = -1;
        else this->velocity_MetersPerSec = 0;

        // Update turn_amt
        if (movement_state_vector[2]) this->turn_amt_Radians = 0.436332; //25deg
        else if (movement_state_vector[3]) this->turn_amt_Radians = -0.436332; //25deg
        else this->turn_amt_Radians = 0;
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
  //  |          X <-Veh Frame      D   -     ---------->          |-->x
  // Width       -                      -
  //  |          -                      -
  // \|/         =======================X <- (Length, Width)      D = Sensor facing right
  //             +++                  +++

  
    float veh_length;
    float veh_width;
    PoseFrame veh_pose_in_globalMap;

    float velocity_MetersPerSec;
    float turn_amt_Radians = -10; // Left +ve, right-ve

    // === Frames ===
    PoseFrame veh_poseframe;

    // Frames with respect to veh_frame
    PoseFrame TOFsensor1_frame;
  
    cv::Point ConvertMapToOpenCVPoint(cv::Point point_in_map) {
    cv::Point point_in_OpenCV;

    point_in_OpenCV.x = point_in_map.x;
    point_in_OpenCV.y = -point_in_map.y;

    return point_in_OpenCV;
    }

    // TODO:
    float wheel_diam;

    PoseFrame leftFrontWheel_frame;
    PoseFrame rightFrontWheel_frame;
    PoseFrame leftBackWheel_frame;
    PoseFrame rightBackWheel_frame;
  };