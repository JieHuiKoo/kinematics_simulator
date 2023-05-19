#include <opencv2/opencv.hpp>
#include "PoseFrame.h"
#include <math.h> 

class Vehicle {
public:
    void DrawPosition(cv::Mat *map) {
        
        PoseFrame vehCentre_in_globalMap1 = TransformFromVehFrameToGlobalMapFrame(this->centreIndicator_in_vehFrame);
        PoseFrame vehLeftIndicator_in_globalMap1 = TransformFromVehFrameToGlobalMapFrame(this->leftIndicator_in_vehFrame);
        PoseFrame vehRightIndicator_in_globalMap1 = TransformFromVehFrameToGlobalMapFrame(this->rightIndicator_in_vehFrame);
        PoseFrame vehForwardIndicator_in_globalMap1 = TransformFromVehFrameToGlobalMapFrame(this->forwardIndicator_in_vehFrame);
        PoseFrame vehBackwardIndicator_in_globalMap1 = TransformFromVehFrameToGlobalMapFrame(this->backwardIndicator_in_vehFrame);
        PoseFrame TOFsensor1_in_globalMap1 = TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame);

        cv::Point veh_position_in_OpenCVMap1 = ConvertMapToOpenCVPoint(cv::Point(this->veh_pose_in_globalMap.x, this->veh_pose_in_globalMap.y));
        cv::Point vehCentreIndicator_in_OpenCVMap1 = ConvertMapToOpenCVPoint(cv::Point(vehCentre_in_globalMap1.x, vehCentre_in_globalMap1.y));
        cv::Point vehLeftIndicator_in_OpenCVMap1 = ConvertMapToOpenCVPoint(cv::Point(vehLeftIndicator_in_globalMap1.x, vehLeftIndicator_in_globalMap1.y));
        cv::Point vehRightIndicator_in_OpenCVMap1 = ConvertMapToOpenCVPoint(cv::Point(vehRightIndicator_in_globalMap1.x, vehRightIndicator_in_globalMap1.y));
        cv::Point vehForwardIndicator_in_OpenCVMap1 = ConvertMapToOpenCVPoint(cv::Point(vehForwardIndicator_in_globalMap1.x, vehForwardIndicator_in_globalMap1.y));
        cv::Point vehBackwardIndicator_in_OpenCVMap1 = ConvertMapToOpenCVPoint(cv::Point(vehBackwardIndicator_in_globalMap1.x, vehBackwardIndicator_in_globalMap1.y));
        cv::Point TOFsensor1_in_OpenCVMap1 = ConvertMapToOpenCVPoint(cv::Point(TOFsensor1_in_globalMap1.x, TOFsensor1_in_globalMap1.y));
        
        cv::circle(*map, veh_position_in_OpenCVMap1, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehCentreIndicator_in_OpenCVMap1, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehLeftIndicator_in_OpenCVMap1, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehRightIndicator_in_OpenCVMap1, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehForwardIndicator_in_OpenCVMap1, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehBackwardIndicator_in_OpenCVMap1, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, TOFsensor1_in_OpenCVMap1, 4, cv::Scalar(255, 255, 255), -1);

        // PoseFrame vehCentre_in_globalMap = TransformFromVehFrameToGlobalMapFrame1(this->centreIndicator_in_vehFrame);
        // PoseFrame vehLeftIndicator_in_globalMap = TransformFromVehFrameToGlobalMapFrame1(this->leftIndicator_in_vehFrame);
        // PoseFrame vehRightIndicator_in_globalMap = TransformFromVehFrameToGlobalMapFrame1(this->rightIndicator_in_vehFrame);
        // PoseFrame vehForwardIndicator_in_globalMap = TransformFromVehFrameToGlobalMapFrame1(this->forwardIndicator_in_vehFrame);
        // PoseFrame vehBackwardIndicator_in_globalMap = TransformFromVehFrameToGlobalMapFrame1(this->backwardIndicator_in_vehFrame);
        // PoseFrame TOFsensor1_in_globalMap = TransformFromVehFrameToGlobalMapFrame1(this->TOFsensor1_in_vehFrame);

        // cv::Point veh_position_in_OpenCVMap = ConvertMapToOpenCVPoint(cv::Point(this->veh_pose_in_globalMap.x, this->veh_pose_in_globalMap.y));
        // cv::Point vehCentreIndicator_in_OpenCVMap = ConvertMapToOpenCVPoint(cv::Point(vehCentre_in_globalMap.x, vehCentre_in_globalMap.y));
        // cv::Point vehLeftIndicator_in_OpenCVMap = ConvertMapToOpenCVPoint(cv::Point(vehLeftIndicator_in_globalMap.x, vehLeftIndicator_in_globalMap.y));
        // cv::Point vehRightIndicator_in_OpenCVMap = ConvertMapToOpenCVPoint(cv::Point(vehRightIndicator_in_globalMap.x, vehRightIndicator_in_globalMap.y));
        // cv::Point vehForwardIndicator_in_OpenCVMap = ConvertMapToOpenCVPoint(cv::Point(vehForwardIndicator_in_globalMap.x, vehForwardIndicator_in_globalMap.y));
        // cv::Point vehBackwardIndicator_in_OpenCVMap = ConvertMapToOpenCVPoint(cv::Point(vehBackwardIndicator_in_globalMap.x, vehBackwardIndicator_in_globalMap.y));
        // cv::Point TOFsensor1_in_OpenCVMap = ConvertMapToOpenCVPoint(cv::Point(TOFsensor1_in_globalMap.x, TOFsensor1_in_globalMap.y));
        
        // cv::circle(*map, veh_position_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        // cv::circle(*map, vehCentreIndicator_in_OpenCVMap, 4, cv::Scalar(0, 255, 255), -1);
        // cv::circle(*map, vehLeftIndicator_in_OpenCVMap, 4, cv::Scalar(0, 0, 255), -1);
        // cv::circle(*map, vehRightIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        // cv::circle(*map, vehForwardIndicator_in_OpenCVMap, 4, cv::Scalar(0, 255, 255), -1);
        // cv::circle(*map, vehBackwardIndicator_in_OpenCVMap, 4, cv::Scalar(0, 255, 255), -1);
        // cv::circle(*map, TOFsensor1_in_OpenCVMap, 4, cv::Scalar(255, 0, 0), -1);

    }
    
    void UpdatePosition(double scale) {
        
        
    }
    void UpdateMovementState(const std::vector<bool>& movement_state_vector) {
        
        // Update velocity
        if (movement_state_vector[0]) this->velocity_MetersPerSec = 1;
        else if (movement_state_vector[1]) this->velocity_MetersPerSec = -1;
        else this->velocity_MetersPerSec = 0;

        // Update turn_amt
        if (movement_state_vector[2]) this->turn_amt_Radians = 0.785398; //45deg
        else if (movement_state_vector[3]) this->turn_amt_Radians = -0.785398; //45deg
        else this->turn_amt_Radians = 0;
    }

    Vehicle() : veh_length(10), veh_width(10) { }
    Vehicle(double veh_length, double veh_width, PoseFrame veh_pose_in_globalMap, double scale) 
    {
        this->veh_length = veh_length*scale; 
        this->veh_width = veh_width*scale; 
        this->veh_pose_in_globalMap = veh_pose_in_globalMap;

        this->TOFsensor1_in_vehFrame.x = veh_length*0.8*scale;
        this->TOFsensor1_in_vehFrame.y = 0;
        this->TOFsensor1_in_vehFrame.orientation = 0;

        this->leftIndicator_in_vehFrame.x = veh_length*0.8*scale;
        this->leftIndicator_in_vehFrame.y = -veh_width/4*scale;
        this->leftIndicator_in_vehFrame.orientation = 0;

        this->rightIndicator_in_vehFrame.x = veh_length*0.8*scale;
        this->rightIndicator_in_vehFrame.y = veh_width/4*scale;
        this->rightIndicator_in_vehFrame.orientation = 0;

        this->forwardIndicator_in_vehFrame.x = veh_length*0.9*scale;
        this->forwardIndicator_in_vehFrame.y = 0;
        this->forwardIndicator_in_vehFrame.orientation = 0;

        this->backwardIndicator_in_vehFrame.x = veh_length*0.1*scale;
        this->backwardIndicator_in_vehFrame.y = 0;
        this->backwardIndicator_in_vehFrame.orientation = 0;

        this->centreIndicator_in_vehFrame.x = veh_length*0.5*scale;
        this->centreIndicator_in_vehFrame.y = 0;
        this->centreIndicator_in_vehFrame.orientation = 0;
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

  
    double veh_length;
    double veh_width;
    PoseFrame veh_pose_in_globalMap;

    double velocity_MetersPerSec;
    double turn_amt_Radians; // Left +ve, right-ve

    // === Frames ===
    // Frames with respect to veh_frame
    PoseFrame TOFsensor1_in_vehFrame;
    PoseFrame leftIndicator_in_vehFrame;
    PoseFrame rightIndicator_in_vehFrame;
    PoseFrame forwardIndicator_in_vehFrame;
    PoseFrame backwardIndicator_in_vehFrame;
    PoseFrame centreIndicator_in_vehFrame;

    cv::Point ConvertMapToOpenCVPoint(cv::Point point_in_map) 
    {
        cv::Point point_in_OpenCV;

        point_in_OpenCV.x = point_in_map.x ;
        point_in_OpenCV.y = point_in_map.y - point_in_map.y*2; // Translate downwards as opencv y axis is inverted
        // std::cout << point_in_OpenCV.x << std::endl;
        return point_in_OpenCV;
    }

    PoseFrame TransformFromVehFrameToGlobalMapFrame (PoseFrame input_frame)
    {   
        // 
        double angle_offset = (atan(input_frame.y/input_frame.x)+1.5708);

        cv::Point input_frame_position = cv::Point(input_frame.x, input_frame.y);
        cv::Point input_frame_position_in_globalMap = cv::Point(this->veh_pose_in_globalMap.x + input_frame_position.x, this->veh_pose_in_globalMap.y + input_frame_position.y);
        double dist_from_point_of_rotation = CalculateEuclideanDist(input_frame_position_in_globalMap, cv::Point(this->veh_pose_in_globalMap.x, this->veh_pose_in_globalMap.y));
        PoseFrame input_poseFrame_in_globalMap(
            dist_from_point_of_rotation*sin(-this->veh_pose_in_globalMap.orientation + angle_offset)+this->veh_pose_in_globalMap.x, 
            dist_from_point_of_rotation*cos(-this->veh_pose_in_globalMap.orientation + angle_offset)+this->veh_pose_in_globalMap.y, 
            input_frame.orientation+veh_pose_in_globalMap.orientation);

        return input_poseFrame_in_globalMap;
    }

    PoseFrame TransformFromVehFrameToGlobalMapFrame1 (PoseFrame input_frame)
    {
        cv::Point input_frame_position = cv::Point(input_frame.x, input_frame.y);
        cv::Point input_frame_position_in_globalMap = cv::Point(this->veh_pose_in_globalMap.x + input_frame_position.x, this->veh_pose_in_globalMap.y + input_frame_position.y);
        double dist_from_point_of_rotation = CalculateEuclideanDist(input_frame_position_in_globalMap, cv::Point(this->veh_pose_in_globalMap.x, this->veh_pose_in_globalMap.y));

        PoseFrame input_poseFrame_in_globalMap(input_frame_position_in_globalMap.x, input_frame_position_in_globalMap.y, 0);

        return input_poseFrame_in_globalMap;
    }

    double CalculateEuclideanDist(cv::Point pt1, cv::Point pt2)
    {
        return sqrt((pow((pt1.x-pt2.x),2)) + (pow((pt1.y-pt2.y),2)));
    }

    // TODO:
    double wheel_diam;

    PoseFrame leftFrontWheel_frame;
    PoseFrame rightFrontWheel_frame;
    PoseFrame leftBackWheel_frame;
    PoseFrame rightBackWheel_frame;
  };