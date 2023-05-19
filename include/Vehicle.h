#include <opencv2/opencv.hpp>
#include "PoseFrame.h"
#include <math.h> 

class Vehicle {
public:
    void DrawPosition(cv::Mat *map) {
        
        PoseFrame vehCentre_in_globalMap            = TransformFromVehFrameToGlobalMapFrame(this->centreIndicator_in_vehFrame);
        PoseFrame vehLeftIndicator_in_globalMap     = TransformFromVehFrameToGlobalMapFrame(this->leftIndicator_in_vehFrame);
        PoseFrame vehRightIndicator_in_globalMap    = TransformFromVehFrameToGlobalMapFrame(this->rightIndicator_in_vehFrame);
        PoseFrame vehForwardIndicator_in_globalMap  = TransformFromVehFrameToGlobalMapFrame(this->forwardIndicator_in_vehFrame);
        PoseFrame vehBackwardIndicator_in_globalMap = TransformFromVehFrameToGlobalMapFrame(this->backwardIndicator_in_vehFrame);
        PoseFrame TOFsensor1_in_globalMap           = TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame);

        cv::Point veh_position_in_OpenCVMap         = ConvertMapToOpenCVPoint(cv::Point(this->veh_pose_in_globalMap.x, this->veh_pose_in_globalMap.y));
        cv::Point vehCentreIndicator_in_OpenCVMap   = ConvertMapToOpenCVPoint(cv::Point(vehCentre_in_globalMap.x, vehCentre_in_globalMap.y));
        cv::Point vehLeftIndicator_in_OpenCVMap     = ConvertMapToOpenCVPoint(cv::Point(vehLeftIndicator_in_globalMap.x, vehLeftIndicator_in_globalMap.y));
        cv::Point vehRightIndicator_in_OpenCVMap    = ConvertMapToOpenCVPoint(cv::Point(vehRightIndicator_in_globalMap.x, vehRightIndicator_in_globalMap.y));
        cv::Point vehForwardIndicator_in_OpenCVMap  = ConvertMapToOpenCVPoint(cv::Point(vehForwardIndicator_in_globalMap.x, vehForwardIndicator_in_globalMap.y));
        cv::Point vehBackwardIndicator_in_OpenCVMap = ConvertMapToOpenCVPoint(cv::Point(vehBackwardIndicator_in_globalMap.x, vehBackwardIndicator_in_globalMap.y));
        cv::Point TOFsensor1_in_OpenCVMap           = ConvertMapToOpenCVPoint(cv::Point(TOFsensor1_in_globalMap.x, TOFsensor1_in_globalMap.y));
        
        cv::circle(*map, veh_position_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehCentreIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehLeftIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehRightIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehForwardIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehBackwardIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, TOFsensor1_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);

        cv::Size veh_size                           = cv::Size(this->veh_length, this->veh_width);
        cv::Size left_right_indicator_size          = cv::Size(this->veh_length*0.1, this->veh_width*0.25);
        cv::Size forward_backward_indicator_size    = cv::Size(this->veh_length*0.1, this->veh_width);
        cv::Size TOFsensor1_size                    = cv::Size(this->veh_length*0.1, this->veh_width*0.1);
        
        bool forward_fill = false;
        bool backward_fill = false;
        bool left_fill = false;
        bool right_fill = false;

        // Draw Forward Back Indicators
        if (this->velocity_MetersPerSec > 0)
        {
            forward_fill = true;
        }
        else if (this->velocity_MetersPerSec < 0)
        {
            backward_fill = true;
        }

        // Draw Turning indicators
        if (this->turn_amt_Radians > 0) // Left
        {
            left_fill = true;
            this->leftIndicator_in_vehFrame.orientation = -0.785398;
            this->rightIndicator_in_vehFrame.orientation = -0.785398;

        }
        else if (this->turn_amt_Radians < 0) // Right
        {
            right_fill = true;
            this->leftIndicator_in_vehFrame.orientation = 0.785398;
            this->rightIndicator_in_vehFrame.orientation = 0.785398;
        }
        else if (this->turn_amt_Radians == 0) // Straight
        {
            right_fill = false;
            this->leftIndicator_in_vehFrame.orientation = 0;
            this->rightIndicator_in_vehFrame.orientation = 0;
        }

        DrawRotatedRect(map, vehCentreIndicator_in_OpenCVMap, veh_size, vehCentre_in_globalMap.orientation, false);
        DrawRotatedRect(map, vehLeftIndicator_in_OpenCVMap, left_right_indicator_size, vehLeftIndicator_in_globalMap.orientation, left_fill);
        DrawRotatedRect(map, vehRightIndicator_in_OpenCVMap, left_right_indicator_size, vehRightIndicator_in_globalMap.orientation, right_fill);
        DrawRotatedRect(map, vehForwardIndicator_in_OpenCVMap, forward_backward_indicator_size, vehForwardIndicator_in_globalMap.orientation, forward_fill);
        DrawRotatedRect(map, vehBackwardIndicator_in_OpenCVMap, forward_backward_indicator_size, vehBackwardIndicator_in_globalMap.orientation, backward_fill);
        DrawRotatedRect(map, TOFsensor1_in_OpenCVMap, TOFsensor1_size, TOFsensor1_in_globalMap.orientation, false);
    }
    
    void UpdatePosition() {
        
        double veh_distBtwnFrontBackWheel = this->leftIndicator_in_vehFrame.x;
        double turn_radius_Meters = abs(veh_distBtwnFrontBackWheel/tan(this->turn_amt_Radians)) + this->veh_width/2;
        
        double front_phi_Radians = atan(veh_distBtwnFrontBackWheel/turn_radius_Meters);

        if (this->turn_amt_Radians < 0) front_phi_Radians = -front_phi_Radians;

        this->veh_pose_in_globalMap.x = this->veh_pose_in_globalMap.x + this->velocity_MetersPerSec*cos(this->veh_pose_in_globalMap.orientation);
        this->veh_pose_in_globalMap.y = this->veh_pose_in_globalMap.y + this->velocity_MetersPerSec*sin(this->veh_pose_in_globalMap.orientation);
        this->veh_pose_in_globalMap.orientation = this->veh_pose_in_globalMap.orientation + this->velocity_MetersPerSec/veh_distBtwnFrontBackWheel*front_phi_Radians;
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

    void AnnotateSensorReading(const std::vector<std::array<cv::Point, 2>> &obstacles, cv::Mat *car_drawing, cv::Mat *map)
    {
        int num_of_obstacles = obstacles.size();
        PoseFrame TOFsensor1_in_globalMap = TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame);

        std::vector<double> sensor_forward_dist_to_obstacles;
        std::vector<bool> is_sensor_pointingAt_wall;

        for (int i = 0; i < num_of_obstacles; i++)
        {
            cv::Point endpoint1 = obstacles[i][0];
            cv::Point endpoint2 = obstacles[i][1];

            cv::line(*car_drawing, ConvertMapToOpenCVPoint(endpoint1), ConvertMapToOpenCVPoint(endpoint2), cv::Scalar(0, 255, 0), 2);

            double sensor_forward_dist_to_obstacle = ((endpoint2.y - endpoint1.y)*(endpoint1.x-TOFsensor1_in_globalMap.x) - (endpoint2.x - endpoint1.x)*(endpoint1.y-TOFsensor1_in_globalMap.y))/((endpoint2.y-endpoint1.y)*cos(-TOFsensor1_in_globalMap.orientation)-(endpoint2.x-endpoint1.x)*sin(-TOFsensor1_in_globalMap.orientation));
            sensor_forward_dist_to_obstacles.push_back(sensor_forward_dist_to_obstacle);

            cv::Point position_of_wall_hit = cv::Point(
                TOFsensor1_in_globalMap.x + sensor_forward_dist_to_obstacle*cos(-TOFsensor1_in_globalMap.orientation),
                TOFsensor1_in_globalMap.y + sensor_forward_dist_to_obstacle*sin(-TOFsensor1_in_globalMap.orientation));

            // If sensor hits the wall and the distance is more than 0
            if (CalculateEuclideanDist(position_of_wall_hit, endpoint1) + CalculateEuclideanDist(position_of_wall_hit, endpoint2) == CalculateEuclideanDist(endpoint1, endpoint2) &&
                sensor_forward_dist_to_obstacle > 0                                               )
            {
                cv::line(*car_drawing, ConvertMapToOpenCVPoint(position_of_wall_hit), ConvertMapToOpenCVPoint(cv::Point(TOFsensor1_in_globalMap.x, TOFsensor1_in_globalMap.y)), cv::Scalar(255, 255, 255), 2);
                cv::circle(*map, ConvertMapToOpenCVPoint(position_of_wall_hit), 4, cv::Scalar(255, 255, 255), -1);

                cv::putText(*car_drawing, std::to_string(int(sensor_forward_dist_to_obstacle)), ConvertMapToOpenCVPoint(cv::Point(position_of_wall_hit.x + 10, position_of_wall_hit.y+10)), cv::FONT_HERSHEY_SIMPLEX, 
                   1, cv::Scalar(255,255,255), 2, cv::LINE_AA);

                is_sensor_pointingAt_wall.push_back(true);
            }
            else
            {
                is_sensor_pointingAt_wall.push_back(false);
            }

            std::cout << i << " Dist To Wall: " << sensor_forward_dist_to_obstacle << " X: " << position_of_wall_hit.x << " Y: " << position_of_wall_hit.y << " Detected: " << is_sensor_pointingAt_wall[i] << std::endl;
        }
        std::cout << "\n\n===\n\n" << std::endl;
    }

    Vehicle() : veh_length(10), veh_width(10) { }
    Vehicle(double veh_length, double veh_width, PoseFrame veh_pose_in_globalMap, double scale) 
    {
        this->veh_length = veh_length*scale; 
        this->veh_width = veh_width*scale; 
        this->veh_pose_in_globalMap = veh_pose_in_globalMap;

        this->TOFsensor1_in_vehFrame.x = veh_length*0.75*scale;
        this->TOFsensor1_in_vehFrame.y = 0;
        this->TOFsensor1_in_vehFrame.orientation = 0;

        this->leftIndicator_in_vehFrame.x = veh_length*0.75*scale;
        this->leftIndicator_in_vehFrame.y = -veh_width/4*scale;
        this->leftIndicator_in_vehFrame.orientation = 0;

        this->rightIndicator_in_vehFrame.x = veh_length*0.75*scale;
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

        point_in_OpenCV.x = point_in_map.x;
        point_in_OpenCV.y = -point_in_map.y;// - point_in_map.y*2; // Translate downwards as opencv y axis is inverted
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
            input_frame.orientation-veh_pose_in_globalMap.orientation);

        return input_poseFrame_in_globalMap;
    }

    double CalculateEuclideanDist(cv::Point pt1, cv::Point pt2)
    {
        return sqrt((pow((pt1.x-pt2.x),2)) + (pow((pt1.y-pt2.y),2)));
    }

    void DrawRotatedRect(cv::Mat *image, cv::Point center, cv::Size size, double rotation_Radians, bool fill)
    {       
        double rotation_degrees = rotation_Radians*180/M_PI;
        
        cv::Scalar colour = cv::Scalar(0,255,0);

        cv::Point2f vertices2f[4];
        cv::RotatedRect rRect = cv::RotatedRect(center, size, rotation_degrees);
        rRect.points(vertices2f);

        for (int i = 0; i < 4; i++)
            cv::line(*image, vertices2f[i], vertices2f[(i+1)%4], colour, 2);
        if (fill)
        {   
            cv::Point vertices[4];    
            for(int i = 0; i < 4; ++i) vertices[i] = vertices2f[i];
            cv::fillConvexPoly(*image, vertices, 4, colour);
        }

    }
  };