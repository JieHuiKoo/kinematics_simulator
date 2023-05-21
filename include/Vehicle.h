#include <opencv2/opencv.hpp>
#include "PoseFrame.h"
#include <math.h> 

class SensorReading {

public:
    double sensor_forward_dist_to_obstacle;
    cv::Point position_of_wall_hit;
    cv::Mat layer_of_wall_hits;

    SensorReading() : sensor_forward_dist_to_obstacle(0), position_of_wall_hit(0) { }

    void GenerateLayerOfWallHits(cv::Size layer_size)
    {
        this->layer_of_wall_hits = cv::Mat::zeros(layer_size, CV_8UC3);
    }
};

class Vehicle {
public:
    void DrawPosition(cv::Mat *map) {
        
        // Transform vehicle components in veh frame to globalMapFrame
        PoseFrame vehCentre_in_globalMap            = TransformFromVehFrameToGlobalMapFrame(this->centreIndicator_in_vehFrame);
        PoseFrame vehLeftIndicator_in_globalMap     = TransformFromVehFrameToGlobalMapFrame(this->leftIndicator_in_vehFrame);
        PoseFrame vehRightIndicator_in_globalMap    = TransformFromVehFrameToGlobalMapFrame(this->rightIndicator_in_vehFrame);
        PoseFrame vehForwardIndicator_in_globalMap  = TransformFromVehFrameToGlobalMapFrame(this->forwardIndicator_in_vehFrame);
        PoseFrame vehBackwardIndicator_in_globalMap = TransformFromVehFrameToGlobalMapFrame(this->backwardIndicator_in_vehFrame);
        PoseFrame TOFsensor1_in_globalMap           = TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame);

        // Convert GlobalMapFrame position to OpenCVFrame (y axis is inverted)
        cv::Point veh_position_in_OpenCVMap         = ConvertMapToOpenCVPoint(cv::Point(this->veh_pose_in_globalMap.x, this->veh_pose_in_globalMap.y));
        cv::Point vehCentreIndicator_in_OpenCVMap   = ConvertMapToOpenCVPoint(cv::Point(vehCentre_in_globalMap.x, vehCentre_in_globalMap.y));
        cv::Point vehLeftIndicator_in_OpenCVMap     = ConvertMapToOpenCVPoint(cv::Point(vehLeftIndicator_in_globalMap.x, vehLeftIndicator_in_globalMap.y));
        cv::Point vehRightIndicator_in_OpenCVMap    = ConvertMapToOpenCVPoint(cv::Point(vehRightIndicator_in_globalMap.x, vehRightIndicator_in_globalMap.y));
        cv::Point vehForwardIndicator_in_OpenCVMap  = ConvertMapToOpenCVPoint(cv::Point(vehForwardIndicator_in_globalMap.x, vehForwardIndicator_in_globalMap.y));
        cv::Point vehBackwardIndicator_in_OpenCVMap = ConvertMapToOpenCVPoint(cv::Point(vehBackwardIndicator_in_globalMap.x, vehBackwardIndicator_in_globalMap.y));
        cv::Point TOFsensor1_in_OpenCVMap           = ConvertMapToOpenCVPoint(cv::Point(TOFsensor1_in_globalMap.x, TOFsensor1_in_globalMap.y));
        
        // Draw Center Points of Veh Components in OpenCVFrame
        cv::circle(*map, veh_position_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehCentreIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehLeftIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehRightIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehForwardIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, vehBackwardIndicator_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        cv::circle(*map, TOFsensor1_in_OpenCVMap, 4, cv::Scalar(255, 255, 255), -1);
        
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

        DrawRotatedRect(map, vehCentreIndicator_in_OpenCVMap, this->veh_size, vehCentre_in_globalMap.orientation, false);
        DrawRotatedRect(map, vehLeftIndicator_in_OpenCVMap, this->left_right_indicator_size, vehLeftIndicator_in_globalMap.orientation, left_fill);
        DrawRotatedRect(map, vehRightIndicator_in_OpenCVMap, this->left_right_indicator_size, vehRightIndicator_in_globalMap.orientation, right_fill);
        DrawRotatedRect(map, vehForwardIndicator_in_OpenCVMap, this->forward_backward_indicator_size, vehForwardIndicator_in_globalMap.orientation, forward_fill);
        DrawRotatedRect(map, vehBackwardIndicator_in_OpenCVMap, this->forward_backward_indicator_size, vehBackwardIndicator_in_globalMap.orientation, backward_fill);
        DrawRotatedRect(map, TOFsensor1_in_OpenCVMap, this->TOFsensor1_size, TOFsensor1_in_globalMap.orientation, false);
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
        if (movement_state_vector[0]) this->velocity_MetersPerSec = 1.3;
        else if (movement_state_vector[1]) this->velocity_MetersPerSec = -1.3;
        else this->velocity_MetersPerSec = 0;

        // Update turn_amt
        if (movement_state_vector[2]) this->turn_amt_Radians = 1.309; //75 deg
        else if (movement_state_vector[3]) this->turn_amt_Radians = -11.309; //75deg
        else this->turn_amt_Radians = 0;
    }

    void CalculateLookaheadPoint(PoseFrame LookaheadCentre_PoseFrame_GlobalMapFrame)
    {        
        this->lookahead_point.x = this->look_ahead_dist_Meters * sin(LookaheadCentre_PoseFrame_GlobalMapFrame.orientation+1.57) +  LookaheadCentre_PoseFrame_GlobalMapFrame.x;
        this->lookahead_point.y = this->look_ahead_dist_Meters * cos(LookaheadCentre_PoseFrame_GlobalMapFrame.orientation+1.57) +  LookaheadCentre_PoseFrame_GlobalMapFrame.y;
    }

    void CalculateSensorReading(const std::vector<std::array<cv::Point, 2>> &obstacles)
    {       
        int num_of_obstacles = obstacles.size();
        PoseFrame TOFsensor1_in_globalMap = TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame);

        std::vector<double> sensor_forward_dist_to_obstacles;
        std::vector<bool> is_sensor_pointingAt_wall;

        for (int i = 0; i < num_of_obstacles; i++)
        {
            cv::Point wall_endpoint1 = obstacles[i][0];
            cv::Point wall_endpoint2 = obstacles[i][1];   

            double sensor_forward_dist_to_obstacle = ((wall_endpoint2.y - wall_endpoint1.y)*(wall_endpoint1.x-TOFsensor1_in_globalMap.x) - (wall_endpoint2.x - wall_endpoint1.x)*(wall_endpoint1.y-TOFsensor1_in_globalMap.y))/((wall_endpoint2.y-wall_endpoint1.y)*cos(-TOFsensor1_in_globalMap.orientation)-(wall_endpoint2.x-wall_endpoint1.x)*sin(-TOFsensor1_in_globalMap.orientation));
            sensor_forward_dist_to_obstacles.push_back(sensor_forward_dist_to_obstacle);

            cv::Point position_of_wall_hit = cv::Point(
                TOFsensor1_in_globalMap.x + sensor_forward_dist_to_obstacle*cos(-TOFsensor1_in_globalMap.orientation),
                TOFsensor1_in_globalMap.y + sensor_forward_dist_to_obstacle*sin(-TOFsensor1_in_globalMap.orientation));

            // If sensor is between end points and the distance is more than 0
            if (CalculateEuclideanDist(position_of_wall_hit, wall_endpoint1) + CalculateEuclideanDist(position_of_wall_hit, wall_endpoint2) == CalculateEuclideanDist(wall_endpoint1, wall_endpoint2) &&
                sensor_forward_dist_to_obstacle > 0)
            {
                this->TOFsensor1_Reading.sensor_forward_dist_to_obstacle = sensor_forward_dist_to_obstacle;
                this->TOFsensor1_Reading.position_of_wall_hit = position_of_wall_hit;
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

    void DrawSensorReading(cv::Mat *car_layer)
    {
        // Draw Sensor Line Of Sight
        PoseFrame TOFsensor1_in_GlobalMapFrame = TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame);
        cv::Point TOFsensor1_position_in_GlobalMapFrame = cv::Point(TOFsensor1_in_GlobalMapFrame.x, TOFsensor1_in_GlobalMapFrame.y);
        cv::line(*car_layer, ConvertMapToOpenCVPoint(TOFsensor1_position_in_GlobalMapFrame), ConvertMapToOpenCVPoint(this->TOFsensor1_Reading.position_of_wall_hit), cv::Scalar(255, 255, 255), 2);

        // Draw Sensor Position of wall hit
        cv::circle(*car_layer, ConvertMapToOpenCVPoint(this->TOFsensor1_Reading.position_of_wall_hit), 4, cv::Scalar(255, 255, 255), -1);

        // Draw Sensor Text Reading
        cv::putText(*car_layer, 
                    std::to_string(int(this->TOFsensor1_Reading.sensor_forward_dist_to_obstacle)), 
                    ConvertMapToOpenCVPoint(cv::Point(this->TOFsensor1_Reading.position_of_wall_hit.x + 10, this->TOFsensor1_Reading.position_of_wall_hit.y + 10)), 
                    cv::FONT_HERSHEY_SIMPLEX, 
                    1, 
                    cv::Scalar(255,255,255), 
                    2, 
                    cv::LINE_AA);  

        // Record Sensor Position 
        cv::circle(this->TOFsensor1_Reading.layer_of_wall_hits, ConvertMapToOpenCVPoint(this->TOFsensor1_Reading.position_of_wall_hit), 4, cv::Scalar(255, 255, 255), -1);

        cv::bitwise_or(this->TOFsensor1_Reading.layer_of_wall_hits, *car_layer, *car_layer);
    }

    void AddPathWaypoints(cv::Point waypoint, cv::Size map_size)
    {
        this->waypoints.push_back(waypoint);
        
        this->waypoints_map = cv::Mat::zeros(map_size, CV_8UC1); // Declare b/w map of size 1000x1000 pixels

        int num_of_waypoints = this->waypoints.size();

        for (int i=0; i<num_of_waypoints-1; i++)
        {
            // Draw the waypoints within an internal map
            cv::line(this->waypoints_map, ConvertMapToOpenCVPoint(this->waypoints[i]), ConvertMapToOpenCVPoint(this->waypoints[i+1]), 255, 2);
        }
    }

    void DrawWaypoints(cv::Mat *image)
    {
        int num_of_waypoints = this->waypoints.size();

        for (int i=0; i<num_of_waypoints-1; i++)
        {
            cv::line(*image, ConvertMapToOpenCVPoint(this->waypoints[i]), ConvertMapToOpenCVPoint(this->waypoints[i+1]), cv::Scalar(0, 255, 255), 2);
            cv::circle(*image, ConvertMapToOpenCVPoint(this->waypoints[i]), 4, cv::Scalar(110, 110, 255), -1);
            cv::circle(*image, ConvertMapToOpenCVPoint(this->waypoints[i+1]), 4, cv::Scalar(110, 110, 255), -1);
        }
    }

    void UpdatePositionPathPursuit()
    {
        PoseFrame centre_of_lookahead_GlobalMapFrame = TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame);
        this->centre_of_lookahead_circle_GlobalMapFrame = cv::Point(centre_of_lookahead_GlobalMapFrame.x, centre_of_lookahead_GlobalMapFrame.y);
        cv::Point centre_of_lookahead_point_OpenCVFrame = ConvertMapToOpenCVPoint(this->centre_of_lookahead_circle_GlobalMapFrame);
        CalculateLookaheadPoint(centre_of_lookahead_GlobalMapFrame);

        this->target_point = FindLookAheadIntersection(this->TOFsensor1_Reading.layer_of_wall_hits.size(), centre_of_lookahead_point_OpenCVFrame);
    }

    void AnnotatePathPursuit(cv::Mat *car_drawing)
    {
        cv::circle(*car_drawing, ConvertMapToOpenCVPoint(this->centre_of_lookahead_circle_GlobalMapFrame), this->look_ahead_dist_Meters, cv::Scalar(110, 110, 255), 4);

        cv::circle(*car_drawing, this->target_point, 10, cv::Scalar(110, 110, 255), 4);

        cv::circle(*car_drawing, ConvertMapToOpenCVPoint(this->lookahead_point), 10, cv::Scalar(110,110,255), 4);

        int num_of_waypoints = this->waypoints.size();
        for (int i=0; i<num_of_waypoints-1; i++)
        {
            // Draw the waypoints within an internal map
            cv::line(*car_drawing, ConvertMapToOpenCVPoint(this->waypoints[i]), ConvertMapToOpenCVPoint(this->waypoints[i+1]), 255, 2);
        }
    }

    Vehicle() : veh_length(10), veh_width(10) { }
    Vehicle(double veh_length, double veh_width, PoseFrame veh_pose_in_globalMap, double scale, double look_ahead_dist_Meters, cv::Size base_environment_map_size)
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

        this->look_ahead_dist_Meters = look_ahead_dist_Meters*scale;

        this->TOFsensor1_Reading.GenerateLayerOfWallHits(base_environment_map_size);

        this->veh_size                           = cv::Size(this->veh_length, this->veh_width);
        this->left_right_indicator_size          = cv::Size(this->veh_length*0.1, this->veh_width*0.25);
        this->forward_backward_indicator_size    = cv::Size(this->veh_length*0.1, this->veh_width);
        this->TOFsensor1_size                    = cv::Size(this->veh_length*0.1, this->veh_width*0.1);

        PoseFrame TOFsensor1_poseframe = TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame);
        this->centre_of_lookahead_circle_GlobalMapFrame = cv::Point(TOFsensor1_poseframe.x, TOFsensor1_poseframe.y);
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

    cv::Size veh_size;
    cv::Size left_right_indicator_size;
    cv::Size forward_backward_indicator_size;
    cv::Size TOFsensor1_size;

    PoseFrame veh_pose_in_globalMap;

    double velocity_MetersPerSec;
    double turn_amt_Radians; // Left +ve, right-ve

    // Path Pursuit
    double look_ahead_dist_Meters;
    cv::Point lookahead_point;
    cv::Point target_point;
    cv::Point centre_of_lookahead_circle_GlobalMapFrame;

    // === Frames ===
    // Frames with respect to veh_frame
    PoseFrame TOFsensor1_in_vehFrame;
    PoseFrame leftIndicator_in_vehFrame;
    PoseFrame rightIndicator_in_vehFrame;
    PoseFrame forwardIndicator_in_vehFrame;
    PoseFrame backwardIndicator_in_vehFrame;
    PoseFrame centreIndicator_in_vehFrame;

    std::vector<cv::Point> waypoints;
    cv::Mat waypoints_map;
    
    SensorReading TOFsensor1_Reading;

    // TODO: Remove repeated code from map.h
    cv::Point ConvertMapToOpenCVPoint(cv::Point point_in_map) 
    {
        cv::Point point_in_OpenCV;

        point_in_OpenCV.x = point_in_map.x;
        point_in_OpenCV.y = point_in_map.y - point_in_map.y*2; // Translate downwards as opencv y axis is inverted (Not sure why inverting point_in_map also works)
        return point_in_OpenCV;
    }

    PoseFrame TransformFromVehFrameToGlobalMapFrame (PoseFrame input_frame)
    {   
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

    cv::Point FindLookAheadIntersection(cv::Size map_size, cv::Point centre_of_lookahead_point)
    {
        cv::Mat look_ahead_circle_map = cv::Mat::zeros(map_size, CV_8UC1); // Declare map of size 1000x1000 pixels

        cv::circle(look_ahead_circle_map, centre_of_lookahead_point, this->look_ahead_dist_Meters, 255, 4);
        
        cv::Mat intersection_map;
        cv::bitwise_and(look_ahead_circle_map, this->waypoints_map, intersection_map);

        std::vector<cv::Point> intersections_OpenCVFrame = FindBlobCentroids(&intersection_map);

        cv::Point intersection = FindValidIntersection(intersections_OpenCVFrame, centre_of_lookahead_point);


        cv::bitwise_or(look_ahead_circle_map, this->waypoints_map, look_ahead_circle_map);

        // cv::imshow("look_ahead_circle_map", look_ahead_circle_map);

        cv::waitKey(1);

        // cv::imshow("intersection_map", intersection_map);
        cv::waitKey(1);

        return intersection;
    }

    cv::Point FindValidIntersection(std::vector<cv::Point> intersections, cv::Point center_of_lookahead)
    {       
        double min_rotation_amt = std::numeric_limits<double>::max();;
        cv::Point valid_intersection;

        for (int i = 0; i<intersections.size(); i++)
        {
            cv::Point intersection = intersections[i];
            double rotation_amt = RotationAmtOfPointFromVeh(intersection);
            std::cout << int(rotation_amt * (180/M_PI)) << std::endl;
            std::cout << "Car Rotation" << TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame).orientation << std::endl;
            if (min_rotation_amt > rotation_amt)
            {
                min_rotation_amt = rotation_amt;
                valid_intersection = intersection;
            }
        }
        return valid_intersection;
    }

    double RotationAmtOfPointFromVeh(cv::Point input_point_OpenCVFrame)
    {   
        double rotation_amt;

        if (input_point_OpenCVFrame.x == this->centre_of_lookahead_circle_GlobalMapFrame.x)
            rotation_amt = 0;
        else
            rotation_amt = atan(-(input_point_OpenCVFrame.y - ConvertMapToOpenCVPoint(this->centre_of_lookahead_circle_GlobalMapFrame).y)/ (input_point_OpenCVFrame.x - ConvertMapToOpenCVPoint(this->centre_of_lookahead_circle_GlobalMapFrame).x));
        return rotation_amt - TransformFromVehFrameToGlobalMapFrame(this->TOFsensor1_in_vehFrame).orientation;
    }

    std::vector<cv::Point> FindBlobCentroids (cv::Mat *image)
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(*image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // get the moments
        std::vector<cv::Moments> mu(contours.size());
        for( int i = 0; i<contours.size(); i++ )
        { mu[i] = moments( contours[i], false ); }

        // get the centroid of figures.
        std::vector<cv::Point> mc(contours.size());
        for( int i = 0; i<contours.size(); i++)
        { mc[i] = cv::Point( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

        return mc;
    }
  };