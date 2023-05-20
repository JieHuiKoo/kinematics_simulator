#include <opencv2/opencv.hpp>

class Map {
public:
    cv::Size map_size;
    cv::Mat map_layer;
    std::vector<std::array <cv::Point, 2>> obstacles;

    Map() : map_size(cv::Size(1000, 1000)) { }
    Map(cv::Size map_size) 
    {
        this->map_size = map_size;
        this->map_layer = cv::Mat::zeros(map_size, CV_8UC3); // Declare map of size 1000x1000 pixels
    } 

    void AddObstacle(std::array <cv::Point, 2> obstacle)
    {
        obstacles.push_back(obstacle);

        cv::line(this->map_layer, ConvertMapToOpenCVPoint(obstacle[0]), ConvertMapToOpenCVPoint(obstacle[1]), cv::Scalar(0, 255, 0), 2);
    }

private:
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
    
    // TODO: Remove repeated code
    cv::Point ConvertMapToOpenCVPoint(cv::Point point_in_map) 
    {
        cv::Point point_in_OpenCV;

        point_in_OpenCV.x = point_in_map.x;
        point_in_OpenCV.y = point_in_map.y - point_in_map.y*2; // Translate downwards as opencv y axis is inverted (Not sure why inverting point_in_map also works)
        return point_in_OpenCV;
    }
};