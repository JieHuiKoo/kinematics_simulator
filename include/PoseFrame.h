class PoseFrame {

public:

    double x;
    double y;
    double orientation;

    PoseFrame() : x(0), y(0), orientation(0) { }
    PoseFrame(double x, double y, double orientation) 
    {
        this->x = x; 
        this->y = y; 
        this->orientation = orientation;
    } 
};