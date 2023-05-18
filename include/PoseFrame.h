class PoseFrame {

public:

    float x;
    float y;
    float orientation;

    PoseFrame() : x(0), y(0), orientation(0) { }
    PoseFrame(float x, float y, float orientation) 
    {
        this->x = x; 
        this->y = y; 
        this->orientation = orientation;
    } 
};