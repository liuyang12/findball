/// demo of finding the ball(round)
// finball_demo.cpp

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>
#include <findball/ball.h>

#define COLOR_Orange    1 * 32  // ball
#define COLOR_Yellow    2 * 32  // gatepost
#define COLOR_Blue      3 * 32  //
#define COLOR_Green     4 * 32  // field
#define COLOR_White     5 * 32  // field line
#define COLOR_Black     0 * 32  //
#define WIDTH   400
#define HEIGHT  400

struct coor{    // coordinate 坐标 (x, y)
    int x, y;
};

struct circle{  // circle 圆 (X-x)^2 + (Y-y)^2 = (R-r)^2
    double x, y;
    double r;
};

struct line{    // line 直线 ax + by + cz = 0
    double a, b, c;
};

