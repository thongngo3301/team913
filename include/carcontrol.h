#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>
#include <queue>

#include "detectlane.h"

using namespace std;
using namespace cv;

class CarControl
{
  public:
    CarControl();
    ~CarControl();
    void driveCar(const vector<Point> &left, const vector<Point> &right, float velocity, SIGN_TYPE sign, float proportion);
    float getVelocity();

    int n = 0;

  private:
    float errorAngle(const Point &dst);
    int calcFrSum(queue<int> frQueue);

    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;

    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    Point carPos;

    float laneWidth = 40;

    float minVelocity = 10;
    float maxVelocity = 100;
    float currVelocity;

    float preError;

    float kP;
    float kI;
    float kD;

    int t_kP;
    int t_kI;
    int t_kD;

    queue<int> frQueue;

    int prevFrCounter;
    int frSum;

    SIGN_TYPE true_sign = NONE;

};

#endif
