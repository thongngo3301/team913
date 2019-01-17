#include "carcontrol.h"

CarControl::CarControl()
{
    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("team913_steerAngle", 10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("team913_speed", 10);
}

CarControl::~CarControl() {}

float CarControl::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x)
        return 0;
    if (dst.y == carPos.y)
        return (dst.x < carPos.x ? -50 : 50);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y;
    if (dx < 0)
        return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarControl::driveCar(const vector<Point> &left, const vector<Point> &right, float velocity, int sign)
{
    const int LEFT = 1;
    const int RIGHT = 2;

    int i = left.size() - 11;
    float error = preError;
    while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
        i--;
        if (i < 0)
            return;
    }
    if (sign) {
        switch (sign) {
            case LEFT:
                break;
            case RIGHT:
                velocity = 10;
                error = 2;
                break;
        }
    } else {
        if (left[i] != DetectLane::null && right[i] != DetectLane::null) {
            error = errorAngle((left[i] + right[i]) / 2);
        }
        else if (left[i] != DetectLane::null) {
            error = errorAngle(left[i] + Point(laneWidth / 2.5, 0));
            velocity /= 2.5;
        }
        else if (right[i] != DetectLane::null) {
            error = errorAngle(right[i] - Point(laneWidth / 1.5, 0));
            velocity /= 2.5;
        }
        else {
            error = errorAngle(Point(laneWidth * 1.0 / 2.0, 0));
            velocity /= 2.0;
        }
    }

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    currVelocity = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
}

float CarControl::getVelocity() {
    return currVelocity;
}