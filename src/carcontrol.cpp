#include "carcontrol.h"

CarControl::CarControl()
{
    prevFrCounter = 0;
    frSum = 0;
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

void CarControl::driveCar(const vector<Point> &left, const vector<Point> &right, float velocity, SIGN_TYPE sign)
{
    const int TURN_LEFT_THRESHOLD = -12;
    const int TURN_RIGHT_THRESHOLD = 12;
    const int QUEUE_LIMIT = 30;
    int i = left.size() - 11;
    float error = preError;
    while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
        i--;
        if (i < 0)
            return;
    }
    if (frQueue.size() == QUEUE_LIMIT) {
        frQueue.pop();
    }
    switch (sign) {
        case LEFT:
            frQueue.push(-1);
            break;
        case RIGHT:
            frQueue.push(1);
            break;
        default:
            frQueue.push(0);
            break;
    }

    frSum = calcFrSum(frQueue);
    // cout << "fr sum: " << frSum << endl;

    if (sign != NONE) {
        if (frSum != 0) {
            velocity /= 2.0;
            if (frSum == TURN_LEFT_THRESHOLD) {
                error = -30;
            } else if (frSum == TURN_LEFT_THRESHOLD - 5) {
                error = -30;
                frSum = 0;
            } else if (frSum > TURN_LEFT_THRESHOLD && frSum <= TURN_LEFT_THRESHOLD / 2) {
                error = errorAngle(left[i] + Point(laneWidth / 4.0, 0));
            } else {
                if (left[i] != DetectLane::null && right[i] != DetectLane::null) {
                    error = errorAngle((left[i] + right[i]) / 2);
                }
                else if (left[i] != DetectLane::null) {
                    error = errorAngle(left[i] + Point(laneWidth / 2.0, 0));
                }
                else if (right[i] != DetectLane::null) {
                    error = errorAngle(right[i] - Point(laneWidth / 2.0, 0));
                }
                else {
                    error = errorAngle(Point(laneWidth * 1.0 / 2.0, 0));
                }
            }
            if (frSum == TURN_RIGHT_THRESHOLD) {
                error = 30;
            } else if (frSum == TURN_RIGHT_THRESHOLD + 5) {
                error = 30;
                frSum = 0;
            } else if (frSum >= TURN_RIGHT_THRESHOLD / 2 && frSum < TURN_RIGHT_THRESHOLD) {
                error = errorAngle(right[i] - Point(laneWidth / 4.0, 0));
            } else {
                if (left[i] != DetectLane::null && right[i] != DetectLane::null) {
                    error = errorAngle((left[i] + right[i]) / 2);
                }
                else if (left[i] != DetectLane::null) {
                    error = errorAngle(left[i] + Point(laneWidth / 2.0, 0));
                }
                else if (right[i] != DetectLane::null) {
                    error = errorAngle(right[i] - Point(laneWidth / 2.0, 0));
                }
                else {
                    error = errorAngle(Point(laneWidth * 1.0 / 2.0, 0));
                }
            }
        }
    }

/*
    if (sign != NONE) {
        prevFrCounter++;
        cout << "prev fr counter: " << prevFrCounter << endl;
        if (prevFrCounter == 10) {
            // velocity /= 2.0;
            switch (sign) {
                case LEFT:
                    error = errorAngle(left[i] + Point(laneWidth / 2.0, 0));
                    // error = errorAngle(right[i] - Point(laneWidth / 2.0, 0));
                    break;
                case RIGHT:
                    // error = errorAngle(left[i] + Point(laneWidth / 2.0, 0));
                    error = errorAngle(right[i] - Point(laneWidth / 2.0, 0));
                    break;
            }
        }
        else if (prevFrCounter == 30) {
            prevFrCounter = 0;
            velocity /= 2.0;
            switch (sign) {
                case LEFT:
                    error = -30;
                    break;
                case RIGHT:
                    error = 30;
                    break;
            }
        }
    }
*/
    else {
        if (left[i] != DetectLane::null && right[i] != DetectLane::null) {
            error = errorAngle((left[i] + right[i]) / 2);
        }
        else if (left[i] != DetectLane::null) {
            error = errorAngle(left[i] + Point(laneWidth / 2.0, 0));
            velocity /= 2.0;
        }
        else if (right[i] != DetectLane::null) {
            error = errorAngle(right[i] - Point(laneWidth / 2.0, 0));
            velocity /= 2.0;
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

int CarControl::calcFrSum(queue<int> frQueue) {
    int sum = 0;
    while (!frQueue.empty()) {
        sum += frQueue.front();
        frQueue.pop();
    }
    return sum;
}