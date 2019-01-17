#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"

#include <string>
#include <unistd.h>

bool STREAM = true;

VideoCapture capture("video.avi");
DetectLane *detect;
CarControl *car;
int skipFrame = 1;
int idx = 0;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        waitKey(1);
        // float currVelocity = car->getVelocity();
        // if (currVelocity > 0) {
        //     int delayTime = 1 / currVelocity * 1000000;
        //     usleep(delayTime);
        //     string name = "/home/thongnd/Desktop/images/" + to_string(idx) + ".jpg";
        //     cout << name << endl;
        //     imwrite(name, cv_ptr->image);
        //     idx++;
        // }
        detect->update(cv_ptr->image);
        // ========== LAY ANH O DAY ==========
        if (idx % 30 == 0) {
            string name = "/home/thongnd/Desktop/images/3/" + to_string(idx/30) + ".jpg";
            cout << name << endl;
            // imwrite(name, cv_ptr->image);
            imwrite(name, detect->getImgThresholded());
        }
        idx++;
        car->driveCar(detect->getLeftLane(), detect->getRightLane(), 40, 0);
        // cv::imshow("View", cv_ptr->image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void videoProcess()
{
    Mat src;
    while (true)
    {
        capture >> src;
        if (src.empty())
            break;

        detect->update(src);
        // imshow("View", src);
        waitKey(30);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    // cv::namedWindow("View");
    // cv::namedWindow("Binary");
    // cv::namedWindow("Threshold");
    // cv::namedWindow("Bird View");
    // cv::namedWindow("Lane Detect");

    detect = new DetectLane();
    car = new CarControl();

    if (STREAM)
    {
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("team913_image", 1, imageCallback);

        ros::spin();
    }
    else
    {
        videoProcess();
    }
    cv::destroyAllWindows();
}
