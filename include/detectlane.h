#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;
using namespace cv;

enum SIGN_TYPE {
  LEFT  = -1,
  NONE  = 0,
  RIGHT = 1
};

const int MIN_CMP_VAL = 25;
const uchar COLOR_THRESHOLD = 240;

class DetectLane
{
  public:
    DetectLane();
    ~DetectLane();

    void update(Mat &src);
    Mat getImgThresholded();
    SIGN_TYPE getTrafficSign(const Mat &src);
    float getSignProportion();
    bool isSign(Mat img, vector<Point> elem);

    vector<Point> getLeftLane();
    vector<Point> getRightLane();

    static int slideThickness;

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static int VERTICAL;
    static int HORIZONTAL;

    static Point null; //

  private:
    Mat preProcess(const Mat &src);

    // bool cmp(vector<Point> a, vector<Point> b);

    Mat morphological(const Mat &imgHSV);
    Mat birdViewTranform(const Mat &source);
    void drawLanes(Mat &img);
    void fillLane(Mat &src);
    vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point>> centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    void detectLeftRight(const vector<vector<Point>> &points);
    Mat laneInShadow(const Mat &src);

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};
    int minShadowTh[3] = {90, 43, 36};
    int maxShadowTh[3] = {120, 81, 171};
    int minLaneInShadow[3] = {90, 43, 97};
    int maxLaneInShadow[3] = {120, 80, 171};
    int binaryThreshold = 180;

    int skyLine = 120;
    int shadowParam = 40;

    float proportion = 0;

    vector<Point> leftLane, rightLane;
};

#endif
