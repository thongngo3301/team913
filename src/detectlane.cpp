#include "detectlane.h"
#include "math.h"

int min(int a, int b)
{
    return a < b ? a : b;
}

int DetectLane::slideThickness = 10;
int DetectLane::BIRDVIEW_WIDTH = 240;
int DetectLane::BIRDVIEW_HEIGHT = 320;
int DetectLane::VERTICAL = 0;
int DetectLane::HORIZONTAL = 1;
Point DetectLane::null = Point();

Mat imgThresholded;

DetectLane::DetectLane()
{
    cvCreateTrackbar("LowH", "Threshold", &minThreshold[0], 240);
    cvCreateTrackbar("HighH", "Threshold", &maxThreshold[0], 240);

    cvCreateTrackbar("LowS", "Threshold", &minThreshold[1], 255);
    cvCreateTrackbar("HighS", "Threshold", &maxThreshold[1], 255);

    cvCreateTrackbar("LowV", "Threshold", &minThreshold[2], 255);
    cvCreateTrackbar("HighV", "Threshold", &maxThreshold[2], 255);

    cvCreateTrackbar("Shadow Param", "Threshold", &shadowParam, 255);
}

DetectLane::~DetectLane() {}

vector<Point> DetectLane::getLeftLane()
{
    return leftLane;
}

Mat DetectLane::getImgThresholded() {
    return imgThresholded;
}

vector<Point> DetectLane::getRightLane()
{
    return rightLane;
}

void DetectLane::drawLanes(Mat &img)
{
    // Mat lane = Mat::zeros(img.size(), CV_8UC3);

    // imshow("threshold", imgThresholded);

    // cvtColor(imgThresholded, dst, COLOR_HSV2BGR);

    // for (int i = imgThresholded.rows - skyLine; i < imgThresholded.rows; i++) {
    //     for (int j = 0; j < imgThresholded.cols; j++) {
    //         // Vec3b pHSV = imgThresholded.at<Vec3b>(j, i);
    //         circle(img, Point(j, i), 1, Scalar(0, 0, 255), 2, 8, 0);
    //     }
    // }

    // Mat grayImg;
    // cvtColor(img, grayImg, COLOR_BGR2GRAY);

    // Ptr<LineSegmentDetector> det;
    // det = createLineSegmentDetector();

    // Mat lines;
    // det->detect(grayImg, lines);

    // det->drawSegments(imgThresholded, lines);

    // const int KERNEL_SIZE = 31;
    // Mat GBImg;
    // GaussianBlur(grayImg, GBImg, Size(KERNEL_SIZE, KERNEL_SIZE), 0, 0);
    // imshow("gbimg", GBImg);
    // const double L_THRESHOLD = 50.0;
    // const double H_THRESHOLD = 150.0;
    // Mat edges;
    // Canny(GBImg, edges, L_THRESHOLD, H_THRESHOLD);

    // vector<Vec4i> lines;
    // HoughLinesP(edges, lines, 1, CV_PI / 180, 1);
    // for (size_t i = 0; i < lines.size(); i++)
    // {
    //     Vec4i l = lines[i];
    //     line(img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
    // }

    // imshow("lanes", img);
    // imshow("threshold", imgThresholded);

    // vector<Point> _leftLane = getLeftLane();
    // vector<Point> _rightLane = getRightLane();

    // for (int i = 1; i < _leftLane.size(); i++)
    // {
    //     if (_leftLane[i] != null)
    //     {
    //         circle(img, _leftLane[i], 1, Scalar(0, 0, 255), 2, 8, 0);
    //     }
    // }

    // for (int i = 1; i < _rightLane.size(); i++)
    // {
    //     if (_rightLane[i] != null)
    //     {
    //         circle(img, _rightLane[i], 1, Scalar(255, 0, 0), 2, 8, 0);
    //     }
    // }
}

// Main processor
void DetectLane::update(Mat &src)
{
    Mat img = preProcess(src);

    // getTrafficSign(src);

    vector<Mat> layers1 = splitLayer(img);
    vector<vector<Point>> points1 = centerRoadSide(layers1);
    // vector<Mat> layers2 = splitLayer(img, HORIZONTAL);
    // vector<vector<Point> > points2 = centerRoadSide(layers2, HORIZONTAL);

    detectLeftRight(points1);

    // Mat birdView, lane;
    // birdView = Mat::zeros(img.size(), CV_8UC3);
    // lane = Mat::zeros(img.size(), CV_8UC3);

    // for (int i = 0; i < points1.size(); i++)
    // {
    //     for (int j = 0; j < points1[i].size(); j++)
    //     {
    //         circle(birdView, points1[i][j], 1, Scalar(0, 0, 255), 2, 8, 0);
    //     }
    // }

    // for (int i = 1; i < leftLane.size(); i++)
    // {
    //     if (leftLane[i] != null)
    //     {
    //         circle(lane, leftLane[i], 1, Scalar(0, 0, 255), 2, 8, 0);
    //     }
    // }

    // for (int i = 1; i < rightLane.size(); i++)
    // {
    //     if (rightLane[i] != null)
    //     {
    //         circle(lane, rightLane[i], 1, Scalar(255, 0, 0), 2, 8, 0);
    //     }
    // }

    imshow("view", src);
    // drawLanes(src);
}

Mat DetectLane::preProcess(const Mat &src)
{
    Mat imgHSV, dst;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    // imshow("hsv", imgHSV);

    inRange(imgHSV, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
            Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]),
            imgThresholded);

    Mat imgThresholdedShadow = laneInShadow(src);

    Mat fullImgThresholded;

    // imshow("normal threshold", imgThresholded);
    // imshow("shadow threshold", imgThresholdedShadow);

    // bitwise_or to to get lanes in normal situation and shadow situation
    bitwise_or(imgThresholded, imgThresholdedShadow, fullImgThresholded);

    imshow("threshold", fullImgThresholded);

    // dst = birdViewTranform(imgThresholded);
    dst = birdViewTranform(fullImgThresholded);

    // imshow("Bird View", dst);

    fillLane(dst);

    return dst;
}

Mat DetectLane::laneInShadow(const Mat &src)
{
    Mat shadowMask, shadow, imgHSV, shadowHSV, laneShadow;
    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(minShadowTh[0], minShadowTh[1], minShadowTh[2]),
            Scalar(maxShadowTh[0], maxShadowTh[1], maxShadowTh[2]),
            shadowMask);

    src.copyTo(shadow, shadowMask);

    cvtColor(shadow, shadowHSV, COLOR_BGR2HSV);

    inRange(shadowHSV, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]),
            Scalar(maxLaneInShadow[0], maxLaneInShadow[1], maxLaneInShadow[2]),
            laneShadow);

    // imshow("lane shadow", laneShadow);

    return laneShadow;
}

void DetectLane::fillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI / 180, 1);
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
    }
}

vector<Mat> DetectLane::splitLayer(const Mat &src, int dir)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;

    if (dir == VERTICAL)
    {
        for (int i = 0; i < rowN - slideThickness; i += slideThickness)
        {
            Mat tmp;
            Rect crop(0, i, colN, slideThickness);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    else
    {
        for (int i = 0; i < colN - slideThickness; i += slideThickness)
        {
            Mat tmp;
            Rect crop(i, 0, slideThickness, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }

    return res;
}

vector<vector<Point>> DetectLane::centerRoadSide(const vector<Mat> &src, int dir)
{
    vector<std::vector<Point>> res;
    int inputN = src.size();
    for (int i = 0; i < inputN; i++)
    {
        std::vector<std::vector<Point>> cnts;
        std::vector<Point> tmp;
        findContours(src[i], cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0)
        {
            res.push_back(tmp);
            continue;
        }

        for (int j = 0; j < cntsN; j++)
        {
            int area = contourArea(cnts[j], false);
            if (area > 3)
            {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float>(M1.m10 / M1.m00), static_cast<float>(M1.m01 / M1.m00));
                if (dir == VERTICAL)
                {
                    center1.y = center1.y + slideThickness * i;
                }
                else
                {
                    center1.x = center1.x + slideThickness * i;
                }
                if (center1.x > 0 && center1.y > 0)
                {
                    tmp.push_back(center1);
                }
            }
        }
        res.push_back(tmp);
    }

    return res;
}

void DetectLane::detectLeftRight(const vector<vector<Point>> &points)
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();

    leftLane.clear();
    rightLane.clear();
    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i++)
    {
        leftLane.push_back(null);
        rightLane.push_back(null);
    }

    int pointMap[points.size()][20];
    int prePoint[points.size()][20];
    int postPoint[points.size()][20];
    int dis = 10;
    int max = -1, max2 = -1;
    Point2i posMax, posMax2;

    memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }

    for (int i = points.size() - 2; i >= 0; i--)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            int err = 320;
            for (int m = 1; m < min(points.size() - 1 - i, 5); m++)
            {
                bool check = false;
                for (int k = 0; k < points[i + 1].size(); k++)
                {
                    if (abs(points[i + m][k].x - points[i][j].x) < dis &&
                        abs(points[i + m][k].x - points[i][j].x) < err)
                    {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        postPoint[i + m][k] = j;
                        check = true;
                    }
                }
                break;
            }

            if (pointMap[i][j] > max)
            {
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax.x || j != posMax.y) && postPoint[i][j] == -1)
            {
                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }

    if (max == -1)
        return;

    while (max >= 1)
    {
        lane1.push_back(points[posMax.x][posMax.y]);
        if (max == 1)
            break;

        posMax.y = prePoint[posMax.x][posMax.y];
        posMax.x += 1;

        max--;
    }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1)
            break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;

        max2--;
    }

    vector<Point> subLane1(lane1.begin(), lane1.begin() + 5);
    vector<Point> subLane2(lane2.begin(), lane2.begin() + 5);

    Vec4f line1, line2;

    fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);

    int lane1X = (BIRDVIEW_WIDTH - line1[3]) * line1[0] / line1[1] + line1[2];
    int lane2X = (BIRDVIEW_WIDTH - line2[3]) * line2[0] / line2[1] + line2[2];

    if (lane1X < lane2X)
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
        for (int i = 0; i < lane2.size(); i++)
        {
            rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
    }
    else
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
        for (int i = 0; i < lane1.size(); i++)
        {
            rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
    }
}

Mat DetectLane::morphological(const Mat &img)
{
    Mat dst;

    // erode(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );
    // dilate( dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );

    dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)));
    erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)));

    // blur(dst, dst, Size(3, 3));

    return dst;
}

void transform(Point2f *src_vertices, Point2f *dst_vertices, Mat &src, Mat &dst)
{
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

Mat DetectLane::birdViewTranform(const Mat &src)
{
    Point2f src_vertices[4];

    int width = src.size().width;
    int height = src.size().height;

    src_vertices[0] = Point(0, skyLine);
    src_vertices[1] = Point(width, skyLine);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - 105, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(105, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    imshow("bird view", dst);

    return dst;
}

SIGN_TYPE DetectLane::getTrafficSign(const Mat &src)
{
    SIGN_TYPE type = NONE;
    Mat imgHSV, dst, trafficSignImgThresholded;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV,
            // Scalar(85, 100, 100),
            // Scalar(135, 255, 255),
            Scalar(85, 100, 100),
            Scalar(105, 255, 255),
            trafficSignImgThresholded);

    imshow("traffic sign thresholded", trafficSignImgThresholded);

    std::vector<std::vector<Point>> cnts;
    findContours(trafficSignImgThresholded, cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    int cntsN = cnts.size();

    if (cntsN > 0)
    {
        vector<Point> sign_elem = *max_element(cnts.begin(), cnts.end(), [](vector<Point> a, vector<Point> b) {
            return a.size() > b.size();
        });
        // 7 - 12 frame
        // vector<Point> sign_elem;
        // for (int idx = 0; idx < cntsN; idx++)
        // {
        //     if (isSign(trafficSignImgThresholded, cnts[idx])) {
        //         sign_elem = cnts[idx];
        //     }
        // }
        // vector<Point> sign_elem = cnts[idx];
        if (sign_elem.empty()) return NONE;
        if (sign_elem.size() > MIN_CMP_VAL)
        {
            Rect bounding_box = boundingRect(Mat(sign_elem));
            unsigned long p_counter_left = 0;
            unsigned long p_counter_right = 0;

            // if (bounding_box.height * 1.0 / bounding_box.width > 1.1 || bounding_box.width * 1.0 / bounding_box.height > 1.1) {
            //     return NONE;
            // }

            unsigned long h = bounding_box.height >> 1, w = bounding_box.width >> 1;
            proportion = (bounding_box.height * bounding_box.width * 1.0) / (src.rows * src.cols);
            for (int p_i = 0; p_i < h; p_i++)
            {
                for (int p_j = 0; p_j < bounding_box.width; p_j++)
                {
                    uchar pixel = trafficSignImgThresholded.at<uchar>(bounding_box.y + p_i, bounding_box.x + p_j);
                    if (p_j < w)
                    {
                        p_counter_left += (pixel > COLOR_THRESHOLD);
                    }
                    else
                    {
                        p_counter_right += (pixel > COLOR_THRESHOLD);
                    }
                }
            }
            if (p_counter_left < p_counter_right) type = LEFT;
            else if (p_counter_left > p_counter_right) type = RIGHT;
            else type = NONE;
        }
    }
    return type;
}

float DetectLane::getSignProportion()
{
    return proportion;
}

bool DetectLane::isSign(Mat img, vector<Point> elem)
{
    Rect bounding_box = boundingRect(Mat(elem));
    if (bounding_box.height * 1.0 / bounding_box.width > 1.1 || bounding_box.width * 1.0 / bounding_box.height > 1.1) return false;
    float rate = 0.2;
    int h_size = ceil(bounding_box.height * rate);
    int w_size = ceil(bounding_box.width * rate);
    for (int y = 0; y < h_size; y++) {
        for (int x = 0; x < w_size; x++) {
            if (
                // TL
                img.at<uchar>(bounding_box.y + y, bounding_box.x + x) < 5 ||
                // TR
                img.at<uchar>(bounding_box.y + (bounding_box.height - h_size + y + 1), bounding_box.x + x) < 5 ||
                // BL
                img.at<uchar>(bounding_box.y, bounding_box.x + (bounding_box.width - w_size + x + 1)) < 5 ||
                // BR
                img.at<uchar>(bounding_box.y + (bounding_box.height - h_size + y + 1), bounding_box.x + (bounding_box.width - w_size + x + 1)) < 5
            ) return false;
        }
    }
    return true;
}