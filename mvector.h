// mvector.h
// OWL - Raspberry Pi based autopilot for iRobot Roomba
// 
//
// rev 1.0 - 2016.04.20
//    - initial version
//
// boredman@boredomprojects.net

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

class MotionVec
{
private:
    float Angle(cv::Point ptStart, cv::Point ptStop);

    bool _valid;

public:
    float speed;
    float angle;

    MotionVec(void);
    MotionVec(cv::Point ptStart, cv::Point ptStop, float vel=0);
    ~MotionVec(void);

    void CalcAngle(cv::Point ptStart, cv::Point ptStop);
    bool valid(void);
    void invalidate(void);
};


