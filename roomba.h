// Roomba.h
// OWL - Raspberry Pi based autopilot for iRobot Roomba
// 
//
// rev 1.0 - 2016.04.20
//    - initial version
//
// boredman@boredomprojects.net

#include <stdint.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <stdlib.h>
#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>

//#include "mvector.h"


#define DEBUG_DUMP

//#define DEBUG_VIDEO


class Roomba
{
private:
    std::vector<cv::Point> perimeter;

    std::vector<cv::Point> FindSmallBlobs(cv::Mat imIn);
    std::vector<cv::Point> FindLargeBlobs(cv::Mat imIn);
    std::vector<cv::Point> ValidateKeypoints(std::vector<cv::Point> keypoints, std::vector<cv::Point> contour);
    std::vector<cv::Point> RangeKeypoints(std::vector<cv::Point> keypoints);

    std::vector<cv::Point> FindShapeCircles(cv::Mat im);

    float Distance(cv::Point p1, cv::Point p2);

    std::vector<cv::Point> motionPath;
    std::vector<cv::Point>::iterator motionItr;

    std::deque<cv::Point> locationHistory;
    void SaveOldLocation(void);

    bool valid(cv::Point loc);
    void invalidate(cv::Point &loc);

    // file descriptor of the serial port
    int fd;

cv::Mat drawing;

public:
    cv::Point location;
    float location_radius;
    MotionVec motion_vector;

    Roomba(void);
    ~Roomba(void);

    void SetSearchArea(std::vector<cv::Point> contour);
    int LocateIn(cv::Mat image);

    int UpdateMotionVec(int depth=0);

    void SetMotionPath(std::vector<cv::Point> path);
    int FollowPath(void);
    int GotoPoint(cv::Point destination);
    float TurnRadius(float angle, float mm_s, float ms);

    int configPort(const char* portName);
    int driveCmd(int velocity, int radius, int vel_duration, int rad_duration, int ntry=3);
    int driveCmdStatus(int ntry=3);
    int generalCmd(unsigned char *data, int len, int ntry=3);
    int transmit(unsigned char *data, int size, int ntry=1);
    int receive(unsigned char *data, int size, int ntry=1);
};
