// mvector.cpp
// OWL - Raspberry Pi based autopilot for iRobot Roomba
// 
//
// rev 1.0 - 2016.04.20
//    - initial version
//
// boredman@boredomprojects.net

#include "mvector.h"

using namespace cv;
using namespace std;

MotionVec::MotionVec()
{
    speed = 0;
    angle = 0;
    _valid = false;
}

MotionVec::MotionVec(Point ptStart, Point ptStop, float vel/*=0*/)
{
    speed = vel;
    angle = Angle(ptStart, ptStop);
    _valid = true;
}

MotionVec::~MotionVec()
{
}


void MotionVec::CalcAngle(Point ptStart, Point ptStop)
{
    angle = Angle(ptStart, ptStop);
    _valid = true;
}


float MotionVec::Angle(Point ptStart, Point ptStop)
{
    Point diff = ptStop - ptStart;
    if( (diff.x == 0) && (diff.y == 0) )
        return 0;
    else
        return atan2f(diff.y, diff.x);  // in [rad]  !!! why not atan2f(x,y) ???
}


bool MotionVec::valid()
{
    return _valid;
}


void MotionVec::invalidate()
{
    _valid = false;
}
