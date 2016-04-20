// Roomba.cpp
// OWL - Raspberry Pi based autopilot for iRobot Roomba
// 
//
// rev 1.0 - 2016.04.20
//    - initial version
//
// boredman@boredomprojects.net

#include "mvector.h"
#include "roomba.h"

using namespace cv;
using namespace std;


Roomba::Roomba()
{
    fd = -1;    // invalid file descriptor

    invalidate(location);
    motion_vector.invalidate();
}


Roomba::~Roomba()
{
    close(fd);
}



/***************************************************************************
 * SetSearchArea()
 *   configures area where to search for Roomba
 ***************************************************************************/
void Roomba::SetSearchArea(vector<Point> contour)
{
    perimeter = contour;
}



/***************************************************************************
 * LocateIn()
 *   attempts to detect Roomba in image
 * returns:
 *   integer representing confidence level of successful detection
 *   0 if roomba not found
 ***************************************************************************/
int Roomba::LocateIn(Mat image)
{
    int status = 0;

    drawing = image;

    // find shape circle
    vector<Point> circles = FindShapeCircles(image);

//  // find large blobs
//  vector<Point> large_keypoints = FindLargeBlobs(image);
//  large_keypoints = ValidateKeypoints(large_keypoints, perimeter);

    // find small blobs
    vector<Point> small_keypoints = FindSmallBlobs(image);
    small_keypoints = ValidateKeypoints(small_keypoints, perimeter);
    small_keypoints = RangeKeypoints(small_keypoints);


    // combine both arrays
    vector<Point> keypoints = circles;
    keypoints.insert(keypoints.end(), small_keypoints.begin(), small_keypoints.end());

    // archive previous location
    SaveOldLocation();

    // temporary storage
    Point2f new_location;

#ifdef DEBUG_VIDEO
    vector<vector<Point> > perimeterVec;
    perimeterVec.push_back(perimeter);
    drawContours(drawing, perimeterVec, 0, Scalar(200,0,0));
    polylines(image, motionPath, false, Scalar(100,0,0), 1);
#endif

    if( small_keypoints.size() == 0 )
    {
        invalidate(location);
        return 0;   // not found
    }
    else
    {
        // find center of it all
        minEnclosingCircle(keypoints, new_location, location_radius);
        if( location_radius < 60 )
        {
            // set new location
            location = new_location;
            // return confidence level
            return circles.size() * 100 + small_keypoints.size();
        }
        else
        {
            invalidate(location);
            return 0;   // not found
        }
    }
}



/***************************************************************************
 * SaveOldLocation()
 *   pushes current location onto history stack
 ***************************************************************************/
void Roomba::SaveOldLocation()
{
    locationHistory.push_front(location);

    if( locationHistory.size() > 10 )  // max 10 element size reached
        locationHistory.pop_back();
}


/***************************************************************************
 * UpdateMotionVec()
 *   calculates current motion vector by taking current location and
 *   the latest valid point in history.
 * returns: 0 if success, -1 otherwise 
 * (depth not implemented yet)
 ***************************************************************************/
int Roomba::UpdateMotionVec(int depth/*=0*/)
{
    deque<Point>::iterator i;

    // find first valid location in locationHistory
    for( i=locationHistory.begin(); i!=locationHistory.end(); i++ )
    {
        if( valid(*i) )
            break;
    }

    if( valid(location) && i!=locationHistory.end() )
    {
        motion_vector.CalcAngle( *i, location );
        return 0;   // success
    }
    else
    {
        motion_vector.invalidate();
        return -1;  // nothing in history
    }
}


/***************************************************************************
 * SetMotionPath()
 *   configures path for Roomba to follow
 *   sets motionItr to the first point in that path
 *
 * commented section attempted to turn on Roomba lights, but it didn't work
 ***************************************************************************/
void Roomba::SetMotionPath(vector<Point> path)
{
    motionPath = path;
    motionItr = motionPath.begin();
/*
    unsigned char buf[10];

    buf[0] = 139;   // LEDs
    buf[1] = 0x0E;
    buf[2] = 255;
    buf[3] = 255;
    generalCmd(buf,4);
    usleep(1000);

    buf[1] = 162;   // Scheduling LEDs
    buf[2] = 0x7F;
    buf[3] = 0x1F;
    generalCmd(buf,3);
    usleep(1000);
*/
}


/***************************************************************************
 * FollowPath()
 *   starts action for Roomba to follow path and manages path progress.
 *   returns immediately after initiating next action,
 *   therefore it must be called repeatedly.
 * returns:
 *   -1 = error
 *    0 = arrive to the last point, path completed.
 * 1..n = path in progress, returns index to path segment + 1
 ***************************************************************************/
int Roomba::FollowPath()
{
    int status = GotoPoint(*motionItr);

    if( status < 0 )
        return status;  // error

    if( status == 0 )   // arrived to this point
    {
        // set next point and check for the end
        if( ++motionItr == motionPath.end() )
            return 0;   // path completed
    }

    // still going, return index + 1
    return ( motionItr - motionPath.begin() + 1 );
}


/***************************************************************************
 * GotoPoint()
 *   prepare and execute maneuver to drive to a specific location.
 *   checks whether previous command has completed, before issuing the next.
 *
 * returns:
 *  -1 = error
 *   0 = arrived
 *   1 = in progress, waiting for valid location
 *   2 = in progress, still executing previous maneuver
 *   3 = in progress, just started new maneuver
 ***************************************************************************/
int Roomba::GotoPoint(Point destination)
{
    const int velocity = 100;   // [mm/s]
    const int duration = 500;   // [ms]

    if( !valid(location) || !motion_vector.valid() )
        return 1;   // Roomba not found, waiting...

    // are we there yet?
    if( Distance(location, destination) < 30 )
        return 0;   // Roomba has reached this destination point

    // check status of previous maneuver
    int status = driveCmdStatus();
    if( status < 0 )
        return status;  // error
    if( status > 0 )
        return 2;       // still executing previous maneuver

    // *** now we are ready for new maneuver ***

    // here we should've used "motion_vector", however, currently is gives inaccurate results,
    // since averaging across several old locations in history is not yet implemented.
    // therefore, for now just make a temporary "old_location" to hold position since last maneuver.
    static Point old_location;
    MotionVec old_vector(old_location, location);

    // prepare new maneuver
    MotionVec new_vector(location, destination, velocity);
    int vel_duration = 0;
    int rad_duration = duration;
    float radius = 3 * TurnRadius( (old_vector.angle-new_vector.angle), new_vector.speed, rad_duration );

#ifdef DEBUG_DUMP
    printf("  ==> old:%d,%d; here:%d,%d; dest:%d,%d;\n", old_location.x, old_location.y, location.x, location.y, destination.x, destination.y);
    printf("  ==> vec:%f; new:%f; dif:%f\n", old_vector.angle, new_vector.angle, (old_vector.angle-new_vector.angle));
    printf("  ==> rad:%f\n", radius);
#endif

#ifdef DEBUG_VIDEO
    circle(drawing, location, 5, Scalar(255,0,0), 2, 8);
    circle(drawing, destination, 8, Scalar(127,0,0), 2, 8);
    arrowedLine( drawing, old_location, location, Scalar(0,0,0), 1, 8 );
    arrowedLine( drawing, location, destination, Scalar(0,0,0), 1, 8 );
#endif

    // save current location for future
    old_location = location;

/*
printf("==> pt:%d,%d; dest:%d,%d;\n", location.x, location.y, destination.x, destination.y);
printf("==> vec:%f; new:%f; dif:%f\n", motion_vector.angle, new_vector.angle, (motion_vector.angle-new_vector.angle));
printf("==> rad:%f   (enter?)\n", radius);
cin.get();
*/

    // execute new maneuver
    status = driveCmd(cvRound(velocity), cvRound(radius), vel_duration, rad_duration);
    if( status < 0 )
        return status;  // error

    return 3;   // maneuver in progress
}


/***************************************************************************
 * TurnRadius()
 *   calculates turn radius, as required by roomba drive command.
 *   the longer radius makes Roomba drive straighter, while the shorter radius
 *   makes Roomba turn more.
 * parameters:
 *   angle - required turn angle in [radians]
 *   mm_s  - average forward speed in [mm/s]
 *   ms    - time allocated for this turn in [ms]
 * returns:
 *   turn radius in [mm]
 ***************************************************************************/
float Roomba::TurnRadius(float angle, float mm_s, float ms)
{
    const float  PI_F=3.14159265358979f;

    float adj_angle;

    if ( angle > PI_F )
        adj_angle = angle - 2 * PI_F;
    else if ( angle < -PI_F )
        adj_angle = 2 * PI_F + angle;
    else
        adj_angle = angle;

    return ( mm_s * (ms / 1000.0f) / adj_angle );       // in [mm]
}



bool Roomba::valid(Point loc)
{
    return !((loc.x == 10000) || (loc.y == 10000));
}


void Roomba::invalidate(Point &loc)
{
    loc = Point(10000, 10000);
}




//////////////////////////////////////////////////////////////////////////////////
// The following set of methods implement radio communication with Roomba
//////////////////////////////////////////////////////////////////////////////////


/***************************************************************************
 * configPort()
 *   prepares serial port for communication
 * parameter:
 *   portName - string, such as "/dev/ttyAMA0"
 * returns:
 *   0 - success
 *  -1 - error
 ***************************************************************************/
int Roomba::configPort(const char* portName)
{
    fd = open (portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if( fd == -1 )
        return -1;

    fcntl(fd, F_SETFL, 0);

    // options:
    struct termios options;
    int status;

    tcgetattr(fd, &options);

    // set "raw" mode: no echo, no special control characters, no parity
    cfmakeraw(&options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);    // Rx enable, no modem control
    options.c_cflag &= ~PARENB;    // no parity
    options.c_cflag &= ~CSTOPB;    // one stop bit
    options.c_cflag |= CS8;        // 8 data bits

    options.c_cc[VMIN]  =  0;
    options.c_cc[VTIME] = 10;    // One second general timeout (10 deciseconds)

    tcsetattr(fd, TCSANOW | TCSAFLUSH, &options);

    return 0;
}


/***************************************************************************
 * driveCmd()
 *   executes roomba Drive Command (opcode: 137)
 *   waits for the acknowledge, after sending the command
 * returns:
 *   0 - success
 * -20 - error during transmission of command
 * -21 - error during reception of acknowledge
 * -22 - acknowledge not received and retrys (def=3) are exhausted
 ***************************************************************************/
int Roomba::driveCmd(int velocity, int radius, int vel_duration, int rad_duration, int ntry/*=3*/)
{
    unsigned char buf[10];

    if( velocity < -500 )
        velocity = -500;
    if( velocity > 500 )
        velocity = 500;
    if( radius < -2000 )
        radius = -2000;
    if( radius > 2000 )
        radius = 2000;

    buf[0] = '~';
    buf[1] = (velocity>>8) & 0xFF;
    buf[2] = velocity & 0xFF;
    buf[3] = (radius>>8) & 0xFF;
    buf[4] = radius & 0xFF;
    buf[5] = (vel_duration>>8) & 0xFF;
    buf[6] = vel_duration & 0xFF;
    buf[7] = (rad_duration>>8) & 0xFF;
    buf[8] = rad_duration & 0xFF;

    unsigned char c = 0;

    while( c != '!' )
    {   // send command
        if( transmit(buf,9) != 0 )
            return -20;

        // get local ack
        if( receive(&c,1) != 0 )
            return -21;

        if( ntry-- == 0 )
            return -22;
    }

#ifdef DEBUG_DUMP
    printf("  (v=%d,r=%d,vd=%d,rd=%d)\n",velocity,radius,vel_duration,rad_duration);
#endif

    return 0;
}


/***************************************************************************
 * driveCmdStatus()
 *   sends a request to get status of currently executed maneuver
 * returns:
 *   0 - last maneuver completed
 *   1 - maneuver in progress
 * -20 - error during transmission of command
 * -21 - error during reception of acknowledge
 * -22 - acknowledge not received and retrys (def=3) are exhausted
 * -23 - error during reception of status
 ***************************************************************************/
int Roomba::driveCmdStatus(int ntry/*=3*/)
{
    unsigned char buf[5];
    unsigned char c = 0;

    while( c != '!' )
    {   // send status request command
        c = '#';
        if( transmit(&c,1) != 0 )
            return -20;

        // get local ack
        if( receive(&c,1) != 0 )
            return -21;

        if( ntry-- == 0 )
            return -22;
    }

    // read remote status
    if( receive(&c,1) != 0 )
        return -23;

#ifdef DEBUG_DUMP
    printf("  [cmd.status=%d]\n", c);
#endif

    if( c == 0 )
        return 0;   // maneuver completed
    else
        return 1;   // still going
}



/***************************************************************************
 * generalCmd()
 *   send general command to Roomba
 ***************************************************************************/
int Roomba::generalCmd(unsigned char *data, int len, int ntry/*=3*/)
{
    unsigned char buf[2];

    buf[0] = '$';
    buf[1] = len;

    unsigned char c = 0;

    while( c != '!' )
    {
        // send out command
        if( transmit(buf,2) != 0 )
            return -1;
        if( transmit(data,len) != 0 )
            return -2;

        // get confirmation
        if( receive(&c,1) != 0 )
            return -3;

        if( ntry-- == 0 )
            return -4;
    }

    return 0;
}


/***************************************************************************
 * transmits data to serial port
 * returns:  0 - if all bytes have been sent out
 *          >0 - if not all bytes were sent out and retries were exhausted
 *          <0 - if error occured
 ***************************************************************************/
int Roomba::transmit(unsigned char *data, int size, int ntry/*=1*/)
{
    int i, n;

    for(i=0; i<size && ntry; i+=n)
    {
        n = write(fd, data+i, size-i);
        if( n < 0 )
            return -1;
        if( n == 0 )
            ntry--;
    }
    return (size-i);
}


/***************************************************************************
 * receives data from serial port
 * returns:  0 - if all bytes have been received
 *          >0 - if not all bytes were received and retries were exhausted
 *          <0 - if error occured
 ***************************************************************************/
int Roomba::receive(unsigned char *data, int size, int ntry/*=1*/)
{
    int i, n;

    for(i=0; i<size && ntry; i+=n)
    {
        n = read(fd, data+i, size-i);
        if( n < 0 )
            return -1;
        if( n == 0 )
            ntry--;
    }
    return (size-i);
}





//////////////////////////////////////////////////////////////////////////////////
// The following set of methods deal with recognizing and locating Roomba in frame
//////////////////////////////////////////////////////////////////////////////////



/***************************************************************************
 * returns *first* circle found by HoughCircles
 * the first one most probably represents Roomba's shape
 ***************************************************************************/
vector<Point> Roomba::FindShapeCircles(Mat im)
{
    vector<Vec3f> circles;
    double db = 1;
    double minDist = 50;
    double param1 = 100;
    double param2 = 10;
    int minRadius = 18;
    int maxRadius = 38;

    HoughCircles( im, circles, CV_HOUGH_GRADIENT, db, minDist, param1, param2, minRadius, maxRadius );

    // convert to points
    vector<Point> points;

    // take only first point, if exists
    if( circles.size() != 0 )
        points.push_back( Point(cvRound(circles[0][0]), cvRound(circles[0][1])) );

    //for( size_t i = 0; i < circles.size(); i++ )
    //{
    //  points.push_back( Point(cvRound(circles[i][0]), cvRound(circles[i][1])) );
    //}

    return points;
}


/***************************************************************************
 * returns blobs representing Roomba's shape
 * *unreliable*
 ***************************************************************************/
vector<Point> Roomba::FindLargeBlobs(Mat imIn)
{
    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 50;
    params.maxThreshold = 200;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 2000;
    params.maxArea = 4000;

    params.minDistBetweenBlobs = 1000;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.3f;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.8f;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01f;

    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // Detect blobs
    vector<KeyPoint> keypoints;
    detector->detect( imIn, keypoints);

    vector<Point> points;
    for(vector<KeyPoint>::iterator i = keypoints.begin(); i != keypoints.end(); i++)
    {
        points.push_back(i->pt);
    }

    return points;
}


/***************************************************************************
 * returns blobs representing Roomba's IR sensor lights
 *
 ***************************************************************************/
vector<Point> Roomba::FindSmallBlobs(Mat imIn)
{
    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 50;
    params.maxThreshold = 200;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 10;
    params.maxArea = 100;

    params.minDistBetweenBlobs = 5;

    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.2f;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.8f;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.1f;

    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // invert image to find bright spots
    Mat imOut;
    bitwise_not(imIn, imOut);

    // Detect blobs
    vector<KeyPoint> keypoints;
    detector->detect( imOut, keypoints);

    vector<Point> points;
    for(vector<KeyPoint>::iterator i = keypoints.begin(); i != keypoints.end(); i++)
    {
        points.push_back(i->pt);
    }

    return points;
}



/***************************************************************************
 * checks whether keypoints are inside the contour
 ***************************************************************************/
vector<Point> Roomba::ValidateKeypoints(vector<Point> keypoints, vector<Point> contour)
{
    vector<Point> valid_keypoints;

    for(vector<Point>::iterator i = keypoints.begin(); i != keypoints.end(); i++)
    {
        if( pointPolygonTest(contour, *i, false) >= 0 )
            valid_keypoints.push_back(*i);
    }

    return valid_keypoints;
}



/***************************************************************************
 * groups keypoints by calculating their relative distance from each other.
 * returns only keypoints that are close to their neightbours. 
 ***************************************************************************/
vector<Point> Roomba::RangeKeypoints(vector<Point> keypoints)
{
    float max_distance = 60;
    float min;
    vector<Point> good_keypoints;

    if( keypoints.size() <= 2 )
        return keypoints;

    for( vector<Point>::iterator i = keypoints.begin(); i != keypoints.end(); i++ )
    {
        min = 1000000;
        for( vector<Point>::iterator j = keypoints.begin(); j != keypoints.end(); j++ )
        {
            // skip if same point
            if( j == i )
                continue;
            // calculate distance
            float dist = Distance(*i, *j);
            // save minimum
            if( dist < min )
                min = dist;
        }
        if( min < max_distance )
            good_keypoints.push_back(*i);
    }

    return good_keypoints;
}



/***************************************************************************
 * Calculates pythagorean distance petween two points
 ***************************************************************************/
float Roomba::Distance(Point p1, Point p2)
{
    Point diff = p1 - p2;
    return sqrt( fabs( float(diff.x*diff.x + diff.y*diff.y) ) );
}


