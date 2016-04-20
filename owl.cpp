// Owl.cpp
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



#define OWL_FRM_WIDTH  325
#define OWL_FRM_HEIGHT 775

Mat CorrectPerspective(Mat input)
{
    Point2f inputQuad[4], outputQuad[4];
    inputQuad[0] = Point2f(228,240);
    inputQuad[1] = Point2f(422,249);
    inputQuad[2] = Point2f(221,442);
    inputQuad[3] = Point2f(500,441);
    outputQuad[0] = Point2f(50,500);
    outputQuad[1] = Point2f(250,500);
    outputQuad[2] = Point2f(50,750);
    outputQuad[3] = Point2f(250,750);
    Mat lambda = getPerspectiveTransform( inputQuad, outputQuad );

    Mat output = Mat::zeros( OWL_FRM_HEIGHT, OWL_FRM_WIDTH, input.type() );
    warpPerspective(input,output,lambda,output.size(),INTER_LINEAR);

    return output;
}


int main(int argc, char* argv[])
{
    VideoCapture cap(0); // open the video camera no. 0

    if( ! cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    //cap.set(CV_CAP_PROP_FRAME_WIDTH,160);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT,120);
    cap.set(CV_CAP_PROP_FPS,10);

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    double dFps = cap.get(CV_CAP_PROP_FPS);
    cout << "Frame size : " << dWidth << " x " << dHeight << " - " << dFps << "fps" << endl;

    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

#ifdef DEBUG_VIDEO
    VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'), 3, Size(OWL_FRM_WIDTH,OWL_FRM_HEIGHT), true); // B/W does not work here!
    Mat video_frame;
#endif

    Roomba roomba;

    if( roomba.configPort("/dev/ttyAMA0") == -1 )
    {
        cout << "Cannot open port" << endl;
        return -1;
    }

    // build contour for search area
    vector<Point> contour;
    contour.push_back(Point2f(2, 30));
    contour.push_back(Point2f(110, 30));
    contour.push_back(Point2f(140, 230));
    contour.push_back(Point2f(140, 500));
    contour.push_back(Point2f(190, 500));
    contour.push_back(Point2f(240, 300));
    contour.push_back(Point2f(320, 300));
    contour.push_back(Point2f(320, 440));
    contour.push_back(Point2f(310, 460));
    contour.push_back(Point2f(310, 510));
    contour.push_back(Point2f(270, 600));
    contour.push_back(Point2f(270, 770));
    contour.push_back(Point2f(2, 770));

    roomba.SetSearchArea(contour);

    double tf = getTickFrequency();
    double t0=0, t1, t2, t3, t4;

    // build path
    vector<Point> path;
//    path.push_back(Point2f(270, 370));
    path.push_back(Point2f(250, 470));
    path.push_back(Point2f(230, 570));
    path.push_back(Point2f(200, 630));
    path.push_back(Point2f(120, 630));
    path.push_back(Point2f(70, 575));
    path.push_back(Point2f(70, 265));
    path.push_back(Point2f(60, 150));

    if( argc > 1 )
        reverse(path.begin(), path.end());

    roomba.SetMotionPath(path);

    while(1)
    {
        Mat frame, gray;

        bool bSuccess = cap.read(frame); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        // execution time counter
        t1 = (double)getTickCount();

	    cvtColor(frame, gray, CV_BGR2GRAY);

        Mat warped = CorrectPerspective(gray);
        GaussianBlur( warped, warped, Size(3, 3), 0, 0 );

        // execution time counter
        t2 = (double)getTickCount();

        // get current location
        int loc_certainty = roomba.LocateIn(warped);

        // get current motion vector
        roomba.UpdateMotionVec();  //

        // let's go!
        int path_state = roomba.FollowPath();
        if( path_state < 0 )
        {
            cout << "error in FollowPath() " << path_state << endl;
            break;
        }
        else if( path_state == 0 )
        {
            cout << "path completed!" << endl;
            roomba.driveCmd(0,0,0,0);  // stop!
            break;
        }
        else
        {   // still going
            printf("-> point %d of %d \n", path_state, path.size());
        }

        // execution time counters
        t3 = (double)getTickCount();
        t4 = (double)getTickCount()-t0;
        t0 = (double)getTickCount();

#ifdef DEBUG_DUMP
        printf("t: %0.3f + %0.3f => %0.3f   ", (t2-t1)/tf, (t3-t2)/tf, t4/tf);
        printf("W=% 3d\n", loc_certainty);
#endif

#ifdef DEBUG_VIDEO
        cvtColor(warped, video_frame, CV_GRAY2BGR);
        video.write(video_frame);
        if( waitKey(30) == 27 )   // ESC key pressed
            break;                // needed for proper closure of AVI file
#endif
    }

    return 0;

}
