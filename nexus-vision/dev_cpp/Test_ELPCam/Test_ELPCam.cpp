#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, const char* argv[])
{
    // class represent for capturing video from camera and reading video file or image sequences
    VideoCapture videoCapture;
    Mat videoFrame;

    // open camera
    videoCapture.open(0);
    namedWindow("VideoCapture", WINDOW_AUTOSIZE);

    // check open camera open sucessed or failed
    if (!videoCapture.isOpened())
    {
        cout << "Can't open camera" << endl;
    }
    else
    {
        while (true)
        {
            //read video frame from camera and show in windows
            videoCapture.read(videoFrame);
            imshow("VideoCapture", videoFrame);
            if (waitKey(30) >= 0) break;
        }
    }
    return 0;
}