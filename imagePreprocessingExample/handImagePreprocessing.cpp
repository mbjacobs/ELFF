#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

//int captureHandImage (VideoStream sVideoStream, Mat sFrame);

int main (int argc, char **argv)
{
    const char QUIT = 113;
    const string FILE_NAME = "peace.jpg";
    const int MAX_THRESHOLD_VAL = 255;
    const int THRESHOLD_VAL = 100;
    VideoCapture sVideoStream (0);
    Mat sFrame, sImage, sImageGreyscale, sImageBlurred, sImageThresholded;
    Mat sImageCannyEdge, sDrawContours;
    vector <vector <Point>> vContours;
    vector <Vec4i> vHierarchy; //NOTE: look up what Vec4i is.
    RNG rng (12345); //NOTE: What is this??

    if (!sVideoStream.isOpened())
    {
        perror ("Error opening video stream: ");
    
    }

    while (1)
    {   
        //Store the latest image from the videa stream as a camera frame.
        //sVideoStream.read (sCameraFrame);
        sVideoStream >> sFrame;

        //Exit on empty frame.
        if (sFrame.empty ())
        {
            perror ("Empty camera frame: ");
            break;
        }

        //Display the frame to the screen as an image.
        imshow ("Current frame:", sFrame);

        //Pause the loop for a short amount of time before next iteration.
        //Wait for specified time, return ASCII value of key pressed during wait time.
        if (QUIT == waitKey (30))
        {   
            //Write the current frame to an image file.
            imwrite (FILE_NAME, sFrame);
                
            break;
        }
    }   

    sVideoStream.release ();
    
    sImage = imread (FILE_NAME, 1);

    //Image pre-procesessing: binarize the image in three steps.
    //1. Greyscale
    //2. Blur
    //3. Threshold

    //1. Convert image to grey, saving it as a new mat object.
    //Save new image as a jpg for demo purposes.
    cvtColor (sImage, sImageGreyscale, CV_BGR2GRAY);
    imwrite ("peace_greyscale.jpg", sImageGreyscale);

    //2. Blur the greyscale image to remove noise.
    //Save the new image as a jpg for demo purposes.
    blur (sImageGreyscale, sImageBlurred, Size(3,3));
    imwrite ("peace_greyscale_blur.jpg", sImageBlurred);

    //3. Threshold the blurred, greyscale image, saving it as a new mat object.
    //Save the new image as a jpg for demo purposes.
    
    //double threshold (InputArray src, OutputArray dst, double thresh, double maxval, int type)
    threshold (sImageBlurred, sImageThresholded, THRESHOLD_VAL, MAX_THRESHOLD_VAL, THRESH_BINARY);
    imwrite ("peace_greyscale_blur_threshold_binary.jpg", sImageThresholded);

    threshold (sImageBlurred, sImageThresholded, THRESHOLD_VAL, MAX_THRESHOLD_VAL, THRESH_BINARY_INV);
    imwrite ("peace_greyscale_blur_threshold_binary_inv.jpg", sImageThresholded);

    threshold (sImageBlurred, sImageThresholded, THRESHOLD_VAL, MAX_THRESHOLD_VAL, THRESH_TRUNC);
    imwrite ("peace_greyscale_blur_threshold_trunc.jpg", sImageThresholded);

    threshold (sImageBlurred, sImageThresholded, THRESHOLD_VAL, MAX_THRESHOLD_VAL, THRESH_TOZERO);
    imwrite ("peace_greyscale_blur_threshold_tozero.jpg", sImageThresholded);

    threshold (sImageBlurred, sImageThresholded, THRESHOLD_VAL, MAX_THRESHOLD_VAL, THRESH_TOZERO_INV);
    imwrite ("peace_greyscale_blur_threshold_tozero_inv.jpg", sImageThresholded);


    //Find contours of the greyscale image and one thresholded image.
    Canny (sImageGreyscale, sImageCannyEdge, THRESHOLD_VAL, THRESHOLD_VAL * 2, 3);
    imwrite ("peace_greyscale_canny_edge.jpg", sImageCannyEdge);

    findContours (sImageCannyEdge, vContours, vHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point (0,0));
    sDrawContours = Mat::zeros (sImageCannyEdge.size(), CV_8UC3); //NOTE: second parameter??
    imwrite ("peace_greyscale_draw_contours.jpg", sDrawContours);

 //   for( int i = 0; i < vContours.size(); i++ )
   // {
     // Scalar color = Scalar  (rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)); //NOTE: This stuff is still magic. Make them constants.
     // drawContours (sDrawContours, vContours, i, color, 2, 8, vHierarchy, 0, Point() );
   // }                            }

    return EXIT_SUCCESS;
}
