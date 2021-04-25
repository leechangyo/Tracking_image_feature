#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat()); //corner
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);

    // TODO: Your task is to locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.
    // extract the the strongest corners in a local neighborhood around each pixel.
    
    
    // iterately check keypoint with decriptor around them
    // step 1: response over threshold
    // step 2: overlap over threshold
    // step 3: on boverloop(if not, inset new keypoint from Keypoint list to compare)
    // step 4: compare response of nrmalized harris corners data (and if it bigger than in the list of keypoints )
    // step 5: if normolized harris detection value is whether greater than max overlap
    // step 5-1: if not iteratly overlap again with next keypoint from keypoints list
    // step 6: value is over threshold, compare response of nomorized harris detection value and current keypoint value in keypoints list.
    // step 6-1: if over, move to step 0, with new keypoint
    // step 6-2: if not check next keypoint
    // step 7: if boveroop false after end for loop, add newkeypoint to keypoints
    
    
    vector<cv::KeyPoint> Keypoints;
    double maxOverlap = 0.0; //max, permissible overlap between two features in %, used during non-maxima suppressio
    // check neighght pixel of corner pixel.
    for(size_t r = 0; r<dst_norm.rows; r++)
    {
        for(size_t c=0; c<dst_norm.cols;c++)
        {
            int response = (int)dst_norm.at<float>(r,c);// step 0 // corner response.
            
            if(response>minResponse) // step 1 over reponse threshold.
            {
                //store points above a threshold.
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(c,r);
                newKeyPoint.size = 2*apertureSize; //newkeypoint.size : diameter of the meaningful keypoint neighborhood(number of pixels to left /right and top/down to investigte
                // make double isze of sober operator(detect x,y coordinate)
                newKeyPoint.response = response; // see response(strength of keypoint)
                // aperture parameter for Sobel operator (must be odd)

                // perform non-maxinum suppression(NMS) in local neighbourhood around new key point
                bool Boverlap = false;
                // check picxel is local maximum.
                for(auto it = Keypoints.begin(); it!= Keypoints.end(); it++)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it); // the method computes overlap for pair of keypoints.
                    // Overlap is the ratio between area of keypoint regions'intersection and area of keypoint regions'union(considering keypoint region as circle)
                    // if they don't overlap, we get zero. if they coincide at same location with same size, we get 1.
                    if(kptOverlap > maxOverlap)
                    {
                        Boverlap = true;
                        if(newKeyPoint.response> (*it).response)
                        {
                            // if overlap is > t and response is higher for new kpt
                            // replace okld key point with new on_exit
                            // quit loop over keypoints
                            *it = newKeyPoint;
                            break;
                            // do the next.
                        }

                    }
                }
                if(!Boverlap)
                    Keypoints.push_back(newKeyPoint); 
                    // only add new key point if no overlap has been found in previous NMS
                    //store new keypoint in dynamic list

            }
        } // eof loop over cols
    } // eof loop over rows

    windowName = "Harris Corner Detection Results";
    cv::namedWindow(windowName,5);
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, Keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName,visImage);
    cv::waitKey(0);
    // EOF Student CODE


}

int main()
{
    cornernessHarris();
}
