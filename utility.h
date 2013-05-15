#include <cstdlib>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#define PI 3.141592653

using namespace cv;
using namespace std;

/*
 * function: void randomWarping(Mat& in, Mat& out)
 * ----------------------------------------------
 * Take an image as input, generate a random rotations about three axis
 * about horizontal axis: (-45, 45)
 * about vertical axis: (-45,45)
 * about inplane rotation: (-180, 180)
 * Get the homography transformation matrix from the three angle
 * apply the transformation to the input image and out the result and the
 * and the randomly generated true H
 */
void randomWarping(Mat& in, Mat& out, Mat& true_H);

/* 
 * function: float matchingRate(KeyPoint& keypoint, vector<DMatch>& matches, Mat& true_H)
 * -------------------------------------------------------------------------------------
 * Input the keypoint detected in the original image
 * Warp the keypoints using the true_H to find its correct correspondings in warped image
 * Find the estimated correspondings from the matches
 * Compare the position of the correct correspondings and the estimated correspondings 
 * If within a distance, mark as a good match, otherwise a bad match
 * The rate is calculated as: good match/total match 
 */
float matchingRate(vector<Point2f>& points1, vector<Point2f>& points2,vector<char>& outlier_mask, Mat& true_H);

/*
 * function: void mainTest(String detector, String descriptor, String matcher, 
			   Mat& img1, Mat& img2, Mat& true_H,
			   float& dec_time, float& desc_time, float& match_time,
			   float& match_rate);
 * --------------------------------------------------------------------------
 * This function is the main test process
 * it takes two images, and performance keypoint detection, descriptor generation and match
 * RANSAC is used to find the best homography H from the correspondances
 * The result is evaluted and time and match rate is the output
 */
void mainTest(const char* detector, const char* descriptor, const char* matcher,
	      Mat& img1, Mat& img2, Mat& true_H,
    	      double& dec_time, double& desc_time, double& match_time,
	      float& match_rate);

