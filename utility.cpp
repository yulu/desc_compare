#include "utility.h"

void runByImageBorder(vector<KeyPoint>& kpts, int width, int height, int boundary){
	int i = 0;
	while(i != kpts.size()){
		int posx = kpts[i].pt.x;
		int posy = kpts[i].pt.y;
		if(posx < boundary || posx > (width - boundary) || posy < boundary || posy > (height - boundary)){
			kpts[i] = kpts.back();
			kpts.pop_back();
			continue;
		}
		i++;
	}
}

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
void randomWarping(Mat& in, Mat& out, Mat& true_H)
{
	float angle1, angle2, angle3;
	true_H.create(3,3,CV_32FC1);

	//generate random number between -45 to 45
	angle1 = 0.0/180.0*PI;
	angle2 = 0.0/180.0*PI;	
	//generate ramdom number between -180 to 180
	angle3 = 4.0/180.0*PI;
	//generate true_H from angles
	float c1 = cos(angle1);
	float c2 = cos(angle2);
	float c3 = cos(angle3);
	float s1 = sin(angle1);
	float s2 = sin(angle2);
	float s3 = sin(angle3);

	true_H.at<float>(0,0) =      c2*c3;
	true_H.at<float>(0,1) = -1.0*c2*s3;
	true_H.at<float>(0,2) =      s2;
	true_H.at<float>(1,0) =      c1*s3 + c3*s1*s2;
	true_H.at<float>(1,1) =	     c1*c3 - s1*s2*s3;
	true_H.at<float>(1,2) = -1.0*c2*s1;
	true_H.at<float>(2,0) =      s1*s3 - c1*c3*s2;
	true_H.at<float>(2,1) =      c3*s1 + c1*s2*s3;
	true_H.at<float>(2,2) =      c1*c2;

	//warp the image
	Mat K = (Mat_<float>(3,3) << 1057.1789, 0.0, 493.0726, 0.0, 1062.7981, 354.5415, 0.0, 0.0, 1.0);
	Mat invK;
	invert(K, invK, DECOMP_LU);
	true_H = true_H*invK;
	true_H = K*true_H;

	warpPerspective(in, out, true_H, in.size());	
}




/*
 * function: float findRate(vector<DMatch>& dmatches, int N)
 * -------------------------------------------------------
 * calculate the rate of correct matches
 */
float findRate(vector<DMatch>& dmatches, int N)
{
	int rate = 0;
	for(int i = 0; i < dmatches.size(); i++){
//		cout<<dmatches[i].trainIdx <<" "<<dmatches[i].queryIdx<<endl;
		if(dmatches[i].trainIdx == dmatches[i].queryIdx)
			rate++;
	}
	return 1.0*rate/N;
}

/*
 * function: void findInferPoints(vector<KeyPoint>& kpts1, vector<Keypoint>& kpts2,int width, int height,  Mat& true_H)
 * --------------------------------------------------------------------------------
 * find the correponding keypoints in the second image from the true homography
 */
 void findInferPoints(vector<KeyPoint>& kpts1, vector<KeyPoint>& kpts2, int width, int height, Mat& true_H)
{
	int i = 0;
	
	kpts2.clear();
	while(i != kpts1.size()){
		float x = true_H.at<float>(0,0)*kpts1[i].pt.x + true_H.at<float>(0,1)*kpts1[i].pt.y + true_H.at<float>(0,2);
		float y = true_H.at<float>(1,0)*kpts1[i].pt.x + true_H.at<float>(1,1)*kpts1[i].pt.y + true_H.at<float>(1,2);
		float z = true_H.at<float>(2,0)*kpts1[i].pt.x + true_H.at<float>(2,1)*kpts1[i].pt.y + true_H.at<float>(2,2);

		x /= z;
		y /= z;

		if(x < width && y < height){ //if the infer point is inside img2
			KeyPoint kpts = KeyPoint();
			kpts.pt.x = x;
			kpts.pt.y = y;
			kpts.size = kpts1[i].size;
			kpts2.push_back(kpts);
			i++;
		}else{ //if the infer point is outside img2, discard it in img1
			kpts1[i] = kpts1.back();
			kpts1.pop_back();
		}	
	}
}
	

void drawMatchTrueH(Mat& im1, vector<KeyPoint>& kpts_1,
	   	    Mat& im2, vector<KeyPoint>& kpts_2){
	
	int width = im1.cols;
	int height = im1.rows;
	Mat mstacked(height, width*2, CV_8UC3);

	Rect roi_1(0, 0, width, height);
	Rect roi_2(width, 0, width, height);
	Mat mstacked_roi1 = Mat(mstacked, roi_1);
	Mat mstacked_roi2 = Mat(mstacked, roi_2);
	im1.copyTo(mstacked_roi1);
	im2.copyTo(mstacked_roi2);
	
	//draw matches
	for (int i = 0; i < kpts_1.size(); i++)
	{
		Point kp1, kp2;
		kp1.x = (int)kpts_1[i].pt.x;
		kp1.y = (int)kpts_1[i].pt.y;
		kp2.x = (int)kpts_2[i].pt.x+width;
		kp2.y = (int)kpts_2[i].pt.y;

		int rand_r = rand()%256;
		int rand_g = rand()%256;
		int rand_b = rand()%256;

		line(mstacked, kp1, kp2, Scalar(rand_r,rand_g,rand_b,255),1, 8, 0);
		circle(mstacked, kp1, 10, Scalar(rand_r,rand_g,rand_b, 255));
		circle(mstacked, kp2, 10, Scalar(rand_r,rand_g,rand_b, 255));
	}
	
	imshow("matches from true H", mstacked);
	
	waitKey();
}
/* function: void mainTest(String det_tyoe, String desc_type, String matcher_type, 
			   Mat& img1, Mat& img2, Mat& true_H,
			   float& dec_time, float& desc_time, float& match_time,
			   float& match_rate);
 * --------------------------------------------------------------------------
 * This function is the main test process
 * it takes two images, and performance keypoint detection, descriptor generation and match
 * RANSAC is used to find the best homography H from the correspondances
 * The result is evaluted and time and match rate is the output
 */
void mainTest(const char* det_type, const char* desc_type, const char* matcher_type,
	      Mat& img1, Mat& img2, Mat& true_H,
    	      double& det_time, double& desc_time, double& match_time,
	      float& match_rate)
{
	double t;
	int width, height;
	width = img1.cols;
	height = img1.rows;

	Ptr<FeatureDetector> detector = FeatureDetector::create(det_type);
	Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create(desc_type);
	Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create(matcher_type);
	
	t = (double)getTickCount();
	if(detector.empty() || descriptorExtractor.empty() || descriptorMatcher.empty())
	{
		cout<<"Can not create detector or descriptor exstractor or descriptor matcher of given types" <<endl;	
		return;
	}
	cout << endl<<"< Extracting keypoints from images..."<<endl;
	vector<KeyPoint> keypoints1;
	vector<KeyPoint> keypoints2;
	detector->detect(img1, keypoints1);
	detector->detect(img2, keypoints2);
	t = ((double)getTickCount() - t) / getTickFrequency()*1000;
	det_time = t;
	cout<<"img1: "<<keypoints1.size() << " points"<<endl;
	cout<<"img2: "<<keypoints2.size() << " points"<<endl<< ">" <<endl;

	t = (double)getTickCount();
	cout << endl<<"< Computing descriptors for keypoints..."<<endl;
	Mat descriptors1;
	Mat descriptors2;
	descriptorExtractor->compute(img1, keypoints1, descriptors1);
	descriptorExtractor->compute(img2, keypoints2, descriptors2);
	t = ((double)getTickCount() - t) / getTickFrequency()*1000;
	desc_time = t;
	cout<< ">" <<endl;

	t = (double)getTickCount();
	cout<<endl<<"< Do the matching..."<<endl;
	vector<DMatch> matches;
	descriptorMatcher->match(descriptors1, descriptors2, matches);
	t = ((double)getTickCount() - t) / getTickFrequency()*1000;
	match_time = t;
	cout<<"number of matches: "<<matches.size()<< ">" <<endl;	
	
	cout<<endl<<"< Find homography..."<<endl;
	vector<int> queryIdxs(matches.size()), trainIdxs(matches.size());
	for(size_t i = 0; i < matches.size(); i++)
	{
		queryIdxs[i] = matches[i].queryIdx;
		trainIdxs[i] = matches[i].trainIdx;
	}
	vector<Point2f> points1; KeyPoint::convert(keypoints1, points1, queryIdxs);
	vector<Point2f> points2; KeyPoint::convert(keypoints2, points2, trainIdxs);
	vector<char> outlier_mask(matches.size(), 0);
	Mat Homo = findHomography(Mat(points1), Mat(points2),  RANSAC, 1);

	cout << ">" <<endl;			

	double maxInlierDist = 3.0;
	Mat points1t;
	perspectiveTransform(Mat(points1), points1t, Homo);
	for(size_t i1 = 0; i1 < points1.size(); i1++)
	{
		if(norm(points2[i1] - points1t.at<Point2f>((int)i1, 0)) <= maxInlierDist)
			outlier_mask[i1] = 1;
	}	
	
	Mat drawImg;
	drawMatches(img1, keypoints1, img2, keypoints2, matches, drawImg, CV_RGB(0, 0, 255), CV_RGB(255, 0, 0), outlier_mask);
	imshow("matches", drawImg);
	waitKey(0);

	cout<<"< Calculate the matching rate: "<<endl;
	
	vector<KeyPoint> infer_kpts;
	Mat infer_descriptors;
	vector<DMatch> infer_matches;

	findInferPoints(keypoints1, infer_kpts, width, height, true_H);
	Mat drawImg2;
	drawMatchTrueH(img1, keypoints1, img2, infer_kpts);	
	
	runByImageBorder(keypoints1, width, height, 35);
	runByImageBorder(infer_kpts, width, height, 35);
	cout<<infer_kpts.size()<<endl;
	cout<<keypoints1.size()<<endl;

	int i = 0;
	descriptorExtractor->compute(img2, infer_kpts, infer_descriptors);
	descriptorExtractor->compute(img1, keypoints1, descriptors1);
	cout<<descriptors1.size() <<" "<<infer_descriptors.size()<<endl;
	descriptorMatcher->match(descriptors1,infer_descriptors, infer_matches);
	float rate = findRate(infer_matches, keypoints1.size());
	cout<<"The matching rate is "<<rate<<endl;
}
