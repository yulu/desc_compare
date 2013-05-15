/* Descriptor Comparison by YU LU
 * May, 2013
 */

#include "utility.h"

const char *DETECTOR[6];
const char *DESCRIPTOR[6];
const char *MATCHER[5];

void init_type()
{
	DETECTOR[0] = "FAST";
	DETECTOR[1] = "SIFT";
	DETECTOR[2] = "SURF";
	DETECTOR[3] = "ORB";
	DETECTOR[4] = "BRISK";
	DETECTOR[5] = "FREAK";

	DESCRIPTOR[0] = "SIFT";
	DESCRIPTOR[1] = "SURF";
	DESCRIPTOR[2] = "ORB";
	DESCRIPTOR[3] = "BRIEF";
	DESCRIPTOR[4] = "BRISK";
	DESCRIPTOR[5] = "FREAK";

	MATCHER[0] = "BruteForce";
	MATCHER[1] = "BruteForce-L1";
	MATCHER[2] = "BruteForce-Hamming";
	MATCHER[3] = "BruteForce-Hamming(2)";
	MATCHER[4] = "FlannBased";
}

int main(int argc, char** argv)
{
	double dec_time, desc_time, match_time;
	float match_rate;
	init_type();
	
	//input image
	if(argc != 2)
	{
		cout<<"input ONE image for test"<<endl;
		return -1;
	}

	Mat src=imread(argv[1]);

	if(src.empty()){
		cout<<"cannot read image"<<endl;
		return -1;
	}

	//generate the warp image
	Mat dst;
	Mat true_H;
	randomWarping(src, dst, true_H);	

	//the main test

	mainTest(DETECTOR[4], DESCRIPTOR[4], MATCHER[0], 
		 src, dst, true_H,
		 dec_time, desc_time, match_time,
		 match_rate);	

	return 0;
}
