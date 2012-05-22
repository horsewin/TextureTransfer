/*
 * Transfer.cc
 *
 *  Created on: 2012/05/22
 *      Author: umakatsu
 */

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include "TransferController.h"


//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
using namespace std;

//-------------------------------------------------------------------
//Code
//-------------------------------------------------------------------
TransferController::TransferController()
{

}

TransferController::~TransferController()
{

}

void TransferController::SetContourPoints()
{
	//Load selected images
	mInput1 = cvLoadImage("mesh.bmp", 0);
	mInput2 = cvLoadImage("Hatune.bmp", 0);

	//prepare gray-scale images for binary transform
	cv::Ptr<IplImage> src1   = cvCreateImage(cvGetSize(mInput1), 8, 1);
	cv::Ptr<IplImage> src2   = cvCreateImage(cvGetSize(mInput2), 8, 1);

	//get white-black images
	cvThreshold(mInput1, src1, 200, 255, CV_THRESH_BINARY_INV);
	cvThreshold(mInput2, src2, 200, 255, CV_THRESH_BINARY_INV);

	//Iplimage -> cv::Mat
	cv::Mat src1_mat(src1);
	cv::Mat src2_mat(src2);

	//get contours with images
	cv::findContours(src1_mat, mContours1, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	cv::findContours(src2_mat, mContours2, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	//for finding out largest area
	pair<int, double> area1, area2; //first:index, second:the number of area
	area1.first = -1; area1.second = 0;
	area2.first = -1; area2.second = 0;

	//find out largest in src1
	for (unsigned int i=0; i<mContours1.size(); i++){
		vector<cv::Point> contour = mContours1[i];
		cv::Mat contourMat = cv::Mat(contour);
		double a = cv::contourArea(contourMat);
		if( area1.second < a)
		{
			area1.first  = i;
			area1.second = a;
		}
	}

	//find out largest in src2
	for (unsigned int i=0; i<mContours2.size(); i++){
		vector<cv::Point> contour = mContours2[i];
		cv::Mat contourMat = cv::Mat(contour);
		double a = cv::contourArea(contourMat);
		// select the largest area as the selected mesh
		if( area2.second < a)
		{
			area2.first  = i;
			area2.second = a;
		}
	}

	//set　each largest contour part
	mContour1 = mContours1[area1.first];
	mContour2 = mContours2[area2.first];
}

void TransferController::AcquireMatching()
{
	cvNamedWindow("1", 1 );
	cvNamedWindow("2", 1 );

//	cvWaitKey(0);
	double c1=0, c2=0;
	bool oneLarger = mContour1.size() > mContour2.size();
//	cout << mContour1.size() << endl;
//	cout << mContour2.size() << endl;
	double ratio = oneLarger?
	(double)mContour1.size() / mContour2.size() :
	(double)mContour2.size() / mContour1.size();

	while(1){
		if(c1<mContour1.size() && c2<mContour2.size()){
//		while(c1<mContour1.size() && c2<mContour2.size()){
			cvCircle(mInput1, mContour1[static_cast<int>(c1)], 3, cv::Scalar(0,0,255), 3);
			cvCircle(mInput2, mContour2[static_cast<int>(c2)], 3, cv::Scalar(0,0,255), 3);

			pair<cv::Point, cv::Point> tmpMatching;
			tmpMatching.first  = mContour1[static_cast<int>(c1)];
			tmpMatching.second = mContour1[static_cast<int>(c2)];
			this->matchingPoints.push_back(tmpMatching);

			if(oneLarger){
				c1 += ratio;
				c2 += 1.0;
			}else{
				c1 += 1.0;
				c2 += ratio;
			}
		}
		else
		{
			break;
		}

		cvShowImage( "1", mInput1 );
		cvShowImage( "2", mInput2 );
		int key = cvWaitKey(0);
//		char c = cvWaitKey (0);
//		if (c == '\x1b') break;
	}
}
