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
#include "MLS.h"

//#include "TickCounter.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
#define VISUALIZE 0
//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
using namespace std;

//-------------------------------------------------------------------
//Code
//-------------------------------------------------------------------
namespace TextureTransfer
{

	TransferController::TransferController()
	{
		mMeshes[0].clear();
		mMeshes[1].clear();
	}

	TransferController::~TransferController()
	{

	}

	void TransferController::SetContourPoints()
	{
		//Load selected images
		mInput1 = cvLoadImage("mesh1.bmp", 0);
		mInput2 = cvLoadImage("mesh2.bmp", 0);

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
//		TickCountAverageBegin();
		IplImage * src = cvCloneImage(mInput1);
	#if VISUALIZE == 1
		cvNamedWindow("1", 1 );
		cvNamedWindow("2", 1 );
	#endif
		double c1=0, c2=0;
		bool oneLarger = mContour1.size() > mContour2.size();

		double ratio = oneLarger?
		(double)mContour1.size() / mContour2.size() :
		(double)mContour2.size() / mContour1.size();

		mMatchingPoints.clear();

#if VISUALIZE == 1
		//Load selected images
		IplImage * input1 = cvLoadImage("mesh1.bmp", 1);
		IplImage * input2 = cvLoadImage("mesh2.bmp", 1);
#endif
		while(1){
			if(c1<mContour1.size() && c2<mContour2.size()){
//			if(c1<mContour1.size()  c2<mContour2.size()){
	//		while(c1<mContour1.size() && c2<mContour2.size()){
	#if VISUALIZE == 1
				cvCircle(input1, mContour1[static_cast<int>(c1)], 3, cv::Scalar(0,0,255), 3);
				cvCircle(input2, mContour2[static_cast<int>(c2)], 3, cv::Scalar(0,0,255), 3);
	#endif
				pair<cv::Point, cv::Point> tmpMatching;
				tmpMatching.first  = mContour1[static_cast<int>(c1)];
				tmpMatching.second = mContour2[static_cast<int>(c2)];
				mMatchingPoints.push_back(tmpMatching);

				int step = 30;
				if(oneLarger){
					c1 += (ratio*step);
					c2 += step;
				}else{
					c1 += step;
					c2 += (ratio*step);
				}
			}
			else
			{
				break;
			}
	#if VISUALIZE == 1
			cvShowImage( "1", input1 );
			cvShowImage( "2", input2 );
	//		cvWaitKey(0);
	#endif
		}


		MLS mls;
		IplImage * dst = cvCreateImage(cvGetSize(mInput1), 8, 3);
		vector<CvPoint> input;
		vector<CvPoint> output;
		REP(i,mMatchingPoints.size()){
			input.push_back(mMatchingPoints[i].first);
			output.push_back(mMatchingPoints[i].second);
		}

		IplImage * i1 = cvLoadImage("mesh1.bmp", 1);

		mls.MLSWarpImage(i1, &input, dst, &output);

//		TickCountAverageEnd();

		cvSaveImage("warping1.bmp", dst);
	#if VISUALIZE == 1
//		REP(i,mls.m_dst.size()){
//			cvCircle(dst, mls.m_dst[i], 3, cv::Scalar(0,0,255), 3);
//		}
		REP(i,output.size()){
			cvCircle(dst, output[i], 3, cv::Scalar(0,0,255), 3);
		}
		cvNamedWindow("MLS", 1 );
		cvShowImage( "MLS", dst);
		cvWaitKey(0);
	#endif
		cvReleaseImage(&src);
		cvReleaseImage(&dst);
	}

//	void TransferController::SetHashmap( const int & x, const int & y, const int & val, const int & modelNumber)
//	{
//		if( modelNumber > 1){
//			cerr << "Set Error in TransferController: Access to out of range of array" << endl;
//			cerr << "model number = " << modelNumber << endl;
//			return;
//		}
//		if(0<= x && x < W_WIDTH/2
//		&& 0<= y && y < W_HEIGHT/2){
//			mIndexHashmap[x][y][modelNumber] = val;
//	//		printf("Model%d's (%d,%d)=%d\n", modelNumber, x, y, val);
//		}
//		else
//		{
//			cerr << "Set Error in TransferController: Access to out of range of array" << endl;
//			cerr << "x=" << x << ",y=" << y << endl;
//		}
//	}
//
//	int TransferController::GetHashmap( int x, int y, const int & modelNumber)
//	{
//		if( modelNumber > 1){
//			cerr << "Get Error in TransferController: Access to out of range of array : ";
//			cerr << "model number = " << modelNumber << endl;
//			return 0;
//		}
//		if(0<= x && x < W_WIDTH/2 && 0<= y && y < W_HEIGHT/2){
//			double dist = 99999;
//			int index = -1;
//			REP(i,mMeshes[modelNumber].size()){
//				double curDist = sqrt( pow( (double)mMeshes[modelNumber].at(i).second.x - x, 2) + pow((double)mMeshes[modelNumber].at(i).second.y - y,2));
//				if( dist >= curDist){
//					index = mMeshes[modelNumber].at(i).first;
//					dist = curDist;
//				}
//				if( dist < 1) break;
//			}
//			return index;
//	//		if(mIndexHashmap[x][y][modelNumber] == 0){
//	//			for(int i=-2; i<= 2; i++){
//	//				y+=i;
//	//				for(int j=-2; j<= 2; j++){
//	//					x+=j;
//	//					if(0<= x && x < W_WIDTH/2 && 0<= y && y < W_HEIGHT/2){
//	//						if(mIndexHashmap[x][y][modelNumber] != 0) return mIndexHashmap[x][y][modelNumber];
//	//					}
//	//					x-=j;
//	//				}
//	//				y-=i;
//	//			}
//	//		}
//	//		else
//	//		{
//	//			return mIndexHashmap[x][y][modelNumber];
//	//		}
//		}
//		else
//		{
//			cerr << "Get Error in TransferController: Access to out of range of array" << endl;
//			cerr << "x=" << x << ",y=" << y << endl;
//			return 0;
//		}
//	}
}
