/*
 * TransferController.h
 *
 *  Created on: 2012/05/22
 *      Author: umakatsu
 */

#ifndef TRANSFERCONTROLLER_H_
#define TRANSFERCONTROLLER_H_

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------

#include "OpenCV.h"
#include "main.h"

#include <vector>

//-------------------------------------------------------------------
// Class definition
//-------------------------------------------------------------------
namespace TextureTransfer
{
	class TransferController
	{
	public:
		TransferController();
		~TransferController();

		void SetContourPoints();
		void AcquireMatching();
		void InitHashmap( void );
		void SetHashmap( const int & x, const int & y, const int & val, const int & modelNumber);
		int GetHashmap( int x, int y, const int & modelNumber);

	private:

	public:
		std::vector< std::vector<cv::Point> > mContours1, mContours2;	// a set of contours
		std::vector<cv::Point> mContour1, mContour2; 					// objective contour for transferring texture
		std::vector< std::pair<cv::Point, cv::Point> > mMatchingPoints;	// match points between contour1 and contour2
		//first : an index of a point , second: coordinates of a point
		std::vector< std::pair<int, cv::Point> > mMeshes[2];

	private:
		cv::Ptr<IplImage> mInput1;
		cv::Ptr<IplImage> mInput2;

		int mIndexHashmap[W_WIDTH/2][W_HEIGHT/2][2];
	};

}
#endif /* TRANSFER_H_ */
