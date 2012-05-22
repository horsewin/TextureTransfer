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
#include <vector>

//-------------------------------------------------------------------
// Class definition
//-------------------------------------------------------------------
class TransferController
{
public:
	TransferController();
	~TransferController();

	void SetContourPoints();
	void AcquireMatching();

public:
    std::vector< std::vector<cv::Point> > mContours1, mContours2;	// a set of contours
    std::vector<cv::Point> mContour1, mContour2; 					// objective contour for transferring texture
    std::vector< std::pair<cv::Point, cv::Point> > matchingPoints;	// match points between contour1 and contour2

private:
	cv::Ptr<IplImage> mInput1;
	cv::Ptr<IplImage> mInput2;
};

#endif /* TRANSFER_H_ */
