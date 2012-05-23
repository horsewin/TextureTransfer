//
//   MLS : Moving Least Suqres Image Doformation Class
//      
//   Based on "Image Deformation Using Moving Least Squares" in SIGGRAPH
//   Implemented by Atsushi Nakazawa (nakazawa@cmc.osaka-u.ac.jp)
//

#ifndef MLS_H
#define MLS_H

#ifndef OPENCV_2_3_1
#define OPENCV_2_3_1
	#include "cv.h"
	#include "highgui.h"
//	#include <opencv2/opencv.hpp>
#endif /* OPENCV_H_ */

#include <cmath>
#include <vector>

using std::vector;

class MLS
{
public:
	MLS(){};
	~MLS(){};

	void	MLSWarpImage( IplImage *src, vector<CvPoint> *spts, IplImage *dst, vector<CvPoint> *dpts );

private:
//	int		calcMLS( MLS &mls, vector<CvPoint> *src, vector<CvPoint> *dst );
//	int		MLSProjection( MLS &mls, int x, int y, float *tx, float *ty );
	int		calcMLS(vector<CvPoint> *src, vector<CvPoint> *dst );
	int		MLSProjection(int x, int y, float *tx, float *ty );


public:
	vector<CvPoint> m_src;
	vector<CvPoint> m_dst;
};
#endif
