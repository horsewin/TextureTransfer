//
//   MLS : Moving Least Suqres Image Doformation Class
//      
//   Based on "Image Deformation Using Moving Least Squares" in SIGGRAPH2006
//   Implemented by Atsushi Nakazawa (nakazawa@cmc.osaka-u.ac.jp)
//

#include "MLS.h"

#define RIGID_DEFORM

int MLS::calcMLS(vector<CvPoint> *src, vector<CvPoint> *dst )
{
	m_src = *src;
	m_dst = *dst;

	return 1;
}

int MLS::MLSProjection(int x, int y, float *tx, float *ty )
{
	float			*w = new float[ m_src.size() ], wsum = 0.0;
	float			ms = 0.0;
	CvPoint2D32f	cp, cq;			// Weighted Center
	CvPoint2D32f	*pdiff = new CvPoint2D32f[ m_src.size() ];
	CvPoint2D32f	*qdiff = new CvPoint2D32f[ m_src.size() ];

	// calc weight
	for( uint i = 0; i < m_src.size(); i++ ){
		w[i] = 1.0 / ( pow( float(x - m_src[i].x) + 0.5, 2) + pow( float(y-m_src[i].y) +  0.5, 2) );
		wsum += w[i];
	}

	// calc weighted center
	cp.x = 0.0; cp.y = 0.0;
	cq.x = 0.0; cq.y = 0.0;
	for( uint j = 0; j < m_src.size(); j++ ){
		cp.x += ( w[j] * m_src[j].x );
		cp.y += ( w[j] * m_src[j].y );
		cq.x += ( w[j] * m_dst[j].x );
		cq.y += ( w[j] * m_dst[j].y );
	}
	cp.x /= wsum;
	cp.y /= wsum;
	cq.x /= wsum;
	cq.y /= wsum;

	// calc difference from src and dst points to weighted centers
	for( uint i = 0; i < m_src.size(); i++ ){
		pdiff[i].x = m_src[i].x - cp.x;
		pdiff[i].y = m_src[i].y - cp.y;
		qdiff[i].x = m_dst[i].x - cq.x;
		qdiff[i].y = m_dst[i].y - cq.y;
	}

	// calc meu-s (ms)
	for( uint i = 0; i < m_src.size(); i++ ){
		ms += ( w[i]*( pdiff[i].x * qdiff[i].x + pdiff[i].y * qdiff[i].y ) );
	}

	// calc mA(i)
	CvMat **mA = new CvMat*[m_src.size()];
	CvMat *mP = cvCreateMat(2,2,CV_32F), *mX = cvCreateMat(2,2,CV_32F), *mXt = cvCreateMat(2,2,CV_32F);
	CvMat *mQ = cvCreateMat(1,2,CV_32F), *mQ2 = cvCreateMat(1,2,CV_32F);

	for( uint i = 0; i < m_src.size(); i++ ){
		cvmSet( mP, 0, 0, pdiff[i].x );
		cvmSet( mP, 0, 1, pdiff[i].y );
		cvmSet( mP, 1, 0, pdiff[i].y );
		cvmSet( mP, 1, 1, -pdiff[i].x );

		cvmSet( mX, 0, 0, x - cp.x );
		cvmSet( mX, 0, 1, y - cp.y );
		cvmSet( mX, 1, 0, y - cp.y );
		cvmSet( mX, 1, 1, -(x - cp.x) );
		cvmTranspose( mX, mXt );
		mA[i] = cvCreateMat(2,2,CV_32F);

		cvmMul( mP, mXt, mA[i] );
		cvmSet( mA[i], 0, 0, cvmGet(mA[i],0,0)*w[i] );
		cvmSet( mA[i], 0, 1, cvmGet(mA[i],0,1)*w[i] );
		cvmSet( mA[i], 1, 0, cvmGet(mA[i],1,0)*w[i] );
		cvmSet( mA[i], 1, 1, cvmGet(mA[i],1,1)*w[i] );
	}

#ifdef RIGID_DEFORM

	// Calc Point using Rigid Deformation
	CvMat *mFr = cvCreateMat(1, 2, CV_32F);
	float len, len2;

	for( uint i = 0; i < m_src.size(); i++ ){
		cvmSet( mQ, 0, 0, qdiff[i].x );
		cvmSet( mQ, 0, 1, qdiff[i].y );
		cvmMul( mQ, mA[i], mQ2 );
		cvmSet( mFr, 0, 0, cvmGet(mFr, 0, 0) + cvmGet(mQ2, 0, 0) );
		cvmSet( mFr, 0, 1, cvmGet(mFr, 0, 1) + cvmGet(mQ2, 0, 1) );
	}

	len = sqrt( cvmGet(mFr, 0, 0)*cvmGet(mFr, 0, 0) + cvmGet(mFr, 0, 1)*cvmGet(mFr, 0, 1) );
	cvmSet( mFr, 0, 0, cvmGet(mFr,0,0)/len );
	cvmSet( mFr, 0, 1, cvmGet(mFr,0,1)/len );

	len2 = sqrt( (x-cp.x)*(x-cp.x) + (y-cp.y)*(y-cp.y) );

	*tx = len2 * cvmGet(mFr, 0, 0) + cq.x;
	*ty = len2 * cvmGet(mFr, 0, 1) + cq.y;

	delete[] w;
	delete[] pdiff;
	delete[] qdiff;

	cvReleaseMat(&mP);
	cvReleaseMat(&mX);
	cvReleaseMat(&mXt);
	cvReleaseMat(&mQ);
	cvReleaseMat(&mQ2);
	for( uint i = 0; i < m_src.size(); i++ ){
		cvReleaseMat(&mA[i]);
	}
	cvReleaseMat(&mFr);

#else

	// Calc Point using Similarity Deformation
	*tx = *ty = 0.0;

	for( int i = 0; i < m_src.size(); i++ ){
		cvmSet( mQ, 0, 0, qdiff[i].x );
		cvmSet( mQ, 0, 1, qdiff[i].y );

		cvmMul( mQ, mA[i], mQ2 );

		*tx += cvmGet(mQ2, 0, 0);
		*ty += cvmGet(mQ2, 0, 1);
	}

	*tx = (*tx)/ms + cq.x;
	*ty = (*ty)/ms + cq.y;

	delete[] w;
	delete[] pdiff;
	delete[] qdiff;

	cvReleaseMat(&mP);
	cvReleaseMat(&mX);
	cvReleaseMat(&mXt);
	cvReleaseMat(&mQ);
	cvReleaseMat(&mQ2);
	for( int i = 0; i < m_src.size(); i++ ){
		cvReleaseMat(&mA[i]);
	}
#endif

	return 1;
}

void MLS::MLSWarpImage( IplImage *src, vector<CvPoint> *spts, IplImage *dst, vector<CvPoint> *dpts )
{
	int STEP = 10;
	float tx, ty;
	CvPoint2D32f **pts;
	unsigned char *p, *q;
	MLS	mls;

	// dstination points to source points projection
	calcMLS(dpts, spts );

	pts = new CvPoint2D32f*[dst->height/STEP + 10];
	for( int i = 0; i < dst->height/STEP + 10; i++ ){
		pts[i] = new CvPoint2D32f[dst->width/STEP + 1];
	}

	// Project GRID Points
	for( int y = 0; y < dst->height + STEP - 1; y += STEP ){
		for( int x = 0; x < dst->width + STEP - 1; x += STEP ){
			MLSProjection(x, y, &tx, &ty );
			pts[y/STEP][x/STEP].x = tx;
			pts[y/STEP][x/STEP].y = ty;
		}
	}

	// Warp Image using MLS + bilinear interpolation
	for( int y = 0; y < dst->height; y++ ){
		for( int x = 0; x < dst->width; x++ ){

			// bilinear interpolation
			CvPoint2D32f f00, f10, f01, f11;
			int   xx, yy;
			float dx, dy;

			f00 = pts[y/STEP][x/STEP];
			f01 = pts[y/STEP][x/STEP+1];
			f10 = pts[y/STEP+1][x/STEP];
			f11 = pts[y/STEP+1][x/STEP+1];

			dx = (float)(x - STEP*(x/STEP))/(float)STEP;
			dy = (float)(y - STEP*(y/STEP))/(float)STEP;

			xx = ( f00.x*(1.0-dy) + f10.x*dy )*(1.0-dx) + ( f01.x*(1.0-dy) + f11.x*dy )*dx;
			yy = ( f00.y*(1.0-dy) + f10.y*dy )*(1.0-dx) + ( f01.y*(1.0-dy) + f11.y*dy )*dx;
			
			if( xx < 0 || xx > dst->width || yy < 0 || yy > dst->height )
				continue;

			p = (unsigned char*)&dst->imageData[ y*src->widthStep + x*src->nChannels ];
			q = (unsigned char*)&src->imageData[ yy*dst->widthStep + xx*dst->nChannels ];

			memcpy( p, q, src->nChannels );
		}
	}
}

