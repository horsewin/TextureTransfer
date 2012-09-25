// to use EIGEN sparse solver
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#ifndef VIEWINGMODEL_H_
#define VIEWINGMODEL_H_

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include "OpenGL.h"
#include "BasicFace.h"
#include "IndexedMesh.h"

#include <Eigen/Sparse>
#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/shared_ptr.hpp>

#include <vector>
#include <deque>
#include <cstring>
#include <cfloat>
#include <cmath>

//-------------------------------------------------------------------
// Structs
//-------------------------------------------------------------------
class Lib3dsFile;
namespace TextureTransfer
{
	class LSCM;
	class Texture;

	//メッシュ構造体
	struct Mesh
	{
	  unsigned int nIndex;
	  std::vector<float> vertex;
	  std::vector<float> normal;
	  std::vector<std::pair<int,Vector2> > mTextureCoords; // first: the index of selected vertex, second: the texture coords of selected vertex
	  std::vector<unsigned int> index;
	  std::vector<unsigned int> ind;

	  std::vector<Vector3> ambient, diffuse, specular;
	  std::vector<int> materials;

	  int ind_max;
	  int flag;
	  void reset(){
		  vertex.clear();
		  normal.clear();
		  mTextureCoords.clear();
		  index.clear();
		  ind.clear();
		  nIndex = 0;
		  ind_max = 0;
		  flag = 0;
	  }
	};

	//-------------------------------------------------------------------
	// Class definition
	//-------------------------------------------------------------------
	class ViewingModel
	{
		class BoundingBox
		{
		public:
			BoundingBox(double minx, double maxx, double miny, double maxy, double minz, double maxz)
			:mMinX(minx), mMaxX(maxx), mMinY(miny), mMaxY(maxy), mMinZ(minz), mMaxZ(maxz)
			, mScale(.0), mUnitScale(14.0){}
			BoundingBox()
			:mMinX(DBL_MAX), mMaxX(DBL_MIN), mMinY(DBL_MAX), mMaxY(DBL_MIN), mMinZ(DBL_MAX), mMaxZ(DBL_MIN)
			, mScale(.0), mUnitScale(14.0){}

			void BoundaryCheck(Vector3 vec)
			{
				if(vec.x > mMaxX) mMaxX = vec.x;
				if(vec.x < mMinX) mMinX = vec.x;
				if(vec.y > mMaxY) mMaxY = vec.y;
				if(vec.y < mMinY) mMinY = vec.y;
				if(vec.z > mMaxZ) mMaxZ = vec.z;
				if(vec.z < mMinZ) mMinZ = vec.z;
			}

			void CalcScale()
			{
				double xScale, yScale, zScale;
				xScale = fabs( mMaxX - mMinX);
				yScale = fabs( mMaxY - mMinY);
				zScale = fabs( mMaxZ - mMinZ);
				mScale = mUnitScale / std::max( std::max(xScale, yScale), zScale);
			}

		public:
			double mMinX, mMaxX;
			double mMinY, mMaxY;
			double mMinZ, mMaxZ;
			double mScale, mUnitScale;

		};

	public:
		ViewingModel(const char * name = NULL);
		~ViewingModel();

		void Save3DModel(const char * filename);

		void LoadTexture(const char * filename);
		bool CheckFittingVertices(GLint *viewport, GLdouble *modelview, GLdouble *projection, cv::Point3d start_point, cv::Point3d end_point, bool glMouse = true);
		void UpdateMatrix();
		void CorrespondTexCoord(GLint *viewport, GLdouble *modelview, GLdouble *projection,
			  cv::Point3d start_point, cv::Point3d end_point, Vector2 & t1, Vector2 & t2, Vector3 & p1, Vector3 & p2, bool glMouse = true);
		void RenewMeshDataConstruct();

		void  QueryNormal(const int & outer_loop, const int & mesh_index, const int & vertexIdx, GLdouble * normal);
		void  QueryVertex(const int & outer_loop, const int & mesh_index, const int & vertexIdx, GLdouble * vertex);
		void  QueryAmbient(const int & outer_loop, const int & mesh_index, GLfloat * ambient);
		void  QueryDiffuse(const int & outer_loop, const int & mesh_index, GLfloat * diffuse);
		void  QuerySpecular(const int & outer_loop, const int & mesh_index, GLfloat  * specular);
		double QueryVertexColor(const int & outer_loop, const int & mesh_index, const int & vertexIdx) const;

		int GetMeshSize() const;
		int GetMeshFacesSize(const int & outer_loop) const;
		int GetMeshInnerFacesSize(const int & outer_loop, const int & inner_loop) const;
		int GetMeshFlag(const int & outer_loop) const;
		void IncrementSumOfStrokes();

		bool isNewTexture() const { return mNewTexture; }
		bool IsMeshSelected() const { return mMeshSelected; }
		void SetMeshSelected(bool meshSelected) { mMeshSelected = meshSelected; }

	private:
		/* Load Functions */
		void Load3DModel();
		void ConvertDataStructure();
		bool LoadMatrixFrom3ds();
		bool LoadMatrixFromObj();
		void Load3DSModel();
		void LoadObjModel(const char* modelName = NULL);
		std::deque<Texture*> LoadTextures(Lib3dsFile * pModel);
		void AdjustmentSizeAndPosition();

		/* Set Functions */
		void SetSelectedMeshData(const int& loopVer);
		void SetSelectedFaces(const int& loopTex);

		public:
		boost::shared_ptr<LSCM> mLSCM;

		std::deque<Texture *> mTexture;			// A set of Textures 2011.6.7

		std::pair<int,IndexedMesh> mSelectedMesh;	//first : an index for the matrix having harmonic field(mTexparts), second: a set of meshes
		std::vector<int> mSelectedFace;				//corresponding to mLSCM's face index
		float  mScales;
		double mAngles[3];
		double mTrans[3];

		std::vector< boost::shared_ptr<IndexedMesh> > mMesh;

	private:

		std::string mModelname;
		int mSumOfVertices; 									// the number of vertices of this model
		int mSumOfStrokes; 									// the number of strokes added to this model
		std::vector<int> mMinStartIndex;
		std::vector<int> mMinEndIndex;

		Eigen::SparseMatrix<double> sparse_laplacian;
		Eigen::VectorXd b, mHarmonicValue;

		bool mIsConvert;
		bool mHasTexture;										//texture image is loaded?
		bool mMeshSelected;
		bool mNewTexture;										//assign new texture image?

		BoundingBox mBoundingBox;
		Vector3 mGravityVector;
	};
}
#endif
