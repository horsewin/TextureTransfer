#include "main.h"
#include "ViewingModel.h"
#include "LSCM.h"
#include "Modelling/Texture.h"

#include <Eigen/CholmodSupport>
#include <Eigen/UmfPackSupport>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>

#include <cvd/image_io.h>

#include <lib3ds.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cstring>
#include <string.h>


const int weight = 1000;

using namespace std;
using namespace CVD;
using namespace boost::numeric;
using namespace Eigen;

int ind1, ind2;

namespace{
  inline void SetAdjacentValue(const int & i1, const int & i2, const int & i3, boost::numeric::ublas::mapped_matrix<int> & mat)
  {
    mat(i1,i2) = 1;
    mat(i2,i1) = 1;
    mat(i1,i3) = 1;
    mat(i3,i1) = 1;
    mat(i3,i2) = 1;
    mat(i2,i3) = 1;
  }
  
  inline void SetFrequencyValue(const int & i1, const int & i2, const int & i3, const boost::numeric::ublas::mapped_matrix<int> & mat_adj, boost::numeric::ublas::mapped_matrix<int> & mat_fre)
  {
    if( mat_adj(i1,i2) == 0 ){
      mat_fre(i1,i1) += 1;
      mat_fre(i2,i2) += 1;
    }
    if( mat_adj(i1,i3) == 0 ){
      mat_fre(i1,i1) += 1;
      mat_fre(i3,i3) += 1;
    }
    if( mat_adj(i2,i3) == 0 ){
      mat_fre(i2,i2) += 1;
      mat_fre(i3,i3) += 1;
    }  
  }
}

//Vector2 operator=(float t[2]){
//    return Vector2(
//        t[0], t[1]
//    ) ;
//}

bool ViewingModel::CheckFittingVertices
(GLint *viewport, GLdouble *modelview, GLdouble *projection, 
 cv::Point2d start_point, cv::Point2d end_point)
{
  double winX,winY,winZ,objX,objY,objZ;

  // ストロークの三次元位置から最も近い頂点を探すことにする
  double min_start = 999999;
  double min_end   = 999999;

  // resize a set of stroke
  const unsigned int size_set_stroke = mMinStartIndex.size();
  mMinStartIndex.push_back(0);
  mMinEndIndex.push_back(0);

  REP(mesh,mMesh.size()){
    cv::Point3d mesh_vertex;
    double dist;
    for(unsigned int loopVer=0; loopVer < mMesh[mesh].vertex.size(); loopVer+=3){
      mesh_vertex.x = mMesh[mesh].vertex[loopVer + 0];
      mesh_vertex.y = mMesh[mesh].vertex[loopVer + 1];
      mesh_vertex.z = mMesh[mesh].vertex[loopVer + 2];
      objX = mesh_vertex.x;
      objY = mesh_vertex.y;
      objZ = mesh_vertex.z;

      gluProject(objX,objY,objZ,modelview,projection,viewport,&winX,&winY,&winZ);
      winY = viewport[3]*2 - winY;
      
      // Judgement of pixel
      dist = sqrt( pow( (start_point.x - winX) , 2) + pow( (start_point.y - winY) , 2) );
      if( min_start > dist ){
		min_start = dist;
		mMinStartIndex[size_set_stroke] = mMesh[mesh].ind[loopVer/3];
      }
      
      dist = sqrt( pow( (end_point.x - winX) , 2) + pow( (end_point.y - winY) , 2) );
      if( min_end > dist ){
		min_end = dist;
		mMinEndIndex[size_set_stroke] = mMesh[mesh].ind[loopVer/3];
      } 
    }
  }	
  cout << "Strokes' indices : " << mMinEndIndex[size_set_stroke] << "--" << mMinStartIndex[size_set_stroke] << endl;
  return true;
}

void ViewingModel::Load3DModel()
{
	char *dot;

	//check whether .3ds, .obj or others
	dot = strrchr(mModelname, '.');

	if(!strcmp(dot, ".3ds") || !strcmp(dot, ".3DS")){
		Load3DSModel();
		mIsLoadMatrix = LoadMatrix();
	}
	else if(!strcmp(dot, ".obj") || !strcmp(dot, ".OBJ")){
		LoadObjModel();
	}
	else
	{
		cerr << "Not supported 3DS file format : " << dot << endl;
		return;
	}
}

void ViewingModel::Load3DSModel( void )
{
	  Lib3dsFile *m_model; //モデル全体
	  Lib3dsMesh *mesh; //メッシュ単位

	  //モデル読み込み
	  m_model = lib3ds_file_open(mModelname);
	  if(m_model==NULL){
	    std::cerr << "can't Open file\n";
	    exit(0);
	  }

	  mMesh.resize(m_model->nmeshes);//メッシュ分メモリ確保

//      Vector2 tex1 = mesh->texcos[mesh->faces[loopX].index[0] ];
//      Vector2 tex2 = mesh->texcos[mesh->faces[loopX].index[1] ];
//      mMesh[loop].mTextureCoords.push_back(tex1);
//      mMesh[loop].mTextureCoords.push_back(tex2);

	  cout << "Materials : " << m_model->nmaterials << endl;
	  for(int loop = 0; loop < m_model->nmeshes;++loop){
	    mesh = m_model->meshes[loop];//mLoop番目のメッシュへのポインタ
	    if(mesh->nfaces == 0) {
	      mMesh[loop].flag = 1;
	      mMesh[loop].ind_max = 0;
	      mMesh[loop].nIndex = 0;
	      continue;
	    }//メッシュが無い場合はカット

	    //法線データの取り出し
	    float (*normal)[3] = new float[mesh->nfaces][3];
	    lib3ds_mesh_calculate_face_normals(mesh,&normal[0]);//面法線の取り出し

	    mMesh[loop].normal.resize(mesh->nfaces*3*3);//法線用メモリ確保(面の数*3頂点*3座標)
	    mMesh[loop].vertex.resize(mesh->nfaces*3*3);//頂点用メモリ確保

	    mMesh[loop].nIndex = mesh->nfaces * 3;
	//    m_mesh[loop].nIndex = mesh->nvertices;
	    mMesh[loop].ind.resize( mMesh[loop].nIndex );//面の数*3頂点分=総インデックス数

	    //インデックス最大値の初期化
	    mMesh[loop].ind_max = 0;
	    //頂点データと法線データをインデックスに合わせて格納
	    for(int loopX = 0; loopX < mesh->nfaces;++loopX){
	      //1頂点目
	      memcpy(&mMesh[loop].normal[loopX*9],&normal[loopX][0],sizeof(float)*3);
	      memcpy(&mMesh[loop].vertex[loopX*9],&mesh->vertices[ mesh->faces[loopX].index[0] ][0],sizeof(float)*3);
	      //２頂点目
	      memcpy(&mMesh[loop].normal[loopX*9+3],&normal[loopX][0],sizeof(float)*3);
	      memcpy(&mMesh[loop].vertex[loopX*9+3],&mesh->vertices[ mesh->faces[loopX].index[1] ][0],sizeof(float)*3);
	      //3頂点目
	      memcpy(&mMesh[loop].normal[loopX*9+6],&normal[loopX][0],sizeof(float)*3);
	      memcpy(&mMesh[loop].vertex[loopX*9+6],&mesh->vertices[ mesh->faces[loopX].index[2] ][0],sizeof(float)*3);

	      // 現在のメッシュの3頂点に対してインデックスの設定
	      for(int idmesh=0;idmesh<3;idmesh++){
				mMesh[loop].ind[loopX*3+idmesh] = 0;
				// インデックス番号の重複をさけるために
				// これまでのメッシュグループのインデックス最大値を足す
				REP(i,loop){
					mMesh[loop].ind[loopX*3+idmesh] += mMesh[i].ind_max;
				}
				//
				// mesh->faces[loopX].index[idmesh] <- 頂点のインデックス値
				//
				mMesh[loop].ind[loopX*3+idmesh] += mesh->faces[loopX].index[idmesh];
//				mMesh[loop].ind[loopX*3+idmesh] += (loopX*3 + idmesh);
	//			mMesh[loop].ind_max = loopX*3+idmesh;
				// 現在のメッシュグループの中で最大のインデックスを探索
				if( mMesh[loop].ind_max < mesh->faces[loopX].index[idmesh]){
					mMesh[loop].ind_max = mesh->faces[loopX].index[idmesh];
				}
	      }
	    }

	//    m_mesh[loop].nIndex = mesh->nfaces * 3;
	//    m_mesh[loop].index.resize( m_mesh[loop].nIndex );//面の数*3頂点分=総インデックス数

	    delete [] normal;
	  }
	  lib3ds_file_free(m_model);
}

void ViewingModel::LoadObjModel( void )
{
	std::ifstream input(mModelname);

//	IndexedMesh * tmpMesh = mLSCM[0]->mesh_.get();
	IndexedMesh * tmpMesh = mLSCM->mesh_.get();
	tmpMesh->clear();
	while(input) {
		char line[1024] ;
		input.getline(line, 1024) ;
		std::stringstream line_input(line) ;
		std::string keyword ;
		line_input >> keyword ;
		if(keyword == "v") {
			Vector3 p ;
			line_input >> p.x >> p.y >> p.z ;
			tmpMesh->add_vertex(p, Vector2(0,0)) ;
		}else if(keyword == "vt") {
		 // Ignore tex vertices
		}else if(keyword == "f") {
			tmpMesh->begin_facet() ;
			while(line_input) {
				std::string s ;
				line_input >> s ;
				if(s.length() > 0) {
					std::stringstream v_input(s.c_str()) ;
					int index ;
					v_input >> index ;
					tmpMesh->add_vertex_to_facet(index - 1) ;
					char c ;
					v_input >> c ;
					if(c == '/') {
						v_input >> index ;
						// Ignore tex vertex index
					}
				}
			}
			tmpMesh->end_facet() ;
		}
	}
	std::cout << "Loaded OBJ file :" << tmpMesh->mVertices.size() << " vertices and "
	   << tmpMesh->mFaces.size() << " facets" << std::endl ;

	//Convert from OBJ format to 3DS format
	mMesh.clear();
	Mesh tmp3DSMesh;
	int nFaces= static_cast<int>(tmpMesh->mFaces.size());

	tmp3DSMesh.normal.resize(nFaces*3*3);//法線用メモリ確保(面の数*3頂点*3座標)
	tmp3DSMesh.vertex.resize(nFaces*3*3);//頂点用メモリ確保
	tmp3DSMesh.nIndex = nFaces * 3;
	tmp3DSMesh.ind.resize( tmp3DSMesh.nIndex );//面の数*3頂点分=総インデックス数

	for(int face=0; face<nFaces; face++){
		REP(i,3){
			Vector3 tmpVertex = tmpMesh->mVertices[face+i].point;
			tmp3DSMesh.vertex[face*9 + i*3 + 0] = tmpVertex.x;
			tmp3DSMesh.vertex[face*9 + i*3 + 1] = tmpVertex.y;
			tmp3DSMesh.vertex[face*9 + i*3 + 2] = tmpVertex.z;
//			cout << face*3 + i*3 << endl;
		}
	}
	mMesh.push_back(tmp3DSMesh);
	cout << "Converted OBJ to 3DS : " << tmp3DSMesh.vertex.size()/3 << " vertices and "
	   << tmp3DSMesh.nIndex/3 << " facets" << std::endl ;
	mIsConvert = true;
}

deque<Texture *> ViewingModel::LoadTextures(Lib3dsFile * pModel, string dirpath) {
	assert( pModel );
	// Creation of the texture's list
	deque<Texture *> texList;
	texList.clear();

	// Load a set of textures
	for (int ii = 0; ii < pModel->nmaterials; ++ii) {
		// Acquire a texture name
		string sTexFile = pModel->materials[ii]->texture1_map.name;

		if (!sTexFile.empty()) {
			string textureFilename = dirpath + "/" + sTexFile;
			const char * sp = strrchr(sTexFile.c_str(), '.');
			if (strcmp(sp, ".gif") == 0 || strcmp(sp, ".GIF") == 0) {
				cerr << "cvLoadImage does not support GIF format! -> "
						<< textureFilename.c_str() << endl;
				continue;
			}
			cout << "Load : " << textureFilename.c_str() << endl;
			// Create a texture object and set it to the list
			::ImageType TextureRGB = (img_load(textureFilename));
			Texture* tmpTexture = new Texture(
					static_cast<const ::ImageType> (TextureRGB));
			texList.push_back(tmpTexture);
		}
	}
	return (texList);
}

void ViewingModel::VertexCorrection( void )
{
//	REP(loop,GetMeshSize()){
//		REP(id,GetMeshIndicesSum(loop) ){
//		}
//	}
}

bool ViewingModel::LoadMatrix( void )
{
  char load_file_name[100];
  sprintf(load_file_name,"%s.txt",mModelname);
  const int MeshNum = mMesh.size();

  REP(id,MeshNum){
    //indexは0からはじまるので頂点数は+1される
    mSumOfVertices += (mMesh[id].ind_max + 1);
  }
  
  cout << "SUM OF VERTICES:" << mSumOfVertices << endl;
  // Create frequency matrix
  // Create adjacent matrix
  ublas::mapped_matrix<int> mat_frequency(mSumOfVertices,mSumOfVertices);
  ublas::mapped_matrix<int> mat_adjacent(mSumOfVertices,mSumOfVertices);

  REP(meshloop,MeshNum){       
    for(int verloop=0; verloop<(int)mMesh[meshloop].nIndex; verloop+=3){
      int ind1 = mMesh[meshloop].ind[verloop + 0];
      int ind2 = mMesh[meshloop].ind[verloop + 1];
      int ind3 = mMesh[meshloop].ind[verloop + 2];
      
      // Set frequency value
      SetFrequencyValue(ind1, ind2, ind3, mat_adjacent, mat_frequency);

      // Set adjacent value <TODO> cotangent matrixにすること
      SetAdjacentValue(ind1, ind2, ind3, mat_adjacent);
    }
  }

  ublas::matrix<int> mat_laplacian(mSumOfVertices,mSumOfVertices);
  mat_laplacian = mat_frequency - mat_adjacent;

  SparseMatrix<double> tmp_laplacian(mSumOfVertices,mSumOfVertices);
  //  SparseMatrix<double> sparse_laplacian_t(sum_of_vertex,sum_of_vertex);
  tmp_laplacian.reserve(mSumOfVertices*5);
  REP(i,mSumOfVertices){
    //    cout << i << endl;
    REP(j,mSumOfVertices){
      if(mat_laplacian(i,j) != 0 ){
	tmp_laplacian.insert(i,j) = mat_laplacian(i,j);
      }
    }
  }
  tmp_laplacian.finalize();

  // set calculated sparse laplacian 
  sparse_laplacian = SparseMatrix<double>(tmp_laplacian.transpose()) * tmp_laplacian;

//   // save laplacian matrix as text data
//   cout << "Save : matrix data..." << endl;
// //   g_mat_loader.SetSumVertices(sum_of_vertex);
// //   char save_file_name[100];
// //   sprintf(save_file_name,"%s.txt",modelname);
// //   g_mat_loader.SaveMatrixFile(&save_file_name[0], sparse_laplacian);  

//   // init Poisson parameter
  b = VectorXd::Zero(mSumOfVertices);
  mHarmonicValue = VectorXd::Zero(mSumOfVertices);

  return true;
}

// refer to paper equation(2)
// "Mesh Decomposition with Cross-Boundary Brushes"
void ViewingModel::UpdateMatrix()
{
	if( !mIsLoadMatrix ){
		cerr << "No loaded matrix has found!" << endl;
		return;
	}
	ublas::matrix<int> mat_b(mSumOfVertices + 2 * mSumOfIndices , 1);
	ublas::matrix<int> mat_wp(2 * mSumOfIndices , mSumOfVertices);

	// solve equation Ax=b
	// A:sparse matrix, x:unknown parameter vector, b:known vector
	// A = L^T*L + Q^T*Q (L:Laplacian matrix, Q:Weight Matrix)
	SparseMatrix<double> Q(mSumOfVertices,mSumOfVertices);
	b = VectorXd::Zero(mSumOfVertices);
	mHarmonicValue = VectorXd::Zero(mSumOfVertices);

	Q.reserve(2*mSumOfIndices); // desinate the number of non-zero approximately

	// a set of indices of strokes
	int * hash_table = new int[mSumOfVertices];
	REP(i,mSumOfVertices) hash_table[i] = 0;
	REP(i,mSumOfIndices){
		//     cout << "start ind:" << min_start_ind[i]<<endl;
		//     cout << "end ind:" << min_end_ind[i]<<endl;
		if( hash_table[mMinStartIndex[i]] == 0){
		  Q.insert(mMinStartIndex[i],mMinStartIndex[i]) = weight * weight;
		  hash_table[mMinStartIndex[i]] = 1;
		}
		if( hash_table[mMinEndIndex[i]] == 0){
		  Q.insert(mMinEndIndex[i],mMinEndIndex[i]) = weight * weight;
		  hash_table[mMinEndIndex[i]] = 1;
		}
	}
	delete hash_table;
	//  cout << Q << endl;
	// set rhs value
	REP(i,mSumOfIndices){
		b[mMinStartIndex[i]] = weight * weight; // estimatable vector value
	}

	// calculate matrix A
	SparseMatrix<double> A(mSumOfVertices,mSumOfVertices);
	A.reserve(mSumOfVertices*10);
	A = sparse_laplacian + Q;

	//  cout << A << endl;
	// sparse linear system solver
	SparseLLT<SparseMatrix<double>, Cholmod> llt(A);
	llt.solveInPlace(b);
	mHarmonicValue=b;
	//  SparseLU<SparseMatrix<double>,UmfPack> llt(A);
	//   if( !llt.succeeded() ){
	//     cerr << "Decomposition Failed!!" << endl;
	//     return;
	//   }
	//   // use the memory storage 'b' is placed in as x's
	//   if( !llt.solve(b,&x) ){
	//     cerr << "Solving Failed!!" << endl;
	//     return;
	//   }
	cout << "Finish : sparse linear solver calculation" << endl;
	//  cout << x << endl;
}

void ViewingModel::CorrespondTexCoord
(GLint *viewport, GLdouble *modelview, GLdouble *projection,
 cv::Point2d pStart, cv::Point2d pEnd, Vector2 & t1, Vector2 & t2
 , Vector3 & p1, Vector3 & p2)
{
	double winX,winY,winZ,objX,objY,objZ;

	// ストロークの三次元位置から最も近い頂点を探すことにする
	double minStartDist = 999999;
	double minEndDist   = 999999;

	IndexedMesh * im = mLSCM->mesh_.get();

//	REP(mesh,mMesh.size()){
	cv::Point3d meshVertex;
	double dist;
//	cout << "vertex ; " << im->vertex.size() << endl;

	cout << pStart.x << " " << pStart.y << endl;
	cout << pEnd.x << " " << pEnd.y << endl;
	for(unsigned int loopVer=0; loopVer<im->mVertices.size(); loopVer++){
		meshVertex.x = im->mVertices[loopVer].point.x;
		meshVertex.y = im->mVertices[loopVer].point.y;
		meshVertex.z = im->mVertices[loopVer].point.z;
		objX = meshVertex.x;
		objY = meshVertex.y;
		objZ = meshVertex.z;

		//得られるwindow座標は右下原点
		//  y
		//   |
		//   |
		//  ----------x
		//   |
		gluProject(objX,objY,objZ,modelview,projection,viewport,&winX,&winY,&winZ);
		winY = viewport[3]*2 - winY;

		// Judgement of pixel
		dist = sqrt( pow( (pStart.x - winX) , 2) + pow( (pStart.y - winY) , 2) );
		if( minStartDist > dist )
		{
		  minStartDist = dist;
		  t1.x = im->mVertices[loopVer].tex_coord.x;
		  t1.y = im->mVertices[loopVer].tex_coord.y;
		  p1.x = objX;
		  p1.y = objY;
		  p1.z = objZ;
		  ind1 = im->mVertices[loopVer].id;
		}

		dist = sqrt( pow( (pEnd.x - winX) , 2) + pow( (pEnd.y - winY) , 2) );
		if( minEndDist > dist )
		{
		  minEndDist = dist;
		  t2.x = im->mVertices[loopVer].tex_coord.x;
		  t2.y = im->mVertices[loopVer].tex_coord.y;
		  p2.x = objX;
		  p2.y = objY;
		  p2.z = objZ;
		  ind2 = im->mVertices[loopVer].id;
		}
//		cout << "Window = " << winX << "," << winY << endl;
	}
//	}
  cout << "Strokes' indices : " << ind1 << "--" << ind2 << endl;
}

/*
 * @separateNumber : separate model into the number of mesh parts
 */
void ViewingModel::RenewMeshDataConstruct( const int & separateNumber)
{
//	REP(nMeshes, mLSCM.size()){
//
//	}
	mLSCM->mesh_->mTexParts.clear();

	int num = GetMeshSize();
	int sumIndices = 0;
	REP(loop,num){
		int nIndices = GetMeshIndicesSum(loop);
		for(int loopv=0; loopv<nIndices; loopv++){
			double decomValue = static_cast<double>(mHarmonicValue[mMesh[loop].ind[loopv]]);
			mLSCM->mesh_->mTexParts.push_back(decomValue);

//			// a texture coord is changed in the case of another part
//			if(decomValue <= 0.5 ){
//				cout << sumIndices + loopv*3<< endl;
//				REP(id,3){
//					mLSCM->mesh_->mTextureCoords[sumIndices + loopv*3 + id].x += (mLSCM->mesh_->mTexMax.x - mLSCM->mesh_->mTexMin.x);
//					mLSCM->mesh_->mTextureCoords[sumIndices + loopv*3 + id].y += (mLSCM->mesh_->mTexMax.y - mLSCM->mesh_->mTexMin.y);
//				}
//			}
//			cout << sumIndices + loopv*3<< endl;
		}
		sumIndices += nIndices*3;
	}
}

void ViewingModel::ConvertDataStructure()
{
	if(mIsConvert) return;

	//for texture deployment
	cout << "Convert from 3DS to OBJ..." << endl;

	int num = GetMeshSize();
	int sumIndices = 0;
	REP(loop,num){
		int nFaces = GetMeshIndicesSum(loop)/3;
		for(int loopv=0; loopv<nFaces; loopv++){
//			mLSCM[0]->mesh_->begin_facet();
			mLSCM->mesh_->begin_facet();
			REP(id,3){
				Vector3 vertex;
				vertex.x = mMesh[loop].vertex[loopv*9 + id*3 + 0];
				vertex.y = mMesh[loop].vertex[loopv*9 + id*3 + 1];
				vertex.z = mMesh[loop].vertex[loopv*9 + id*3 + 2];

				//set vertex value
//				mLSCM->mesh_->add_vertex(vertex, Vector2(0,0));
//				mLSCM[0]->mesh_->add_vertex(vertex, Vector2(0,0));
				mLSCM->mesh_->add_vertex(vertex, Vector2(0,0));

				//set index corresponding to the vertex
				mLSCM->mesh_->add_vertex_to_facet(sumIndices + loopv*3 + id);
//				mLSCM[0]->mesh_->add_vertex_to_facet(sumIndices + loopv*3 + id);
//				cout << sumIndices + loopv*3 + id << endl;

				mLSCM->mesh_->mTexParts.push_back(0.0);
			}
//			mLSCM[0]->mesh_->end_facet();
			mLSCM->mesh_->end_facet();
		}
		sumIndices += nFaces*3;
	}

//	std::cout << "Converted : " << mLSCM[0]->mesh_->vertex.size() << " vertices and "
//	<< mLSCM[0]->mesh_->facet.size() << " facets" << std::endl ;
	std::cout << "Converted : " << mLSCM->mesh_->mVertices.size() << " vertices and "
	<< mLSCM->mesh_->mFaces.size() << " facets" << std::endl ;

	mIsConvert = true;
}

// *********** Query method for rendering the model *********** //
void ViewingModel::QueryNormal(const int & outer_loop, const int & mesh_index, GLdouble * normal)
{
  REP(id,3){
    normal[id] = static_cast<GLdouble>(mMesh[outer_loop].normal[mesh_index*3 + id]);
  }
}

void ViewingModel::QueryVertex(const int & outer_loop, const int & mesh_index, GLdouble * vertex)
{
  REP(id,3){
    vertex[id] = static_cast<GLdouble>(mMesh[outer_loop].vertex[mesh_index*3 + id]);
  }
  //  printf("%lf %lf %lf\n",vertex[0],vertex[1],vertex[2]);
}

double ViewingModel::QueryVertexColor(const int & outer_loop, const int & mesh_index) const
{
	if(mMesh.size() <= outer_loop)
		cout << "MeshSize:" << mMesh.size() << " OuterLoop:" << outer_loop << endl;
//	if(mMesh[outer_loop].ind.size() <= mesh_index )
//		cout << "IndSize:" << mMesh[outer_loop].ind.size() << " MeshIndex:" << mesh_index << endl;

	if( mSumOfVertices <= static_cast<int>(mMesh[outer_loop].ind[mesh_index])){
		cerr << "Error(QueryVertexColor): out of range of mesh array" << endl;
		return 1;
	}
	return mHarmonicValue[mMesh[outer_loop].ind[mesh_index]];
}
// *********** The end of Query method for rendering the model *********** //


int ViewingModel::GetMeshSize() const 
{ 
  return ( static_cast<int>(mMesh.size()) ); 
}

//int ViewingModel::GetMeshSize() const
//{
//  return ( static_cast<int>(mMesh.size()) );
//}

int ViewingModel::GetMeshIndicesSum(const int & outer_loop) const 
{
  if( static_cast<int>(mMesh.size()) <= outer_loop){
    cerr << "Error(GetMeshIndicesSum): out of range of mesh array" << endl;
    return 1;
  }
  return ( mMesh[outer_loop].nIndex ); 
}

int ViewingModel::GetMeshFlag(const int & outer_loop) const
{
  if( static_cast<int>(mMesh.size()) <= outer_loop){
    cerr << "Error(GetMeshFlag): out of range of mesh array" << endl;
    return 1;
  }
  return mMesh[outer_loop].flag;
}

void ViewingModel::IncrementSumOfStrokes()
{
  this->mSumOfIndices++;
}


/*
 *
 */
bool ViewingModel::RunLSCM(void)
{
  if(!mIsConvert){
    cerr << "Not converted yet! " << endl;
    return false;
  }

  const char * solver = "CG";
//  mLSCM->run(solver, " ");
  //  lscm->mesh_->save("test.obj");
  
  return true;
}

/*
 * @name : model name for loading
 */
ViewingModel::ViewingModel(char * name)
  :mSumOfVertices(0), mSumOfIndices(0), mScales(5.0), mIsConvert(false), mIsLoadMatrix(false)
, mHasTexture(false)
{
  mModelname = name;
  REP(i,3) mAngles[i] = 0.0;
//  mLSCM.clear();
//  boost::shared_ptr<LSCM> lscm = boost::shared_ptr<LSCM>(new LSCM());
//  mLSCM.push_back(lscm);

  mLSCM = boost::shared_ptr<LSCM>(new LSCM());
  //  mLSCM = new LSCM();

  //for mesh decomposition
  Load3DModel(); 
}

/*
 *
 */
ViewingModel::~ViewingModel()
{
//  delete mLSCM;
}
