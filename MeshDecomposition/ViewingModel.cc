/*
 * ViewingModel.cc
 *
 *  Created on: 2012/04/01
 *      Author: umakatsu
 */

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
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

//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
const int weight = 1000;

#define OVERLAID 1
//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
using namespace std;
using namespace CVD;
using namespace boost::numeric;
using namespace Eigen;

int ind1, ind2;

namespace {
	inline void SetAdjacentValue(const int & i1, const int & i2, const int & i3,
			boost::numeric::ublas::mapped_matrix<int> & mat) {
		mat(i1, i2) = 1;
		mat(i2, i1) = 1;
		mat(i1, i3) = 1;
		mat(i3, i1) = 1;
		mat(i3, i2) = 1;
		mat(i2, i3) = 1;
	}

	inline void SetFrequencyValue(const int & i1, const int & i2, const int & i3,
			const boost::numeric::ublas::mapped_matrix<int> & mat_adj,
			boost::numeric::ublas::mapped_matrix<int> & mat_fre) {
		if (mat_adj(i1, i2) == 0) {
			mat_fre(i1, i1) += 1;
			mat_fre(i2, i2) += 1;
		}
		if (mat_adj(i1, i3) == 0) {
			mat_fre(i1, i1) += 1;
			mat_fre(i3, i3) += 1;
		}
		if (mat_adj(i2, i3) == 0) {
			mat_fre(i2, i2) += 1;
			mat_fre(i3, i3) += 1;
		}
	}

	inline TextureTransfer::Vector3 OperatorEqual(float *t) {
		return TextureTransfer::Vector3(t[0], t[1], t[2]);
	}
}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
namespace TextureTransfer
{
	bool ViewingModel::CheckFittingVertices(GLint *viewport, GLdouble *modelview,
			GLdouble *projection, cv::Point2d start_point, cv::Point2d end_point)
	{
		double winX, winY, winZ, objX, objY, objZ;

		// ストロークの三次元位置から最も近い頂点を探すことにする
		double min_start = 999999;
		double min_end = 999999;

		// resize a set of stroke
		const unsigned int size_set_stroke = mMinStartIndex.size();
		mMinStartIndex.push_back(0);
		mMinEndIndex.push_back(0);

		REP(mesh,mMesh.size()) {
			Vector3 mesh_vertex;
			double dist;
			for (unsigned int loopVer = 0; loopVer < mMesh[mesh]->mVertices.size(); loopVer++)
			{
				mesh_vertex = mMesh[mesh]->mVertices[loopVer].point;
				objX = mesh_vertex.x;
				objY = mesh_vertex.y;
				objZ = mesh_vertex.z;

				gluProject(objX, objY, objZ, modelview, projection, viewport, &winX,
						&winY, &winZ);
				winY = viewport[3] * 2 - winY;

				// Judgement of pixel
				dist = sqrt(
						pow((start_point.x - winX), 2)
								+ pow((start_point.y - winY), 2));
				if (min_start > dist) {
					min_start = dist;
					mMinStartIndex[size_set_stroke] = mMesh[mesh]->mVertices[loopVer].allIndex;
				}

				dist = sqrt(
						pow((end_point.x - winX), 2)
								+ pow((end_point.y - winY), 2));
				if (min_end > dist) {
					min_end = dist;
					mMinEndIndex[size_set_stroke] = mMesh[mesh]->mVertices[loopVer].allIndex;
				}
			}
		}
		//  cout << "Strokes' indices : " << mMinEndIndex[size_set_stroke] << "--" << mMinStartIndex[size_set_stroke] << endl;
		return true;
	}

	void ViewingModel::Load3DModel() {
		char *dot;

		//check whether .3ds, .obj or others
		dot = strrchr(mModelname, '.');

		if (!strcmp(dot, ".3ds") || !strcmp(dot, ".3DS")) {
			Load3DSModel();
			mIsLoadMatrix = LoadMatrix();
		} else if (!strcmp(dot, ".obj") || !strcmp(dot, ".OBJ")) {
			LoadObjModel();
			mIsLoadMatrix = LoadMatrix();
		} else {
			cerr << "Not supported 3DS file format : " << dot << endl;
			return;
		}
	}

	void ViewingModel::Load3DSModel(void)
	{
		::Lib3dsFile *m_model; //モデル全体
		::Lib3dsMesh *mesh; //メッシュ単位

		//モデル読み込み
		m_model = lib3ds_file_open(mModelname);
		if (m_model == NULL) {
			std::cerr << "can't Open file\n";
			exit(0);
		}

		//メッシュ分メモリ確保
		REP(i,m_model->nmeshes)
		{
			mMesh.push_back( boost::shared_ptr<IndexedMesh>(new IndexedMesh()) );
		}

		REP(mats,m_model->nmaterials)
		{
			Vector3 tmpAmbient, tmpDiffuse, tmpSpecular;
			//		  tmpAmbient  = m_model->materials[mats]->ambient;
			//		  tmpDiffuse  = m_model->materials[mats]->diffuse;
			//		  tmpSpecular = m_model->materials[mats]->specular;
			tmpAmbient = OperatorEqual(m_model->materials[mats]->ambient);
			tmpDiffuse = OperatorEqual(m_model->materials[mats]->diffuse);
			tmpSpecular = OperatorEqual(m_model->materials[mats]->specular);
	//		mMesh[0].ambient.push_back(tmpAmbient);
	//		mMesh[0].diffuse.push_back(tmpDiffuse);
	//		mMesh[0].specular.push_back(tmpSpecular);
		}

		for (int loop = 0; loop < m_model->nmeshes; ++loop)
		{
			mesh = m_model->meshes[loop]; //mLoop番目のメッシュへのポインタ
			if (mesh->nfaces == 0)
			{
				mMesh[loop]->mIdxMax = 0;
				mMesh[loop]->mNumIndex = 0;
	//			mMesh[loop] = NULL;  //IndexedMeshをNULLにする
				continue;
			} //メッシュが無い場合はカット

			//法線データの取り出し
			float (*normal)[3] = new float[mesh->nfaces][3];
			lib3ds_mesh_calculate_face_normals(mesh, &normal[0]); //面法線の取り出し

	//		mMesh[loop]->mVertices.resize(mesh->nvertices); //vertex用メモリ確保
			mMesh[loop]->mNumIndex = mesh->nvertices;

	//		//インデックス最大値の初期化
			mMesh[loop]->mIdxMax = 0;

			//頂点データと法線データをインデックスに合わせて格納
			for (int loopVer = 0; loopVer < mesh->nvertices; ++loopVer)
			{
				Vector3 tmp(mesh->vertices[loopVer][0],mesh->vertices[loopVer][1],mesh->vertices[loopVer][2]);
//				tmp = mesh->vertices[loopVer];
//				printf("%f,%f,%f\n",mesh->vertices[loopVer][0], mesh->vertices[loopVer][1], mesh->vertices[loopVer][2]);
				mMesh[loop]->add_vertex(tmp, Vector2(0,0));
			}

			//面データに対応する頂点情報を格納
			for (int loopFace = 0; loopFace < mesh->nfaces; ++loopFace)
			{
				mMesh[loop]->begin_facet();
				//reserver vertex information
				REP(loopVer,3)
				{
					mMesh[loop]->add_vertex_to_facet(mesh->faces[loopFace].index[loopVer]);
					mMesh[loop]->mVertices[mesh->faces[loopFace].index[loopVer]].normal = normal[loopFace];
				}
				mMesh[loop]->end_facet();

	//			mMesh[loop].materials.push_back(mesh->faces[loopFace].material);

				// 現在のメッシュの3頂点に対してインデックスの設定
				REP(loopVer,3)
				{
					mMesh[loop]->mVertices[mesh->faces[loopFace].index[loopVer]].allIndex = 0;
					// インデックス番号の重複をさけるために
					// これまでのメッシュグループのインデックス最大値を足す
					REP(i,loop)
					{
						mMesh[loop]->mVertices[mesh->faces[loopFace].index[loopVer]].allIndex += mMesh[i]->mIdxMax;
					}
					//
					// mesh->faces[loopX].index[idmesh] <- 頂点のインデックス値
					//
					mMesh[loop]->mVertices[mesh->faces[loopFace].index[loopVer]].allIndex +=
							mesh->faces[loopFace].index[loopVer];
					//				mMesh[loop].ind[loopX*3+idmesh] += (loopX*3 + idmesh);
					//			mMesh[loop]->ind_max = loopX*3+idmesh;
					// 現在のメッシュグループの中で最大のインデックスを探索
					if (mMesh[loop]->mIdxMax < mesh->faces[loopFace].index[loopVer])
					{
						mMesh[loop]->mIdxMax = mesh->faces[loopFace].index[loopVer];
					}
				}
			}

			//    m_mesh[loop]->nIndex = mesh->nfaces * 3;
			//    m_mesh[loop].index.resize( m_mesh[loop]->nIndex );//面の数*3頂点分=総インデックス数

			delete[] normal;
		}

		//release the pointer having 3DS information
		lib3ds_file_free(m_model);
	}

	void ViewingModel::LoadObjModel(void)
	{
		std::ifstream input(mModelname);

		//mesh data structure for LSCM
		//	IndexedMesh * tmpMesh = mLSCM[0]->mesh_.get();
		IndexedMesh * tmpMesh = mLSCM->mMesh.get();
		tmpMesh->clear();

		//mesh data structure for displaying 3D model
		mMesh.clear();
		mMesh.push_back( boost::shared_ptr<IndexedMesh>(new IndexedMesh()) );
		mMesh[0]->mIdxMax = 0;
		mMesh[0]->mNumIndex  = 0;

		while (input) //until input data continues
		{
			char line[1024];
			input.getline(line, 1024);
			std::stringstream line_input(line);

			std::string keyword;
			line_input >> keyword;

			//in the case of vertex information
			if (keyword == "v")
			{
				Vector3 p;
				line_input >> p.x >> p.y >> p.z;

				tmpMesh->add_vertex(p, Vector2(0, 0));
				mMesh[0]->add_vertex(p, Vector2(0, 0));
				tmpMesh->mVertices[tmpMesh->mVertices.size()-1].textureNumber = 0;

			}
			//in the case of texture coord information
			else if (keyword == "vt")
			{
				// Ignore tex vertices
			}
			//in the case of face connectivity information
			else if (keyword == "f")
			{
				tmpMesh->begin_facet();
				mMesh[0]->begin_facet();

				while (line_input)
				{
					std::string s;
					line_input >> s;
					if (s.length() > 0)
					{
						std::stringstream v_input(s.c_str());
						int index;
						v_input >> index;

						//set face connectivity with a vertex
						tmpMesh->add_vertex_to_facet(index - 1);
						mMesh[0]->add_vertex_to_facet(index - 1);

						//set max value of indices
						if( mMesh[0]->mIdxMax < index - 1 )
						{
							mMesh[0]->mIdxMax = index - 1;
						}

						char c;
						v_input >> c;
						if (c == '/') {
							v_input >> index;
							// Ignore tex vertex index
						}
					}
				}

				mMesh[0]->end_facet();
				tmpMesh->end_facet();
			}
		}

		mMesh[0]->mNumIndex = mMesh[0]->mIdxMax;

		std::cout << "Loaded OBJ file :" << tmpMesh->mVertices.size()
				<< " vertices and " << tmpMesh->mFaces.size() << " facets"
				<< std::endl;


		if( static_cast<int>(mMesh[0]->mVertices.size()) <= mMesh[0]->mIdxMax)
		{
			cerr << "The size of vertices is less than max value of indices" << endl;
			return;
		}

		REP(loopFace, mMesh[0]->mFaces.size())
		{
			REP(loopVer, mMesh[0]->mFaces[loopFace].size())
			{
				mMesh[0]->mVertices[mMesh[0]->mFaces[loopFace].at(loopVer)].allIndex
					= mMesh[0]->mFaces[loopFace].at(loopVer);
			}
		}

		cout << "Converted OBJ to 3DS : " << mMesh[0]->mVertices.size()
				<< " vertices and " << mMesh[0]->mFaces.size() << " facets"
				<< std::endl;
		mIsConvert = true;
	}

	void ViewingModel::Save3DModel(const char * filename)
	{
		// Creation of a 3DS file
		Lib3dsFile * sModel = lib3ds_file_new();

		strcpy(sModel->name, filename);

		// Reserve the material setting
		sModel->nmaterials = static_cast<uint>(mTexture.size());
		sModel->materials = new Lib3dsMaterial*[sModel->nmaterials];

		cout << "The number of textures = " << sModel->nmaterials << endl;

		REP(m,sModel->nmaterials) {
			ostringstream textureFilename;
			textureFilename << "LSCM_texture" << m;
			sModel->materials[m] = lib3ds_material_new(
					textureFilename.str().c_str());
			textureFilename << ".bmp";
			strcpy(sModel->materials[m]->texture1_map.name,
					textureFilename.str().c_str());
		}

		ostringstream com1, com2;
		com1 << "cp texture1.bmp " << sModel->materials[0]->texture1_map.name;
		if (system(com1.str().c_str())) {
			cerr << "Error in ViewingModel: System command is not valid";
		}
		com2 << "cp warping1.bmp " << sModel->materials[1]->texture1_map.name;
		if (system(com2.str().c_str())) {
			cerr << "Error in ViewingModel: System command is not valid";
		}

		sModel->nmeshes = sModel->nmaterials;
		sModel->meshes = new Lib3dsMesh*[sModel->nmeshes];

		IndexedMesh * lscmMesh = mLSCM->mMesh.get();

		if(!lscmMesh){
			cerr << "Error in Save3dsModel: tmpMesh is null pointer!!" << endl;
		}


		// Mesh loop
		for(int texNumber=0; texNumber<sModel->nmeshes; texNumber++)
		{
			ostringstream meshFilename;
			meshFilename << "mesh" << texNumber;
			sModel->meshes[texNumber] = lib3ds_mesh_new(meshFilename.str().c_str());

			vector<int> indices;
			vector<Vertex> vertices;
			vector<int>	 indexVertex;
			vector<Facet>  faces;

			indexVertex.resize(lscmMesh->mVertices.size());

			//texNumber番目に属する頂点情報のみとりだす
			REP(verIdx, lscmMesh->mVertices.size())
			{
				if(lscmMesh->mVertices[verIdx].textureNumber == texNumber)
				{
					vertices.push_back(lscmMesh->mVertices[verIdx]);
					indexVertex[verIdx] = vertices.size() - 1;
				}
			}

			//メッシュに属する頂点数を数えてインデックスを割り振る
			REP(faceIdx, lscmMesh->mFaces.size() )
			{
				int index = lscmMesh->mFaces[faceIdx].at(0);
				if(lscmMesh->mVertices[index].textureNumber == texNumber)
				{
					Facet face;
					REP(verIdx, lscmMesh->mFaces[faceIdx].size())
					{
						face.push_back(indexVertex[ lscmMesh->mFaces[faceIdx].at(verIdx)]);
						indices.push_back(indexVertex[ lscmMesh->mFaces[faceIdx].at(verIdx)]);
					}
					faces.push_back(face);
				}
			}

//			for (unsigned int loopVer = 0; loopVer < lscmMesh->mVertices.size(); loopVer += 3)
//			{
//				//TODO 正しいデータ構造の値に修正
//				if (lscmMesh->mVertices[loopVer + 0].textureNumber == texNumber
//				||  lscmMesh->mVertices[loopVer + 1].textureNumber == texNumber
//				||	 lscmMesh->mVertices[loopVer + 2].textureNumber == texNumber)
//				{
//					REP(id,3) {
//						indices.push_back(loopVer + id);
//						nVertices++;
//					}
//					nFaces++;
//				}
//			}

			cout << "The number of vertices = " << vertices.size() << endl;
			cout << "The number of faces= " << faces.size() << endl;

			// Creating temporary memory for data of face
			sModel->meshes[texNumber]->faces = new Lib3dsFace[faces.size()];
			sModel->meshes[texNumber]->vertices = new float[vertices.size()][3];
			sModel->meshes[texNumber]->texcos = new float[vertices.size()][2];
			sModel->meshes[texNumber]->nfaces = faces.size();
			sModel->meshes[texNumber]->nvertices = vertices.size();

			//for warping texture mapping
			double ratio_x = (W_WIDTH * 0.5 - 1)
					/ (lscmMesh->mTexMax.x - lscmMesh->mTexMin.x);
			double ratio_y = (W_HEIGHT * 0.5 - 1)
					/ (lscmMesh->mTexMax.y - lscmMesh->mTexMin.y); //

			REP(faceIdx, faces.size() )
			{
				// Reserve the face setting
				sModel->meshes[texNumber]->faces[faceIdx].material = texNumber;

				// Reserve for each vertex
				REP(verIdx, faces[faceIdx].size())
				{
					int index = faces[faceIdx].at(verIdx);
					sModel->meshes[texNumber]->faces[faceIdx].index[verIdx] = index;

					sModel->meshes[texNumber]->vertices[index][0] = vertices[index].point.x;
					sModel->meshes[texNumber]->vertices[index][1] = vertices[index].point.y;
					sModel->meshes[texNumber]->vertices[index][2] = vertices[index].point.z;

					sModel->meshes[texNumber]->texcos[index][0] =
							(vertices[index].tex_coord.x
									- lscmMesh->mTexMin.x) * ratio_x / (W_WIDTH / 2);
					sModel->meshes[texNumber]->texcos[index][1] = ((W_HEIGHT / 2
							- (lscmMesh->mVertices[index].tex_coord.y
									- lscmMesh->mTexMin.y) * ratio_y) - 1)
							/ (W_HEIGHT / 2);
				}
			}
		}

		ostringstream saveName;
		saveName << filename << ".3ds";
		lib3ds_file_save(sModel, saveName.str().c_str());

		ostringstream com;
		com << "cp " << saveName.str().c_str()
				<< " LSCM_texture0.bmp LSCM_texture1.bmp ~/NewARDiorama/ARDiorama/ARMM/Data/rec/";
		if (system(com.str().c_str())){

		}
		cout << "Save 3DS..." << saveName.str().c_str() << endl;
	}

	deque<Texture *> ViewingModel::LoadTextures(::Lib3dsFile * pModel,
			string dirpath) {
		assert( pModel);
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
						static_cast<const ::ImageType>(TextureRGB));
				texList.push_back(tmpTexture);
			}
		}
		return (texList);
	}

	void ViewingModel::VertexCorrection(void) {
		//	REP(loop,GetMeshSize()){
		//		REP(id,GetMeshIndicesSum(loop) ){
		//		}
		//	}
	}

	bool ViewingModel::LoadMatrix(void) {
		char load_file_name[100];
		sprintf(load_file_name, "%s.txt", mModelname);

		REP(id,mMesh.size())
		{
			//indexは0からはじまるので頂点数は+1される
			mSumOfVertices += mMesh[id]->mVertices.size();
		}

		cout << "SUM OF VERTICES:" << mSumOfVertices << endl;
		// Create frequency matrix
		// Create adjacent matrix
		ublas::mapped_matrix<int> mat_frequency(mSumOfVertices, mSumOfVertices);
		ublas::mapped_matrix<int> mat_adjacent(mSumOfVertices, mSumOfVertices);

		REP(loopMesh,mMesh.size())
		{
			REP(loopFace, mMesh[loopMesh]->mFaces.size())
			{
				assert(mMesh[loopMesh]->mFaces[loopFace].size() == 3);

				int ind1 = mMesh[loopMesh]->mFaces[loopFace].at(0);
				int ind2 = mMesh[loopMesh]->mFaces[loopFace].at(1);
				int ind3 = mMesh[loopMesh]->mFaces[loopFace].at(2);

				// Set frequency value
				SetFrequencyValue(ind1, ind2, ind3, mat_adjacent, mat_frequency);

				// Set adjacent value <TODO> cotangent matrixにすること
				SetAdjacentValue(ind1, ind2, ind3, mat_adjacent);
			}
		}

		ublas::matrix<int> mat_laplacian(mSumOfVertices, mSumOfVertices);
		mat_laplacian = mat_frequency - mat_adjacent;

		SparseMatrix<double> tmp_laplacian(mSumOfVertices, mSumOfVertices);
		//  SparseMatrix<double> sparse_laplacian_t(sum_of_vertex,sum_of_vertex);
		tmp_laplacian.reserve(mSumOfVertices * 5);
		REP(i,mSumOfVertices) {
			REP(j,mSumOfVertices) {
				if (mat_laplacian(i, j) != 0) {
					tmp_laplacian.insert(i, j) = mat_laplacian(i, j);
				}
			}
		}
		tmp_laplacian.finalize();

		// set calculated sparse laplacian
		sparse_laplacian = SparseMatrix<double>(tmp_laplacian.transpose())
				* tmp_laplacian;

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
	void ViewingModel::UpdateMatrix() {
		if (!mIsLoadMatrix) {
			cerr << "No loaded matrix has found!" << endl;
			return;
		}
		ublas::matrix<int> mat_b(mSumOfVertices + 2 * mSumOfStrokes, 1);
		ublas::matrix<int> mat_wp(2 * mSumOfStrokes, mSumOfVertices);

		// solve equation Ax=b
		// A:sparse matrix, x:unknown parameter vector, b:known vector
		// A = L^T*L + Q^T*Q (L:Laplacian matrix, Q:Weight Matrix)
		SparseMatrix<double> Q(mSumOfVertices, mSumOfVertices);
		b = VectorXd::Zero(mSumOfVertices);
		mHarmonicValue = VectorXd::Zero(mSumOfVertices);

		Q.reserve(2 * mSumOfStrokes); // desinate the number of non-zero approximately

		// a set of indices of strokes			TransferController controller;

		int * hash_table = new int[mSumOfVertices];
		REP(i,mSumOfVertices)
			hash_table[i] = 0;
		REP(i,mSumOfStrokes) {
			//     cout << "start ind:" << min_start_ind[i]<<endl;
			//     cout << "end ind:" << min_end_ind[i]<<endl;
			if (hash_table[mMinStartIndex[i]] == 0) {
				Q.insert(mMinStartIndex[i], mMinStartIndex[i]) = weight * weight;
				hash_table[mMinStartIndex[i]] = 1;
			}
			if (hash_table[mMinEndIndex[i]] == 0) {
				Q.insert(mMinEndIndex[i], mMinEndIndex[i]) = weight * weight;
				hash_table[mMinEndIndex[i]] = 1;
			}
		}
		delete hash_table;
		//  cout << Q << endl;
		// set rhs value
		REP(i,mSumOfStrokes) {
			b[mMinStartIndex[i]] = weight * weight; // estimatable vector value
		}

		// calculate matrix A
		SparseMatrix<double> A(mSumOfVertices, mSumOfVertices);
		A.reserve(mSumOfVertices * 10);
		A = sparse_laplacian + Q;

		//  cout << A << endl;
		// sparse linear system solver
		SparseLLT<SparseMatrix<double>, Cholmod> llt(A);
		llt.solveInPlace(b);
		mHarmonicValue = b;
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

	void ViewingModel::SetSelectedMeshData(const int& loopVer)
	{
		Vector3 tmpVertex   = mLSCM->mMesh->mVertices[loopVer].point;
		Vector2 tmpTexcoord = mLSCM->mMesh->mVertices[loopVer].tex_coord;
		mSelectedMesh.second.add_vertex(tmpVertex, tmpTexcoord);
		mSelectedMesh.second.mVertices.at(mSelectedMesh.second.mVertices.size()-1).id = loopVer;
		mSelectedMesh.second.mVertices.at(mSelectedMesh.second.mVertices.size()-1).locked = false;
		mSelectedMesh.second.mVertices.at(mSelectedMesh.second.mVertices.size()-1).harmonicValue
				= mLSCM->mMesh->mVertices[loopVer].harmonicValue;

		//vertex info
//		mSelectedMesh.second.mVertices.push_back(
//				mLSCM->mMesh->mVertices[loopVer].point.x);
//		mSelectedMesh.second.vertex.push_back(
//				mLSCM->mMesh->mVertices[loopVer].point.y);
//		mSelectedMesh.second.vertex.push_back(
//				mLSCM->mMesh->mVertices[loopVer].point.z);

	}

	void ViewingModel::SetSelectedFaces(const int& loopFace)
	{
//		int index = mLSCM->mMesh->mTextureFaces.at(loopFace) - 1;
//
//		//index info
//		mSelectedMesh.second.index.push_back(loopFace);
//
//		//texture info
//		std::pair<int, Vector2> t;
//		t.first = 0;
//		t.second = mLSCM->mMesh->mTextureCoords[index];
//		mSelectedMesh.second.mTextureCoords.push_back(t);
		mSelectedMesh.second.add_vertex_to_facet(loopFace);

	}

	void ViewingModel::CorrespondTexCoord(GLint *viewport, GLdouble *modelview,
												GLdouble *projection, cv::Point2d pStart, cv::Point2d pEnd,
												Vector2 & t1, Vector2 & t2, Vector3 & p1, Vector3 & p2)
	{
		double winX, winY, winZ, objX, objY, objZ;

		// ストロークの三次元位置から最も近い頂点を探すことにする
		double minStartDist = 999999;
		double minEndDist = 999999;

		IndexedMesh * im = mLSCM->mMesh.get();

		//	REP(mesh,mMesh.size()){
		cv::Point3d meshVertex;
		double dist;

		//init selected mesh information
		mSelectedMesh.second.clear();

		for (unsigned int loopVer = 0; loopVer < im->mVertices.size(); loopVer++)
		{
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
			gluProject(objX, objY, objZ, modelview, projection, viewport, &winX,
					&winY, &winZ);
			winY = viewport[3] * 2 - winY;

			// Judgement of pixel
			dist = sqrt(pow((pStart.x - winX), 2) + pow((pStart.y - winY), 2));
			if (minStartDist > dist)
			{
				minStartDist = dist;
				t1.x = im->mVertices[loopVer].tex_coord.x;
				t1.y = im->mVertices[loopVer].tex_coord.y;
				p1.x = objX;
				p1.y = objY;
				p1.z = objZ;
				ind1 = im->mVertices[loopVer].id;

				//始点を選択したメッシュとする
				mSelectedMesh.first = ind1;
			}

			dist = sqrt(pow((pEnd.x - winX), 2) + pow((pEnd.y - winY), 2));
			if (minEndDist > dist)
			{
				minEndDist = dist;
				t2.x = im->mVertices[loopVer].tex_coord.x;
				t2.y = im->mVertices[loopVer].tex_coord.y;
				p2.x = objX;
				p2.y = objY;
				p2.z = objZ;
				ind2 = im->mVertices[loopVer].id;
			}
		}

		cout << "Clicked vertes index = " << mSelectedMesh.first << " : " << mLSCM->mMesh->mVertices[mSelectedMesh.first].harmonicValue << endl; //for debug

		//whether harmonic field have been already calculated?
//		if (!mLSCM->mMesh->mTexParts.empty())
		if(true)
		{
			//		cout << "harmonic value=" << mLSCM->mesh_->mTexParts[ mSelectedMesh.first ] << endl;
			bool hVal = mLSCM->mMesh->mVertices[mSelectedMesh.first].harmonicValue >= 0.5 ? true : false;
			REP(loopVer,im->mVertices.size())
			{
				SetSelectedMeshData(loopVer);
			}

			REP(loopFace, mLSCM->mMesh->mFaces.size())
			{
				//面の最初の頂点を対象に探索する
				int index = mLSCM->mMesh->mFaces[loopFace].at(0);

				if( (hVal && mLSCM->mMesh->mVertices[index].harmonicValue >= 0.5)
				 || (!hVal && mLSCM->mMesh->mVertices[index].harmonicValue < 0.5) )
				{
//					mLSCM->mMesh->begin_facet();
					Facet face;
					REP(loopVer, mLSCM->mMesh->mFaces[loopFace].size())
					{
						int idx = mLSCM->mMesh->mFaces[loopFace].at(loopVer);
//						SetSelectedFaces(loopFace);
						face.push_back(idx);
						mSelectedMesh.second.mVertices[idx].locked = true;
					}
					mSelectedMesh.second.mFaces.push_back(face);
//					mLSCM->mMesh->end_facet();
				}
			}
			cout << "mSelectedMesh.second.mFaces >> " << mSelectedMesh.second.mFaces.size() << endl;
		}
	}

	/*
	 * @separateNumber : separate model into the number of mesh parts
	 */
	void ViewingModel::RenewMeshDataConstruct(const int & separateNumber)
	{
		//for dynamic LSCM
		//	REP(nMeshes, mLSCM.size()){
		//
		//	}
		REP(loopMesh,GetMeshSize())
		{
			REP(loopFace,GetMeshFacesSize(loopMesh))
			{
				REP(loopVer, GetMeshInnerFacesSize(loopMesh, loopFace))
				{
					int verIdx = mMesh[loopMesh]->mFaces[loopFace].at(loopVer);
					double decomValue = static_cast<double>(mHarmonicValue[mMesh[loopMesh]->mVertices[verIdx].allIndex]);

					mLSCM->mMesh->mVertices[mMesh[loopMesh]->mVertices[verIdx].allIndex].harmonicValue = decomValue;
				}
			}
		}
	}

	/*
	 * 複数のメッシュ集合を1つにする
	 * TODO mesh synthesis
	 */
	void ViewingModel::ConvertDataStructure()
	{
		//already converted?
		if (mIsConvert){
			return;
		}

		//for texture deployment
		cout << "Convert from 3DS to OBJ..." << endl;

		//頂点情報の格納
//		REP(loopMesh, mMesh.size())
//		{
//			REP(faceIdx, mMesh[loopMesh]->mFaces.size())
//			{
//				REP(vertexIdx, mMesh[loopMesh]->mFaces[faceIdx].size())
//				{
//					int index = mMesh[loopMesh]->mFaces[faceIdx].at(vertexIdx);
//					mLSCM->mMesh->add_vertex(mMesh[loopMesh]->mVertices[index].point, Vector2(0,0));
//					mLSCM->mMesh->mVertices[]
//				}
//			}
//		}
		REP(loopMesh, mMesh.size())
		{
			REP(verIdx, mMesh[loopMesh]->mVertices.size())
			{
				mLSCM->mMesh->add_vertex(mMesh[loopMesh]->mVertices[verIdx].point, Vector2(0,0));
				mLSCM->mMesh->mVertices[verIdx].textureNumber = loopMesh;
			}
		}

//			}
//
//		}

		int sumOfVertices = 0;
		//面との連結情報の格納
		REP(loopMesh, mMesh.size())
		{
			REP(faceIdx, mMesh[loopMesh]->mFaces.size())
			{
				mLSCM->mMesh->begin_facet();
				REP(vertexIdx, mMesh[loopMesh]->mFaces[faceIdx].size())
				{
					int index = mMesh[loopMesh]->mFaces[faceIdx].at(vertexIdx);
					mLSCM->mMesh->add_vertex_to_facet(index + sumOfVertices);
				}
				mLSCM->mMesh->end_facet();
			}
			sumOfVertices += mMesh[loopMesh]->mIdxMax;
		}
//		mLSCM->mMesh = mMesh[0];
		std::cout << "Converted : " << mLSCM->mMesh->mVertices.size()
				<< " vertices and " << mLSCM->mMesh->mFaces.size() << " facets"
				<< std::endl;

		mIsConvert = true;
	}

	// *********** Query method for rendering the model *********** //
	void ViewingModel::QueryNormal(const int & outer_loop, const int & faceIdx, const int & vertexIdx, GLdouble * normal)
	{
		if( outer_loop >= static_cast<int>(mMesh.size()) )
		{
			cerr << "Error in ViewingModel: out of range of array, mMesh" << endl;
			return;
		}
		if( faceIdx >= static_cast<int>(mMesh[outer_loop]->mFaces.size()) )
		{
			cerr << "Error in ViewingModel: out of range of array, mFaces" << endl;
			return;
		}
		int index = mMesh[outer_loop]->mFaces[faceIdx].at(vertexIdx);
		normal[0] = static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].normal.x);
		normal[1] = static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].normal.y);
		normal[2] = static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].normal.z);
	}

	void ViewingModel::QueryVertex(const int & outer_loop, const int & faceIdx, const int & vertexIdx, GLdouble * vertex)
	{
		if( outer_loop >= static_cast<int>(mMesh.size()) )
		{
			cerr << "Error in ViewingModel: out of range of array, mMesh" << endl;
			return;
		}
		if( faceIdx >= static_cast<int>(mMesh[outer_loop]->mFaces.size()) )
		{
			cerr << "Error in ViewingModel: out of range of array, mFaces" << endl;
			return;
		}
		int index = mMesh[outer_loop]->mFaces[faceIdx].at(vertexIdx);
		vertex[0] = static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].point.x);
		vertex[1] = static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].point.y);
		vertex[2] = static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].point.z);
	}

	double ViewingModel::QueryVertexColor(const int & outer_loop, const int & faceIdx, const int & vertexIdx) const
	{

		if (static_cast<int>(mMesh.size()) <= outer_loop)
		{
			cout << "MeshSize:" << mMesh.size() << " OuterLoop:" << outer_loop << endl;
		}
		if( faceIdx >= static_cast<int>(mMesh[outer_loop]->mFaces.size()) )
		{
			cerr << "Error in ViewingModel: out of range of array, mFaces" << endl;
			return -1;
		}
		int index = mMesh[outer_loop]->mFaces[faceIdx].at(vertexIdx);

		return mHarmonicValue[mMesh[outer_loop]->mVertices[index].allIndex];
	}
//	void ViewingModel::QueryAmbient(const int & outer_loop, const int & mesh_index, GLfloat * ambient)
//	{
//		int ambientIndex = mMesh[outer_loop].materials[mesh_index];
//		if (ambientIndex >= 0
//				&& ambientIndex < static_cast<int>(mMesh[0].ambient.size())) {
//			//		cout << ambientIndex << endl;
//			ambient[0] = static_cast<GLfloat>(mMesh[0].ambient[ambientIndex].x);
//			ambient[1] = static_cast<GLfloat>(mMesh[0].ambient[ambientIndex].y);
//			ambient[2] = static_cast<GLfloat>(mMesh[0].ambient[ambientIndex].z);
//			ambient[3] = .0;
//		} else {
//			REP(i,3)
//				ambient[i] = 1.0f;
//		}
//		ambient[3] = .0;
//	}
//
//	void ViewingModel::QueryDiffuse(const int & outer_loop, const int & mesh_index,
//			GLfloat * diffuse) {
//		int diffuseIndex = mMesh[outer_loop].materials[mesh_index];
//		if (diffuseIndex >= 0
//				&& diffuseIndex < static_cast<int>(mMesh[0].diffuse.size())) {
//			//		cout << mMesh[outer_loop].diffuse.size() << endl;
//			diffuse[0] = static_cast<GLfloat>(mMesh[0].diffuse[diffuseIndex].x);
//			diffuse[1] = static_cast<GLfloat>(mMesh[0].diffuse[diffuseIndex].y);
//			diffuse[2] = static_cast<GLfloat>(mMesh[0].diffuse[diffuseIndex].z);
//		} else {
//			REP(i,3)
//				diffuse[i] = 1.0f;
//		}
//		diffuse[3] = .0;
//	}
//
//	void ViewingModel::QuerySpecular(const int & outer_loop, const int & mesh_index,
//			GLfloat * specular) {
//		int specularIndex = mMesh[outer_loop].materials[mesh_index];
//		if (specularIndex >= 0
//				&& specularIndex < static_cast<int>(mMesh[0].specular.size())) {
//			//		cout << mMesh[outer_loop].specular.size() << endl;
//			specular[0] = static_cast<GLfloat>(mMesh[0].specular[specularIndex].x);
//			specular[1] = static_cast<GLfloat>(mMesh[0].specular[specularIndex].y);
//			specular[2] = static_cast<GLfloat>(mMesh[0].specular[specularIndex].z);
//			specular[3] = .0;
//		} else {
//			REP(i,3)
//				specular[i] = 0.5f;
//		}
//		specular[3] = .0;
//	}


	// *********** The end of Query method for rendering the model *********** //

	int ViewingModel::GetMeshSize() const
	{
		return (static_cast<int>(mMesh.size()));
	}

	int ViewingModel::GetMeshFacesSize(const int & outer_loop) const
	{
		if (static_cast<int>(mMesh.size()) <= outer_loop) {
			cerr << "Error(GetMeshIndicesSum): out of range of mesh array" << endl;
			return 1;
		}
		return (mMesh[outer_loop]->mFaces.size());
	}

	void ViewingModel::IncrementSumOfStrokes() {
		this->mSumOfStrokes++;
	}

	/*
	 *
	 */
	bool ViewingModel::RunLSCM(void) {
		if (!mIsConvert) {
			cerr << "Not converted yet! " << endl;
			return false;
		}

		//  const char * solver = "CG";
		//  mLSCM->run(solver, " ");
		//  lscm->mesh_->save("test.obj");

		return true;
	}

	bool ViewingModel::LoadTexture(const char * filename) {
		::ImageType TextureRGB = (img_load(filename));
		Texture * tmpTexture = new Texture(
				static_cast<const ::ImageType>(TextureRGB));

		mTexture.push_back(tmpTexture);

		return true;
	}

	/*
	 * @name : model name for loading
	 */
	ViewingModel::ViewingModel(const char * name)
	: mScales(5.0), mModelname(name), mSumOfVertices(0), mSumOfStrokes(0),  mIsConvert(false), mIsLoadMatrix(false),
	  mHasTexture(false), mMeshSelected(false)
	{
		//  mModelname = name;
		REP(i,3) {
			mTrans[i] = 0.0;
			mAngles[i] = 0.0;
		}
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
	ViewingModel::~ViewingModel() {
		//  delete mLSCM;
	}
}
