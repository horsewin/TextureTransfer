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
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <cvd/image_io.h>

#include <lib3ds.h>
#include <iostream>
#include <fstream>
#include <cstdio>

//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
const int weight = 1000;

#define OVERLAID 1
#define FILE_WRITE 0		//for debug to check the coord of each vertex
#define SHIFT_SCALING 0	//for displaying model with shifting and scaling depending on the gravity and size
#define DEBUG_SAVEMODEL 0

//#define DEBUG_TEXTURE_COORD
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
			GLdouble *projection, cv::Point3d start_point, cv::Point3d end_point, bool glMouse)
	{
		double winX, winY, winZ, objX, objY, objZ;

		// ストロークの三次元位置から最も近い頂点を探すことにする
		double min_start = 999999;
		double min_end = 999999;

		// resize a set of stroke
		const unsigned int size_set_stroke = mMinStartIndex.size();
		mMinStartIndex.push_back(0);
		mMinEndIndex.push_back(0);

		//OpenGL上でマウス入力した値を使う場合(for test)
		if(glMouse)
		{
			REP(mesh,mMesh.size()) {
				Vector3 mesh_vertex;
				double dist;
				for (unsigned int loopVer = 0; loopVer < mMesh[mesh]->mVertices.size(); loopVer++)
				{
					//temporary variable to store the vertex pos to input simply
					mesh_vertex = mMesh[mesh]->mVertices[loopVer].point;
					objX = mesh_vertex.x;
					objY = mesh_vertex.y;
					objZ = mesh_vertex.z;

					gluProject(objX, objY, objZ, modelview, projection, viewport, &winX,
							&winY, &winZ);
					winY = viewport[3] * 2 - winY;

					//judging of pixels
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
		}
		// 直接入力された3次元バリューを使う場合(for AR)
		else
		{
			REP(mesh,mMesh.size())
			{
				Vector3 mesh_vertex;
				double dist;
				for (unsigned int loopVer = 0; loopVer < mMesh[mesh]->mVertices.size(); loopVer++)
				{
					//temporary variable to store the vertex pos to input simply
					mesh_vertex = mMesh[mesh]->mVertices[loopVer].point;

					//judging of pixels
						dist = sqrt(pow((start_point.x - mesh_vertex.x), 2)
									+ pow((start_point.y - mesh_vertex.y), 2)
									+ pow((start_point.z - mesh_vertex.z), 2)
									 );
					if (min_start > dist)
					{
						min_start = dist;
						mMinStartIndex[size_set_stroke] = mMesh[mesh]->mVertices[loopVer].allIndex;
					}

					dist = sqrt(pow((end_point.x - mesh_vertex.x), 2)
								+ pow((end_point.y - mesh_vertex.y), 2)
								+ pow((end_point.z - mesh_vertex.z), 2)
								 );
					if (min_end > dist)
					{
						min_end = dist;
						mMinEndIndex[size_set_stroke] = mMesh[mesh]->mVertices[loopVer].allIndex;
					}

				}
			}

		}
		//  cout << "Strokes' indices : " << mMinEndIndex[size_set_stroke] << "--" << mMinStartIndex[size_set_stroke] << endl;
		return true;
	}

	void ViewingModel::Load3DModel()
	{
		//check whether .3ds, .obj or others
		const char *extension = strrchr(mFullPath.c_str(), '.');

		if (!strcmp(extension, ".3ds") || !strcmp(extension, ".3DS"))
		{
			Load3DSModel();

			//convert to OBJ file format
			ConvertDataStructure();

			//重複頂点削除ー＞ファイルに書き戻すー＞書き戻した内容を読み直す 2012.8.?
			//DONE 冗長な処理をしているので簡単化
			if(mMesh.size() > 1)
			{
				//
				vector<int> replaceIndex = mLSCM->mMesh->VertexSynthesis();

				//スワップインデックスの更新
				if(replaceIndex.size())
				{
					int lscmIdx = 0;
					REP(loopMesh,mMesh.size())
					{
						REP(loopVer, mMesh[loopMesh]->mVertices.size())
						{
							mMesh[loopMesh]->mVertices[loopVer].allIndex = replaceIndex.at(lscmIdx);
							lscmIdx++;
						}
					}
				}
			}
		}
		else if (!strcmp(extension, ".obj") || !strcmp(extension, ".OBJ"))
		{
			LoadObjModel();
		}
		else
		{
			cerr << "Not supported modelling file format : " << extension << endl;
			return;
		}

		mSumOfVertices = mLSCM->mMesh->mVertices.size();

		LoadMatrixFromObj();

		cout << "Mesh(" << mMesh.size() << ") SUM OF VERTICES:" << mSumOfVertices << endl;
	}

	void ViewingModel::Load3DSModel(void)
	 {
	Lib3dsFile* m_model; //モデル全体
	Lib3dsMesh* mesh; //メッシュ単位
	//モデル読み込み
	m_model = lib3ds_file_open(mFullPath.c_str());
	if (m_model == NULL) {
		cerr << "can't Open file : " << mFullPath.c_str() << endl;
		exit (EXIT_FAILURE);
	}
	mTexture = LoadTextures(m_model);
	if (mTexture.size()) {
		mHasTexture = true;
	}
	//メッシュ分メモリ確保
	REP(i, m_model->nmeshes)
	{
		mMesh.push_back(boost::shared_ptr < IndexedMesh > (new IndexedMesh()));
	}
	REP(mats, m_model->nmaterials)
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
	int nface = 0;
	REP(loop, m_model->nmeshes)
	{
		mesh = m_model->meshes[loop]; //mLoop番目のメッシュへのポインタ
		nface += mesh->nfaces;
		if (mesh->nfaces == 0) {
			mMesh[loop]->mNumIndex = 0;
			continue;
		} //メッシュが無い場合はカット

		//法線データの取り出し
		float (*normal)[3] = new float[mesh->nfaces][3];
		lib3ds_mesh_calculate_face_normals(mesh, &normal[0]); //面法線の取り出し

		mMesh[loop]->mNumIndex = mesh->nvertices;

		//頂点データと法線データをインデックスに合わせて格納
		for (int loopVer = 0; loopVer < mesh->nvertices; ++loopVer) {
			Vector3 tmp(mesh->vertices[loopVer][0], mesh->vertices[loopVer][1],
					mesh->vertices[loopVer][2]);

#if SHIFT_SCALING == 1
			//recalculate the boundary of this object
			mBoundingBox.BoundaryCheck(tmp);
#endif

			if (mesh->texcos && mesh->texcos[loopVer]) {
				mMesh[loop]->AddVertex(tmp,
						Vector2(mesh->texcos[loopVer][0],
								mesh->texcos[loopVer][1]));
			} else {
				mMesh[loop]->AddVertex(tmp, Vector2(0, 0));
			}
		}

		//面データに対応する頂点情報を格納
		REP(loopFace, mesh->nfaces)
		{
			mMesh[loop]->BeginFacet();
			//reserve vertex information
#if SHIFT_SCALING == 1
			Vector3 tmpGravityVector(0,0,0);
#endif
			REP(loopVer, 3)
			{
				mMesh[loop]->AddVertex2Facet(
						mesh->faces[loopFace].index[loopVer]);
				mMesh[loop]->mVertices[mesh->faces[loopFace].index[loopVer]].normal =
						normal[loopFace];
#if SHIFT_SCALING == 1
				tmpGravityVector += mMesh[loop]->mVertices[mesh->faces[loopFace].index[loopVer]].point;
#endif
			}
			mMesh[loop]->EndFacet();

#if SHIFT_SCALING == 1
			tmpGravityVector /= 3;
			mGravityVector += tmpGravityVector;
#endif
			// 現在のメッシュの3頂点に対してインデックスの設定
			REP(loopVer, 3)
			{
				mMesh[loop]->mVertices[mesh->faces[loopFace].index[loopVer]].allIndex =
						0;
				// インデックス番号の重複をさけるために
				// これまでのメッシュグループのインデックス最大値を足す
				REP(i, loop)
				{
					mMesh[loop]->mVertices[mesh->faces[loopFace].index[loopVer]].allIndex +=
							mMesh[i]->mVertices.size();
				}
				mMesh[loop]->mVertices[mesh->faces[loopFace].index[loopVer]].allIndex +=
						mesh->faces[loopFace].index[loopVer];
			}
		}
		delete[] normal;
	}
	//calculate the gravity and the bounding box of this model
	//recalculate a position of each vertex based on the gravity vector and bounding box
	//release the pointer having 3DS information
	lib3ds_file_free(m_model);
}

void ViewingModel::LoadObjModel(const char* modelName) {
	std::ifstream input;
	if (modelName) {
		input.open(modelName);
	} else if (!mFullPath.empty()) {
		input.open(mFullPath.c_str());
	} else {
		cerr << "Error: No model is available" << endl;
		return;
	}

	//mesh data structure for LSCM
	//	IndexedMesh * tmpMesh = mLSCM[0]->mesh_.get();
	IndexedMesh* tmpMesh = mLSCM->mMesh.get();
	tmpMesh->Clear();
	//mesh data structure for displaying 3D model
	mMesh.clear();
	mMesh.push_back(boost::shared_ptr < IndexedMesh > (new IndexedMesh()));
	mMesh[0]->mNumIndex = 0;
	while (input) //until input data continues
	{
		char line[1024];
		input.getline(line, 1024);
		std::stringstream line_input(line);
		std::string keyword;
		line_input >> keyword;
		//in the case of vertex information
		if (keyword == "v") {
			Vector3 p;
			line_input >> p.x >> p.y >> p.z;
			tmpMesh->AddVertex(p, Vector2(0, 0));
			mMesh[0]->AddVertex(p, Vector2(0, 0));
			//				tmpMesh->mVertices[tmpMesh->mVertices.size()-1].textureNumber = 0;
			tmpMesh->mVertices[tmpMesh->mVertices.size() - 1].textureNumberArray.push_back(
					0);
		} else
		//in the case of texture coord information
		if (keyword == "vt") {
			// Ignore tex vertices
		} else
		//in the case of face connectivity information
		if (keyword == "f") {
			tmpMesh->BeginFacet();
			mMesh[0]->BeginFacet();
			while (line_input) {
				std::string s;
				line_input >> s;
				if (s.length() > 0) {
					std::stringstream v_input(s.c_str());
					int index;
					v_input >> index;
					//set face connectivity with a vertex(Face connectivity numbered from 1)
					tmpMesh->AddVertex2Facet(index - 1);
					mMesh[0]->AddVertex2Facet(index - 1);
					char c;
					v_input >> c;
					if (c == '/') {
						v_input >> index;
						// Ignore tex vertex index
					}
				}

			}

			mMesh[0]->EndFacet();
			tmpMesh->EndFacet();
		}

	}

	std::cout << "Loaded OBJ file :" << tmpMesh->mVertices.size()
			<< " vertices and " << tmpMesh->mFaces.size() << " facets"
			<< std::endl;
	REP(loopFace, mMesh[0]->mFaces.size())
	{
		REP(loopVer, mMesh[0]->mFaces[loopFace].size())
		{
			mMesh[0]->mVertices[mMesh[0]->mFaces[loopFace].at(loopVer)].allIndex =
					mMesh[0]->mFaces[loopFace].at(loopVer);
		}
	}
	cout << "Converted OBJ to 3DS : " << mMesh[0]->mVertices.size()
			<< " vertices and " << mMesh[0]->mFaces.size() << " facets"
			<< std::endl;
	mIsConvert = true;
}

void ViewingModel::CreateNewTextureFile(Lib3dsFile*& sModel)
{
	ostringstream com1, com2;
	if (!mTexture.empty()) {
		REP(tn, mTexture.size() - 1)
		{
			com1 << "cp " << mTexture[tn].get()->GetName() << " "
					<< sModel->materials[tn]->texture1_map.name;
			if (system(com1.str().c_str())) {
				cerr << "Error in CreateNewTextureFile of ViewingModel: System command is not valid "
						<< endl;
				cerr << "Command -> " << com1.str().c_str() << endl;
			}
			com1.str("");
		}
		com2 << "cp warping1.bmp "
				<< sModel->materials[mTexture.size() - 1]->texture1_map.name;
		if (system(com2.str().c_str())) {
			cerr << "Error in CreateNewTextureFile of ViewingModel: System command is not valid "
					<< endl;
			cerr << "Command -> " << com2.str().c_str() << endl;
		}
	} else {
		com1 << "cp texture2.bmp " << sModel->materials[0]->texture1_map.name;
		if (system(com1.str().c_str())) {
			cerr << "Error in CreateNewTextureFile of ViewingModel: System command is not valid "
					<< endl;
			cerr << "Command -> " << com1.str().c_str() << endl;
		}
		com2 << "cp warping1.bmp " << sModel->materials[1]->texture1_map.name;
		if (system(com2.str().c_str())) {
			cerr << "Error in CreateNewTextureFile of ViewingModel: System command is not valid "
					<< endl;
			cerr << "Command -> " << com2.str().c_str() << endl;
		}
	}
}

void ViewingModel::Save3DModel(const char* filename)
{
	// Creation of a 3DS file
	Lib3dsFile* sModel = lib3ds_file_new();
	strcpy(sModel->name, filename);

	// Reserve the material setting
	sModel->nmaterials	= static_cast<uint>(mTexture.size());
	sModel->materials		= new Lib3dsMaterial*[sModel->nmaterials];
	sModel->nmeshes		= sModel->nmaterials;
	sModel->meshes		= new Lib3dsMesh*[sModel->nmeshes];

	//set the name of filename of each texture
	ostringstream textureFilename[sModel->nmaterials];
	REP(m, sModel->nmaterials)
	{
		textureFilename[m] << "LSCM_" << filename << m;
		sModel->materials[m] = lib3ds_material_new(
				textureFilename[m].str().c_str());
		textureFilename[m] << ".bmp";
		strcpy(sModel->materials[m]->texture1_map.name,
				textureFilename[m].str().c_str());
	}

	CreateNewTextureFile(sModel);

	IndexedMesh* lscmMesh = mLSCM->mMesh.get();
	if (!lscmMesh) {
		cerr << "Error in Save3dsModel: tmpMesh is null pointer!!" << endl;
	}

	// Mesh loop
	for (int texNumber = 0; texNumber < sModel->nmeshes; texNumber++)
	{
		ostringstream meshFilename;
		meshFilename << "mesh" << texNumber;
		sModel->meshes[texNumber] = lib3ds_mesh_new(meshFilename.str().c_str());

		vector<int> indices;
		vector < Vertex > vertices;
		vector<int> indexVertex;
		vector < Facet > faces;

		indexVertex.resize(lscmMesh->mVertices.size());

		//texNumber番目に属する頂点情報のみとりだす
		REP(verIdx, lscmMesh->mVertices.size())
		{
			// ここが問題になっている。境界線頂点は2つ以上のテクスチャ番号をもつことがあるため
			//	それに対応するように書き直す必要あり(2012.10.19)
//				if(lscmMesh->mVertices[verIdx].textureNumber == texNumber)
//				{
//					//push to the array for calculation
//					vertices.push_back(lscmMesh->mVertices[verIdx]);
//					//reserve index info corresponding to the index before calculation
//					indexVertex[verIdx] = vertices.size() - 1;
//				}
			//revised-version to solve above problem
			REP(texID, lscmMesh->mVertices[verIdx].textureNumberArray.size())
			{
				if (lscmMesh->mVertices[verIdx].textureNumberArray.at(texID) == texNumber)
				{
					//push to the array for calculation
					vertices.push_back(lscmMesh->mVertices[verIdx]);
					//reserve index info corresponding to the index before calculation
					indexVertex[verIdx] = vertices.size() - 1;
					break;
				}

			}
		}

		//メッシュに属する頂点数を数えてインデックスを割り振る
		REP(loopFace, lscmMesh->mFaces.size())
		{
			//ある点が選択領域の条件を満たしているかCHECK
			bool compose = false;
			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);

				//revised-version to solve above problem
				//今回は面情報の保存であるためテクスチャ番号配列の先頭のみの参照でよい
				//かつ、すでに参照されている面ではないこと
				if (lscmMesh->mVertices[verIndex].textureNumberArray.at(0)
						== texNumber)
//					if(lscmMesh->mVertices[verIndex].textureNumber == texNumber)
						{
					compose = true;
					break;
				}

			}

			//this face was already restored?
			if (compose) {
				REP(loopVer, lscmMesh->mFaces[loopFace].size())
				{
					int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
					if (lscmMesh->mVertices[verIndex].textureNumberArray.at(0)
							< texNumber) {
						compose = false; //texNumber番目より前のテクスチャメッシュで参照された
						break;
					}
				}
			}

			//if above condition is satisfied
			//2012.10.22 境界線を考慮した保存において、頂点数の整合性はとれたー＞面の数がおかしい
			if (compose) {
				Facet face;

				//reserve face information that should be written into 3DS file
				REP(verIdx, lscmMesh->mFaces[loopFace].size())
				{
					face.push_back(
							indexVertex[lscmMesh->mFaces[loopFace].at(verIdx)]);
					indices.push_back(
							indexVertex[lscmMesh->mFaces[loopFace].at(verIdx)]);
				}
				faces.push_back(face);
			}
		}

		cout << "The number of vertices = " << vertices.size() << endl;
		cout << "The number of faces= " << faces.size() << endl;

		// Creating temporary memory for data of face
		sModel->meshes[texNumber]->faces = new Lib3dsFace[faces.size()];
		sModel->meshes[texNumber]->vertices = new float[vertices.size()][3];
		sModel->meshes[texNumber]->texcos = new float[vertices.size()][2];
		sModel->meshes[texNumber]->nfaces = faces.size();
		sModel->meshes[texNumber]->nvertices = vertices.size();

		//for warping texture mapping
		double ratio_x = (ConstParams::W_WIDTH * 0.5 - 0)
				/ (lscmMesh->mTexMax.x - lscmMesh->mTexMin.x);
		double ratio_y = (ConstParams::W_HEIGHT * 0.5 - 0)
				/ (lscmMesh->mTexMax.y - lscmMesh->mTexMin.y); //

		//頂点情報の保存
		double img_width = (ConstParams::W_WIDTH / 2);
		double img_height = (ConstParams::W_HEIGHT / 2);
		REP(index, vertices.size())
		{
			sModel->meshes[texNumber]->vertices[index][0] =
					vertices[index].point.x;
			sModel->meshes[texNumber]->vertices[index][1] =
					vertices[index].point.y;
			sModel->meshes[texNumber]->vertices[index][2] =
					vertices[index].point.z;

			//u-v座標系を0-1にする
			//2012.9.11 OSGで読み込ませるためにv座標を上下反転させている
			sModel->meshes[texNumber]->texcos[index][0] = (vertices[index].tex_coord.x - lscmMesh->mTexMin.x) * ratio_x / img_width;
			sModel->meshes[texNumber]->texcos[index][1] = 1 - ((img_height - (vertices[index].tex_coord.y - lscmMesh->mTexMin.y) * ratio_y) - 0)
					/ img_height;
#if DEBUG_TEXTURE_COORD
			cout << sModel->meshes[texNumber]->texcos[index][0] << "," << sModel->meshes[texNumber]->texcos[index][1] << endl;
#endif
		}

		//面情報の保存
		REP(faceIdx, faces.size())
		{
			// Reserve the face setting
			sModel->meshes[texNumber]->faces[faceIdx].material = texNumber;

			// Reserve for each vertex
			vector<int> tmpInd;
			REP(verIdx, faces[faceIdx].size())
			{
				int index = faces[faceIdx].at(verIdx);
				tmpInd.push_back(index);

//					sModel->meshes[texNumber]->vertices[index][0] = vertices[index].point.x; // 6\30 21:46
//					sModel->meshes[texNumber]->vertices[index][1] = vertices[index].point.y;
//					sModel->meshes[texNumber]->vertices[index][2] = vertices[index].point.z;
//					sModel->meshes[texNumber]->texcos[index][0] =
//						(vertices[index].tex_coord.x - lscmMesh->mTexMin.x) * ratio_x;
//					sModel->meshes[texNumber]->texcos[index][1] =
//						((W_HEIGHT / 2 - (lscmMesh->mVertices[index].tex_coord.y - lscmMesh->mTexMin.y) * ratio_y) - 1);
//---------------------------
//					sModel->meshes[texNumber]->texcos[index][0] =
//							(vertices[index].tex_coord.x - lscmMesh->mTexMin.x) * ratio_x / (W_WIDTH / 2);
//					sModel->meshes[texNumber]->texcos[index][1] =
//							((W_HEIGHT / 2 - (lscmMesh->mVertices[index].tex_coord.y - lscmMesh->mTexMin.y) * ratio_y) - 1) / (W_HEIGHT / 2);
			}
			sModel->meshes[texNumber]->faces[faceIdx].index[0] = tmpInd[0];
			sModel->meshes[texNumber]->faces[faceIdx].index[1] = tmpInd[1];
			sModel->meshes[texNumber]->faces[faceIdx].index[2] = tmpInd[2];
		}
	}

	ostringstream saveName;
	saveName << filename << ".3ds";
	lib3ds_file_save(sModel, saveName.str().c_str());

	//copy all new materials to ARMM dir
	ostringstream com;
	com << ConstParams::DATABASEDIR << filename;
	boost::filesystem::path dir(com.str().c_str());
	if (boost::filesystem::create_directory(dir))
	{
		cout << "SUCCEEDED:" << com.str().c_str() << endl;
	}
	else
	{
		com.str(""); //clear() is not a fuction to init
		com << "rm -rf " << ConstParams::DATABASEDIR << filename << " && ";
		com << "mkdir " << ConstParams::DATABASEDIR << filename;
		if (system(com.str().c_str())) {
		};
		cout << "Exec: " << com.str().c_str() << endl;
	}

	com.str("");
	com << "cp " << saveName.str().c_str() << " ";
	REP(tn, mTexture.size())
	{
		com << textureFilename[tn].str().c_str() << " ";
	}
	com << ConstParams::DATABASEDIR << filename;
	if (system(com.str().c_str()))
	{
		cerr << "Error in ViewingModel: System command is not valid " << endl;
		cerr << "Command -> " << com.str().c_str() << endl;
	}
	cout << "Exec: " << com.str().c_str() << endl;
	cout << "Save 3DS..." << saveName.str().c_str() << endl;
}

void ViewingModel::Save3DModelRevised(const char* filename)
{
	// Creation of a 3DS file
	Lib3dsFile* sModel = lib3ds_file_new();
	strcpy(sModel->name, filename);

	// Reserve the material setting
	sModel->nmaterials	= static_cast<uint>(mTexture.size());
	sModel->materials		= new Lib3dsMaterial*[sModel->nmaterials];
	sModel->nmeshes		= sModel->nmaterials;
	sModel->meshes		= new Lib3dsMesh*[sModel->nmeshes];

	//set the name of filename of each texture
	ostringstream textureFilename[sModel->nmaterials];
	REP(m, sModel->nmaterials)
	{
		textureFilename[m] << "LSCM_" << filename << m;
		sModel->materials[m] = lib3ds_material_new(
				textureFilename[m].str().c_str());
		textureFilename[m] << ".bmp";
		strcpy(sModel->materials[m]->texture1_map.name,
				textureFilename[m].str().c_str());
	}

	CreateNewTextureFile(sModel);

	IndexedMesh* lscmMesh = mLSCM->mMesh.get();
	if (!lscmMesh) {
		cerr << "Error in Save3dsModel: tmpMesh is null pointer!!" << endl;
	}

	//---->REVISED PART
	REP(texNumber, mTexture.size())
	{
		ostringstream meshFilename;
		meshFilename << "mesh" << texNumber;
		sModel->meshes[texNumber] = lib3ds_mesh_new(meshFilename.str().c_str());

		vector<int> indices;
		vector < Vertex > vertices;
		vector<int> indexVertex;
		vector < Facet > faces;

		indexVertex.resize(lscmMesh->mVertices.size());

		REP(loopFace, lscmMesh->mFaces.size())
		{
			//対象の面が同一のテクスチャ画像を参照しているかチェック
			bool compose = false;

			int faceIdx;
			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				faceIdx = lscmMesh->mTexnumVernum[loopFace].mModelFaceIdx;
				if (lscmMesh->mTexnumVernum[loopFace].mTextureNumber == texNumber)
				{
					compose = true;
					int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
					int corVerIdx = mMesh[texNumber]->mFaces[faceIdx].at(loopVer);

					//push to the array for calculation
					vertices.push_back(lscmMesh->mVertices[verIndex]);
					//reserve index info corresponding to the index before calculation
					indexVertex[verIndex] = vertices.size() - 1;

					//rewrite the tex coord from vertex pos in Monitor to tex coord in Texture mapping
					vertices.back().tex_coord.x = mMesh[texNumber]->mVertices[corVerIdx].tex_coord.x;
					vertices.back().tex_coord.y = mMesh[texNumber]->mVertices[corVerIdx].tex_coord.y;
				}
			}

//			if (!compose) continue;
			//this face was already restored?
			if (compose)
			{
				faceIdx = lscmMesh->mTexnumVernum[loopFace].mModelFaceIdx;
				REP(loopVer, lscmMesh->mFaces[loopFace].size())
				{
					if (lscmMesh->mTexnumVernum[loopFace].mTextureNumber < texNumber)
					{
						compose = false; //texNumber番目より前のテクスチャメッシュで参照された
						break;
					}
				}
			}

			//if above condition is satisfied
			//2012.10.22 境界線を考慮した保存において、頂点数の整合性はとれたー＞面の数がおかしい
			if (compose)
			{
				Facet face;
				//reserve face information that should be written into 3DS file
				REP(verIdx, lscmMesh->mFaces[loopFace].size())
				{
					face.push_back( indexVertex[lscmMesh->mFaces[loopFace].at(verIdx)]);
					indices.push_back( indexVertex[lscmMesh->mFaces[loopFace].at(verIdx)]);
				}
				faces.push_back(face);
			}

		}
#if DEBUG_SAVEMODEL == 1
		cout << "The number of vertices = " << vertices.size() << endl;
		cout << "The number of faces= " << faces.size() << endl;
#endif

		// Creating temporary memory for data of face
		sModel->meshes[texNumber]->faces = new Lib3dsFace[faces.size()];
		sModel->meshes[texNumber]->vertices = new float[vertices.size()][3];
		sModel->meshes[texNumber]->texcos = new float[vertices.size()][2];
		sModel->meshes[texNumber]->nfaces = faces.size();
		sModel->meshes[texNumber]->nvertices = vertices.size();

		REP(index, vertices.size())
		{
			sModel->meshes[texNumber]->vertices[index][0] = vertices[index].point.x;
			sModel->meshes[texNumber]->vertices[index][1] = vertices[index].point.y;
			sModel->meshes[texNumber]->vertices[index][2] = vertices[index].point.z;

			//u-v座標系を0-1にする
			sModel->meshes[texNumber]->texcos[index][0] = vertices[index].tex_coord.x;
			sModel->meshes[texNumber]->texcos[index][1] = vertices[index].tex_coord.y;
#if DEBUG_TEXTURE_COORD
			cout << sModel->meshes[texNumber]->texcos[index][0] << "," << sModel->meshes[texNumber]->texcos[index][1] << endl;
#endif
		}

		//面情報の保存
		REP(faceIdx, faces.size())
		{
			// Reserve the face setting
			sModel->meshes[texNumber]->faces[faceIdx].material = texNumber;

			// Reserve for each vertex
			vector<int> tmpInd;
			REP(verIdx, faces[faceIdx].size())
			{
				int index = faces[faceIdx].at(verIdx);
				tmpInd.push_back(index);
			}
			sModel->meshes[texNumber]->faces[faceIdx].index[0] = tmpInd[0];
			sModel->meshes[texNumber]->faces[faceIdx].index[1] = tmpInd[1];
			sModel->meshes[texNumber]->faces[faceIdx].index[2] = tmpInd[2];
		}
	}
	//<----REVISED PART END

	ostringstream saveName;
	saveName << filename << ".3ds";
	lib3ds_file_save(sModel, saveName.str().c_str());

	//copy all new materials to ARMM dir
	ostringstream com;
	com << ConstParams::DATABASEDIR << filename;
	boost::filesystem::path dir(com.str().c_str());
	if (boost::filesystem::create_directory(dir))
	{
		cout << "SUCCEEDED:" << com.str().c_str() << endl;
	}
	else
	{
		com.str(""); //clear() is not a fuction to init
		com << "rm -rf " << ConstParams::DATABASEDIR << filename << " && ";
		com << "mkdir " << ConstParams::DATABASEDIR << filename;
		if (system(com.str().c_str())) {
		};
		cout << "Exec: " << com.str().c_str() << endl;
	}

	com.str("");
	com << "cp " << saveName.str().c_str() << " ";
	REP(tn, mTexture.size())
	{
		com << textureFilename[tn].str().c_str() << " ";
	}
	com << ConstParams::DATABASEDIR << filename;
	if (system(com.str().c_str()))
	{
		cerr << "Error in ViewingModel: System command is not valid " << endl;
		cerr << "Command -> " << com.str().c_str() << endl;
	}
	cout << "Exec: " << com.str().c_str() << endl;
	cout << "Save 3DS..." << saveName.str().c_str() << endl;
}

void ViewingModel::AdjustmentSizeAndPosition() {
}

bool ViewingModel::LoadMatrixFrom3ds(void) {
	char load_file_name[100];
	sprintf(load_file_name, "%s.txt", mFullPath.c_str());
	// Create frequency matrix
	// Create adjacent matrix
	ublas::mapped_matrix<int> mat_frequency(mSumOfVertices, mSumOfVertices);
	ublas::mapped_matrix<int> mat_adjacent(mSumOfVertices, mSumOfVertices);
	REP(loopMesh, mMesh.size())
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
	REP(i, mSumOfVertices)
	{
		REP(j, mSumOfVertices)
		{
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

bool ViewingModel::LoadMatrixFromObj(void) {
	char load_file_name[100];
	sprintf(load_file_name, "%s.txt", mFullPath.c_str());
	// Create frequency matrix
	// Create adjacent matrix
	ublas::mapped_matrix<int> mat_frequency(mSumOfVertices, mSumOfVertices);
	ublas::mapped_matrix<int> mat_adjacent(mSumOfVertices, mSumOfVertices);
	IndexedMesh* lscmMesh = mLSCM->mMesh.get();
	REP(loopFace, lscmMesh->mFaces.size())
	{
		assert(lscmMesh->mFaces[loopFace].size() == 3);

		int ind1 = lscmMesh->mFaces[loopFace].at(0);
		int ind2 = lscmMesh->mFaces[loopFace].at(1);
		int ind3 = lscmMesh->mFaces[loopFace].at(2);

		// Set frequency value
		SetFrequencyValue(ind1, ind2, ind3, mat_adjacent, mat_frequency);

		// Set adjacent value <TODO> cotangent matrixにすること
		SetAdjacentValue(ind1, ind2, ind3, mat_adjacent);
	}
	ublas::matrix<int> mat_laplacian(mSumOfVertices, mSumOfVertices);
	mat_laplacian = mat_frequency - mat_adjacent;
	SparseMatrix<double> tmp_laplacian(mSumOfVertices, mSumOfVertices);
	//  SparseMatrix<double> sparse_laplacian_t(sum_of_vertex,sum_of_vertex);
	tmp_laplacian.reserve(mSumOfVertices * 20);
	REP(i, mSumOfVertices)
	{
		REP(j, mSumOfVertices)
		{
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
	int* hash_table = new int[mSumOfVertices];
	REP(i, mSumOfVertices)
	{
		hash_table[i] = 0;
	}
	REP(i, mSumOfStrokes)
	{
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
	REP(i, mSumOfStrokes)
	{
		b[mMinStartIndex[i]] = weight * weight; // estimatable vector value
	}
	// calculate matrix A
	SparseMatrix<double> A(mSumOfVertices, mSumOfVertices);
	A.reserve(mSumOfVertices * 10);
	A = sparse_laplacian + Q;
	//  cout << A << endl;
	// sparse linear system solver
	SparseLLT < SparseMatrix<double>, Cholmod > llt(A);
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

void ViewingModel::SetSelectedMeshData(const int& loopVer) {
	Vector3 tmpVertex = mLSCM->mMesh->mVertices[loopVer].point;
	Vector2 tmpTexcoord = mLSCM->mMesh->mVertices[loopVer].tex_coord;
	mSelectedMesh.second.AddVertex(tmpVertex, tmpTexcoord);
	mSelectedMesh.second.mVertices.at(mSelectedMesh.second.mVertices.size() - 1).id =
			loopVer;
	mSelectedMesh.second.mVertices.at(mSelectedMesh.second.mVertices.size() - 1).locked =
			false;
	mSelectedMesh.second.mVertices.at(mSelectedMesh.second.mVertices.size() - 1).harmonicValue =
			mLSCM->mMesh->mVertices[loopVer].harmonicValue;
	//vertex info
	//		mSelectedMesh.second.mVertices.push_back(
	//				mLSCM->mMesh->mVertices[loopVer].point.x);
	//		mSelectedMesh.second.vertex.push_back(
	//				mLSCM->mMesh->mVertices[loopVer].point.y);
	//		mSelectedMesh.second.vertex.push_back(
	//				mLSCM->mMesh->mVertices[loopVer].point.z);
}

void ViewingModel::SetSelectedFaces(const int& loopFace) {
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
	mSelectedMesh.second.AddVertex2Facet(loopFace);
}

void ViewingModel::CorrespondTexCoord(GLint* viewport, GLdouble* modelview,
		GLdouble* projection, cv::Point3d pStart, cv::Point3d pEnd, Vector2& t1,
		Vector2& t2, Vector3& p1, Vector3& p2, bool glMouse) {
	double winX, winY, winZ, objX, objY, objZ;
	// ストロークの三次元位置から最も近い頂点を探すことにする
	double minStartDist = 999999;
	double minEndDist = 999999;
	IndexedMesh* lscmMesh = mLSCM->mMesh.get();
	cv::Point3d meshVertex;
	double dist;
	//init selected mesh information
	mSelectedMesh.second.Clear();
	mSelectedFace.clear();
	if (glMouse) {
		for (unsigned int loopVer = 0; loopVer < lscmMesh->mVertices.size();
				loopVer++) {
			meshVertex.x = lscmMesh->mVertices[loopVer].point.x;
			meshVertex.y = lscmMesh->mVertices[loopVer].point.y;
			meshVertex.z = lscmMesh->mVertices[loopVer].point.z;
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
			if (minStartDist > dist) {
				minStartDist = dist;
				t1.x = lscmMesh->mVertices[loopVer].tex_coord.x;
				t1.y = lscmMesh->mVertices[loopVer].tex_coord.y;
				p1.x = objX;
				p1.y = objY;
				p1.z = objZ;
				ind1 = lscmMesh->mVertices[loopVer].id;
				//始点を選択したメッシュとする
				mSelectedMesh.first = ind1;
			}
			dist = sqrt(pow((pEnd.x - winX), 2) + pow((pEnd.y - winY), 2));
			if (minEndDist > dist) {
				minEndDist = dist;
				t2.x = lscmMesh->mVertices[loopVer].tex_coord.x;
				t2.y = lscmMesh->mVertices[loopVer].tex_coord.y;
				p2.x = objX;
				p2.y = objY;
				p2.z = objZ;
				ind2 = lscmMesh->mVertices[loopVer].id;
			}
		}

	} else //for direct input from a file
	{
		for (unsigned int loopVer = 0; loopVer < lscmMesh->mVertices.size();
				loopVer++) {
			meshVertex.x = lscmMesh->mVertices[loopVer].point.x;
			meshVertex.y = lscmMesh->mVertices[loopVer].point.y;
			meshVertex.z = lscmMesh->mVertices[loopVer].point.z;
			// Judgement of pixel
			//judging of pixels
			dist = sqrt(
					pow((pStart.x - meshVertex.x), 2)
							+ pow((pStart.y - meshVertex.y), 2)
							+ pow((pStart.z - meshVertex.z), 2));
			if (minStartDist > dist) {
				minStartDist = dist;
				t1.x = lscmMesh->mVertices[loopVer].tex_coord.x;
				t1.y = lscmMesh->mVertices[loopVer].tex_coord.y;
				p1.x = pStart.x;
				p1.y = pStart.y;
				p1.z = pStart.z;
				ind1 = lscmMesh->mVertices[loopVer].id;
				//始点を選択したメッシュとする
				mSelectedMesh.first = ind1;
			}
		}

	}

	cout << "Clicked vertes index = " << mSelectedMesh.first << " : "
			<< mLSCM->mMesh->mVertices[mSelectedMesh.first].harmonicValue
			<< endl; //for debug
	//whether harmonic field have been already calculated?
	//		if (!mLSCM->mMesh->mTexParts.empty())
	if (true) {
		//		cout << "harmonic value=" << mLSCM->mesh_->mTexParts[ mSelectedMesh.first ] << endl;
		bool hVal =
				mLSCM->mMesh->mVertices[mSelectedMesh.first].harmonicValue
						>= 0.5 ? true : false;
		REP(loopVer, lscmMesh->mVertices.size())
		{
			SetSelectedMeshData (loopVer);
		}

		REP(loopFace, mLSCM->mMesh->mFaces.size())
		{
			//すべての面が選択領域の条件を満たしているかCHECK
			bool compose = false;
			REP(loopVer, mLSCM->mMesh->mFaces[loopFace].size())
			{
				int index = mLSCM->mMesh->mFaces[loopFace].at(loopVer);
				if ((hVal && mLSCM->mMesh->mVertices[index].harmonicValue >= 0.5)
						|| (!hVal
								&& mLSCM->mMesh->mVertices[index].harmonicValue
										< 0.5)) {
					//do nothing
					compose = true;
				} else {
//						compose = false;
				}
			}

			if (compose) {
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
				mSelectedFace.push_back(loopFace);
//					mLSCM->mMesh->end_facet();
			}
		}
		cout << "mSelectedMesh.second.mFaces >> "
				<< mSelectedMesh.second.mFaces.size() << endl;
	}
}

/*
 * @separateNumber : separate model into the number of mesh parts
 */
void ViewingModel::RenewMeshDataConstruct() {
	REP(loopMesh, mMesh.size())
	{
//			REP(loopFace,GetMeshFacesSize(loopMesh))
		REP(loopFace, mMesh[loopMesh]->mFaces.size())
		{
//				REP(loopVer, GetMeshInnerFacesSize(loopMesh, loopFace))
			REP(loopVer, mMesh[loopMesh]->mFaces[loopFace].size())
			{
				const int verIdx = mMesh[loopMesh]->mFaces[loopFace].at(
						loopVer);
				const double decomValue =
						static_cast<double>(mHarmonicValue[mMesh[loopMesh]->mVertices[verIdx].allIndex]);
				const int allIdx = mMesh[loopMesh]->mVertices[verIdx].allIndex;
				mLSCM->mMesh->mVertices[allIdx].harmonicValue = decomValue; //!!BUG!! 2012.9.22 2:33
				mMesh[loopMesh]->mVertices[verIdx].harmonicValue = decomValue;
			}
		}
	}
}

void ViewingModel::ConvertDataStructure()
{
	//already converted?
	if (mIsConvert) {
		return;
	}

	//storing information of each vertex
	int sumOfVertices = 0;
	REP(loopMesh, mMesh.size())
	{
		REP(verIdx, mMesh[loopMesh]->mVertices.size())
		{
			mLSCM->mMesh->AddVertex(mMesh[loopMesh]->mVertices[verIdx].point,
					mMesh[loopMesh]->mVertices[verIdx].tex_coord);
#if FILE_WRITE == 1
			out << "v "
			<< mLSCM->mMesh->mVertices[verIdx].point.x << "\t"
			<< mLSCM->mMesh->mVertices[verIdx].point.y << "\t"
			<< mLSCM->mMesh->mVertices[verIdx].point.z << "\t"
			<< std::endl;
#endif
			//init texture number information
			mLSCM->mMesh->mVertices[verIdx + sumOfVertices].textureNumberArray.clear();
			mLSCM->mMesh->mVertices[verIdx + sumOfVertices].textureNumberArray.push_back(loopMesh);
		}
		sumOfVertices += mMesh[loopMesh]->mVertices.size();
	}

	//storing information of connectivity about a face with vertices
	sumOfVertices = 0;
	REP(loopMesh, mMesh.size())
	{
		REP(faceIdx, mMesh[loopMesh]->mFaces.size())
		{
			//テクスチャマッピングのため、mModelとmLSCMモデルの頂点の対応付け(1)
			ModelCorresPondense tmpModel(loopMesh, faceIdx);
			mLSCM->mMesh->mTexnumVernum.push_back(tmpModel);

			mLSCM->mMesh->BeginFacet();
			REP(vertexIdx, mMesh[loopMesh]->mFaces[faceIdx].size())
			{
				int index = mMesh[loopMesh]->mFaces[faceIdx].at(vertexIdx);
				mLSCM->mMesh->AddVertex2Facet(index + sumOfVertices);

				//テクスチャマッピングのため、mModelとmLSCMモデルの頂点の対応付け(2)
				mLSCM->mMesh->mTexnumVernum.back().mVertexIdxArray.push_back(vertexIdx);
			}
			mLSCM->mMesh->EndFacet();
		}
		sumOfVertices += mMesh[loopMesh]->mVertices.size();
	}

	std::cout << "Converted : " << mLSCM->mMesh->mVertices.size()
			<< " vertices and " << mLSCM->mMesh->mFaces.size() << " facets"
			<< std::endl;
	mIsConvert = true;
}

// *********** Query method for rendering the model *********** //
void ViewingModel::QueryNormal(const int& outer_loop, const int& faceIdx,
		const int& vertexIdx, GLdouble* normal) {
	if (outer_loop >= static_cast<int>(mMesh.size())) {
		cerr << "Error in ViewingModel: out of range of array, mMesh" << endl;
		return;
	}
	if (faceIdx >= static_cast<int>(mMesh[outer_loop]->mFaces.size())) {
		cerr << "Error in ViewingModel: out of range of array, mFaces" << endl;
		return;
	}
	int index = mMesh[outer_loop]->mFaces[faceIdx].at(vertexIdx);
	normal[0] =
			static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].normal.x);
	normal[1] =
			static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].normal.y);
	normal[2] =
			static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].normal.z);
}

void ViewingModel::QueryVertex(const int& outer_loop, const int& faceIdx,
		const int& vertexIdx, GLdouble* vertex) {
	if (outer_loop >= static_cast<int>(mMesh.size())) {
		cerr << "Error in ViewingModel: out of range of array, mMesh" << endl;
		return;
	}
	if (faceIdx >= static_cast<int>(mMesh[outer_loop]->mFaces.size())) {
		cerr << "Error in ViewingModel: out of range of array, mFaces" << endl;
		return;
	}
	int index = mMesh[outer_loop]->mFaces[faceIdx].at(vertexIdx);
	vertex[0] =
			static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].point.x);
	vertex[1] =
			static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].point.y);
	vertex[2] =
			static_cast<GLdouble>(mMesh[outer_loop]->mVertices[index].point.z);
}

double ViewingModel::QueryVertexColor(const int& outer_loop, const int& faceIdx,
		const int& vertexIdx) const {
	if (static_cast<int>(mMesh.size()) <= outer_loop) {
		cout << "MeshSize:" << mMesh.size() << " OuterLoop:" << outer_loop
				<< endl;
	}
	if (faceIdx >= static_cast<int>(mMesh[outer_loop]->mFaces.size())) {
		cerr << "Error in ViewingModel: out of range of array, mFaces" << endl;
		return -1;
	}
	int index = mMesh[outer_loop]->mFaces[faceIdx].at(vertexIdx);
	return mMesh[outer_loop]->mVertices[index].harmonicValue;
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
int ViewingModel::GetMeshFacesSize(const int& outer_loop) const {
	if (static_cast<int>(mMesh.size()) <= outer_loop) {
		cerr << "Error(GetMeshIndicesSum): out of range of mesh array" << endl;
		return 1;
	}
	return (mMesh[outer_loop]->mFaces.size());
}

int ViewingModel::GetMeshInnerFacesSize(const int& outer_loop,
		const int& inner_loop) const {
	if (static_cast<int>(mMesh.size()) <= outer_loop) {
		cerr << "Error(GetMeshInnerFacesSize): out of range of mesh array"
				<< endl;
		return 1;
	}
	if (static_cast<int>(mMesh[outer_loop]->mFaces.size()) <= inner_loop) {
		cerr << "Error(GetMeshInnerFacesSize): out of range of mFace array"
				<< endl;
		return 1;
	}
	return mMesh[outer_loop]->mFaces[inner_loop].size();
}

void ViewingModel::IncrementSumOfStrokes()
{
	this->mSumOfStrokes++;
}

std::deque< boost::shared_ptr<Texture> > ViewingModel::LoadTextures(Lib3dsFile* pModel)
{
	assert(pModel);

	// create the texture's list
	std::deque< boost::shared_ptr<Texture> >  texList;
	texList.clear();

	// create dirpath of the loaded model
	string dirPath(mFullPath), tmpPath;
	tmpPath = strrchr(dirPath.c_str(), '/');
	dirPath.resize(dirPath.size() - tmpPath.size());

	// Load a set of textures
	REP(material, pModel->nmaterials)
	{
		// Acquire a texture name
		string sTexFile = pModel->materials[material]->texture1_map.name;

		if (!sTexFile.empty())
		{
			string textureFilename = dirPath + "/" + sTexFile;

			const char * sp = strrchr(sTexFile.c_str(), '.');
			if (strcmp(sp, ".gif") == 0 || strcmp(sp, ".GIF") == 0)
			{
				cerr << "cvLoadImage does not support GIF format! -> "
						<< textureFilename.c_str() << endl;
				continue;
			}

//			cout << "Load : " << textureFilename.c_str() << endl;

			// Create a texture object and set it to the list
			::ImageType TextureRGB = (img_load(textureFilename));
			boost::shared_ptr<Texture> tmpTexture = boost::shared_ptr<Texture>
			(
					new Texture( static_cast<const ::ImageType>(TextureRGB), textureFilename.c_str())
			);
			texList.push_back(tmpTexture);
		}
	}

	return (texList);
}

void ViewingModel::LoadTexture(const char* filename)
{
	if (mHasTexture) return;

	::ImageType TextureRGB = (img_load(filename));
	boost::shared_ptr<Texture> tmpTexture = boost::shared_ptr<Texture>
	(
			new Texture(static_cast<const ::ImageType>(TextureRGB), filename)
	);

	mTexture.push_back(tmpTexture);

	IndexedMesh* lscmMesh = mLSCM->mMesh.get();

	REP(texNumber, mTexture.size())
	{
		//for warping texture mapping to size (W_WIDTH/2, W_HEIGHT/2)
		double ratio_x = (ConstParams::W_WIDTH * 0.5 - 0)
				/ (lscmMesh->mTexMax.x - lscmMesh->mTexMin.x);
		double ratio_y = (ConstParams::W_HEIGHT * 0.5 - 0)
				/ (lscmMesh->mTexMax.y - lscmMesh->mTexMin.y);

		REP(loopFace, lscmMesh->mFaces.size())
		{
			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);

				//unified tex coords between 0 and 1
				mMesh[texNumber]->mVertices[verIndex].tex_coord.x =
						((lscmMesh->mVertices[verIndex].tex_coord.x
						- lscmMesh->mTexMin.x) * ratio_x )/ mTexture[texNumber]->getWidth();
				mMesh[texNumber]->mVertices[verIndex].tex_coord.y =
						(((lscmMesh->mVertices[verIndex].tex_coord.y
						- lscmMesh->mTexMin.y) * ratio_y))/mTexture[texNumber]->getHeight();
			}
		}
	}
	//new texture is assigned to this model
	mNewTexture = true;
	cout << "This model has loaded new texture!!!" << endl;
}

	/*
	 * rewrite the loaded model info to file with 3DS format
	 * This function is used effectively to adjust the position and scale of each vertex
	 */
	void ViewingModel::WritebackTo3ds()
	{
		Lib3dsFile *lModel; //モデル全体
		//モデル読み込み
		lModel = lib3ds_file_open(mFullPath.c_str());
		if (lModel == NULL)
		{
			cerr << "can't Open file : " << mFullPath.c_str() << endl;
			exit(EXIT_FAILURE);
		}

		//create a new 3DS file
		Lib3dsFile * sModel = lib3ds_file_new();
		strcpy( sModel->name, mModelname.c_str());

		//reserve the setting of materials
		sModel->nmaterials	= mMesh.size();
		sModel->materials		= new Lib3dsMaterial*[sModel->nmaterials];

		//reserve the setting of meshes
		sModel->nmeshes	= mMesh.size();
		sModel->meshes	= new Lib3dsMesh*[sModel->nmeshes];

		REP(nm, sModel->nmeshes)
		{
			string str(lModel->materials[nm]->texture1_map.name);
			sModel->materials[nm] = lib3ds_material_new(str.c_str() );
			strcpy( sModel->materials[nm]->texture1_map.name, str.c_str());

			ostringstream meshFilename;
			meshFilename << "mesh" << nm;
			sModel->meshes[nm] = lib3ds_mesh_new( meshFilename.str().c_str() );

			// create temporary memory for restoring data
			sModel->meshes[nm]->nfaces		= mMesh[nm]->mFaces.size();
			sModel->meshes[nm]->nvertices	= mMesh[nm]->mVertices.size();
			sModel->meshes[nm]->faces		= new Lib3dsFace[sModel->meshes[nm]->nfaces];
			sModel->meshes[nm]->vertices	= new float[sModel->meshes[nm]->nvertices][3];
			sModel->meshes[nm]->texcos		= new float[sModel->meshes[nm]->nvertices][2];

			//writeback the info of each vertex
			REP(loopVer, mMesh[nm]->mVertices.size())
			{
				sModel->meshes[nm]->vertices[loopVer][0]	= mMesh[nm]->mVertices[loopVer].point.x;
				sModel->meshes[nm]->vertices[loopVer][1]	= mMesh[nm]->mVertices[loopVer].point.y;
				sModel->meshes[nm]->vertices[loopVer][2]	= mMesh[nm]->mVertices[loopVer].point.z;

//				sModel->meshes[nm]->texcos[loopVer][0]		= mMesh[nm]->mVertices[loopVer].tex_coord.x;
//				sModel->meshes[nm]->texcos[loopVer][1]		= mMesh[nm]->mVertices[loopVer].tex_coord.y;

				//to unify the Yasuhara's reconstructed model into a readable model by osgviewer
				sModel->meshes[nm]->texcos[loopVer][0]		= mMesh[nm]->mVertices[loopVer].tex_coord.x/640;
				sModel->meshes[nm]->texcos[loopVer][1]		= 1 - mMesh[nm]->mVertices[loopVer].tex_coord.y/480;
			}

			//writeback the info of connectivity of facets
			REP(loopFace, mMesh[nm]->mFaces.size())
			{
				//set the index of material corresponding to this face
				sModel->meshes[nm]->faces[loopFace].material = nm;

				//set the index of vertex corresponding to each vertex
				REP(id,3)
				{
					sModel->meshes[nm]->faces[loopFace].index[id] = mMesh[nm]->mFaces[loopFace].at(id);
				}
			}
		}
		lib3ds_file_save( sModel , mFullPath.c_str());

		printf("Writeback the info of model -> %s\n", mFullPath.c_str());
	}

	/*
	 * @name : model name for loading
	 */
	ViewingModel::ViewingModel(const char * name, const char * fullpath)
	: mScales(5.0), mModelname(name), mFullPath(fullpath), mSumOfVertices(0), mSumOfStrokes(0),
	  mIsConvert(false), mHasTexture(false), mMeshSelected(false), mNewTexture(false),
	  mGravityVector(0,0,0)
	{
		REP(i,3) {
			mTrans[i] = 0.0;
			mAngles[i] = 0.0;
		}
		//  mLSCM.clear();
		//  boost::shared_ptr<LSCM> lscm = boost::shared_ptr<LSCM>(new LSCM());
		//  mLSCM.push_back(lscm);

		mLSCM = boost::shared_ptr<LSCM>(new LSCM());
		mSelectedFace.clear();

		//for mesh decomposition
		Load3DModel();
	}

	/*
	 *
	 */
	ViewingModel::~ViewingModel()
	{
	}
}
