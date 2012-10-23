//TODO XX->Realのときに,Realオブジェクトのテクスチャ情報

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>

#include "OpenGL.h"
#include "TickCounter.h"

#include <cvd/image_io.h>

#include "main.h"
#include "ViewingModel.h"
#include "IndexedMesh.h"
#include "LSCM.h"
#include "Bitmap.h"
#include "TransferController.h"

#include "Modelling/Texture.h"

//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
#define TEXTURE_TRIANGLES 1
#define VISUALIZE 0 //OpenCVのウィンドウ上にテクスチャ展開の点群を表示する
#define FEEDBACK_VISUALIZE 0 //OpenCVから計算してきた輪郭情報をGL上に持っていくときの輪郭情報を表示する
#define DEBUG_VISUALIZING_IN_AR 0
#define DEBUG_INTERACTION_POINTS 0

const int SEPARATION = 5;
const static GLfloat lit_amb[4] = { 0.4f, 0.4f, 0.4f, 1.0 }; /* 環境光の強さ */
const static GLfloat lit_dif[4] = { 1.0, 1.0, 1.0, 1.0 }; /* 拡散光の強さ */
const static GLfloat lit_spc[4] = { 0.4f, 0.4f, 0.4f, 1.0 }; /* 鏡面反射光の強さ */
const static GLfloat lit_pos[4] = { 0.0, 0.0, -9.0, 1.0 }; /* 光源の位置 */
std::string LOADFILENAME("keyboard");
std::string LOADFILENAME2("NewTorus");
std::string LOADFILEFORMAT1(".3ds");
std::string LOADFILEFORMAT2(".3ds");

namespace TextureTransfer
{
	const int ConstParams::W_WIDTH  = 1280;
	const int ConstParams::W_HEIGHT = 800;

	const char *  ConstParams::DATABASEDIR = "/home/umakatsu/Dropbox/Lab/ModelDatabase/";
}

using namespace std;
using namespace TextureTransfer;

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
short fileInput = 0;
char filename[50];
std::vector < Vector3 > direct_ver[2];

// to control object
int mouse_l = 0;
int mouse_m = 0;
int mouse_r = 0;
int mpos[2];
double trans[3] = { 0.0, 0.0, 0.0 };
double theta[3] = { 0.0, 0.0, 0.0 };
enum CONTROLLER {
	DECOMPOSITE, MANUPLATE, TRANSFER, SELECT
} controllObject;
bool textureOFF;
bool displayTexture;

//to project into window-coordinate
GLint viewport[SEPARATION][4];
GLdouble modelview[SEPARATION][16];
GLdouble projection[SEPARATION][16];

//for debug
TextureTransfer::Vector2 texPoint[2];
TextureTransfer::Vector3 clickPoint[2];

void *font = GLUT_BITMAP_HELVETICA_18;
std::vector<int *> point;

TextureTransfer::ViewingModel * models[2];
TextureTransfer::TransferController controller;
short manipulation;

void DrawModelMonitor(int x, int y, int w, int h,
		TextureTransfer::ViewingModel * model, bool isStroke,
		const int & separationW);
void DrawTextureMonitor(int x, int y, int w, int h,
		TextureTransfer::ViewingModel * model, const int & seprationW);
void PointsDisplay();
void TexturePaste(bool color = false);
void keyboard(unsigned char key, int x, int y);

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
void SetObjectiveNames(char **argv, const int & first, const int & second)
{
	LOADFILENAME.clear();
	LOADFILENAME2.clear();
	LOADFILEFORMAT1.clear();
	LOADFILEFORMAT2.clear();

	LOADFILENAME		= strtok(argv[first], ".");
	LOADFILEFORMAT1	+= ".";
	LOADFILEFORMAT1	+= strtok(NULL, ".");

	LOADFILENAME2		= strtok(argv[second], ".");
	LOADFILEFORMAT2	+= ".";
	LOADFILEFORMAT2	+= strtok(NULL, ".");
}

//---------- display font image ------------//
void DrawString(const char *str, void *font, float x, float y, float z) {
	glRasterPos3f(x, y, z);
	while (*str) {
		glutBitmapCharacter(font, *str);
		++str;
	}

}

void DrawController()
{
	glPushAttrib(GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT); // retrieve color and Z buffer
	glColor3d(0, 0, 0);
	char str[50], textureStr[50];
	switch (controllObject)
	{
	case MANUPLATE:
		sprintf(str, "[r] Mode : Manipulation");
		break;

	case DECOMPOSITE:
		sprintf(str, "[r] Mode : Input Stroke");
		break;

	case TRANSFER:
		sprintf(str, "[r] Mode : Texture Transfer");
		break;

	case SELECT:
		sprintf(str, "[r] Mode : Selected texture");
		break;
	}

	if(textureOFF)
	{
		sprintf(textureStr,"[p]Texture OFF");
	}
	else
	{
		sprintf(textureStr,"[p]Texture ON");
	}

	DrawString(str, font, 10, 20, 0);
	DrawString(textureStr, font, -10, 19.5, 0);
	glPopAttrib(); // write back color and Z buffer
}

void ColorSetting(const double & value, bool harmonic = false) {
	// get the value calculated poisson equation
	//  assert( value <= 1.0);
	GLfloat R, G, B;
	if (harmonic) {
		if (value >= 0.75) {
			R = 1.0;
			G = static_cast<GLfloat>(4.0 - 4.0 * value);
			B = 0.0;
		} else if (value >= 0.5) {
			R = static_cast<GLfloat>(4.0 * value - 2.0);
			G = 1.0;
			B = 0.0;
		} else if (value >= 0.25) {
			R = 0.0;
			G = 1.0;
			B = static_cast<GLfloat>(-4.0 * value + 2.0);
		} else {
			R = 0.0;
			G = static_cast<GLfloat>(4.0 * value);
			B = 1.0;
		}
	} else {
		if (value >= 0.5) {
			R = 0;
			G = 1.0;
			B = 1.0;
		} else {
			R = 1.0;
			G = 1.0;
			B = 0;
		}
	}
	glColor3f(R, G, B);
}

void SetDirectVertex(cv::Point3d& start_point, std::vector<Vector3> direct_ver[2], cv::Point3d& end_point)
{
	start_point.x = direct_ver[manipulation - 1][0].x;
	start_point.y = direct_ver[manipulation - 1][0].y;
	start_point.z = direct_ver[manipulation - 1][0].z;
	end_point.x =
			direct_ver[manipulation - 1][direct_ver[manipulation - 1].size() - 1].x;
	end_point.y =
			direct_ver[manipulation - 1][direct_ver[manipulation - 1].size() - 1].y;
	end_point.z =
			direct_ver[manipulation - 1][direct_ver[manipulation - 1].size() - 1].z;
}

void DirectFileInput()
{
	if (fileInput == 1)
	{
		fileInput = 2;
		//for texture transfer in AR
		cout << "Load -> " << filename << endl;
		std::ifstream input(filename);
		direct_ver[0].clear();
		direct_ver[1].clear();
		int number = 0;
		while (input) //until input data continues
		{
			char line[1024];
			input.getline(line, 1024);
			if (!input)
				break;

			std::stringstream line_input(line);
			std::string keyword;
			line_input >> keyword;
			//in the case of vertex information
			if (keyword == "v") {
				Vector3 ver;
				line_input >> ver.x >> ver.y >> ver.z;
				//			printf("%f %f %f\n",ver.x, ver.y, ver.z);
				direct_ver[number].push_back(ver);
			} else if (keyword == "d") {
				number = 1;
			}
		}

		models[manipulation - 1]->IncrementSumOfStrokes();
		//reserver click points
		cv::Point3d start_point, end_point;
		start_point.x = direct_ver[manipulation - 1][0].x;
		start_point.y = direct_ver[manipulation - 1][0].y;
		start_point.z = direct_ver[manipulation - 1][0].z;
		end_point.x =
				direct_ver[manipulation - 1][direct_ver[manipulation - 1].size()
						- 1].x;
		end_point.y =
				direct_ver[manipulation - 1][direct_ver[manipulation - 1].size()
						- 1].y;
		end_point.z =
				direct_ver[manipulation - 1][direct_ver[manipulation - 1].size()
						- 1].z;
		printf("%f %f %f : %f %f %f \n", start_point.x, start_point.y,
				start_point.z, end_point.x, end_point.y, end_point.z);
		int window = 3;
		if (models[manipulation - 1]->CheckFittingVertices(viewport[window],
				modelview[window], projection[window], start_point, end_point,
				false)) {
			models[manipulation - 1]->UpdateMatrix();
			models[manipulation - 1]->RenewMeshDataConstruct();
			models[manipulation - 1]->mLSCM->mMesh->FindTextureMax();
		}
		models[manipulation - 1]->CorrespondTexCoord(viewport[window],
				modelview[window], projection[window], start_point, end_point,
				texPoint[0], texPoint[1], clickPoint[0], clickPoint[1], false);

		//<-- finished decompotion and source's mesh selection
	}
	else if( fileInput == 2)
	{
		fileInput = 3;
		keyboard('s', 0, 0);
	}
	else if( fileInput == 3)
	{
		fileInput = 4;
		//--->selecting parts of the dest's model
		manipulation = 2;

		cv::Point3d start_point, end_point;
		start_point.x = direct_ver[manipulation - 1][0].x;
		start_point.y = direct_ver[manipulation - 1][0].y;
		start_point.z = direct_ver[manipulation - 1][0].z;
		end_point.x =
				direct_ver[manipulation - 1][direct_ver[manipulation - 1].size()
						- 1].x;
		end_point.y =
				direct_ver[manipulation - 1][direct_ver[manipulation - 1].size()
						- 1].y;
		end_point.z =
				direct_ver[manipulation - 1][direct_ver[manipulation - 1].size()
						- 1].z;
		printf("%f %f %f : %f %f %f \n", start_point.x, start_point.y,
				start_point.z, end_point.x, end_point.y, end_point.z);
		int window = 4;
		models[manipulation - 1]->CorrespondTexCoord(viewport[window],
				modelview[window], projection[window], start_point, end_point,
				texPoint[0], texPoint[1], clickPoint[0], clickPoint[1], false);
	}
	else if( fileInput == 4)
	{
		fileInput = 5;
		keyboard('s', 0, 0);
	}
	else if( fileInput == 5)
	{
		keyboard('q',0,0);
	}

}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (controllObject)
	{
		DrawTextureMonitor(0, ConstParams::W_HEIGHT / 2, ConstParams::W_WIDTH / 2, ConstParams::W_HEIGHT / 2, models[0], 1);
		DrawTextureMonitor(ConstParams::W_WIDTH / 2, ConstParams::W_HEIGHT / 2, ConstParams::W_WIDTH / 2, ConstParams::W_HEIGHT / 2, models[1], 2);
	}
	DrawModelMonitor(0, 0, ConstParams::W_WIDTH / 2, ConstParams::W_HEIGHT / 2, models[0], false, 3);
	DrawModelMonitor(ConstParams::W_WIDTH / 2, 0, ConstParams::W_WIDTH / 2, ConstParams::W_HEIGHT / 2, models[1], false, 4);

	glutSwapBuffers();

	//to control from an external file
	DirectFileInput();
}

void idle() {
	glutPostRedisplay();
}

void specialkey(int key, int x, int y)
{
	switch (key) {
	case GLUT_KEY_UP:
		models[manipulation - 1]->mScales *= 2.f;
		break;

	case GLUT_KEY_DOWN:
		models[manipulation - 1]->mScales *= 0.5f;
		break;
	}
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 'q':
	case '\033':
		exit(0);
		break;

	case 'w':
		models[manipulation - 1]->mTrans[1] += 0.5f;
		break;

	case 'x':
		models[manipulation - 1]->mTrans[1] -= 0.5f;
		break;

	case 'd':
		models[manipulation - 1]->mTrans[0] += 0.5f;
		break;

	case 'a':
		models[manipulation - 1]->mTrans[0] -= 0.5f;
		break;

	case 'r':
		if (controllObject == MANUPLATE) {
			controllObject = DECOMPOSITE;
		} else if (controllObject == DECOMPOSITE) {
			controllObject = TRANSFER;
			displayTexture = false;
		} else if (controllObject == TRANSFER) {
			controllObject = SELECT;
			displayTexture = false;
		} else if (controllObject == SELECT) {
			controllObject = MANUPLATE;
			displayTexture = true;
		}
		break;

	case 'p':
		textureOFF=!textureOFF;
		break;

	case 'i':
		models[manipulation-1]->WritebackTo3ds();
		break;

	case 's':
		if (manipulation == 1)
		{
			const char * f_str = "mesh1.bmp";
			WriteBitmapFromGL(f_str, 0, ConstParams::W_HEIGHT / 2, ConstParams::W_WIDTH / 2, ConstParams::W_HEIGHT / 2);
			cout << "Save decomposed mesh from Obj1 -> " << f_str << endl;
		}
		else
		{
			const char * f_str = "mesh2.bmp";
			WriteBitmapFromGL(f_str, ConstParams::W_WIDTH / 2, ConstParams::W_HEIGHT / 2, ConstParams::W_WIDTH / 2, ConstParams::W_HEIGHT / 2);
			cout << "Save decomposed mesh from Obj2 -> " << f_str << endl;
		}
		models[manipulation - 1]->SetMeshSelected(true);
		PointsDisplay();

		if (models[0]->IsMeshSelected() && models[1]->IsMeshSelected())
		{
			controller.SetContourPoints();
			controller.AcquireMatching();

			TexturePaste(true);
			string newstr("New"); newstr+= LOADFILENAME2;
			models[1]->Save3DModel(newstr.c_str());
		}
		break;
	}
}

void mouse(int button, int state, int x, int y)
{
	if (controllObject == DECOMPOSITE)
	{
		// button ON
		//reserve mouse-coord and project into window-coord
		//while pressing the mouse button
		if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON) {
			int * p = new int[2];
			p[0] = x;
			p[1] = y;
			point.push_back(p);
		}

		//button OFF
		if (state == GLUT_UP && button == GLUT_LEFT_BUTTON)
		{
//			TickCountAverageBegin();

			models[manipulation - 1]->IncrementSumOfStrokes();

			//reserver click points
			cv::Point3d start_point, end_point;
			start_point.x = point[0][0];
			start_point.y = point[0][1];
			end_point.x = point[point.size() - 1][0];
			end_point.y = point[point.size() - 1][1];

			int window;
			if (manipulation - 1 == 0)
				window = 3;
			else
				window = 4;

			ViewingModel * pModel = models[manipulation - 1];

			if( pModel->CheckFittingVertices(viewport[window], modelview[window], projection[window], start_point, end_point))
			{
				pModel->UpdateMatrix();
				pModel->RenewMeshDataConstruct();
				pModel->mLSCM->mMesh->FindTextureMax();
			}
			point.clear();
//			TickCountAverageEnd();
		}
	}
	else if (controllObject == MANUPLATE)
	{
		switch (button) {
		case GLUT_LEFT_BUTTON:
			if (state == GLUT_DOWN) {
				mpos[0] = x;
				mpos[1] = y;
				mouse_l = 1;
			}
			if (state == GLUT_UP) {
				mouse_l = 0;
			}
			break;
		default:
			break;
		}
	}
	//TRANSFER, SELECT
	else
	{
		// button ON
		//reserve mouse-coord and project into window-coord
		//while pressing the mouse button
		if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON) {
			int * p = new int[2];
			p[0] = x;
			p[1] = y;
			point.push_back(p);
		}

		//button OFF
		if (state == GLUT_UP && button == GLUT_LEFT_BUTTON) {

			//get each matrix parameter
//			SetMatrixParam();

			//reserver click points
			cv::Point3d start_point, end_point;
			start_point.x = point[0][0];
			start_point.y = point[0][1];
			end_point.x = point[point.size() - 1][0];
			end_point.y = point[point.size() - 1][1];

			int window;
			if (manipulation - 1 == 0)
				window = 3;
			else
				window = 4;

			models[manipulation - 1]->CorrespondTexCoord(viewport[window],
					modelview[window], projection[window], start_point,
					end_point, texPoint[0], texPoint[1], clickPoint[0],
					clickPoint[1]);

//			cout << " ------------ Texture Transfer Points ------------ " << endl;
//			cout << start_point.x << " " << start_point.y << endl;
//			cout << end_point.x << " " << end_point.y << endl;
//			cout << texPoint[0].x << " " << texPoint[0].y << endl;
//			cout << texPoint[1].x << " " << texPoint[1].y << endl << endl;
			point.clear();
		}

	}
}

void motion(int x, int y) {
	// change control window in accordance with mouse cursor coord
	if (x < ConstParams::W_WIDTH / 2 && 0 <= x) {
		manipulation = 1;
	} else {
		manipulation = 2;
	}

	if (controllObject == DECOMPOSITE || controllObject == TRANSFER) {
		glLineWidth(5);
		int * p = new int[2];
		p[0] = x;
		p[1] = y;
		//    cout << x << " " << y << endl;
		point.push_back(p);
	} else {
		if (mouse_l == 1) {
			theta[0] = (double) (y - mpos[1]) / 5.0;
			theta[1] = (double) (x - mpos[0]) / 5.0;
		}
		if (mouse_l == 1 || mouse_m == 1 || mouse_r == 1) {
			mpos[0] = x;
			mpos[1] = y;
			if (x < ConstParams::W_WIDTH / 2 && 0 <= x) {
				models[0]->mAngles[0] += theta[0];
				models[0]->mAngles[1] += theta[1];
			} else {
				models[1]->mAngles[0] += theta[0];
				models[1]->mAngles[1] += theta[1];
			}
			glutPostRedisplay();
		}
	}
}

void CallbackEntry(void) {
	glutDisplayFunc(display);
	glutSpecialFunc(specialkey);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutIdleFunc(idle);
}

void DrawModelWithNewTexture(ViewingModel*& model)
{
	IndexedMesh * lscmMesh = model->mLSCM->mMesh.get();

	REP(texNumber, model->mTexture.size())
	{
		//テクスチャセット
		model->mTexture[texNumber]->bind();
		//for warping texture mapping to size (ConstParams::W_WIDTH/2, ConstParams::W_HEIGHT/2)
		double ratio_x = (ConstParams::W_WIDTH * 0.5 - 0)
				/ (lscmMesh->mTexMax.x - lscmMesh->mTexMin.x);
		double ratio_y = (ConstParams::W_HEIGHT * 0.5 - 0)
				/ (lscmMesh->mTexMax.y - lscmMesh->mTexMin.y); //

		glBegin (GL_TRIANGLES);
		REP(loopFace, lscmMesh->mFaces.size())
		{
			//対象の面を構成する頂点が同一のテクスチャ画像を参照しているかチェック
			bool compose = false;

			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
				REP(texID, lscmMesh->mVertices[verIndex].textureNumberArray.size())
				{
					if(lscmMesh->mVertices[verIndex].textureNumberArray.at(texID) == texNumber)
//				if (lscmMesh->mVertices[verIndex].textureNumber == texNumber)
					{
						compose = true;
						break;
					}
				}
			}

			if (!compose) continue;

			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
				{
					GLdouble texcos[2], vertex[3];
					//TEXTURE_ARBを用いているためu-v座標を0-1にしなくてもよい
					texcos[0] = (lscmMesh->mVertices[verIndex].tex_coord.x
							- lscmMesh->mTexMin.x) * ratio_x;
					texcos[1] = (ConstParams::W_HEIGHT / 2
							- (lscmMesh->mVertices[verIndex].tex_coord.y
									- lscmMesh->mTexMin.y) * ratio_y) - 0;
					vertex[0] = lscmMesh->mVertices[verIndex].point.x;
					vertex[1] = lscmMesh->mVertices[verIndex].point.y;
					vertex[2] = lscmMesh->mVertices[verIndex].point.z;

					glColor3f(1.0f, 1.0f, 1.0f);
					glTexCoord2dv(texcos);
					glVertex3dv(vertex);
				}
			}
		}
		glEnd();
		model->mTexture[texNumber]->unbind();
	}
}

void DrawModelWithTextures(ViewingModel*& model)
{

	IndexedMesh * lscmMesh = model->mLSCM->mMesh.get();
	REP(texNumber, model->mTexture.size())
	{
		//テクスチャセット
		model->mTexture[texNumber]->bind();

		glBegin (GL_TRIANGLES);
		REP(loopFace, lscmMesh->mFaces.size())
		{
			//対象の面が同一のテクスチャ画像を参照しているかチェック
			bool compose = false;

			int faceIdx;
			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				faceIdx = lscmMesh->mTexnumVernum[loopFace].mFaceIdx[loopVer];
				if (lscmMesh->mTexnumVernum[loopFace].mTextureNumber == texNumber)
				{
					compose = true;
					break;
				}
			}

			if (!compose) continue;

			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
				//対応関係のための指数
				int corVerIdx = model->mMesh[texNumber]->mFaces[faceIdx].at(loopVer);
				GLdouble corTexcos[] = {
						model->mMesh[texNumber]->mVertices[corVerIdx].tex_coord.x,
						1-model->mMesh[texNumber]->mVertices[corVerIdx].tex_coord.y };
				GLdouble vertex[3];
				vertex[0] = lscmMesh->mVertices[verIndex].point.x;
				vertex[1] = lscmMesh->mVertices[verIndex].point.y;
				vertex[2] = lscmMesh->mVertices[verIndex].point.z;

				glColor3f(1.0f, 1.0f, 1.0f);
				glTexCoord2dv(corTexcos);
				glVertex3dv(vertex);
			}
		}
		glEnd();
		model->mTexture[texNumber]->unbind();
	}
}

void DrawModelMonitor(int x, int y, int w, int h, ViewingModel * model,
		bool isStroke, const int & separationW)
{
	//view-port transform
	glViewport(x, y, w, h);

	//projection transform
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30.0, (double) w / (double) h, 0.1, 1000.0);

	//model-view transform
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(20, 30, 80, 0, 0, 0, 0, 1, 0);

	glPushMatrix();

	glRotated(model->mAngles[0], 1.0, 0.0, 0.0); //
	glRotated(model->mAngles[1], 0.0, 1.0, 0.0); //
	glRotated(model->mAngles[2], 0.0, 0.0, 1.0); //
	glTranslated(model->mTrans[0], model->mTrans[1], model->mTrans[2]);
	glScalef(model->mScales, model->mScales, model->mScales);
	glRotatef(90, -1, 0, 0);

	//reserve projection matrices for rendering a model
	glGetIntegerv(GL_VIEWPORT, viewport[separationW]);
	glGetDoublev(GL_PROJECTION_MATRIX, projection[separationW]);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview[separationW]);

	//for debug drawing lines
#if DEBUG_DRAWINGLINES
	glColor3f(1.0,1.0,0.0);
	glLineWidth(5);
	glBegin(GL_LINE_STRIP);
	glVertex3d(clickPoint[0].x, clickPoint[0].y, clickPoint[0].z);
	glVertex3d(clickPoint[1].x, clickPoint[1].y, clickPoint[1].z);
	glEnd();
#endif
//	glEnable(GL_LIGHT0);
//	glEnable(GL_LIGHTING);
//	glEnable(GL_COLOR_MATERIAL);

	//Texture Transfer mode
	//Selected Texture mode
	if (!displayTexture || textureOFF)
	{
		//  Rendering a model
		glLineWidth(1);
		glColor3f(1.0f, 0.0f, 0.0f);

		REP(loopMesh, model->mMesh.size())
		{
			GLdouble normal[3];
			GLdouble vertex[3];

			glBegin(GL_TRIANGLES);
			REP(faceIdx,model->GetMeshFacesSize(loopMesh) )
			{
				REP(verIdx, model->GetMeshInnerFacesSize(loopMesh,faceIdx))
				{
					ColorSetting(model->QueryVertexColor(loopMesh, faceIdx, verIdx), true);
					model->QueryNormal(loopMesh, faceIdx, verIdx, normal);
					model->QueryVertex(loopMesh, faceIdx, verIdx, vertex);
					//				model->QueryAmbient(loop, id, ambient);
					//				model->QueryDiffuse(loop, id, diffuse);
					//				model->QuerySpecular(loop, id, specular);
					//				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
					//				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
					//				glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);

					//      glColor3f(diffuse[0]*lit_dif[0], diffuse[1]*lit_dif[1], diffuse[2]*lit_dif[2]);
					//      glColor3f(diffuse[0], diffuse[1], diffuse[2]);
					glNormal3dv(normal);
					glVertex3dv(vertex);
				}
			}
			glEnd();
		}
		//  glDisable(GL_COLOR_MATERIAL);
		//  glDisable(GL_LIGHTING);
		//  glDisable(GL_LIGHT0);
	}

	//Manipulation mode
	//Input stroke mode
	// with texture mapping
	else
	{
		if(model->isNewTexture())
		{
			DrawModelWithNewTexture(model);
		}
		else
		{
			DrawModelWithTextures(model);
		}

#if DEBUG_VISUALIZING_IN_AR == 1
		//for texture transfer in AR
		glBegin(GL_LINE_STRIP);
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3d(0,0,0);

			std::ifstream input("Model3DS/debug_remote.txt") ;
			while (input) //until input data continues
			{
				char line[1024];
				input.getline(line, 1024);
				std::stringstream line_input(line);

				GLdouble ver[3];
				line_input >> ver[0] >> ver[1] >> ver[2];
				glLineWidth(25);
				glColor3f(.0f, 1.0f, 0.0f);
				glVertex3dv(ver);
			}
		glEnd();
#endif
	}
	//get each transform matrix
	glPopMatrix();

	// draw drag line
	double ox, oy, oz;
	GLint glX, glY, glZ;

	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

	glDisable(GL_DEPTH_TEST);

	glLineWidth(5);
	glColor3f(1.0, 0.0, 0.0);
	if (point.size() > 1) {
		glBegin(GL_LINE_STRIP);
		for (unsigned int i = 0; i < point.size(); i++) {
			glX = point[i][0];
			glY = viewport[3] * 2 - point[i][1];
			//      glReadPixels(glX,glY,1,1,GL_DEPTH_COMPONENT,GL_FLOAT,&z);
			glZ = 0;
			gluUnProject((GLdouble) glX, (GLdouble) glY, (GLdouble) glZ,
					modelview, projection, viewport, &ox, &oy, &oz);
			glVertex3d(ox, oy, oz);
		}
		glEnd();
	}
	DrawController();

	//display a sentence if a mesh in the model have been selected
	if (model->IsMeshSelected())
	{
		// retrieve color and Z buffer
		glPushAttrib(GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT);

		glColor3d(1, 0, 1);
		void *letter = GLUT_BITMAP_TIMES_ROMAN_24;
		DrawString("SELECTED", letter, -10, -10, 0);

		// write back color and Z buffer
		glPopAttrib();

	}
	glEnable(GL_DEPTH_TEST);
}

void DrawAtlasWithNewTexture(ViewingModel*& model)
{
	IndexedMesh* lscmMesh = model->mLSCM->mMesh.get();

	REP(texNumber, model->mTexture.size())
	{
		//テクスチャセット
		model->mTexture[texNumber]->bind();

		//for warping texture mapping to size (W_WIDTH/2, W_HEIGHT/2)
		double ratio_x = (ConstParams::W_WIDTH * 0.5 - 0)
				/ (lscmMesh->mTexMax.x - lscmMesh->mTexMin.x);
		double ratio_y = (ConstParams::W_HEIGHT * 0.5 - 0)
				/ (lscmMesh->mTexMax.y - lscmMesh->mTexMin.y);

//			cout << lscmMesh->mFaces.size() << endl;

		glBegin(GL_TRIANGLES);
		REP(loopFace, lscmMesh->mFaces.size())
		{
			//対象の面を構成する頂点が同一のテクスチャ画像を参照しているかチェック
			bool compose = false;

			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
				REP(texID, lscmMesh->mVertices[verIndex].textureNumberArray.size())
				{
					if(lscmMesh->mVertices[verIndex].textureNumberArray.at(texID) == texNumber)
//				if (lscmMesh->mVertices[verIndex].textureNumber == texNumber)
					{
						compose = true;
						break;
					}
				}
			}

			if (!compose) continue;

			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, lit_amb);
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, lit_dif);
				glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, lit_spc);
				{
					GLdouble texcos[2], vertex[2];
					//TEXTURE_ARBを用いているためu-v座標を0-1にしなくてもよい
					texcos[0] = (lscmMesh->mVertices[verIndex].tex_coord.x
							- lscmMesh->mTexMin.x) * ratio_x;
					texcos[1] = (ConstParams::W_HEIGHT / 2
							- (lscmMesh->mVertices[verIndex].tex_coord.y
									- lscmMesh->mTexMin.y) * ratio_y) - 0;

					vertex[0] = lscmMesh->mVertices[verIndex].tex_coord.x;
					vertex[1] = lscmMesh->mVertices[verIndex].tex_coord.y;

					glColor3f(1.0f, 1.0f, 1.0f);
					glTexCoord2dv(texcos);
					glVertex2dv(vertex);
				}
			}
		}
		glEnd();
		model->mTexture[texNumber]->unbind();
	}
}

void DrawAtlasWithTextures(ViewingModel*& model)
{
	IndexedMesh* lscmMesh = model->mLSCM->mMesh.get();
	REP(texNumber, model->mTexture.size())
	{
		//テクスチャセット
		model->mTexture[texNumber]->bind();

		glColor4f(1.0f, 1.0f, 1.0f, 0.0f);
		glBegin (GL_TRIANGLES);
		REP(loopFace, lscmMesh->mFaces.size())
		{
			//対象の面を構成する頂点が同一のテクスチャ画像を参照しているかチェック
			bool compose = false;
			int faceIdx = 0;
			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				faceIdx = lscmMesh->mTexnumVernum[loopFace].mFaceIdx[loopVer];
				if (lscmMesh->mTexnumVernum[loopFace].mTextureNumber == texNumber) {
					compose = true;
					break;
				}
			}

			if (!compose)
				continue;

			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				//対応関係のための指数
				int corVerIdx = model->mMesh[texNumber]->mFaces[faceIdx].at(
						loopVer);
				GLfloat corTexcos[] =
						{
								model->mMesh[texNumber]->mVertices[corVerIdx].tex_coord.x,
								1 - model->mMesh[texNumber]->mVertices[corVerIdx].tex_coord.y
						};

				int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
				{
					GLfloat vertex[2];
					vertex[0] = lscmMesh->mVertices[verIndex].tex_coord.x;
					vertex[1] = lscmMesh->mVertices[verIndex].tex_coord.y;
					glTexCoord2fv(corTexcos);
					glVertex2fv(vertex);
				}
			}
		}
		glEnd();
		model->mTexture[texNumber]->unbind();
	}
}
void DrawAtlasPartWithNewTexture(ViewingModel*& model)
{
	IndexedMesh* lscmMesh = model->mLSCM->mMesh.get();
	glColor3f(1.0f, 1.0f, 1.0f);

	REP(texNumber, model->mTexture.size())
	{
		//setting appropriate texture
		model->mTexture[texNumber]->bind();

		//for warping texture mapping to size (W_WIDTH/2, W_HEIGHT/2)
		double ratio_x = (ConstParams::W_WIDTH * 0.5 - 0)
				/ (lscmMesh->mTexMax.x - lscmMesh->mTexMin.x);
		double ratio_y = (ConstParams::W_HEIGHT * 0.5 - 0)
				/ (lscmMesh->mTexMax.y - lscmMesh->mTexMin.y);

		glBegin (GL_TRIANGLES);
		assert(
				model->mSelectedMesh.second.mFaces.size()
						== model->mSelectedFace.size());
		REP(loopFace, model->mSelectedMesh.second.mFaces.size())
		{
			REP(loopVer, 3)
			{
				Vector2 tex_coord = model->mSelectedMesh.second.mVertices.at(
						model->mSelectedMesh.second.mFaces[loopFace].at(
								loopVer)).tex_coord;
				GLdouble texcos[2], vertex[2];

				//TEXTURE_ARBを用いているためu-v座標を0-1にしなくてもよい
				texcos[0] = (tex_coord.x - lscmMesh->mTexMin.x) * ratio_x;
				texcos[1] = (ConstParams::W_HEIGHT / 2
						- (tex_coord.y - lscmMesh->mTexMin.y) * ratio_y) - 0;
				vertex[0] = tex_coord.x;
				vertex[1] = tex_coord.y;
				glTexCoord2dv(texcos);
				glVertex2dv(vertex);
			}
		}
		glEnd();
		model->mTexture[texNumber]->unbind();
	}
}

void DrawTextureMonitor(int x, int y, int w, int h, ViewingModel * model,
		const int & seprationW)
{
	glViewport(x, y, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(model->mLSCM->mMesh->mTexMin.x, model->mLSCM->mMesh->mTexMax.x,
			model->mLSCM->mMesh->mTexMin.y, model->mLSCM->mMesh->mTexMax.y, 0,
			1.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0); //default view point

	//render something from here
	// ------>

#if DEBUG_INTERACTION_POINTS == 1
	//for debug (interaction points)
	glColor3d(0, 0, .5);
	glLineWidth(3);
	glBegin(GL_LINES);
	glVertex2f(texPoint[0].x, texPoint[0].y);
	glVertex2f(texPoint[1].x, texPoint[1].y);
	glEnd();
#endif

	if (controllObject != SELECT)
	{
		//showing a texture image
		if(!textureOFF)
		{
			if(model->isNewTexture())
			{
				DrawAtlasWithNewTexture(model);
			}
			else
			{
				DrawAtlasWithTextures(model);
			}
		}
		else
		{
			IndexedMesh * lscmMesh = model->mLSCM->mMesh.get();

			//for rendering texture deployments
			#if TEXTURE_TRIANGLES==0
				glBegin(GL_LINES);
			#else
				glBegin(GL_TRIANGLES);
			#endif

			REP(faceIdx, lscmMesh->mFaces.size() )
			{
	#if TEXTURE_TRIANGLES==0
				Vector2 vertex[3];
				REP(i,3){
					int index = lscmMesh->mFaces[faceIdx].at(i);
					vertex[i].x = lscmMesh->mVertices[index].tex_coord.x;
					vertex[i].y = lscmMesh->mVertices[index].tex_coord.y;
					double val = lscmMesh->mVertices[index].harmonicValue;
					ColorSetting(val, true);
				}
				glVertex2f(vertex[0].x,vertex[0].y);
				glVertex2f(vertex[1].x,vertex[1].y);
				glVertex2f(vertex[1].x,vertex[1].y);
				glVertex2f(vertex[2].x,vertex[2].y);
				glVertex2f(vertex[2].x,vertex[2].y);
				glVertex2f(vertex[0].x,vertex[0].y);
	#else
				REP(verIdx, lscmMesh->mFaces[faceIdx].size()) {
					Vector2 vertex;
					int index = lscmMesh->mFaces[faceIdx].at(verIdx);
					vertex.x = lscmMesh->mVertices[index].tex_coord.x;
					vertex.y = lscmMesh->mVertices[index].tex_coord.y;

					double val = lscmMesh->mVertices[index].harmonicValue;
					ColorSetting(val, true);
					glVertex2f(vertex.x, vertex.y);
				}
	#endif
			}
			glEnd();
		}
	} /* if (controllObject != SELECT) */

	//in case of selecting parts
	else
	{
		//showing a texture image
		if(!textureOFF)
		{
			if(model->isNewTexture())
			{
				DrawAtlasPartWithNewTexture(model);
			}
			else
			{
				IndexedMesh* lscmMesh = model->mLSCM->mMesh.get();
				REP(texNumber, model->mTexture.size())
				{
					//テクスチャセット
					model->mTexture[texNumber]->bind();

					glColor4f(1.0f, 1.0f, 1.0f, 0.0f);
					glBegin (GL_TRIANGLES);

					assert(model->mSelectedMesh.second.mFaces.size() == model->mSelectedFace.size());
					REP(loopFace, model->mSelectedMesh.second.mFaces.size())
					{
						//対象の面を構成する頂点が同一のテクスチャ画像を参照しているかチェック
						bool compose = false;
						int faceIdx = 0;
						int corrFaceIdx = model->mSelectedFace.at(loopFace);
						REP(i,3)
						{
							faceIdx = lscmMesh->mTexnumVernum[corrFaceIdx].mFaceIdx[i];
							if (lscmMesh->mTexnumVernum[corrFaceIdx].mTextureNumber == texNumber)
							{
								compose = true;
								break;
							}
						}

						if (!compose) continue;

						REP(loopVer, 3)
						{
							//対応関係のための指数
							int corVerIdx = model->mMesh[texNumber]->mFaces[faceIdx].at(loopVer);
							GLfloat corTexcos[] = {
											model->mMesh[texNumber]->mVertices[corVerIdx].tex_coord.x,
											1-model->mMesh[texNumber]->mVertices[corVerIdx].tex_coord.y
							};

							Vector2 tex_coord = model->mSelectedMesh.second.mVertices.at(
									model->mSelectedMesh.second.mFaces[loopFace].at(
											loopVer)).tex_coord;
							GLfloat vertex[2];
							vertex[0] = tex_coord.x;
							vertex[1] = tex_coord.y;

							glTexCoord2fv(corTexcos);
							glVertex2fv(vertex);
						}
					}
					glEnd();
					model->mTexture[texNumber]->unbind();
				}
			}
		}
		//with harmonic field
		else
		{
			assert(model->mLSCM->mMesh->mVertices.size() == model->mLSCM->mMesh->mTextureCoords.size() );
			//for rendering texture deployments
			#if TEXTURE_TRIANGLES==0
				glBegin(GL_LINES);
			#else
			glBegin(GL_TRIANGLES);
			#endif

			REP(faceIdx, model->mSelectedMesh.second.mFaces.size())
			{
				Vector2 vertex[3];
				REP(i,3) {
					vertex[i] =
							model->mSelectedMesh.second.mVertices.at(model->mSelectedMesh.second.mFaces[faceIdx].at(i)).tex_coord;
				}

				double value;
				value = model->mLSCM->mMesh->mVertices[model->mSelectedMesh.second.mFaces[faceIdx].at(0)].harmonicValue;

				ColorSetting(value, true);

			#if TEXTURE_TRIANGLES==0
					glVertex2f(vertex[0].x,vertex[0].y);
					glVertex2f(vertex[1].x,vertex[1].y);
					glVertex2f(vertex[1].x,vertex[1].y);
					glVertex2f(vertex[2].x,vertex[2].y);
					glVertex2f(vertex[2].x,vertex[2].y);
					glVertex2f(vertex[0].x,vertex[0].y);
			#else
				glVertex2f(vertex[0].x, vertex[0].y);

				value = model->mLSCM->mMesh->mVertices[model->mSelectedMesh.second.mFaces[faceIdx].at(1)].harmonicValue;
				ColorSetting(value, true);
				glVertex2f(vertex[1].x, vertex[1].y);

				value = model->mLSCM->mMesh->mVertices[model->mSelectedMesh.second.mFaces[faceIdx].at(2)].harmonicValue;
				ColorSetting(value, true);
				glVertex2f(vertex[2].x, vertex[2].y);

			#endif
			}
			glEnd();
		} /* else(textureOFF != 0) */
	}
	//<-------
	//this is the end of rendering part

	glEnable(GL_DEPTH_TEST);
}

void PointsDisplay() {
	IndexedMesh * tmpMesh = models[manipulation - 1]->mLSCM->mMesh.get();
	double ratio_x = (ConstParams::W_WIDTH * 0.5 - 1)
			/ (tmpMesh->mTexMax.x - tmpMesh->mTexMin.x);
	double ratio_y = (ConstParams::W_HEIGHT * 0.5 - 1)
			/ (tmpMesh->mTexMax.y - tmpMesh->mTexMin.y);

	IplImage * input = cvLoadImage("mesh1.bmp", 0);
#if VISUALIZE == 1
	IplImage * src = cvCreateImage( cvGetSize(input), 8, 3);
	const char * winName = "Convex Hull";
	cvZero(src);
#endif

	controller.mMeshes[manipulation - 1].clear();
	assert(input);
	REP(i,models[manipulation-1]->mSelectedMesh.second.mVertices.size()) {
		//実際に使われているvertexかどうか
		if (!models[manipulation - 1]->mSelectedMesh.second.mVertices[i].locked)
			continue;

		Vector2 tmp1 =
				models[manipulation - 1]->mSelectedMesh.second.mVertices[i].tex_coord;

		cv::Point tmp;
		pair<int, cv::Point> tmpPair;

		tmp.x = (tmp1.x - tmpMesh->mTexMin.x) * ratio_x;
		tmp.y = (input->height - (tmp1.y - tmpMesh->mTexMin.y) * ratio_y) - 1;
		tmpPair.first =
				models[manipulation - 1]->mSelectedMesh.second.mVertices[i].id;
		tmpPair.second = tmp;

		controller.mMeshes[manipulation - 1].push_back(tmpPair);
#if VISUALIZE == 1
		cvCircle( src, tmp, 2, CV_RGB( 255, 255, 0 ), CV_FILLED );
#endif
//		cout << "Model" << manupulation-1 << " Index=" << tmpPair.first << " ; HarmonicVal=" << models[manupulation-1]->mLSCM->mMesh->mTexParts[tmpPair.first] << endl;
//		controller.SetHashmap( tmp.x, tmp.y, tmpPair.first, manupulation-1);
	}

#if VISUALIZE == 1
	cvNamedWindow(winName, 1);
	cvShowImage( winName, src);

	cvWaitKey(0);
	cvReleaseImage(&src);
#endif
	cvReleaseImage(&input);
}

void TexturePaste(bool color)
{
	if (color)
	{
#if FEEDBACK_VISUALIZE == 1
		IplImage * input = cvLoadImage("mesh1.bmp", 0);
		IplImage * src = cvCreateImage( cvGetSize(input), 8, 3);
		const char * winName = "Convex Hull";
		cvZero(src);
#endif

		//色情報のTransfer
		//対応点の色情報を移し替える
//		REP(id,controller.mMatchingPoints.size()){
		// should use to transfer color field
//			int idModel0 = controller.GetHashmap((int)controller.mMatchingPoints[id].first.x, (int)controller.mMatchingPoints[id].first.y, 0);
//			int idModel1 = controller.GetHashmap((int)controller.mMatchingPoints[id].second.x, (int)controller.mMatchingPoints[id].second.y, 1);

//				printf(" Matching0(%d,%d):(%d,%d)\n",(int)controller.mMatchingPoints[id].first.x, (int)controller.mMatchingPoints[id].first.y, controller.mMeshes[0].at(idModel0).second.x, controller.mMeshes[0].at(idModel0).second.y);
//				printf(" Matching1(%d,%d):(%d,%d)\n\n",(int)controller.mMatchingPoints[id].second.x, (int)controller.mMatchingPoints[id].second.y, controller.mMeshes[1].at(idModel1).second.x, controller.mMeshes[1].at(idModel1).second.y);
//				printf("(%d , %d) : (%lf -> %lf)\n", idModel1, idModel0, models[1]->mLSCM->mMesh->mTexParts[idModel1], models[0]->mLSCM->mMesh->mTexParts[idModel0]);

//			models[1]->mLSCM->mMesh->mTexParts[idModel1] = models[0]->mLSCM->mMesh->mTexParts[idModel0];
#if FEEDBACK_VISUALIZE == 1
		cvCircle( src, cvPoint((int)controller.mMatchingPoints[id].first.x, (int)controller.mMatchingPoints[id].first.y), 2, CV_RGB( 255, 255, 0 ), CV_FILLED );
#endif
//		}

		//Update texture number corresponding to each vertex
		REP(id, controller.mMeshes[1].size())
		{
			models[1]->mLSCM->mMesh->mVertices[controller.mMeshes[1].at(id).first].textureNumberArray.front() = 1;
		}

		//boundary vertices should be assigned by more than two texture numbers
		IndexedMesh * lscmMesh = models[1]->mLSCM->mMesh.get();
		REP(loopFace, lscmMesh->mFaces.size())
		{
			//対象の面が同一のテクスチャ画像を参照しているかチェック
			//同一ではない場合は、テクスチャ番号を追加する
			//対象の面を構成する頂点が同一のテクスチャ画像を参照しているかチェック
			vector<int> composeArray;
			bool dif = false;
			REP(loopVer, lscmMesh->mFaces[loopFace].size())
			{
				int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
				int texIndex = lscmMesh->mVertices[verIndex].textureNumberArray.front();
				composeArray.push_back(texIndex);

				//異なるテクスチャ番号があった場合はフラグをONにする
				if(!dif && composeArray.at(0) != texIndex)
				{
					dif = true;
				}
			}

			//参照テクスチャ番号の追加
			if(dif)
			{
				//面を構成する頂点についてチェック
				REP(loopVer, lscmMesh->mFaces[loopFace].size())
				{
					int verIndex = lscmMesh->mFaces[loopFace].at(loopVer);
					bool dup = true;
					REP(nn, composeArray.size())
					{
						int comArrIdx = composeArray.at(nn);
						REP(i,lscmMesh->mVertices[verIndex].textureNumberArray.size())
						{
							if(lscmMesh->mVertices[verIndex].textureNumberArray.at(i) != comArrIdx)
							{
								dup = false;
								break;
							}
						}

						//新たな参照テクスチャ番号がある場合は登録する
						if(!dup)
						{
							lscmMesh->mVertices[verIndex].textureNumberArray.push_back(comArrIdx);
						}
					}
				}
			}
		}

		//create the warped texture
		const char * tex1 = "warping1.bmp";
		::ImageType TextureRGB = (CVD::img_load(tex1));
		Texture * tmpTexture = new Texture( static_cast<const ::ImageType>(TextureRGB), tex1);
		models[1]->mTexture.push_back(tmpTexture);

		cout << "Texture Transfer DONE!! Left -> Right" << endl;

		//reset selected mesh
		REP(i,2) models[i]->SetMeshSelected(false);

#if FEEDBACK_VISUALIZE == 1
		cvNamedWindow(winName, 1);
		cvShowImage( winName, src);
		cvWaitKey(0);
		cvReleaseImage(&src);
		cvReleaseImage(&input);
#endif
	}
}

void Init()
{
	glClearColor(1, 1, 1, 1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	glLightfv(GL_LIGHT0, GL_AMBIENT, lit_amb);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lit_dif);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lit_spc);
	glLightfv(GL_LIGHT0, GL_POSITION, lit_pos);

//	glEnable(GL_LIGHT0);
//	glEnable(GL_LIGHTING);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);

	//load 3ds model
	ostringstream model1Name, model2Name;
	model1Name.clear(); model2Name.clear();
	model1Name << ConstParams::DATABASEDIR << LOADFILENAME.c_str() << "/" << LOADFILENAME.c_str() << LOADFILEFORMAT1.c_str();
	model2Name << ConstParams::DATABASEDIR << LOADFILENAME2.c_str() << "/" << LOADFILENAME2.c_str() << LOADFILEFORMAT2.c_str();
	models[0] = new ViewingModel(LOADFILENAME.c_str(), model1Name.str().c_str());
	models[1] = new ViewingModel(LOADFILENAME2.c_str(),model2Name.str().c_str());
	manipulation = 1;

	models[0]->LoadTexture("itimatsu.bmp");
	models[0]->mLSCM->run("CG");
	models[0]->mLSCM->mMesh->FindTextureMax();
//
	models[1]->LoadTexture("wood.bmp");
	models[1]->mLSCM->run("CG");
	models[1]->mLSCM->mMesh->FindTextureMax();

	displayTexture = true;
	controllObject = SELECT;
	textureOFF = false;
}

int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(ConstParams::W_WIDTH, ConstParams::W_HEIGHT);

	glutCreateWindow("Mesh Decomposition using Harmonic Field");

	CallbackEntry();

	switch(argc)
	{
		case 2: //has direct input?
			strcpy(filename, argv[1]);
			fileInput = 1;
			break;

		case 3:
			SetObjectiveNames(argv, 1, 2);
			break;

		case 4:
			strcpy(filename, argv[1]);
			fileInput = 1;
			SetObjectiveNames(argv, 2, 3);
			break;

		default:
			break;
	}

	Init();
	glutMainLoop();
	return 0;
}
