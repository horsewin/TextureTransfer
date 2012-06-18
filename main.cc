//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include <vector>
#include <iostream>
#include <stdio.h>

#include "OpenGL.h"
#include "OpenCV.h"

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

const int SEPARATION = 5;
const static GLfloat lit_amb[4]={0.4f, 0.4f, 0.4f,1.0};	/* 環境光の強さ */
const static GLfloat lit_dif[4]={1.0, 1.0, 1.0, 1.0};	/* 拡散光の強さ */
const static GLfloat lit_spc[4]={0.4f, 0.4f, 0.4f, 1.0};	/* 鏡面反射光の強さ */
const static GLfloat lit_pos[4]={0.0, 0.0, -9.0, 1.0};	/* 光源の位置 */

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
// to control object
int mouse_l = 0;
int mouse_m = 0;
int mouse_r = 0;
int mpos[2];
double trans[3] = {0.0, 0.0, 0.0};
double theta[3] = {0.0, 0.0, 0.0};
enum CONTROLLER{DECOMPOSITE, MANUPLATE, TRANSFER, SELECT} controllObject;
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
short manupulation;

void DrawModelMonitor(int x, int y, int w, int h, TextureTransfer::ViewingModel * model, bool isStroke, const int & separationW);
void DrawTextureMonitor(int x, int y, int w, int h, TextureTransfer::ViewingModel * model, const int & seprationW);
void PointsDisplay();
void TexturePaste(bool color = false);

using namespace std;
using namespace TextureTransfer;
//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
//---------- display font image ------------//
void DrawString(const char *str,void *font,float x,float y,float z)
{
  glRasterPos3f(x,y,z);
  while(*str){
    glutBitmapCharacter(font, *str);
    ++str;
  }	

}

void DrawController()
{
  glPushAttrib(GL_CURRENT_BIT|GL_DEPTH_BUFFER_BIT); // retrieve color and Z buffer
  glColor3d(0,0,0);
  char str[50];
  switch(controllObject){
  	  case MANUPLATE:
  		  sprintf(str,"[r] Mode : Manipulation");
  		  break;

  	  case DECOMPOSITE:
  		  sprintf(str,"[r] Mode : Input Stroke");
  		  break;

  	  case TRANSFER:
  		  sprintf(str,"[r] Mode : Texture Transfer");
  		  break;

  	  case SELECT:
  		  sprintf(str,"[r] Mode : Selected texture");
  		  break;

  }
  DrawString(str,font,10,20,0);
  glPopAttrib(); // write back color and Z buffer 
}

void ColorSetting(const double & value, bool harmonic = false)
{
  // get the value calculated poisson equation  
  //  assert( value <= 1.0);
  GLfloat R,G,B;
  if(harmonic){
	  if( value >= 0.75){
		R = 1.0;
		G = static_cast<GLfloat>( 4.0 - 4.0 * value);
		B = 0.0;
	  }else if( value >= 0.5){
		R = static_cast<GLfloat>( 4.0 * value - 2.0);
		G = 1.0;
		B = 0.0;
	  }else if( value >= 0.25){
		R = 0.0;
		G = 1.0;
		B = static_cast<GLfloat>( -4.0 * value + 2.0);
	  }else{
		R = 0.0;
		G = static_cast<GLfloat>( 4.0 * value);
		B = 1.0;
	  }
  }
  else
  {
	  if(value>=0.5){
		R = 0; G=1.0; B=1.0;
	  }else{
		R=1.0; G=1.0; B=0;
	  }
  }
  glColor3f(R,G,B);
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  DrawModelMonitor(0, 0 , W_WIDTH/2, W_HEIGHT/2, models[0], false, 3);
  DrawModelMonitor(W_WIDTH/2,0, W_WIDTH/2,W_HEIGHT/2, models[1], false, 4);
//  DrawModelMonitor(0, 0, W_WIDTH, W_HEIGHT, models[1]);

  if(controllObject){
    DrawTextureMonitor(0, W_HEIGHT/2, W_WIDTH/2,W_HEIGHT/2, models[0], 1);
    DrawTextureMonitor(W_WIDTH/2,W_HEIGHT/2, W_WIDTH/2,W_HEIGHT/2, models[1], 2);
  }
  glutSwapBuffers();
}

void idle()
{
  glutPostRedisplay();
}

void specialkey(int key,int x, int y)
{
  switch(key){
    case GLUT_KEY_UP:
		models[manupulation-1]->mScales *= 2.f;
		break;
 
    case GLUT_KEY_DOWN:
    	models[manupulation-1]->mScales *= 0.5f;
    	break;
  }
}

void keyboard(unsigned char key, int x, int y)
{
  switch(key){
    case 'q': case '\033':
      exit(0);
      break;

    case 'w':
    	models[manupulation-1]->mTrans[1] += 0.5f;
    	break;

    case 'x':
    	models[manupulation-1]->mTrans[1] -= 0.5f;
    	break;

    case 'd':
    	models[manupulation-1]->mTrans[0] += 0.5f;
    	break;

    case 'a':
    	models[manupulation-1]->mTrans[0] -= 0.5f;
    	break;

    case 'g':
      break;

    case 'r':
    	if(controllObject == MANUPLATE){
    		controllObject = DECOMPOSITE;
    	}
    	else if(controllObject == DECOMPOSITE){
    		controllObject = TRANSFER;
    		displayTexture = false;
    	}
    	else if(controllObject == TRANSFER){
    		controllObject = SELECT;
    		displayTexture = false;
    	}
    	else if(controllObject == SELECT){
    		controllObject = MANUPLATE;
    		displayTexture = true;
    	}
      break;

    case 's':
    	if(manupulation == 1){
        	const char * f_str = "mesh1.bmp";
    		WriteBitmapFromGL(f_str, 0, W_HEIGHT/2, W_WIDTH/2,W_HEIGHT/2);
        	cout << "Save decomposed mesh from Obj1 -> " << f_str << endl;
    	}else{
        	const char * f_str = "mesh2.bmp";
    		WriteBitmapFromGL(f_str, W_WIDTH/2,W_HEIGHT/2, W_WIDTH/2,W_HEIGHT/2);

        	cout << "Save decomposed mesh from Obj2 -> " << f_str << endl;
    	}
		models[manupulation - 1]->SetMeshSelected(true);

    	PointsDisplay();

		if(models[0]->IsMeshSelected() && models[1]->IsMeshSelected())
		{
			controller.SetContourPoints();
			controller.AcquireMatching();
			TexturePaste(true);
			models[1]->Save3DModel("NewModel");
		}
    	break;
  }	
}

void mouse(int button, int state, int x, int y)
{
	if( controllObject == DECOMPOSITE ){
		// button ON
		//reserve mouse-coord and project into window-coord
		//while pressing the mouse button
		if(state == GLUT_DOWN && button == GLUT_LEFT_BUTTON){
			int * p = new int[2];
			p[0] = x;
			p[1] = y;
			point.push_back(p);
		}

		//button OFF
		if(state == GLUT_UP && button == GLUT_LEFT_BUTTON)
		{
			models[manupulation-1]->IncrementSumOfStrokes();

			//reserver click points
			cv::Point2d start_point , end_point;
			start_point.x = point[0][0];
			start_point.y = point[0][1];
			end_point.x   = point[point.size() - 1][0];
			end_point.y   = point[point.size() - 1][1];

			int window;
			if( manupulation-1 == 0) window = 3;
			else window = 4;

			if(models[manupulation-1]->
					CheckFittingVertices(viewport[window],
											modelview[window],
											projection[window],
											start_point, end_point))
			{
				models[manupulation-1]->UpdateMatrix();
				models[manupulation-1]->RenewMeshDataConstruct(2);
				models[manupulation-1]->mLSCM->mMesh->FindTextureMax();
			}

			point.clear();
		}
	}
	else if( controllObject == MANUPLATE )
	{
		switch(button){
		case GLUT_LEFT_BUTTON:
			if(state == GLUT_DOWN){
				mpos[0] = x;
				mpos[1] = y;
				mouse_l = 1;
			}
			if(state == GLUT_UP){
				mouse_l = 0;
			}
			break;
		default:
			break;
		}
	}
	else
	{
		// button ON
		//reserve mouse-coord and project into window-coord
		//while pressing the mouse button
		if(state == GLUT_DOWN && button == GLUT_LEFT_BUTTON){
			int * p = new int[2];
			p[0] = x;
			p[1] = y;
			point.push_back(p);
		}

		//button OFF
		if(state == GLUT_UP && button == GLUT_LEFT_BUTTON){

			//get each matrix parameter
//			SetMatrixParam();

			//reserver click points
			cv::Point2d start_point , end_point;
			start_point.x = point[0][0];
			start_point.y = point[0][1];
			end_point.x   = point[point.size() - 1][0];
			end_point.y   = point[point.size() - 1][1];

			int window;
			if( manupulation-1 == 0) window = 3;
			else window = 4;

			models[manupulation-1]->CorrespondTexCoord(viewport[window], modelview[window], projection[window], start_point, end_point, texPoint[0], texPoint[1], clickPoint[0], clickPoint[1]);

//			cout << " ------------ Texture Transfer Points ------------ " << endl;
//			cout << start_point.x << " " << start_point.y << endl;
//			cout << end_point.x << " " << end_point.y << endl;
//			cout << texPoint[0].x << " " << texPoint[0].y << endl;
//			cout << texPoint[1].x << " " << texPoint[1].y << endl << endl;
			point.clear();
		}

	}
}

void motion(int x, int y)
{
	// change control window in accordance with mouse cursor coord
	if( x < W_WIDTH/2 && 0 <= x){
	  manupulation = 1;
	}else{
	  manupulation = 2;
	}

	if( controllObject == DECOMPOSITE || controllObject == TRANSFER)
	{
		glLineWidth(5);
		int * p = new int[2];
		p[0] = x;
		p[1] = y;
		//    cout << x << " " << y << endl;
		point.push_back(p);
	}
	else
	{
		if(mouse_l == 1){
			theta[0] = (double)(y-mpos[1])/5.0;
			theta[1] = (double)(x-mpos[0])/5.0;
		}
		if(mouse_l == 1 || mouse_m == 1 || mouse_r == 1)
		{
			mpos[0] = x;
			mpos[1] = y;
			if( x < W_WIDTH/2 && 0 <= x)
			{
				models[0]->mAngles[0] += theta[0];
				models[0]->mAngles[1] += theta[1];
			}
			else
			{
				models[1]->mAngles[0] += theta[0];
				models[1]->mAngles[1] += theta[1];
			}
			glutPostRedisplay();
		}
	}
}

void CallbackEntry(void)
{
  glutDisplayFunc(display);
  glutSpecialFunc(specialkey);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutIdleFunc(idle);
}

void DrawModelMonitor(int x, int y, int w, int h, ViewingModel * model, bool isStroke, const int & separationW)
{
  //Viewport transform
  glViewport(x, y, w, h);
  
  //Projection transform
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(30.0, (double)w / (double)h, 0.1, 1000.0);

  //Modelview transform
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(20,30,80,0,0,0,0,1,0);

  glPushMatrix();

  glRotated(model->mAngles[0], 1.0, 0.0, 0.0);  //
  glRotated(model->mAngles[1], 0.0, 1.0, 0.0);  //
  glRotated(model->mAngles[2], 0.0, 0.0, 1.0);  //
  glTranslated(model->mTrans[0], model->mTrans[1], model->mTrans[2]);
  glScalef(model->mScales,model->mScales,model->mScales);
  glRotatef(90,-1,0,0);

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
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	if(!false)
	{
		//  Rendering a model
		glLineWidth(1);
		glColor3f(1.0f, 0.0f, 0.0f);

		REP(loopMesh, model->GetMeshSize())
		{
			GLdouble normal[3];
			GLdouble vertex[3];

			GLfloat ambient[4];
			GLfloat diffuse[4];
			GLfloat specular[4];

			glBegin(GL_TRIANGLES);
			REP(faceIdx,model->GetMeshFacesSize(loopMesh) )
			{
				REP(verIdx, model->GetMeshInnerFacesSize(loopMesh,faceIdx))
				{
					ColorSetting( model->QueryVertexColor(loopMesh, faceIdx, verIdx) , false);
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
	else
	{
		IndexedMesh * tmpMesh = model->mLSCM->mMesh.get();

		int isTriangles = 0;
		REP(texNumber, model->mTexture.size())
		{
			//テクスチャセット
			model->mTexture[texNumber]->bind();

			//for warping texture mapping
			double ratio_x = (W_WIDTH*0.5 -  1) / (tmpMesh->mTexMax.x - tmpMesh->mTexMin.x);
			double ratio_y = (W_HEIGHT*0.5 - 1) / (tmpMesh->mTexMax.y - tmpMesh->mTexMin.y);//

			glBegin(GL_TRIANGLES);
			for(unsigned int loopVer=0; loopVer<tmpMesh->mVertices.size(); loopVer++)
			{
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, lit_amb);
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, lit_dif);
				glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, lit_spc);

				if( tmpMesh->mTextureNumber[loopVer] == texNumber || isTriangles > 0){
					GLfloat texcos[2];
					texcos[0] = (tmpMesh->mTextureCoords[loopVer].x - tmpMesh->mTexMin.x) * ratio_x;
					texcos[1] = (W_HEIGHT/2 - (tmpMesh->mTextureCoords[loopVer].y - tmpMesh->mTexMin.y) * ratio_y) - 1;

					GLdouble vertex[3];
					vertex[0] = tmpMesh->mVertices[loopVer].point.x;
					vertex[1] = tmpMesh->mVertices[loopVer].point.y;
					vertex[2] = tmpMesh->mVertices[loopVer].point.z;
					glColor3f(1.0f, 1.0f, 1.0f);
					glTexCoord2fv(texcos);
					glVertex3dv(vertex);

					isTriangles++;
				}
				isTriangles %= 3;
			}
			glEnd();
			model->mTexture[texNumber]->unbind();
		}
	}
	//get each transform matrix
	glPopMatrix();

	// draw drag line
	double ox, oy, oz;
	GLint glX,glY,glZ;

	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);


	glDisable(GL_DEPTH_TEST);

	glLineWidth(5);
	glColor3f(1.0 , 0.0 , 0.0);
	if( point.size() > 1){
		glBegin(GL_LINE_STRIP);
		for(unsigned int i=0 ; i<point.size() ; i++){
			glX = point[i][0];
			glY = viewport[3]*2 - point[i][1];
			//      glReadPixels(glX,glY,1,1,GL_DEPTH_COMPONENT,GL_FLOAT,&z);
			glZ = 0;
			gluUnProject((GLdouble)glX, (GLdouble)glY, (GLdouble)glZ,modelview, projection, viewport, &ox, &oy, &oz);
			glVertex3d(ox,oy,oz);
		}
		glEnd();
	}
	DrawController();

  //display a sentence if a mesh in the model have been selected
  if(model->IsMeshSelected()){
	  glPushAttrib(GL_CURRENT_BIT|GL_DEPTH_BUFFER_BIT); // retrieve color and Z buffer
	  glColor3d(1,0,1);
	  void *letter = GLUT_BITMAP_TIMES_ROMAN_24;
	  DrawString("SELECTED", letter, -10, -10, 0);
	  glPopAttrib(); // write back color and Z buffer

  }

  // frame
//  glBegin(GL_LINES);
//  glX = W_WIDTH/2;
//  glY = 2*viewport[3];
//  glZ = 0;
//  gluUnProject((GLdouble)glX, (GLdouble)glY, (GLdouble)glZ,modelview, projection, viewport, &ox, &oy, &oz);
//  glVertex3d(ox,oy,oz);
//
//  glX = W_WIDTH/2;
//  glY = viewport[3];
//  glZ = 0;
//  gluUnProject((GLdouble)glX, (GLdouble)glY, (GLdouble)glZ,modelview, projection, viewport, &ox, &oy, &oz);
//  glVertex3d(ox,oy,oz);
//  glEnd();

  glEnable(GL_DEPTH_TEST);
}

void DrawTextureMonitor(int x, int y, int w, int h, ViewingModel * model, const int & seprationW)
{
	glViewport(x, y, w, h);
  
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(model->mLSCM->mMesh->mTexMin.x, model->mLSCM->mMesh->mTexMax.x,
	  model->mLSCM->mMesh->mTexMin.y, model->mLSCM->mMesh->mTexMax.y,
	  0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt (0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0); //default view point

	//render something from here
	// ------>

	//for debug (interaction points)
	glColor3d(0,0,.5);
	glLineWidth(3);
	glBegin(GL_LINES);
	glVertex2f(texPoint[0].x, texPoint[0].y);
	glVertex2f(texPoint[1].x, texPoint[1].y);
	glEnd();

	//for rendering texture deployments
	glLineWidth(1);
#if TEXTURE_TRIANGLES==0
	glBegin(GL_LINES);
#else
	glBegin(GL_TRIANGLES);
#endif

	IndexedMesh * im = model->mLSCM->mMesh.get();
	if( controllObject != SELECT){
	  int size = static_cast<int>(im->mTextureFaces.size());

//	  glColor3d(0,1,0);
	  for(int i=0; i<size; i+=3){
		Vector2 tmp1 = im->mTextureCoords[ im->mTextureFaces.at(i) - 1];
		Vector2 tmp2 = im->mTextureCoords[ im->mTextureFaces.at(i+1) - 1];
		Vector2 tmp3 = im->mTextureCoords[ im->mTextureFaces.at(i+2) - 1];

		double val = model->mLSCM->mMesh->mTexParts[i];
		ColorSetting(val, true);

#if TEXTURE_TRIANGLES==0
		glVertex2f(tmp1.x,tmp1.y);
		glVertex2f(tmp2.x,tmp2.y);
		glVertex2f(tmp2.x,tmp2.y);
		glVertex2f(tmp3.x,tmp3.y);
		glVertex2f(tmp3.x,tmp3.y);
		glVertex2f(tmp1.x,tmp1.y);
#else
		glVertex2f(tmp1.x,tmp1.y);
		glVertex2f(tmp2.x,tmp2.y);
		glVertex2f(tmp3.x,tmp3.y);

#endif
	  }
  }
  //in the case of selecting parts
  else
  {
//	  glColor3d(0,1,0);
	  for(unsigned int i=0; i<model->mSelectedMesh.second.mTextureCoords.size(); i+=3)
	  {
			Vector2 tmp1 = model->mSelectedMesh.second.mTextureCoords[i+0].second;
			Vector2 tmp2 = model->mSelectedMesh.second.mTextureCoords[i+1].second;
			Vector2 tmp3 = model->mSelectedMesh.second.mTextureCoords[i+2].second;

			double value = model->mLSCM->mMesh->mTexParts[model->mSelectedMesh.second.index[i]];
			ColorSetting(value, true);

#if TEXTURE_TRIANGLES==0
			glVertex2f(tmp1.x,tmp1.y);
			glVertex2f(tmp2.x,tmp2.y);
			glVertex2f(tmp2.x,tmp2.y);
			glVertex2f(tmp3.x,tmp3.y);
			glVertex2f(tmp3.x,tmp3.y);
			glVertex2f(tmp1.x,tmp1.y);
#else
			glVertex2f(tmp1.x,tmp1.y);
			glVertex2f(tmp2.x,tmp2.y);
			glVertex2f(tmp3.x,tmp3.y);

#endif
	  }
  	}
	glEnd();

	//drawing frame lines
	//  glColor3d(0,0,0);
	//  glLineWidth(5);
	//  glBegin(GL_LINES);
	//  glVertex2f(im->mTexMin.x, im->mTexMin.y);
	//  glVertex2f(im->mTexMax.x, im->mTexMin.y);
	//  glVertex2f(im->mTexMax.x, im->mTexMax.y);
	//  glVertex2f(im->mTexMax.x, im->mTexMin.y);
	//  glEnd();

	//<-------
	//this is the end of rendering part

	glEnable(GL_DEPTH_TEST);
}

void PointsDisplay()
{
	IndexedMesh * tmpMesh = models[manupulation-1]->mLSCM->mMesh.get();
	double ratio_x = (W_WIDTH*0.5 -  1) / (tmpMesh->mTexMax.x - tmpMesh->mTexMin.x);
	double ratio_y = (W_HEIGHT*0.5 - 1) / (tmpMesh->mTexMax.y - tmpMesh->mTexMin.y);

	IplImage * input = cvLoadImage("mesh1.bmp", 0);
#if VISUALIZE == 1
	IplImage * src   = cvCreateImage( cvGetSize(input), 8, 3);
	const char * winName = "Convex Hull";
	cvZero(src);
#endif

	controller.mMeshes[manupulation-1].clear();
	assert(input);
    for(unsigned int i=0; i<models[manupulation-1]->mSelectedMesh.second.mTextureCoords.size(); i++){
		Vector2 tmp1 = models[manupulation-1]->mSelectedMesh.second.mTextureCoords[i].second;
		cv::Point tmp;
		tmp.x = (tmp1.x - tmpMesh->mTexMin.x) * ratio_x;
		tmp.y = (input->height - (tmp1.y - tmpMesh->mTexMin.y) * ratio_y) - 1;
		pair<int , cv::Point > tmpPair;
		tmpPair.first  = models[manupulation-1]->mSelectedMesh.second.index[i];
		tmpPair.second = tmp;
		controller.mMeshes[manupulation-1].push_back(tmpPair);
#if VISUALIZE == 1
		cvCircle( src, tmp, 2, CV_RGB( 255, 255, 0 ), CV_FILLED );
#endif
//		cout << "Model" << manupulation-1 << " Index=" << tmpPair.first << " ; HarmonicVal=" << models[manupulation-1]->mLSCM->mMesh->mTexParts[tmpPair.first] << endl;
		controller.SetHashmap( tmp.x, tmp.y, tmpPair.first, manupulation-1);
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
	if(color){
#if FEEDBACK_VISUALIZE == 1
		IplImage * input = cvLoadImage("mesh1.bmp", 0);
		IplImage * src   = cvCreateImage( cvGetSize(input), 8, 3);
		const char * winName = "Convex Hull";
		cvZero(src);
#endif

		REP(id,controller.mMatchingPoints.size()){
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
		}

		REP(id, controller.mMeshes[1].size()){
			//テクスチャ画像番号を更新
			models[1]->mLSCM->mMesh->mTextureNumber[controller.mMeshes[1].at(id).first] = 1;//models[1]->mTexture.size();
		}

		::ImageType TextureRGB = (CVD::img_load("warping1.bmp"));
		Texture * tmpTexture = new Texture( static_cast<const ::ImageType> (TextureRGB));

		models[1]->mTexture.push_back(tmpTexture);

		cout << "Texture Transfer DONE!! Left -> Right" << endl;

		//reset selected mesh
		REP(i,2){
			models[i]->SetMeshSelected(false);
		}
		controller.InitHashmap();
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
	glClearColor(1,1,1,1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	glLightfv(GL_LIGHT0, GL_AMBIENT, lit_amb);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lit_dif);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lit_spc);
	glLightfv(GL_LIGHT0, GL_POSITION, lit_pos);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glShadeModel(GL_SMOOTH);

	//load 3ds model
	const char * model1Name = "Model3DS/Torus.3ds";
	const char * model2Name = "Model3DS/cow.obj";
	models[0] = new ViewingModel(model1Name);
	models[1] = new ViewingModel(model2Name);
	manupulation = 1;

	models[0]->LoadTexture("texture1.bmp");
	models[0]->ConvertDataStructure();
	models[0]->mLSCM->run("CG","");
	models[0]->mLSCM->mMesh->save("Model3DS/test2.obj");
	models[0]->mLSCM->mMesh->FindTextureMax();
//
	models[1]->LoadTexture("Hatsune2.bmp");
	models[1]->ConvertDataStructure();
	models[1]->mLSCM->run("CG","");
	models[1]->mLSCM->mMesh->save("Model3DS/voxel3.obj");
	models[1]->mLSCM->mMesh->FindTextureMax();

	controller.InitHashmap();
	displayTexture = true;
}

int main(int argc, char *argv[])
{
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_RGBA|GLUT_DEPTH|GLUT_DOUBLE);
	glutInitWindowSize(W_WIDTH,W_HEIGHT);

	glutCreateWindow("Mesh Decompositon Using Harmonic Field");

	CallbackEntry();

	Init();

	// start to display image
	glutMainLoop();

	return 0;
}
