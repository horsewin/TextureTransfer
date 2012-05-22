#include <vector>
#include <iostream>
#include <stdio.h>

#include "OpenGL.h"
#include "OpenCV.h"

#include "main.h"
#include "ViewingModel.h"
#include "IndexedMesh.h"
#include "LSCM.h"
#include "Bitmap.h"
#include "TransferController.h"

#define HARMONIC 1
#define TEXTURE_TRIANGLES 1

using namespace std;

// to control object
int mouse_l = 0;
int mouse_m = 0;
int mouse_r = 0;
int mpos[2];
double trans[3] = {0.0, 0.0, 0.0};
double theta[3] = {0.0, 0.0, 0.0};
enum CONTROLLER{DECOMPOSITE, MANUPLATE, TRANSFER, SELECT};
CONTROLLER controllObject;

const int SEPARATION = 5;
/////////////////////

GLint viewport[SEPARATION][4];
GLdouble modelview[SEPARATION][16];
GLdouble projection[SEPARATION][16];

static GLfloat lit_amb[4]={0.4f, 0.4f, 0.4f,1.0};	/* 環境光の強さ */
static GLfloat lit_dif[4]={1.0, 1.0, 1.0, 1.0};	/* 拡散光の強さ */
static GLfloat lit_spc[4]={0.4f, 0.4f, 0.4f, 1.0};	/* 鏡面反射光の強さ */
static GLfloat lit_pos[4]={0.0, 0.0, -9.0, 1.0};	/* 光源の位置 */

Vector2 texPoint[2];
Vector3 clickPoint[2];


void *font = GLUT_BITMAP_HELVETICA_18;
vector<int *> point;

ViewingModel * models[2];
short manupulation;

void SetMatrixParam( );
void DrawModelMonitor(int x, int y, int w, int h, ViewingModel * model, bool isStroke, const int & separationW);
void DrawTextureMonitor(int x, int y, int w, int h, ViewingModel * model, const int & seprationW);
void ConvexHull();

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
  glColor3d(1,0,1);
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

void ColorSetting(const double & value)
{
  // get the value calculated poisson equation  
  //  assert( value <= 1.0);
  GLfloat R,G,B;
  //if(value<=1.0);
  //  else cout << value << endl;
#if HARMONIC == 1
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
#else
  if(value>=0.5){
    R = 0; G=1.0; B=1.0;
  }else{
    R=1.0; G=1.0; B=0;
  }
#endif
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
      
    case 'g':
      break;

    case 'r':
    	if(controllObject == MANUPLATE) 		controllObject = DECOMPOSITE;
    	else if(controllObject == DECOMPOSITE)	controllObject = TRANSFER;
    	else if(controllObject == TRANSFER)	controllObject = SELECT;
    	else if(controllObject == SELECT)		controllObject = MANUPLATE;
      break;

    case 's':
    	const char * f_str = "mesh.bmp";
    	if(manupulation == 1){
    		WriteBitmapFromGL(f_str, 0, W_HEIGHT/2, W_WIDTH/2,W_HEIGHT/2);
        	cout << "Save decomposited mesh from Obj1 -> " << f_str << endl;
    	}else{
    		WriteBitmapFromGL(f_str, W_WIDTH/2,W_HEIGHT/2, W_WIDTH/2,W_HEIGHT/2);
        	cout << "Save decomposited mesh from Obj2 -> " << f_str << endl;
    	}
//    	ConvexHull();
    	TransferController controller;
    	controller.SetContourPoints();
    	controller.AcquireMatching();
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
		if(state == GLUT_UP && button == GLUT_LEFT_BUTTON){
			models[manupulation-1]->IncrementSumOfStrokes();

//			//get each matrix parameter
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

			if(models[manupulation-1]->
					CheckFittingVertices(viewport[window],
											modelview[window],
											projection[window],
											start_point, end_point))
			{
				models[manupulation-1]->UpdateMatrix();
				models[manupulation-1]->RenewMeshDataConstruct(2);
				models[manupulation-1]->mLSCM->mesh_->FindTextureMax();
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
  glScalef(model->mScales,model->mScales,model->mScales);
  glRotatef(90,-1,0,0);

  //reserve projection matrices for rendering a model
  glGetIntegerv(GL_VIEWPORT, viewport[separationW]);
  glGetDoublev(GL_PROJECTION_MATRIX, projection[separationW]);
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview[separationW]);
  
//  //for debug drawing lines
//  glColor3f(1.0,1.0,0.0);
//  glLineWidth(5);
//  glBegin(GL_LINE_STRIP);
//  glVertex3d(clickPoint[0].x, clickPoint[0].y, clickPoint[0].z);
//  glVertex3d(clickPoint[1].x, clickPoint[1].y, clickPoint[1].z);
//  glEnd();

  //Rendering a model
  glLineWidth(1);
  glColor3f(1.0f, 0.0f, 0.0f);

  int nMeshs = model->GetMeshSize();
//  int nMaterials =
//  REP(material, )
//  //テクスチャセット
//  texturesList[loop]->bind();

  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);

  REP(loop,nMeshs){
	GLdouble normal[3];
    GLdouble vertex[3];

	GLfloat ambient[4];
	GLfloat diffuse[4];
	GLfloat specular[4];

    glBegin(GL_TRIANGLES);
    REP(id,model->GetMeshIndicesSum(loop) ){
      ColorSetting( model->QueryVertexColor(loop,id) );
      model->QueryNormal(loop, id, normal);
      model->QueryVertex(loop, id, vertex);
      model->QueryAmbient(loop, id, ambient);
      model->QueryDiffuse(loop, id, diffuse);
      model->QuerySpecular(loop, id, specular);

		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
//      glColor3f(diffuse[0]*lit_dif[0], diffuse[1]*lit_dif[1], diffuse[2]*lit_dif[2]);
//      glColor3f(diffuse[0], diffuse[1], diffuse[2]);
      glNormal3dv(normal);
      glVertex3dv(vertex);
    }
    glEnd();
  }

  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);

//	IndexedMesh * im = model->mLSCM->mesh_;
//
//	glBegin(GL_TRIANGLES);
//	for(unsigned int loopVer=0; loopVer<im->vertex.size(); loopVer++){
//		GLdouble normal[3];
//		GLdouble vertex[3];
//		cv::Point3d meshVertex;
//		meshVertex.x = im->vertex[loopVer].point.x;
//		meshVertex.y = im->vertex[loopVer].point.y;
//		meshVertex.z = im->vertex[loopVer].point.z;
//		glVertex3d(meshVertex.x, meshVertex.y, meshVertex.z);
//	}
//	glEnd();
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
  glOrtho(model->mLSCM->mesh_->mTexMin.x, model->mLSCM->mesh_->mTexMax.x, 
 	  model->mLSCM->mesh_->mTexMin.y, model->mLSCM->mesh_->mTexMax.y, 
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

  IndexedMesh * im = model->mLSCM->mesh_.get();
  if( controllObject != SELECT){
	  int size = static_cast<int>(im->mTextureFaces.size());

	  glColor3d(0,1,0);
	  for(int i=0; i<size; i+=3){
		Vector2 tmp1 = im->mTextureCoords[ im->mTextureFaces.at(i) - 1];
		Vector2 tmp2 = im->mTextureCoords[ im->mTextureFaces.at(i+1) - 1];
		Vector2 tmp3 = im->mTextureCoords[ im->mTextureFaces.at(i+2) - 1];

		double val = model->mLSCM->mesh_->mTexParts[i];
		ColorSetting(val);

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
	  glColor3d(0,1,0);
	  for(unsigned int i=0; i<model->mSelectedMesh.second.mTextureCoords.size(); i+=3)
	  {
			Vector2 tmp1 = model->mSelectedMesh.second.mTextureCoords[i+0].second;
			Vector2 tmp2 = model->mSelectedMesh.second.mTextureCoords[i+1].second;
			Vector2 tmp3 = model->mSelectedMesh.second.mTextureCoords[i+2].second;

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

void SetMatrixParam( )
{
//	glPushMatrix();
//
//	glRotated(models[manupulation-1]->mAngles[0], 1.0, 0.0, 0.0);  //
//	glRotated(models[manupulation-1]->mAngles[1], 0.0, 1.0, 0.0);  //
//	glRotated(models[manupulation-1]->mAngles[2], 0.0, 0.0, 1.0);  //
//	glScalef(models[manupulation-1]->mScales,models[manupulation-1]->mScales,models[manupulation-1]->mScales);
//
//	glRotatef(90,-1,0,0);
//	glGetIntegerv(GL_VIEWPORT, viewport);
//	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);	//幾何変換（モデルビュー）行列
//	glGetDoublev(GL_PROJECTION_MATRIX, projection);	//投影変換行列
//	glPopMatrix();
}

void ConvexHull()
{
	IndexedMesh * tmpMesh = models[manupulation-1]->mLSCM->mesh_.get();
	double ratio_x = (W_WIDTH*0.5 - 0) / (tmpMesh->mTexMax.x - tmpMesh->mTexMin.x);
	double ratio_y = (W_HEIGHT*0.5 - 0) / (tmpMesh->mTexMax.y - tmpMesh->mTexMin.y);

	IplImage * input = cvLoadImage("mesh.bmp", 0);
	IplImage* img = cvCreateImage( cvGetSize(input), 8, 3);

    vector<cv::Point> meshes;

    for(unsigned int i=0; i<models[manupulation-1]->mSelectedMesh.second.mTextureCoords.size(); i++){
		Vector2 tmp1 = models[manupulation-1]->mSelectedMesh.second.mTextureCoords[i].second;
		int x = (tmp1.x - tmpMesh->mTexMin.x) * ratio_x;
		int y = input->height - (tmp1.y - tmpMesh->mTexMin.y) * ratio_y;
		cv::Point tmp(x, y);
		meshes.push_back(tmp);
	}
	const char * winName = "Convex Hull";
	cvZero(img);
	REP(i, meshes.size()){
	 cvCircle( img, meshes[i], 2, CV_RGB( 255, 255, 0 ), CV_FILLED );
	}
	cvNamedWindow(winName, 1);
	cvShowImage( winName, img );

	int key = cvWaitKey(0);
	cvReleaseImage(&img);
	cvReleaseImage(&input);

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

  //load 3ds model
  models[0] = new ViewingModel("Model3DS/DORA.3ds");
  models[1] = new ViewingModel("Model3DS/HatuneMiku.3ds");
  manupulation = 1;

  models[0]->ConvertDataStructure();
  models[0]->mLSCM->run("CG","");
  models[0]->mLSCM->mesh_->save("Model3DS/test2.obj");
  models[0]->mLSCM->mesh_->FindTextureMax();

  models[1]->ConvertDataStructure();
  models[1]->mLSCM->run("CG","");
  models[1]->mLSCM->mesh_->save("Model3DS/voxel3.obj");
  models[1]->mLSCM->mesh_->FindTextureMax();
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
