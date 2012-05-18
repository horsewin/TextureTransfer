#include "Modelling/Model3ds.h"
#include "OpenGL.h"

#include <gvars3/instances.h>

#include <iostream>
//#include "myCVmethod.h"
#include "main.h"

#include <cvd/image_io.h>

using namespace std;
using namespace CVD;
using namespace GVars3;

/**
 * Constructor
 */
Model3DS::Model3DS() :
	mbLoaded(false), msName("Default Name"), mdScaleMult(6.0), mdScale(exp(
			-mdScaleMult)), mbDelayedLoad(false) {
}

/**
 * Destructor
 */
Model3DS::~Model3DS() {
	if (mbLoaded) {
		///@TODO appears to be a gl context issue when reseting the map,
		/// and hence the game data from outside of the gl context. Disabling the clean up for now.
		//    glDeleteLists( mnDisplayList[0], 1 );
		//    glDeleteLists( mnDisplayList[1],1 );
	}
}

/**
 * Load a model from a specifed file.
 * This is the internal load function that the other external ones call.
 * @return success
 */
bool Model3DS::_Load() {
	if (mbLoaded) {
		cout << "Model " << msName << " already loaded" << endl;
		return false;
	}

	// attempt to load the model file
	std::string sFullPath = msModelDir + "/" + msModelFile;
	Lib3dsFile * pModel = lib3ds_file_open(sFullPath.c_str());
	//  cout << "Load Model -> " << pModel->name << endl;
	if (pModel == NULL) {
		cout << "Failed to load model " << sFullPath << endl;
		return false;
	}

	// Load the textures
	//Texture loader 2011.6.6
	std::deque<Texture *> texturesList; // A set of Textures 2011.6.7
	texturesList = _LoadTextures(pModel, msModelDir);

	// generate the display lists
	//  mnDisplayList[0] = _GenerateDisplayList( pModel, false ); // polygon
	mnDisplayList[0] = _GenerateDisplayList(pModel, false, texturesList); // polygon
	mnDisplayList[1] = _GenerateDisplayList(pModel, true); // wireframe

	//failed to create display lists
	if (mnDisplayList[0] == 0 || mnDisplayList[1] == 0) {
		lib3ds_file_free(pModel);
		return false;
	}

	//find the object's bounding box
	_FindBoundingBox(pModel);

	//delete the lib3ds model, as not needed now.
	lib3ds_file_free(pModel);

	mbLoaded = true;
	return true;
}

/**
 * Load a model from a specifed file
 * @param sFileName File to load (path + name)
 * @param v3Rotation Rotation required to align model with the axes
 * @return success
 */
bool Model3DS::Load(std::string sModelDir, std::string sFileName, TooN::Vector<
		3> v3Rotation) {
	msModelDir = sModelDir;
	msModelFile = sFileName;
	if (_Load()) {
		mse3ModelOffset.get_rotation() = SO3<>::exp(v3Rotation);

		//set initial scale
		double dMaxLen = max(
				(abs(mv3DimensionMin[0]) + abs(mv3DimensionMax[0])), max((abs(
						mv3DimensionMin[1]) + abs(mv3DimensionMax[1])), (abs(
						mv3DimensionMin[2]) + abs(mv3DimensionMax[2]))));
		mdScale = 0.6 / dMaxLen;
		mdScaleMult = -log(mdScale);

		return true;
	}

	return false;
}

/**
 * Draw the model
 */
void Model3DS::Draw() {
	//is there a load from a saved map waiting?
	if (mbDelayedLoad) {
		_DelayedLoad();
	}

	//has a model been loaded?
	if (!mbLoaded) {
		return;
	}

	// Draw the model's axes?
	if (GV3::get<int>("ModelsGame.DrawModelAxes", "0", SILENT)) {
		glPushMatrix();
		glMultMatrix(mse3MfromW);
		glScaled(mdScale, mdScale, mdScale);
		glTranslate(mv3Offset);
		_DrawAxes();
		glPopMatrix();
	}

	glPushMatrix();
	glMultMatrix(mse3MfromW * mse3ModelOffset);
	glScaled(mdScale, mdScale, mdScale);
	glTranslate(mv3Offset);

	// Draw the wireframe model or the textured/solid model?
	if (GV3::get<int>("ModelsGame.DrawWireframe", "0", SILENT)) {
		glCallList(mnDisplayList[1]);
	} else {
		glCallList(mnDisplayList[0]);
	}

	// Draw the model bounding box?
	if (GV3::get<int>("ModelsGame.DrawBoundingBox", "0", SILENT)) {
		_DrawBoundingBox();
	}

	glPopMatrix();

}

/**
 * Generate the display lists for the 3D model
 * @return the list reference
 */
GLuint Model3DS::_GenerateDisplayList(Lib3dsFile * pModel, bool bWireframe) {
	assert( pModel );

	//some defaults
	static GLfloat ambiant[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
	static GLfloat diffuse[4] = { 0.8f, 0.8f, 0.8f, 1.0f };
	static GLfloat spectal[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

	GLuint list = glGenLists(1);
	if (list != 0) {
		glNewList(list, GL_COMPILE);

		// Loop through every mesh
		for (int mm = 0; mm < pModel->nmeshes; mm++) {
			Lib3dsMesh * mesh = pModel->meshes[mm];
			Vector3D * normals = new Vector3D[3 * mesh->nfaces];
			//calculate the normals
			lib3ds_mesh_calculate_vertex_normals(mesh, normals);

			glDisable(GL_TEXTURE_2D);
			glDisable(GL_BLEND);

			// Begin drawing with our selected mode
			bWireframe ? glBegin(GL_LINES) : glBegin(GL_TRIANGLES);

			// Go through all of the faces (polygons) of the object and draw them
			for (int ff = 0; ff < mesh->nfaces; ff++) {
				Lib3dsFace & face = mesh->faces[ff];

				glColor3ub(255, 255, 255);

				if (face.material >= 0) {
					//			cout << "material : " << face.material << endl;

					Lib3dsMaterial * material =
							pModel->materials[face.material]; // Load Material(Texture)
					glMaterialfv(GL_FRONT, GL_AMBIENT, material->ambient);
					glMaterialfv(GL_FRONT, GL_DIFFUSE, material->diffuse);
					glMaterialfv(GL_FRONT, GL_SPECULAR, material->specular);
					float s = pow(2.0f, 10.0f * material->shininess); // make shininess an openGL value
					if (s > 128.0f) {
						s = 128.0f;
					}
					glMaterialf(GL_FRONT, GL_SHININESS, s);
					///@TODO fix the color, material and light settings.
					glColor3fv(material->diffuse);
				} else // assign default material values
				{
					glMaterialfv(GL_FRONT, GL_AMBIENT, ambiant);
					glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
					glMaterialfv(GL_FRONT, GL_SPECULAR, spectal);
				}

				// Go through each corner of the triangle and draw it.
				for (int vv = 0; vv < 3; vv++) {
					// Get the index for each point of the face
					int index = face.index[vv];

					// Give OpenGL the normal for this vertex.
					Vector3D & v3Norm = normals[3 * ff + vv];
					glNormal3fv(v3Norm);

					// Draw in the current vertex of the object (Corner of current face)
					glVertex3fv(mesh->vertices[index]);
				}
			}

			glEnd();

		}

		glEndList();
	}
	return list;
}

/**
 * Generate the display lists for the 3D model
 * Edited by Umakatsu 2011.06.08
 * @return the list reference
 */

GLuint Model3DS::_GenerateDisplayList(Lib3dsFile * pModel, bool bWireframe, std::deque<Texture *> texturesList)
{
	assert( pModel );

	vector<Materials> materials;//材質データ	構造
	Materials MatDefault;//デフォルト用
	vector<int>::iterator Iter;
	vector<int>::iterator EndIter;
	Lib3dsMaterial *material;
	Lib3dsMesh *mesh;
	//材質データ構造のメモリ確保
	materials.resize(pModel->nmaterials);
	for (int loop = 0; loop < pModel->nmaterials; ++loop) {
		material = pModel->materials[loop];//loop番目のマテリアル
		//ambient
		memcpy(materials[loop].ambient, material->ambient,
				sizeof(material->ambient));
		materials[loop].ambient[3] = 1;

		//diffuse
		memcpy(materials[loop].diffuse, material->diffuse,
				sizeof(material->diffuse));
		materials[loop].diffuse[3] = 1 - material->transparency;//アルファ値はdiffuseのみ

		//shinness
		memcpy(&materials[loop].shininess, &material->shininess,
				sizeof(material->shininess));
		if (materials[loop].shininess == 0.01f) {
			materials[loop].shininess = 20.f;
		} else {
			materials[loop].shininess *= 128;
		}

		//specular
		memcpy(materials[loop].specular, material->specular,
				sizeof(material->specular));
		materials[loop].specular[3] = 1;

		//各材質データ内のメンバでメモリ確保
		materials[loop].meshIdx.resize(pModel->nmeshes);
		for (int n = 0; n < pModel->nmeshes; ++n) {
			mesh = pModel->meshes[n];//n番目のメッシュ
			materials[loop].meshIdx[n].faceId.reserve(mesh->nfaces
					/ pModel->nmaterials);//適当にメモリ確保
		}
	}

	//材質デフォルトのメモリ確保
	MatDefault.meshIdx.resize(pModel->nmeshes);
	for (int loop1 = 0; loop1 < pModel->nmeshes; ++loop1) {
		mesh = pModel->meshes[loop1];
		if (pModel->nmaterials != 0)
			MatDefault.meshIdx[loop1].faceId.reserve(mesh->nfaces
					/ pModel->nmaterials);
		//            else  MatDefault.meshIdx[loop1].faceId.reserve(mesh->nfaces);
	}

	//材質データ構造に対応するメッシュデータIDを格納する
	static int MatId = 0;
	for (int k = 0; k < pModel->nmeshes; ++k) {
		mesh = pModel->meshes[k];//k番目のメッシュ
		for (int i = 0; i < mesh->nfaces; ++i) {
			MatId = mesh->faces[i].material;
			if (MatId == -1) {
				MatDefault.meshIdx[k].faceId.push_back(i);
			} else {
				materials[MatId].meshIdx[k].faceId.push_back(i);
			}
		}
		std::vector<int>(MatDefault.meshIdx[k].faceId).swap(
				MatDefault.meshIdx[k].faceId);
	}

	//shrink - to - fitで確保メモリの調整
	for (int loop = 0; loop < pModel->nmaterials; ++loop) {
		for (int j = 0; j < pModel->nmeshes; ++j) {
			std::vector<int>(materials[loop].meshIdx[j].faceId).swap(
					materials[loop].meshIdx[j].faceId);
		}

	}

	//some defaults
	static GLfloat ambiant[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
	static GLfloat diffuse[4] = { 0.8f, 0.8f, 0.8f, 1.0f };
	static GLfloat spectal[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

	GLuint list = glGenLists(1);
	if (list != 0) {
		glNewList(list, GL_COMPILE);
		cout << "The number of materials : " << pModel->nmaterials << endl;
		// Loop through every mesh
		REP(nm,pModel->nmaterials) {
			bool textureOn = false; // if the material has a texture -> true
			string sFile = pModel->materials[nm]->texture1_map.name;
			if (!sFile.empty() && texturesList.size() > 0) {
				textureOn = true;
				cout << "Texture Name(material" << nm << ") : "
						<< sFile.c_str() << endl;
				// Activate TextureMapping
				texturesList.front()->bind();
			} else {
				glMaterialfv(GL_FRONT, GL_AMBIENT, materials[nm].ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, materials[nm].diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, materials[nm].specular);
				float s = pow(2.0f, 10.0f * materials[nm].shininess); // make shininess an openGL value
				if (s > 128.0f) {
					s = 128.0f;
				}
				glMaterialf(GL_FRONT, GL_SHININESS, s);
			}
			REP(mm,pModel->nmeshes) {
				Lib3dsMesh * mesh = pModel->meshes[mm];
				Vector3D * normals = new Vector3D[3 * mesh->nfaces];

				//calculate the normals
				lib3ds_mesh_calculate_vertex_normals(mesh, normals);

				//			glDisable(GL_TEXTURE_2D);
				//			glDisable(GL_BLEND);
				// Begin drawing with our selected mode
				bWireframe ? glBegin(GL_LINES) : glBegin(GL_TRIANGLES);
				EndIter = materials[nm].meshIdx[mm].faceId.end();
				for (Iter = materials[nm].meshIdx[mm].faceId.begin(); Iter!= EndIter; ++Iter)
				{
					// Go through each corner of the triangle and draw it.
					REP(vv,3) {
						// Get the index for each point of the face
						int index = mesh->faces[(*Iter)].index[vv];

						// Give OpenGL the normal for this vertex.
						// 2012.3.7 法線ベクトルの設定を切ると正しく表示された
						// テクスチャマッピングの時は法線設定しないほうがよい？
//						Vector3D & v3Norm = normals[3 * (*Iter) + vv];
//						glNormal3fv(v3Norm);

						// Draw in the current vertex of the object (Corner of current face)
						if (textureOn){
							glTexCoord2fv(mesh->texcos[index]);
						}
						glVertex3fv(mesh->vertices[index]);
//						cout << mesh->texcos[index][0] << " " << mesh->texcos[index][1] << endl;
					}
				}
				glEnd();
				delete normals;
			}
			if (textureOn) {
				Texture * topTexture = texturesList.front();
				topTexture->unbind();
				texturesList.pop_front();
			}
		}

		//材質未設定部分を描画
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambiant); //面がもつ材質IDを指定する
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spectal);
		for (int n = 0; n < pModel->nmeshes; ++n) {
			Lib3dsMesh *mesh = pModel->meshes[n];//n番目のメッシュ + 材質がk番目のものが対象
			Vector3D * normals = new Vector3D[3 * mesh->nfaces];
			//calculate the normals
			lib3ds_mesh_calculate_vertex_normals(mesh, normals);

			EndIter = MatDefault.meshIdx[n].faceId.end();
			bWireframe ? glBegin(GL_LINES) : glBegin(GL_TRIANGLES);
			for (Iter = MatDefault.meshIdx[n].faceId.begin(); Iter != EndIter; ++Iter) {
				// Go through each corner of the triangle and draw it.
				REP(vv,3) {
					// Get the index for each point of the face
					int index = mesh->faces[(*Iter)].index[vv];

					// Give OpenGL the normal for this vertex.
					Vector3D & v3Norm = normals[3 * (*Iter) + vv];
					glNormal3fv(v3Norm);

					// Draw in the current vertex of the object (Corner of current face)
					glVertex3fv(mesh->vertices[index]);
				}
			}
			glEnd();
			delete normals;
		}
		glEndList();
	}
	return list;
}

/**
 * Reset the model pose.
 */
void Model3DS::Reset() {
	mse3MfromW = SE3<> ();
}

/**
 * Increase the model scale
 */
void Model3DS::ScaleIncrease() {
	mdScaleMult -= 0.04;
	mdScale = exp(-mdScaleMult);

}

/**
 * Decrease the model scale
 */
void Model3DS::ScaleDecrease() {
	mdScaleMult += 0.1;
	mdScale = exp(-mdScaleMult);
}

/**
 * Move forwards
 */
void Model3DS::MoveForwards() {
	double inc = GV3::get<double>("ModelsGame.MoveIncrement", "0.01", SILENT
			| HIDDEN);
	mse3MfromW.get_translation() += mse3MfromW.get_rotation() * makeVector(inc,
			0, 0);
}

/**
 * Move backwards
 */
void Model3DS::MoveBack() {
	double inc = GV3::get<double>("ModelsGame.MoveIncrement", "0.01", SILENT
			| HIDDEN);
	mse3MfromW.get_translation() += mse3MfromW.get_rotation() * makeVector(
			-inc, 0, 0);
}

/**
 * Move to the left
 */
void Model3DS::MoveLeft() {
	double inc = GV3::get<double>("ModelsGame.MoveIncrement", "0.01", SILENT
			| HIDDEN);
	mse3MfromW.get_translation() += mse3MfromW.get_rotation() * makeVector(0,
			inc, 0);
}

/**
 * move to the right
 */
void Model3DS::MoveRight() {
	double inc = GV3::get<double>("ModelsGame.MoveIncrement", "0.01", SILENT
			| HIDDEN);
	mse3MfromW.get_translation() += mse3MfromW.get_rotation() * makeVector(0,
			-inc, 0);
}

/**
 * move up
 */
void Model3DS::MoveUp() {
	double inc = GV3::get<double>("ModelsGame.MoveIncrement", "0.01", SILENT
			| HIDDEN);
	mse3MfromW.get_translation() += mse3MfromW.get_rotation() * makeVector(0,
			0, inc);
}

/**
 * Move down
 */
void Model3DS::MoveDown() {
	double inc = GV3::get<double>("ModelsGame.MoveIncrement", "0.01", SILENT
			| HIDDEN);
	mse3MfromW.get_translation() += mse3MfromW.get_rotation() * makeVector(0,
			0, -inc);
}

/**
 * Increase the pitch angle
 */
void Model3DS::PitchIncrease() {
	double inc = GV3::get<double>("ModelsGame.RotateIncrement", "1.0", SILENT
			| HIDDEN) * M_PI / 180;
	Vector<3> rot = makeVector(0.0, inc, 0.0);
	mse3MfromW.get_rotation() *= SO3<>::exp(rot);
}

/**
 * Decrease the pitch angle
 */
void Model3DS::PitchDecrease() {
	double inc = GV3::get<double>("ModelsGame.RotateIncrement", "1.0", SILENT
			| HIDDEN) * M_PI / 180;
	Vector<3> rot = makeVector(0.0, -inc, 0.0);
	mse3MfromW.get_rotation() *= SO3<>::exp(rot);
}

/**
 * Increase the roll angle
 */
void Model3DS::RollIncrease() {
	double inc = GV3::get<double>("ModelsGame.RotateIncrement", "1.0", SILENT
			| HIDDEN) * M_PI / 180;
	Vector<3> rot = makeVector(inc, 0.0, 0.0);
	mse3MfromW.get_rotation() *= SO3<>::exp(rot);
}

/**
 * Decrease the roll angle
 */
void Model3DS::RollDecrease() {
	double inc = GV3::get<double>("ModelsGame.RotateIncrement", "1.0", SILENT
			| HIDDEN) * M_PI / 180;
	Vector<3> rot = makeVector(-inc, 0.0, 0.0);
	mse3MfromW.get_rotation() *= SO3<>::exp(rot);
}

/**
 * Increase the yaw angle
 */
void Model3DS::YawIncrease() {
	double inc = GV3::get<double>("ModelsGame.RotateIncrement", "1.0", SILENT
			| HIDDEN) * M_PI / 180;
	Vector<3> rot = makeVector(0.0, 0.0, inc);
	mse3MfromW.get_rotation() *= SO3<>::exp(rot);
}

/**
 * Decrease the yaw angle
 */
void Model3DS::YawDecrease() {
	double inc = GV3::get<double>("ModelsGame.RotateIncrement", "1.0", SILENT
			| HIDDEN) * M_PI / 180;
	Vector<3> rot = makeVector(0.0, 0.0, -inc);
	mse3MfromW.get_rotation() *= SO3<>::exp(rot);
}

/**
 * Move the model to the specified location
 * @param v3Position the new location for the model.
 */
void Model3DS::MoveTo(Vector<3> v3Position) {
	mse3MfromW.get_translation() = v3Position;
}

/**
 * Draw the model axes
 */
void Model3DS::_DrawAxes() {
	float w = 0;
	glGetFloatv(GL_LINE_WIDTH, &w);

	glLineWidth(5);
	const double len = 0.5;

	glBegin(GL_LINES);

	glColor4f(1, 0, 0, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(len, 0, 0);

	glColor4f(0, 1, 0, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, len, 0);

	glColor4f(0, 0, 1, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, len);

	glEnd();

	glLineWidth(w);
}

/**
 * Find the bounding box
 * @param pModel the lib3ds model file
 */
void Model3DS::_FindBoundingBox(Lib3dsFile * pModel) {
	assert( pModel );

	float fMinPos[3] = { 0, 0, 0 };
	float fMaxPos[3] = { 0, 0, 0 };

	lib3ds_file_bounding_box_of_objects(pModel, 1, 0, 0, fMinPos, fMaxPos);

	mv3DimensionMin[0] = fMinPos[0];
	mv3DimensionMin[1] = fMinPos[1];
	mv3DimensionMin[2] = fMinPos[2];

	mv3DimensionMax[0] = fMaxPos[0];
	mv3DimensionMax[1] = fMaxPos[1];
	mv3DimensionMax[2] = fMaxPos[2];

	mv3CentreOfMass[0] = mv3DimensionMin[0] + (abs(mv3DimensionMax[0]) + abs(
			mv3DimensionMin[0])) / 2;
	mv3CentreOfMass[1] = mv3DimensionMin[1] + (abs(mv3DimensionMax[1]) + abs(
			mv3DimensionMin[1])) / 2;
	mv3CentreOfMass[2] = mv3DimensionMin[2] + (abs(mv3DimensionMax[2]) + abs(
			mv3DimensionMin[2])) / 2;

	mv3Offset = makeVector(-mv3CentreOfMass[0], -mv3CentreOfMass[1],
			-mv3DimensionMin[2]);

	mdDiameter = max(max(mv3DimensionMax[0], mv3DimensionMax[1]), max(abs(
			mv3DimensionMin[0]), abs(mv3DimensionMin[1])));

}

/**
 * Draw the bounding box
 */
void Model3DS::_DrawBoundingBox() {
	Vector<3> a, b, c, d, e, f, g, h;
	//max face
	a = mv3DimensionMax;
	b = mv3DimensionMax;
	b[2] = mv3DimensionMin[2];
	c = mv3DimensionMin;
	c[0] = mv3DimensionMax[0];
	d = mv3DimensionMax;
	d[1] = mv3DimensionMin[1];
	//min face
	e = mv3DimensionMin;
	e[2] = mv3DimensionMax[2];
	f = mv3DimensionMin;
	g = mv3DimensionMin;
	g[1] = mv3DimensionMax[1];
	h = mv3DimensionMax;
	h[0] = mv3DimensionMin[0];

	glColor4f(1, 0, 0, 1);
	glLineWidth(5);
	glBegin(GL_LINE_LOOP);
	glVertex(a); //max face
	glVertex(b); //
	glVertex(c); //
	glVertex(d); //
	glEnd();
	glBegin(GL_LINE_LOOP);
	glVertex(e); //min face
	glVertex(f); //
	glVertex(g); //
	glVertex(h); //
	glEnd();
	glBegin(GL_LINE_LOOP);
	glVertex(a); //max side face
	glVertex(h); //
	glVertex(g); //
	glVertex(b); //
	glEnd();
	glBegin(GL_LINE_LOOP);
	glVertex(e); //min side face
	glVertex(d); //
	glVertex(c); //
	glVertex(f); //
	glEnd();
}

/**
 * Save a model's state to disk
 * @param modelsNode the tinyXML element to append to
 */
void Model3DS::Save(TiXmlElement * modelsNode) {
	if (!modelsNode) {
		return;
	}
//
//	TiXmlElement * modelNode = new TiXmlElement("Model");
//	modelsNode->LinkEndChild(modelNode);
//
//	modelNode->SetAttribute("name", msName);
//	modelNode->SetAttribute("dir", msModelDir);
//	modelNode->SetAttribute("file", msModelFile);
//	modelNode->SetDoubleAttribute("scale", mdScale);
//	modelNode->SetDoubleAttribute("scaleMult", mdScaleMult);
//
//	ostringstream os;
//	string s;
//
//	os << mse3MfromW.ln();
//	s = os.str();
//	PruneWhiteSpace(s);
//	modelNode->SetAttribute("pose", s);
//
//	os.str("");
//	os << mse3ModelOffset.ln();
//	s = os.str();
//	PruneWhiteSpace(s);
//	modelNode->SetAttribute("offset", s);

}

/**
 * Load a model's state from disk
 * @param modelsNode the TinyXML element to read from
 * @return success
 */
bool Model3DS::Load(TiXmlElement * modelsNode) {
	if (!modelsNode) {
		return false;
	}
//
//	//read the name
//	msName = modelsNode->Attribute("name");
//
//	//read the file path
//	msModelDir = modelsNode->Attribute("dir");
//	if (msModelDir.empty()) {
//		return false;
//	}
//
//	// read the file name
//	msModelFile = modelsNode->Attribute("file");
//	if (msModelFile.empty()) {
//		return false;
//	}
//
//	//read the scale and its mulitplier
//	modelsNode->QueryDoubleAttribute("scale", &mdScale);
//	modelsNode->QueryDoubleAttribute("scaleMult", &mdScaleMult);
//
//	string tmp;
//
//	//read the pose
//	Vector<6> v6Pose;
//	tmp = modelsNode->Attribute("pose");
//	sscanf(tmp.c_str(), "%lf %lf %lf %lf %lf %lf", &v6Pose[0], &v6Pose[1],
//			&v6Pose[2], &v6Pose[3], &v6Pose[4], &v6Pose[5]);
//	mse3MfromW = SE3<>::exp(v6Pose);
//
//	//read the offset
//	Vector<6> v6Offset;
//	tmp = modelsNode->Attribute("offset");
//	sscanf(tmp.c_str(), "%lf %lf %lf %lf %lf %lf", &v6Offset[0], &v6Offset[1],
//			&v6Offset[2], &v6Offset[3], &v6Offset[4], &v6Offset[5]);
//	mse3ModelOffset = SE3<>::exp(v6Offset);
//
//	mbDelayedLoad = true;
	return true;
}

/**
 * Perform a delayed load
 */
void Model3DS::_DelayedLoad() {
	if (mbDelayedLoad) {
		_Load();
		mbDelayedLoad = false;
	}
}

/**
 * Load the model textures
 * @param pModel the lib3ds model file
 */
deque<Texture *> Model3DS::_LoadTextures(Lib3dsFile * pModel, string dirpath) {
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
