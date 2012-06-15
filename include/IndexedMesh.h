#ifndef _INDEXEDMESH_H_
#define _INDEXEDMESH_H_

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include <vector>

#include "BasicFace.h"
#include <string>

//-------------------------------------------------------------------
// Class definition
//-------------------------------------------------------------------
namespace TextureTransfer
{

	class IndexedMesh
	{
	 public:
	  IndexedMesh();
	  ~IndexedMesh();

	  Vertex* add_vertex();
	  Vertex* add_vertex(const Vector3& p, const Vector2& t);

	  void begin_facet();
	  void end_facet();
	  void add_vertex_to_facet(unsigned int i);
	  void clear();

	  void save(const std::string & file_name);

	  void FindTextureMax();

	 public:
	  unsigned int nIndex;
	  std::vector<Vertex> mVertices ;
	  std::vector<Facet>  mFaces ;

	  bool mInFacet ;	//faceを追加するときにmutual exclusion的な使い方をする

	  Vector2 mTexMax, mTexMin;

	  std::vector<Vector2> mTextureCoords;
	  std::vector<double> mTexParts;			//for reserving harmonic field value. This vector size is the same as mVertices vector size
	  std::vector<int> mTextureFaces;

	  std::vector<int> mTextureNumber; 		//to indicate corresponding texture image number. This vector size is the same as mVertices vector size

	  int ind_max;

	};

}
#endif
