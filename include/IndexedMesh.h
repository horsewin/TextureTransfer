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

	  Vertex* AddVertex();
	  Vertex* AddVertex(const Vector3& p, const Vector2& t);

	  void BeginFacet();
	  void EndFacet();
	  void AddVertex2Facet(unsigned int i);
	  void Clear();
	  void Save(const std::string & file_name);
	  void VertexSynthesis( void );

	  void FindTextureMax();

	 public:
	  std::vector<Vertex> mVertices;				//vertex information for creating mesh structure
	  std::vector<Facet>  mFaces;					//面情報の頂点座標の格納順序
	  std::vector<Facet>  mTextureFaces;			//面情報のテクスチャ座標の格納順序

	  bool mInFacet ;	//faceを追加するときにmutual exclusion的な使い方をする

	  Vector2 mTexMax, mTexMin;
	  unsigned int mNumIndex;
	};

}
#endif
