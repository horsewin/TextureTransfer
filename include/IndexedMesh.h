#ifndef _INDEXEDMESH_H_
#define _INDEXEDMESH_H_

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include <vector>

#include "BasicFace.h"
#include "ModelCorrespondence.h"

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
	  std::vector<int> VertexSynthesis( void );

	  void FindTextureMax();

	 public:
	  std::vector<Vertex> mVertices;				//vertex information for creating mesh structure
	  std::vector<Facet>  mFaces;					//面情報の頂点座標の格納順序
	  std::vector<Facet>  mTextureFaces;			//面情報のテクスチャ座標の格納順序
	  std::vector<ModelCorresPondense> mTexnumVernum; //LSCM頂点群と通常頂点群の対応関係
	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 //LSCM群のテクスチャ座標は展開図を表示するためにも使用する
	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 //対して、元々テクスチャマッピングされているモデルの場合、マッピング用のテクスチャ座標も必要。そのための対応関係を表すデータ構造

	  bool mInFacet ;	//faceを追加するときにmutual exclusion的な使い方をする

	  Vector2 mTexMax, mTexMin;
	  unsigned int mNumIndex;
	};

}
#endif
