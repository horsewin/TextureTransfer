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
  std::vector<Vertex> mVertices ;
  std::vector<Facet>  mFaces ;
  bool mInFacet ;
  Vector2 mTexMax, mTexMin;

  std::vector<Vector2> mTextureCoords;
  std::vector<double> mTexParts;
  std::vector<int> mTextureFaces;
  
};

#endif
