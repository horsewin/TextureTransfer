/*
 * IndexedMesh.cc
 *
 *  Created on: 2012/05/22
 *      Author: umakatsu
 */

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include "IndexedMesh.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cassert>

//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
//#define OUTPUT_TEXCOORDS 0

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
namespace TextureTransfer
{

	std::ostream& operator<<(std::ostream& out, const Vector2& v) {
		return out << v.x << " " << v.y ;
	}

	std::ostream& operator<<(std::ostream& out, const Vector3& v) {
		return out << v.x << " " << v.y << " " << v.z ;
	}

	std::istream& operator>>(std::istream& in, Vector2& v) {
		return in >> v.x >> v.y ;
	}

	std::istream& operator>>(std::istream& in, Vector3& v) {
		return in >> v.x >> v.y >> v.z ;
	}

	//---------------------------------------------------------------------------
	// Code
	//---------------------------------------------------------------------------
	IndexedMesh::IndexedMesh()
	: mInFacet(false)
	{
		mVertices.clear();
		mFaces.clear();
		mTextureCoords.clear();
		mTexParts.clear();
		mTextureFaces.clear();
	}

	IndexedMesh::~IndexedMesh()
	{

	}

	void IndexedMesh::FindTextureMax()
	{
	  uint size = mTextureFaces.size();

	  mTexMax.x = -9999;
	  mTexMax.y = -9999;
	  mTexMin.x = 9999;
	  mTexMin.y = 9999;

	  for(uint i=0; i<size; i++){
		Vector2 tmp1 = mTextureCoords[ mTextureFaces.at(i) - 1];
		if( tmp1.x < mTexMin.x) mTexMin.x = tmp1.x;
		if( tmp1.y < mTexMin.y) mTexMin.y = tmp1.y;
		if( tmp1.x > mTexMax.x) mTexMax.x = tmp1.x;
		if( tmp1.y > mTexMax.y) mTexMax.y = tmp1.y;
	  }

	#ifdef OUTPUT_TEXCOORDS
	  std::cout << mTexMin.x << " " << mTexMin.y << std::endl;
	  std::cout << mTexMax.x << " " << mTexMax.y << std::endl;
	#endif
	}

	Vertex* IndexedMesh::add_vertex() {
	  mVertices.push_back(Vertex()) ;
	  mVertices.rbegin()->id = mVertices.size() - 1 ;
	  return &*(mVertices.rbegin()) ;
	}

	Vertex* IndexedMesh::add_vertex(const Vector3& p, const Vector2& t) {
	  mVertices.push_back(Vertex(p,t)) ;
	  mVertices.rbegin()->id = mVertices.size() - 1 ;
	  return &*(mVertices.rbegin()) ;
	}

	void IndexedMesh::begin_facet() {
	  assert(!mInFacet) ;
	  mFaces.push_back(Facet()) ;
	  mInFacet = true ;
	}

	void IndexedMesh::end_facet() {
	  assert(mInFacet) ;
	  mInFacet = false ;
	}

	void IndexedMesh::add_vertex_to_facet(unsigned int i) {
	  assert(mInFacet) ;
	  assert(i < mVertices.size()) ;
	  mFaces.rbegin()->push_back(i) ;
	}

	void IndexedMesh::clear() {
	  mVertices.clear() ;
	  mFaces.clear() ;
	}

	void IndexedMesh::save(const std::string& file_name)
	{
	  std::ofstream out(file_name.c_str()) ;

	  mTextureCoords.clear();
	  mTextureFaces.clear();

	  for(int i=0; i<mVertices.size(); i++) {
		out << "v " << mVertices[i].point << std::endl ;
	  }
	  for(int i=0; i<mVertices.size(); i++) {
		out << "vt " << mVertices[i].tex_coord << std::endl ;
		mTextureCoords.push_back(mVertices[i].tex_coord);
	  }
	  for(int i=0; i<mFaces.size(); i++) {
		out << "f " ;
		const Facet& F = mFaces[i] ;
		for(int j=0; j<F.size(); j++) {
		  out << (F[j] + 1) << "/" << (F[j] + 1) << " " ;
		  mTextureFaces.push_back(F[j]+1);
		}
		out << std::endl ;
	  }
	  for(int i=0; i<mVertices.size(); i++) {
		if(mVertices[i].locked) {
		  out << "# anchor " << i+1 << std::endl ;
		}
	  }
	}

} //<--------- end of namespace TextureTransfer --------------
