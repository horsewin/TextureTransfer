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
#define REP(i,n) for(int i=0;i<(int)n;++i)

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
		mTextureFaces.clear();
	}

	IndexedMesh::~IndexedMesh()
	{

	}

	void IndexedMesh::FindTextureMax()
	{
	  uint size = mVertices.size();

	  mTexMax.x = -9999;
	  mTexMax.y = -9999;
	  mTexMin.x = 9999;
	  mTexMin.y = 9999;

	  for(uint i=0; i<size; i++)
	  {
		Vector2 tmp1 = mVertices[i].tex_coord;
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

	Vertex* IndexedMesh::AddVertex() {
	  mVertices.push_back(Vertex()) ;
	  mVertices.rbegin()->id = mVertices.size() - 1 ;
	  return &*(mVertices.rbegin()) ;
	}

	Vertex* IndexedMesh::AddVertex(const Vector3& p, const Vector2& t) {
	  mVertices.push_back(Vertex(p,t)) ;
	  mVertices.rbegin()->id = mVertices.size() - 1 ;
	  return &*(mVertices.rbegin()) ;
	}

	void IndexedMesh::BeginFacet() {
	  assert(!mInFacet) ;
	  mFaces.push_back(Facet()) ;
	  mInFacet = true ;
	}

	void IndexedMesh::EndFacet() {
	  assert(mInFacet) ;
	  mInFacet = false ;
	}

	void IndexedMesh::AddVertex2Facet(unsigned int i) {
	  assert(mInFacet) ;
	  assert(i < mVertices.size()) ;
	  mFaces.rbegin()->push_back(i) ;
	}

	void IndexedMesh::Clear()
	{
		mNumIndex = 0;

		mVertices.clear() ;
		mFaces.clear() ;
		mTextureFaces.clear();
	}

	void IndexedMesh::Save(const std::string& file_name)
	{
	  unsigned int i,j ;
	  std::ofstream out(file_name.c_str()) ;

	  mTextureFaces.clear();

	  for(i=0; i<mVertices.size(); i++) {
		out << "v " << mVertices[i].point << std::endl ;
	  }
	  for(i=0; i<mVertices.size(); i++) {
		out << "vt " << mVertices[i].tex_coord << std::endl ;
	  }
	  for(i=0; i<mFaces.size(); i++) {
		out << "f " ;
		const Facet& F = mFaces[i] ;
		for(j=0; j<F.size(); j++) {
		  out << (F[j] + 1) << "/" << (F[j] + 1) << " " ;
//		  mTextureFaces.push_back(F[j]+1);
		}
		out << std::endl ;
	  }
	  for(i=0; i<mVertices.size(); i++) {
		if(mVertices[i].locked) {
		  out << "# anchor " << i+1 << std::endl ;
		}
	  }
	}

	//to correct calculation of LSCM in the case of reconstructed objects
	void IndexedMesh::VertexSynthesis( void )
	{
		uint current_id = 0;

		std::vector<Vertex> newVertices;

		REP(id_seek, mVertices.size())
		{
			bool duplicate = false;
			REP(id_pair, id_seek)
			{
				if(mVertices[id_pair].point == mVertices[id_seek].point)
				{
					//found out duplicating of vertices
					mVertices[id_seek].id = mVertices[id_pair].id;
					duplicate = true;
					break;
				}

			}
			if( !duplicate )
			{
				mVertices[id_seek].id = current_id;
				current_id++;
				newVertices.push_back(mVertices[id_seek]);
			}
		}

		//revise face info
		REP(id_face, mFaces.size())
		{
			Facet& F = mFaces[id_face];
			//Face内のvertex id は0から始まる
			REP(id_innerF, F.size())
			{
				F[id_innerF] = mVertices[F[id_innerF]].id;
			}
		}

		//resize array of vertex
		mVertices.swap(newVertices);
	}

} //<--------- end of namespace TextureTransfer --------------
