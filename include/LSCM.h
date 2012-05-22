#ifndef _LSCM_H_
#define _LSCM_H_

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include <cstdlib>
#include <cstring>

#include "BasicFace.h"
#include "IndexedMesh.h"

#include <boost/shared_ptr.hpp>

/*************************************************************************************/
/* Basic functions */

template <class T> inline T nl_min(T x, T y) { return x < y ? x : y ; }
template <class T> inline T nl_max(T x, T y) { return x > y ? x : y ; }

//-------------------------------------------------------------------
// Class definition
//-------------------------------------------------------------------
class LSCM 
{
 public: 
  LSCM();
  ~LSCM(void);
  void run(const char * t_solver, const char * filename);
  void apply();

 protected:
  void setup_lscm();
  void setup_lscm(const Facet& F);
  void project_triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2,
			       Vector2& z0, Vector2& z1, Vector2& z2);
  void setup_conformal_map_relations(const Vertex& v0, const Vertex& v1, const Vertex& v2);
  void solver_to_mesh();
  void mesh_to_solver();
  void project();

 public:
  boost::shared_ptr<IndexedMesh> mesh_;
  //IndexedMesh * mesh_;
};

#endif
