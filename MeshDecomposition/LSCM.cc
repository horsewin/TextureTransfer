/*
 *  Copyright (c) 2004-2010, Bruno Levy
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ALICE Project-Team nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Bruno Levy
 *
 *     levy@loria.fr
 *
 *     ALICE Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 */

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include <NL/nl.h>

#include <vector>
#include <set>
#include <iostream>

#include "LSCM.h"

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
char* type_solver;
using namespace std;

/* dot product */
double operator*(const Vector3& v1, const Vector3& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z ;
}

/* cross product */
Vector3 operator^(const Vector3& v1, const Vector3& v2) {
    return Vector3(
        v1.y*v2.z - v2.y*v1.z,
        v1.z*v2.x - v2.z*v1.x,
        v1.x*v2.y - v2.x*v1.y
    ) ;
}

Vector3 operator+(const Vector3& v1, const Vector3& v2) {
    return Vector3(
        v1.x + v2.x,
        v1.y + v2.y,
        v1.z + v2.z
    ) ;
}

Vector3 operator-(const Vector3& v1, const Vector3& v2) {
    return Vector3(
        v1.x - v2.x,
        v1.y - v2.y,
        v1.z - v2.z
    ) ;
}

Vector2 operator+(const Vector2& v1, const Vector2& v2) {
    return Vector2(
        v1.x + v2.x,
        v1.y + v2.y
    ) ;
}

Vector2 operator-(const Vector2& v1, const Vector2& v2) {
    return Vector2(
        v1.x - v2.x,
        v1.y - v2.y
    ) ;
}


//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

// Outline of the algorithm:

// 1) Find an initial solution by projecting on a plane
// 2) Lock two vertices of the mesh
// 3) Copy the initial u,v coordinates to OpenNL
// 3) Construct the LSCM equation with OpenNL
// 4) Solve the equation with OpenNL
// 5) Copy OpenNL solution to the u,v coordinates

void LSCM::apply() {
  int nb_vertices = mMesh->mVertices.size() ;
  project() ;
  nlNewContext() ;
        
//  std::cout <<  type_solver << std::endl;
  
  if (!strcmp(type_solver,"CG")) {
    nlSolverParameteri(NL_SOLVER, NL_CG) ;
    nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
  }
  else if (!strcmp(type_solver,"BICGSTAB")) {
    nlSolverParameteri(NL_SOLVER, NL_BICGSTAB) ;
    nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
  }
  else if (!strcmp(type_solver,"GMRES")) {
    nlSolverParameteri(NL_SOLVER, NL_GMRES) ;
  }
  else if (!strcmp(type_solver,"SUPERLU")) {
    if(nlInitExtension("SUPERLU")) {
      nlSolverParameteri(NL_SOLVER, NL_PERM_SUPERLU_EXT) ;
    } else {
      std::cerr << "OpenNL has not been compiled with SuperLU support." << std::endl;
      exit(-1);
    }
  }else if(
	   !strcmp(type_solver,"FLOAT_CRS") 
	   || !strcmp(type_solver,"DOUBLE_CRS")
	   || !strcmp(type_solver,"FLOAT_BCRS2")
	   || !strcmp(type_solver,"DOUBLE_BCRS2")
	   || !strcmp(type_solver,"FLOAT_ELL")
	   || !strcmp(type_solver,"DOUBLE_ELL")
	   || !strcmp(type_solver,"FLOAT_HYB")
	   || !strcmp(type_solver,"DOUBLE_HYB")
	   ) {
    if(nlInitExtension("CNC")) {
      if (!strcmp(type_solver,"FLOAT_CRS")) {
	nlSolverParameteri(NL_SOLVER, NL_CNC_FLOAT_CRS) ;
	nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
      }
      else if (!strcmp(type_solver,"DOUBLE_CRS")) {
	nlSolverParameteri(NL_SOLVER, NL_CNC_DOUBLE_CRS) ;
	nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
      }
      else if (!strcmp(type_solver,"FLOAT_BCRS2")) {
	nlSolverParameteri(NL_SOLVER, NL_CNC_FLOAT_BCRS2) ;
	nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
      }
      else if (!strcmp(type_solver,"DOUBLE_BCRS2")) {
	nlSolverParameteri(NL_SOLVER, NL_CNC_DOUBLE_BCRS2) ;
	nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
      }
      else if (!strcmp(type_solver,"FLOAT_ELL")) {
	nlSolverParameteri(NL_SOLVER, NL_CNC_FLOAT_ELL) ;
	nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
      } 
      else if (!strcmp(type_solver,"DOUBLE_ELL")) {
	nlSolverParameteri(NL_SOLVER, NL_CNC_DOUBLE_ELL) ;
                nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
      }
      else if (!strcmp(type_solver,"FLOAT_HYB")) {
	nlSolverParameteri(NL_SOLVER, NL_CNC_FLOAT_HYB) ;
	nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
      }
      else if (!strcmp(type_solver,"DOUBLE_HYB")) {
	nlSolverParameteri(NL_SOLVER, NL_CNC_DOUBLE_HYB) ;
	nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI) ;
      }
      
    } else {
      std::cerr << "OpenNL has not been compiled with CNC support." << std::endl;  
            exit(-1);
    }
  } else {
	  std::cerr << "type_solver must belong to { CG | BICGSTAB | GMRES | "
		    << "SUPERLU | FLOAT_CRS | FLOAT_BCRS2 | DOUBLE_CRS | "
		    << "DOUBLE_BCRS2 | FLOAT_ELL | DOUBLE_ELL | FLOAT_HYB |"
		    << "DOUBLE_HYB } "
		    << std::endl;
	  exit(-1);
  }
  nlSolverParameteri(NL_NB_VARIABLES, 2*nb_vertices) ;
  nlSolverParameteri(NL_LEAST_SQUARES, NL_TRUE) ;
  nlSolverParameteri(NL_MAX_ITERATIONS, 5*nb_vertices) ;
  nlSolverParameterd(NL_THRESHOLD, 1e-10) ;
  nlBegin(NL_SYSTEM) ;
  mesh_to_solver() ;
  nlBegin(NL_MATRIX) ;
  setup_lscm() ;
  nlEnd(NL_MATRIX) ;
  nlEnd(NL_SYSTEM) ;
//  std::cout << "Solving ..." << std::endl ;
  nlSolve() ;
  solver_to_mesh() ;
  double time ;
  NLint iterations;
  nlGetDoublev(NL_ELAPSED_TIME, &time) ;
  nlGetIntergerv(NL_USED_ITERATIONS, &iterations);
  std::cout << "Solver time: " << time <<" , ";
  std::cout << "Used iterations: " << iterations << std::endl ;
  nlDeleteContext(nlGetCurrent()) ;
}

void LSCM::setup_lscm() {
  for(unsigned int i=0; i<mMesh->mFaces.size(); i++) {
    const Facet& F = mMesh->mFaces[i] ;
    setup_lscm(F) ;
  }
}

// Note: no-need to triangulate the facet,
// we can do that "virtually", by creating triangles
// radiating around vertex 0 of the facet.
// (however, this may be invalid for concave facets)
void LSCM::setup_lscm(const Facet& F) {
  for(unsigned int i=1; i < F.size() - 1; i++) {
    setup_conformal_map_relations(mMesh->mVertices[F[0]], mMesh->mVertices[F[i]], mMesh->mVertices[F[i+1]]) ;
  }
}

// Computes the coordinates of the vertices of a triangle
// in a local 2D orthonormal basis of the triangle's plane.
void LSCM::project_triangle(
			    const Vector3& p0, 
			    const Vector3& p1, 
			    const Vector3& p2,
			    Vector2& z0,
			    Vector2& z1,
			    Vector2& z2
			    ) {
  Vector3 X = p1 - p0 ;
  X.normalize() ;
  Vector3 Z = X ^ (p2 - p0) ;
  Z.normalize() ;
  Vector3 Y = Z ^ X ;
  const Vector3& O = p0 ;
  
  double x0 = 0 ;
  double y0 = 0 ;
  double x1 = (p1 - O).length() ;
  double y1 = 0 ;
  double x2 = (p2 - O) * X ;
  double y2 = (p2 - O) * Y ;        
  
  z0 = Vector2(x0,y0) ;
  z1 = Vector2(x1,y1) ;
  z2 = Vector2(x2,y2) ;        
}

// LSCM equation, geometric form :
// (Z1 - Z0)(U2 - U0) = (Z2 - Z0)(U1 - U0)
// Where Uk = uk + i.vk is the complex number 
//                       corresponding to (u,v) coords
//       Zk = xk + i.yk is the complex number 
//                       corresponding to local (x,y) coords
// cool: no divide with this expression,
//  makes it more numerically stable in
//  the presence of degenerate triangles.

void LSCM::setup_conformal_map_relations(
					 const Vertex& v0, const Vertex& v1, const Vertex& v2
					 ) {
  
  int id0 = v0.id ;
  int id1 = v1.id ;
  int id2 = v2.id ;
  
  const Vector3& p0 = v0.point ; 
  const Vector3& p1 = v1.point ;
  const Vector3& p2 = v2.point ;
  
  Vector2 z0,z1,z2 ;
  project_triangle(p0,p1,p2,z0,z1,z2) ;
  Vector2 z01 = z1 - z0 ;
  Vector2 z02 = z2 - z0 ;
  double a = z01.x ;
  double b = z01.y ;
  double c = z02.x ;
  double d = z02.y ;
  assert(b == 0.0) ;
  
  // Note  : 2*id + 0 --> u
  //         2*id + 1 --> v
  int u0_id = 2*id0     ;
  int v0_id = 2*id0 + 1 ;
  int u1_id = 2*id1     ;
  int v1_id = 2*id1 + 1 ;
  int u2_id = 2*id2     ;
  int v2_id = 2*id2 + 1 ;
  
  // Note : b = 0
  
  // Real part
  nlBegin(NL_ROW) ;
  nlCoefficient(u0_id, -a+c)  ;
  nlCoefficient(v0_id,  b-d)  ;
  nlCoefficient(u1_id,   -c)  ;
  nlCoefficient(v1_id,    d)  ;
  nlCoefficient(u2_id,    a) ;
  nlEnd(NL_ROW) ;
  
  // Imaginary part
  nlBegin(NL_ROW) ;
  nlCoefficient(u0_id, -b+d) ;
  nlCoefficient(v0_id, -a+c) ;
  nlCoefficient(u1_id,   -d) ;
  nlCoefficient(v1_id,   -c) ;
  nlCoefficient(v2_id,    a) ;
  nlEnd(NL_ROW) ;
}

/**
 * copies u,v coordinates from OpenNL solver to the mesh.
 */
void LSCM::solver_to_mesh() {
  for(unsigned int i=0; i<mMesh->mVertices.size(); i++) {
    Vertex& it = mMesh->mVertices[i] ;
    double u = nlGetVariable(2 * it.id    ) ;
    double v = nlGetVariable(2 * it.id + 1) ;
    it.tex_coord = Vector2(u,v) ;
  }
}

/**
 * copies u,v coordinates from the mesh to OpenNL solver.
 */
void LSCM::mesh_to_solver() {
  for(unsigned int i=0; i<mMesh->mVertices.size(); i++) {
    Vertex& it = mMesh->mVertices[i] ;
    double u = it.tex_coord.x ;
    double v = it.tex_coord.y ;
    nlSetVariable(2 * it.id    , u) ;
    nlSetVariable(2 * it.id + 1, v) ;
    if(it.locked) {
      nlLockVariable(2 * it.id    ) ;
      nlLockVariable(2 * it.id + 1) ;
    } 
  }
}


// Chooses an initial solution, and locks two vertices
void LSCM::project() {
  // Get bbox
  unsigned int i ;
  
  double xmin =  1e30 ;
  double ymin =  1e30 ;
  double zmin =  1e30 ;
  double xmax = -1e30 ;
  double ymax = -1e30 ;
  double zmax = -1e30 ;
  
  for(i=0; i<mMesh->mVertices.size(); i++) {
    const Vertex& v = mMesh->mVertices[i] ;
    xmin = nl_min(v.point.x, xmin) ;
    ymin = nl_min(v.point.y, xmin) ;
    zmin = nl_min(v.point.z, xmin) ;
    
    xmax = nl_max(v.point.x, xmin) ;
    ymax = nl_max(v.point.y, xmin) ;
    zmax = nl_max(v.point.z, xmin) ;
  }
  
  double dx = xmax - xmin ;
  double dy = ymax - ymin ;
  double dz = zmax - zmin ;
  
  Vector3 V1,V2 ;
  
  // Find shortest bbox axis
  if(dx < dy && dx < dz) {
    if(dy > dz) {
      V1 = Vector3(0,1,0) ;
      V2 = Vector3(0,0,1) ;
    } else {
      V2 = Vector3(0,1,0) ;
      V1 = Vector3(0,0,1) ;
    }
  } else if(dy < dx && dy < dz) {
    if(dx > dz) {
      V1 = Vector3(1,0,0) ;
      V2 = Vector3(0,0,1) ;
    } else {
      V2 = Vector3(1,0,0) ;
      V1 = Vector3(0,0,1) ;
    }
  } else if(dz < dx && dz < dy) {
    if(dx > dy) {
      V1 = Vector3(1,0,0) ;
      V2 = Vector3(0,1,0) ;
    } else {
      V2 = Vector3(1,0,0) ;
      V1 = Vector3(0,1,0) ;
    }
  }
  
  // Project onto shortest bbox axis,
  // and lock extrema vertices
  
  Vertex* vxmin = NULL ;
  double  umin = 1e30 ;
  Vertex* vxmax = NULL ;
  double  umax = -1e30 ;
  
  for(i=0; i<mMesh->mVertices.size(); i++) {
    Vertex& V = mMesh->mVertices[i] ;
    double u = V.point * V1 ;
    double v = V.point * V2 ;
    V.tex_coord = Vector2(u,v) ;
    if(u < umin) {
      vxmin = &V ;
      umin = u ;
    } 
    if(u > umax) {
      vxmax = &V ;
      umax = u ;
    } 
  }
  
  vxmin->locked = true ;
  vxmax->locked = true ;
}

void LSCM::run(const char * t_solver, const char * filename) {
  int len = strlen(t_solver);
  type_solver = new char[len];
  strcpy(type_solver, t_solver);

//  std::cout << "Running LSCM ..." << std::endl ;
  //  mesh_->load(filename) ;

  apply() ;

//   std::cout << "Saving ..." << std::endl ;
//   mesh.save(argv[3]) ;
}

LSCM::LSCM(void)
{
	mMesh = boost::shared_ptr<IndexedMesh>(new IndexedMesh());
  //mesh_ = new IndexedMesh();
}

LSCM::~LSCM(void)
{
//  delete mesh_;
}
