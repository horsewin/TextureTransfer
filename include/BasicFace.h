#ifndef _BASICFACE_H_
#define _BASICFACE_H_

/* Basic geometric types */

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include <cmath>
#include <vector>

//-------------------------------------------------------------------
// Class definition
//-------------------------------------------------------------------
class Vector2 {
public:
    Vector2(double x_in, double y_in) : x(x_in), y(y_in) { }
    Vector2() : x(0), y(0) { }
    double x ;
    double y ;
} ;

class Vector3 {
public:
    Vector3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) { }
    Vector3() : x(0), y(0), z(0) { }
    double length() const { 
        return sqrt(x*x + y*y + z*z) ;
    }
    void normalize() {
        double l = length() ;
        x /= l ; y /= l ; z /= l ;
    }

    Vector3 operator=(float *t){
        return Vector3(
            t[0], t[1], t[2]
        ) ;
    }

    double x ;
    double y ;
    double z ;
} ;

/***********************************************************************************/
/* Mesh class */

// Note1: this is a minimum mesh class, it does not have facet adjacency information
// (we do not need it for LSCM). This is just like an Inventor indexed face set.
//
// Note2: load() and save() use Alias|Wavefront .obj file format

class Vertex {
public:
    Vertex() : locked(false), id(-1) { }
    Vertex(
        const Vector3& p, const Vector2& t
    ) : point(p), tex_coord(t), locked(false), id(-1) { }
    Vector3 point ;
    Vector3 normal ;
    Vector2 tex_coord ;
    bool    locked ;
    int     id ;
};

class Facet : public std::vector<int> {
	public:
};

#endif
