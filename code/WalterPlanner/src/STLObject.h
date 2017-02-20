/*
 * STLObject.h
 *
 * Represents a CAD Object in STL format, it can be displayed via opengl.
 *
 *  Author: JochenAlt
 */

#include <iostream>
#include <string>
#include <cstring>
#include <fstream>
#include <vector>
#include <math.h>

#include <GL/glut.h>

using namespace std;

#ifndef OBJECT_H
#define OBJECT_H


// Plain 3D coordinate
class Coordinate
{
    public:
	Coordinate() {
            x = 0.0;
            y = 0.0;
            z = 0.0;
	}
	virtual ~Coordinate() {};
	Coordinate(GLfloat px, GLfloat py, GLfloat pz) {
		x = px;
		y = py;
		z = pz;
	}

	void operator=(const Coordinate&  coord) {
		x = coord.x;
		y = coord.y;
		z = coord.z;
	}

	GLfloat x;
	GLfloat y;
	GLfloat z;
};

// 3D-triangle consisting of three vertexes and a normal
class Triangle {
public:
	Triangle() {};
	Triangle(const Triangle& par) {
		vertex1 = par.vertex1;
		vertex2 = par.vertex2;
		vertex3 = par.vertex3;
		normal = par.normal;
	};
    void operator=(const Triangle&  par) {
    	vertex1 = par.vertex1;
    	vertex2 = par.vertex2;
    	vertex3 = par.vertex3;
    	normal = par.normal;
    };

	virtual ~Triangle() {};

	Coordinate vertex1;
	Coordinate vertex2;
	Coordinate vertex3;
	Coordinate normal;
};

// STL object to be displayed
class STLObject
{
    public:
        STLObject() {};

        // load STL file in ascii or binary format
        bool loadFile(string filename);

        // display loaded object via opengl
        void display(const GLfloat* color,const GLfloat* accentColor);
    private:
        bool parseSTLAsciiFormat();
        bool parseSTLBinaryFormat();

        Coordinate computeFaceNormal(const Coordinate&  vec1, const Coordinate& vec2 ,const Coordinate& vec3);

        vector<Triangle> triangles;
        string filename;
};

#endif // OBJECT_H
