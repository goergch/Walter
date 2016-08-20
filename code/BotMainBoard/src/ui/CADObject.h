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


class GLCoordinate
{
    public:
	GLCoordinate() {
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }
        virtual ~GLCoordinate() {};
        GLCoordinate(GLfloat px, GLfloat py, GLfloat pz) {
            x = px;
            y = py;
            z = pz;
        }

        void operator=(const GLCoordinate&  coord) {
            x = coord.x;
            y = coord.y;
            z = coord.z;
        }

       GLfloat x;
       GLfloat y;
       GLfloat z;

};

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

	GLCoordinate vertex1;
	GLCoordinate vertex2;
	GLCoordinate vertex3;
	GLCoordinate normal;
};

class CADObject
{
    public:
        CADObject() {};
        bool loadFile(string filename);
        void display(GLfloat* color,GLfloat* accentColor);


    private:
        bool parseAsciiFormat();
        bool parseBinaryFormat();


        GLCoordinate computeFaceNormal(const GLCoordinate&  vec1, const GLCoordinate& vec2 ,const GLCoordinate& vec3);

        vector<Triangle> triangles;
        string filename;
};

#endif // OBJECT_H
