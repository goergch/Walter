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
            xyz[0] = 0.0;
            xyz[1] = 0.0;
            xyz[2] = 0.0;
        }
        virtual ~GLCoordinate() {};
        GLCoordinate(GLfloat *_xyz) {
            xyz[0] = _xyz[0];
            xyz[1] = _xyz[1];
            xyz[2] = _xyz[2];
        }
        void setCoordinate(GLfloat *_xyz) {
            xyz[0] = _xyz[0];
            xyz[1] = _xyz[1];
            xyz[2] = _xyz[2];
        }
        GLfloat* getCoordinate() {
            return xyz;
        }
        GLfloat getCoordinate(int _index) {
            if(_index >=0 && _index<3)
                return xyz[_index];

            return -1;
        }
    protected:
    private:
        GLfloat xyz[3];
};

class CADObject
{
    public:
        CADObject() {};
        void loadFile(string filename);
        void display(GLfloat* color);


    private:
        void parse();

        vector<GLCoordinate> getVertex();
        vector<GLCoordinate> getNormals();

        GLCoordinate computeFaceNormal(GLfloat*  vec1, GLfloat* vec2 ,GLfloat* vec3);

        vector<string> rows;
        vector<GLCoordinate> vertex;
        vector<GLCoordinate> normal;
};

#endif // OBJECT_H
