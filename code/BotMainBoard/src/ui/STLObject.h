#include <iostream>
#include <string>
#include <cstring>
#include <fstream>
#include <vector>
#include <math.h>
#include "GLCoordinate.h"

#include <GL/glut.h>

using namespace std;

#ifndef OBJECT_H
#define OBJECT_H


class STLObject
{
    public:
        STLObject();
        virtual ~STLObject();
        void loadFile(char *filename);
        void parse();
        vector<GLCoordinate> getVertex();
        vector<GLCoordinate> getNormals();

        GLCoordinate computeFaceNormal(GLfloat*  vec1, GLfloat* vec2 ,GLfloat* vec3);


    protected:
    private:
    vector<string> rows;
    vector<GLCoordinate> vertex;
    vector<GLCoordinate> normal;

    enum { X, Y, Z};
};

#endif // OBJECT_H
