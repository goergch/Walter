#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif



#ifndef COORDINATE_H
#define COORDINATE_H


class GLCoordinate
{
    public:
	GLCoordinate() {
            xyz[X] = 0.0;
            xyz[Y] = 0.0;
            xyz[Z] = 0.0;
        }
        virtual ~GLCoordinate() {};
        GLCoordinate(GLfloat *_xyz) {
            xyz[X] = _xyz[X];
            xyz[Y] = _xyz[Y];
            xyz[Z] = _xyz[Z];
        }
        void setCoordinate(GLfloat *_xyz) {
            xyz[X] = _xyz[X];
            xyz[Y] = _xyz[Y];
            xyz[Z] = _xyz[Z];
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

        enum
        {
            X,
            Y,
            Z
        };
};

#endif // COORDINATE_H
