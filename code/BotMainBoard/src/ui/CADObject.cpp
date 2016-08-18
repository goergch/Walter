#include "stdio.h"

#include "CADObject.h"
#include "util.h"

using namespace std;


void CADObject::loadFile(string filename)
{
    ifstream file;
    file.open(filename.c_str());
    if(!file.is_open())
    {
    	LOG(ERROR) << "file " << filename << " not found" << filename;
    }
    else
    {
        string line;
        while(!file.eof())
        {
            getline(file, line);
            rows.push_back(line);
        }
    }

    file.close();
    parse();
}



void CADObject::parse()
{
    string line;
    for(unsigned int i=0; i<rows.size(); i++)
    {
        line = rows[i];
        char *ch;
        ch = new char[line.size()+1];
        strcpy(ch,line.c_str());

        GLfloat xyz[3];

        int idx = 0;
        while ((ch[idx] == ' ') || (ch[idx] == '\t')) idx++;
        if ((ch[idx]=='v' && ch[idx+1]=='e' && ch[idx+2]=='r' && ch[idx+3]=='t' && ch[idx+4]=='e' && ch[idx+5]=='x'))
        {
            sscanf(&(ch[idx]),"vertex %f %f %f", &xyz[0], &xyz[1], &xyz[2]);
            GLCoordinate coord(&xyz[0]);
            vertex.push_back(coord);
        }

        if ( (ch[idx]=='f' && ch[idx+1]=='a' && ch[idx+2]=='c' && ch[idx+3]=='e' && ch[idx+4]=='t'))
        {
            sscanf(&(ch[idx]), "facet normal %f %f %f", &xyz[0], &xyz[1], &xyz[2]);

            GLCoordinate coord(&xyz[0]);
            normal.push_back(coord);
        }
    }
}



vector<GLCoordinate> CADObject::getVertex()
{
    return vertex;
}

vector<GLCoordinate> CADObject::getNormals()
{
    return normal;
}


GLCoordinate CADObject::computeFaceNormal(GLfloat*  vec1, GLfloat* vec2 ,GLfloat* vec3)
{
	GLCoordinate result;

    GLfloat v1x, v1y, v1z, v2x, v2y, v2z;
    GLfloat nx, ny, nz;
    GLfloat vLen;

    // Calculate vectors
    v1x = vec1[0] - vec2[0];
    v1y = vec1[1] - vec2[1];
    v1z = vec1[2] - vec2[2];

    v2x = vec2[0] - vec3[0];
    v2y = vec2[1] - vec3[1];
    v2z = vec2[2] - vec3[2];

    // cross product
    nx = (v1y * v2z) - (v1z * v2y);
    ny = (v1z * v2x) - (v1x * v2z);
    nz = (v1x * v2y) - (v1y * v2x);

    // Normalise to get len 1
    vLen = sqrt( (nx * nx) + (ny * ny) + (nz * nz) );
    GLfloat val[3];
    if(vLen!=0)
    {
        val[0] = (nx / vLen);
        val[1] = (ny / vLen);
        val[2] = (nz / vLen);
    }
    else
    {
        val[0] = 0;
        val[1] = 0;
        val[2] = 0;
    }
    result.setCoordinate(val);

    return result;
}

void CADObject::display(GLfloat* color) {

        for(unsigned int i=0; i<vertex.size(); i+=3)
        {
            glBegin(GL_TRIANGLES);
            	// glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
            	glColor3fv(color);

        	GLfloat *fnormal = normal[(i)/3].getCoordinate();
            GLfloat *fvertex1 = vertex[i].getCoordinate();
            GLfloat *fvertex12 = vertex[i+1].getCoordinate();
            GLfloat *fvertex13 = vertex[i+2].getCoordinate();

            if( fnormal[0] == 0 && fnormal[1] == 0 && fnormal[2] == 0 )
            {
            	GLCoordinate coord;
                coord = computeFaceNormal(&fvertex1[0], &fvertex12[0], &fvertex13[0]);
                glNormal3f(coord.getCoordinate(0), coord.getCoordinate(1), coord.getCoordinate(2));
            }
            else
            {
                glNormal3f(fnormal[0], fnormal[1], fnormal[2]);
            }
                glVertex3fv(fvertex1);
                glVertex3fv(fvertex12);
                glVertex3fv(fvertex13);
                glEnd();

        }
}

