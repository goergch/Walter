#include "STLObject.h"

#include "stdio.h"
using namespace std;

STLObject::STLObject()
{

}

STLObject::~STLObject()
{

}

void STLObject::loadFile(char *filename)
{
    ifstream file;
    file.open(filename);
    if(!file.is_open())
    {
        cout << "fle locked: " << filename << endl;
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
}

/**
vertexleri -> vertex vector e ata
normalleri -> normal vector e ata
*/
void STLObject::parse()
{
    string line;
    for(int i=0; i<rows.size(); i++)
    {
        line = rows[i];
        char *ch;
        ch = new char[line.size()+1];
        strcpy(ch,line.c_str());

        GLfloat xyz[3];

        // ======== VERTEX ========
        int idx = 0;
        while ((ch[idx] == ' ') || (ch[idx] == '\t')) idx++;
        if ((ch[idx]=='v' && ch[idx+1]=='e' && ch[idx+2]=='r' && ch[idx+3]=='t' && ch[idx+4]=='e' && ch[idx+5]=='x'))
        {
            sscanf(&(ch[idx]),"vertex %f %f %f", &xyz[0], &xyz[1], &xyz[2]);
            GLCoordinate coord(&xyz[0]);
            vertex.push_back(coord);
        }

        // ======== FACE NORMAL ========
        if ( (ch[idx]=='f' && ch[idx+1]=='a' && ch[idx+2]=='c' && ch[idx+3]=='e' && ch[idx+4]=='t'))
        {
            sscanf(&(ch[idx]), "facet normal %f %f %f", &xyz[0], &xyz[1], &xyz[2]);

            GLCoordinate coord(&xyz[0]);
            normal.push_back(coord);
        }
    }
}



vector<GLCoordinate> STLObject::getVertex()
{
    return vertex;
}

vector<GLCoordinate> STLObject::getNormals()
{
    return normal;
}


GLCoordinate STLObject::computeFaceNormal(GLfloat*  vec1, GLfloat* vec2 ,GLfloat* vec3)
{
	GLCoordinate result;

    GLfloat v1x, v1y, v1z, v2x, v2y, v2z;
    GLfloat nx, ny, nz;
    GLfloat vLen;

    // Calculate vectors
    v1x = vec1[X] - vec2[X];
    v1y = vec1[Y] - vec2[Y];
    v1z = vec1[Z] - vec2[Z];

    v2x = vec2[X] - vec3[X];
    v2y = vec2[Y] - vec3[Y];
    v2z = vec2[Z] - vec3[Z];

    // Get cross product of vectors
    nx = (v1y * v2z) - (v1z * v2y);
    ny = (v1z * v2x) - (v1x * v2z);
    nz = (v1x * v2y) - (v1y * v2x);

    // Normalise final vector

    vLen = sqrt( (nx * nx) + (ny * ny) + (nz * nz) );
    GLfloat val[3];
    if(vLen!=0)
    {
        val[X] = (nx / vLen);
        val[Y] = (ny / vLen);
        val[Z] = (nz / vLen);
    }
    else
    {
        val[X] = 0;
        val[Y] = 0;
        val[Z] = 0;
    }
    result.setCoordinate(val);

    return result;
}
