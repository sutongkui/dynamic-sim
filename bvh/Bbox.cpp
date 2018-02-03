#include <iostream>

#include <GL/glew.h>

#include "BBox.h"

using namespace std;

void BBox::draw() const 
{
	// top
	glBegin(GL_LINE_STRIP);
	glVertex3d(_max.x, _max.y, _max.z);
	glVertex3d(_max.x, _max.y, _min.z);
	glVertex3d(_min.x, _max.y, _min.z);
	glVertex3d(_min.x, _max.y, _max.z);
	glVertex3d(_max.x, _max.y, _max.z);
	glEnd();

	// bottom
	glBegin(GL_LINE_STRIP);
	glVertex3d(_min.x, _min.y, _min.z);
	glVertex3d(_min.x, _min.y, _max.z);
	glVertex3d(_max.x, _min.y, _max.z);
	glVertex3d(_max.x, _min.y, _min.z);
	glVertex3d(_min.x, _min.y, _min.z);
	glEnd();

	// side
	glBegin(GL_LINES);
	glVertex3d(_max.x, _max.y, _max.z);
	glVertex3d(_max.x, _min.y, _max.z);
	glVertex3d(_max.x, _max.y, _min.z);
	glVertex3d(_max.x, _min.y, _min.z);
	glVertex3d(_min.x, _max.y, _min.z);
	glVertex3d(_min.x, _min.y, _min.z);
	glVertex3d(_min.x, _max.y, _max.z);
	glVertex3d(_min.x, _min.y, _max.z);
	glEnd();

}

void BBox::print() const
{
	cout << _min.x << "  " << _min.y << " " << _min.z << ";  ";
	cout << _max.x << "  " << _max.y << " " << _max.z << endl;
}