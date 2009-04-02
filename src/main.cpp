/**
 * Main module form skeletal animation library
 */

//STL includes
#include <iostream>
#include <string>
#include <vector>

//Eigen includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

//OpenGL
#include <GL/glut.h>

//Project includes
#include <skeleton.hpp>

using namespace std;
using namespace Eigen;
using namespace Skeletal;


#define PAN_RATE	100.

//Window parameters
int		width = 800,
		height = 800;

//Perspective matrix
double	fov = 45., 
		z_near = 0.1,
		z_far = 10000.;

//Camera transformation
Vector3d	object_center;
Quaterniond camera_rot;

//Motion capture data
Motion mocap;

void init()
{
	//Parse out motion capture data
	mocap = parseBVH(cin);
}


void draw()
{
	//Extract matrices
	vector<Transform3d> xform( mocap.skeleton.size() );
	mocap.skeleton.interpret_pose(
		xform.begin(), 
		mocap.frames[1].pose.begin(),
		mocap.frames[1].pose.end());
	
	//Draw a floor grid
	glColor3f(1.,1.,1.);
	glBegin(GL_LINES);
		for(float t=-100.; t<=100.; t+=10.)
		{
			glVertex3f(t, 0., -100.);
			glVertex3f(t, 0.,  100.);
			glVertex3f(-100., 0., t);
			glVertex3f( 100., 0., t);
		}
	glEnd();
	
	//Draw origin for each coordinate system
	glPointSize(5);
	glBegin(GL_POINTS);
	glColor3f(1, 0, 0);
	for(int i=0; i<xform.size(); i++)
	{
		Matrix4d m = xform[i].matrix();
		glVertex3f(m(0, 3), m(1, 3), m(2, 3));
	}
	glEnd();
}


void display()
{
	//Set window bounds
	glViewport(0, 0, width, height);

	//Clear buffer
	glClearColor(0., 0., 0., 0.);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	//Set up camera
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fov, (double)width / (double)height, z_near, z_far);
	
	//Create base camer matrix from mouse controls
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(-object_center(0), -object_center(1), -object_center(2));
	Matrix4d tr = Transform3d(camera_rot).matrix().transpose();
	glMultMatrixd(tr.data());
	
	//Set other flags....
	
	//Do actual drawing code
	draw();
	
	//Finish up drawing
	glFlush();
	glutSwapBuffers();
}


void keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
		case 27: exit(0);
		default: break;
	}
}

//Arcball parameters
bool moving_arcball = false, zoom_camera = false, pan_camera = false;
int rotation_pos[2], pan_base[2], zoom_base[2];
Quaterniond previous_rotation;

void mouse_button(int button, int state, int x, int y)
{
	if(button == 0 && state == GLUT_DOWN)
	{
		moving_arcball = true;
		previous_rotation.setIdentity();
		rotation_pos[0] = x;
		rotation_pos[1] = y;
	}
	else if(button == 0 && state != GLUT_DOWN)
	{
		moving_arcball = false;
	}
	
	if(button == 2 && state == GLUT_DOWN)
	{
		pan_camera = true;
		pan_base[0] = x;
		pan_base[1] = y;
	}
	else if(button == 2 && state != GLUT_DOWN)
	{
		pan_camera = false;
	}
	
	if(button == 1 && state == GLUT_DOWN)
	{
		zoom_camera = true;
		zoom_base[0] = x;
		zoom_base[1] = y;
	}
	else if(button == 1 && state != GLUT_DOWN)
	{
		zoom_camera = false;
	}
}

void mouse_motion(int x, int y)
{
	if(moving_arcball)
	{
		camera_rot *= previous_rotation.inverse().normalized();
		
		Vector3d A = Vector3d(
					-0.5 + (double)x / (double)width, 
					 0.5 - (double)y / (double)height, 0.5),
				 B = Vector3d(
				 	-0.5 + (double)rotation_pos[0] / (double)width, 
				 	 0.5 - (double)rotation_pos[1] / (double)height, 0.5);
				
		Quaterniond next;
		next.setFromTwoVectors(A, B).normalize();
		camera_rot *= next;
			
		previous_rotation = next;
	}
	
	if(pan_camera)
	{
		Vector3d delta = 
			Vector3d((double)(pan_base[0] - x) / width, 
					-(double)(pan_base[1] - y) / (double)height,
					0.0);
		
		object_center += delta * PAN_RATE;
		
		pan_base[0] = x;
		pan_base[1] = y;
	}
	
	if(zoom_camera)
	{
		Vector3d delta = 
			Vector3d((double)(zoom_base[0] - x) / width, 
					0.,
					-(double)(zoom_base[1] - y) / (double)height);
		
		object_center += delta * PAN_RATE;
		
		zoom_base[0] = x;
		zoom_base[1] = y;
	}
}

//Resize the screen
void resize(int nx, int ny)
{
	width = nx;
	height = ny;
}

int main(int argc, char ** argv)
{
	//Clear camera rotation
	object_center = Vector3d(0, 0, 500);
	camera_rot.setIdentity();

	//Initialize glut stuff
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	
	glutCreateWindow("Project 2");
	
	//Set callbacks
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(display);
	glutIdleFunc(glutPostRedisplay);
	glutReshapeFunc(resize);
	glutMouseFunc(mouse_button);
	glutMotionFunc(mouse_motion);
	
	
	//Do initialization
	init();
	
	//Go
	glutMainLoop();
	
	return 0;
}

