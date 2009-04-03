#ifndef SKIN_H
#define SKIN_H

//STL
#include <iostream>
#include <string>
#include <vector>
#include <cassert>

//Vector arithmetic
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

//OpenGL
#include <GL/glut.h>

//Skeleton includes
#include <skeleton.hpp>

namespace Skeletal
{
	using namespace Eigen;
	using namespace std;
	
	//Draws the line skeleton
	template<class XformIter>
		void _draw_line_skeleton_impl(Transform3d xform, const Joint& skeleton, XformIter& pose_begin, XformIter pose_end)
		{
			assert(pose_begin != pose_end);

			Transform3d base_xform = *(pose_begin++);

			xform.translate(skeleton.offset);
			xform = xform * base_xform;
		
			glPushMatrix();
		
			Matrix4d tr = xform.matrix();
			glMultMatrixd(tr.data());
			
			//Point of parent
			Vector4d base_pt =  base_xform.matrix().inverse() * 
				Vector4d(-skeleton.offset(0), -skeleton.offset(1), -skeleton.offset(2), 0);

			glColor3f(1., 1., 1.);
			glBegin(GL_LINES);
				glVertex3f(0., 0., 0.);
				glVertex3dv(base_pt.data());
			glEnd();
			
	
			glColor3f(1., 0., 0.);
			glEnable(GL_POINT_SMOOTH);
			glPointSize(5);
			glBegin(GL_POINTS);
				glVertex3d(0., 0., 0.);
			glEnd();
	
			glPopMatrix();
	

			for(int i=0; i<skeleton.children.size(); i++)
				_draw_line_skeleton_impl(xform, skeleton.children[i], pose_begin, pose_end);
		}

	template<class XformIter>
		void draw_line_skeleton(const Joint& skeleton, XformIter pose_begin, XformIter pose_end)
		{
			Transform3d xform;
			xform.setIdentity();
			_draw_line_skeleton_impl(xform, skeleton, pose_begin, pose_end);
		}

};

#endif
