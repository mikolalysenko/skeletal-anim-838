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

			//Construct joint transform
			Transform3d base_xform = *(pose_begin++);
			xform.translate(skeleton.offset);
			xform = xform * base_xform;
		
			//Set up OGL transform
			glPushMatrix();
			Matrix4d tr = xform.matrix();
			glMultMatrixd(tr.data());
			
			//Start point for bone
			Vector4d base_pt =  base_xform.matrix().inverse() * 
				Vector4d(-skeleton.offset(0), -skeleton.offset(1), -skeleton.offset(2), 0);

			//Draw bone
			glColor3f(1., 1., 1.);
			glBegin(GL_LINES);
				glVertex3f(0., 0., 0.);
				glVertex3dv(base_pt.data());
			glEnd();
			
			//Draw point for joint location
			glColor3f(1., 0., 0.);
			glEnable(GL_POINT_SMOOTH);
			glPointSize(5);
			glBegin(GL_POINTS);
				glVertex3d(0., 0., 0.);
			glEnd();
	
			//Undo OGL transform
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



	//Draws the line skeleton
	template<class XformIter>
		void _draw_ellipsoid_skeleton_impl(Transform3d& xform_ref, const Joint& skeleton, XformIter& pose_begin, XformIter pose_end)
		{
			assert(pose_begin != pose_end);

      Transform3d xform = xform_ref;

			//Construct joint transform
			Transform3d base_xform = *(pose_begin++);
			xform.translate(skeleton.offset);
			xform = xform * base_xform;
		
			//Set up OGL transform
			glPushMatrix();
			Matrix4d tr = xform.matrix();
			glMultMatrixd(tr.data());
			
			//Start point for bone
			Vector4d base_pt =  base_xform.matrix().inverse() * 
				Vector4d(-skeleton.offset(0), -skeleton.offset(1), -skeleton.offset(2), 0);
				
			Vector3d x = Vector3d(base_pt(0), base_pt(1), base_pt(2)) * 0.5;
			Vector3d u = x.cross(Vector3d::Random()).normalized();
			Vector3d v = x.cross(u).normalized();

			//Draw an ellipsoid
			const double 
				delta_phi = M_PI / 16.,
				delta_theta = M_PI / 8.;
			
			glBegin(GL_TRIANGLES);
			for(double phi0=0., phi1=delta_phi; 
					   phi1<=M_PI; 
					   phi0+=delta_phi,phi1+=delta_phi)
			for(double theta0=-M_PI, theta1=delta_theta-M_PI; 
					   theta1<=M_PI; 
					   theta0+=delta_theta,theta1+=delta_theta)
			{
				Vector3d p00 = sin(phi0) * (cos(theta0) * u + sin(theta0) * v) + (1 - cos(phi0)) * x,
						 p01 = sin(phi1) * (cos(theta0) * u + sin(theta0) * v) + (1 - cos(phi1)) * x,
						 p10 = sin(phi0) * (cos(theta1) * u + sin(theta1) * v) + (1 - cos(phi0)) * x,
						 p11 = sin(phi1) * (cos(theta1) * u + sin(theta1) * v) + (1 - cos(phi1)) * x;
				
				Vector3d n0 = (p01 - p00).cross(p10 - p00).normalized(),
						 n1 = (p10 - p01).cross(p11 - p01).normalized();
				
				glNormal3dv(n0.data());
				glVertex3dv(p00.data());
				glVertex3dv(p01.data());
				glVertex3dv(p10.data());
				
				glNormal3dv(n1.data());
				glVertex3dv(p01.data());
				glVertex3dv(p10.data());				
				glVertex3dv(p11.data());
			}
			glEnd();
	
	
			//Undo OGL transform
			glPopMatrix();
	

			for(int i=0; i<skeleton.children.size(); i++)
				_draw_ellipsoid_skeleton_impl(xform, skeleton.children[i], pose_begin, pose_end);
		}

	template<class XformIter>
		void draw_ellipsoid_skeleton(const Joint& skeleton, XformIter pose_begin, XformIter pose_end)
		{
			Transform3d xform;
			xform.setIdentity();
			_draw_ellipsoid_skeleton_impl(xform, skeleton, pose_begin, pose_end);
		}


};

#endif
