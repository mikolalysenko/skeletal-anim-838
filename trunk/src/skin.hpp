#ifndef SKIN_H
#define SKIN_H

//Vector arithmetic
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

//STL
#include <iostream>
#include <string>
#include <vector>
#include <cassert>


//OpenGL
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

//Skeleton includes
#include <skeleton.hpp>

namespace Skeletal
{
  using namespace Eigen;
  using namespace std;
  
  //Draws the line skeleton
  template<class XformIter>
    void _draw_line_skeleton_impl(Transform3d& xform_ref, const Joint& skeleton, XformIter& pose_begin, XformIter pose_end, 
      bool disable_color, float alpha)
    {
      assert(pose_begin != pose_end);

      //Construct joint transform
      Transform3d xform = xform_ref;
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
      if(!disable_color) glColor4f(1., 1., 1., alpha);
      glBegin(GL_LINES);
        glVertex3f(0., 0., 0.);
        glVertex3dv(base_pt.data());
      glEnd();
      
      //Draw point for joint location
      if(!disable_color) glColor4f(1., 0., 0., alpha);
      glEnable(GL_POINT_SMOOTH);
      glPointSize(5);
      glBegin(GL_POINTS);
        glVertex3d(0., 0., 0.);
      glEnd();
  
      //Undo OGL transform
      glPopMatrix();
  

      for(int i=0; i<skeleton.children.size(); i++)
        _draw_line_skeleton_impl(xform, skeleton.children[i], pose_begin, pose_end, disable_color, alpha);
    }

  template<class XformIter>
    void draw_line_skeleton(const Joint& skeleton, XformIter pose_begin, XformIter pose_end, 
      bool disable_color = false, float alpha = 1.0)
    {
      Transform3d xform;
      xform.setIdentity();
      _draw_line_skeleton_impl(xform, skeleton, pose_begin, pose_end, disable_color, alpha);
    }



  //Draws the line skeleton
  template<class XformIter>
    void _draw_ellipsoid_skeleton_impl(Transform3d& xform_ref, const Joint& skeleton, XformIter& pose_begin, XformIter pose_end, 
      bool disable_color, float alpha)
    {
      assert(pose_begin != pose_end);

      if(!disable_color) glColor4f(.2, 1., .2, alpha);

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
        _draw_ellipsoid_skeleton_impl(xform, skeleton.children[i], pose_begin, pose_end, disable_color, alpha);
    }

  template<class XformIter>
    void draw_ellipsoid_skeleton(const Joint& skeleton, XformIter pose_begin, XformIter pose_end, 
      bool disable_color = false, float alpha = 1.0)
    {
      Transform3d xform;
      xform.setIdentity();
      _draw_ellipsoid_skeleton_impl(xform, skeleton, pose_begin, pose_end, disable_color, alpha);
    }



  //Draws the stick figure skeleton
  template<class XformIter>
    void _draw_stick_skeleton_impl(Transform3d& xform_ref, const Joint& skeleton, XformIter& pose_begin, XformIter pose_end, 
      bool disable_color, float alpha)
    {
      assert(pose_begin != pose_end);

      
      Transform3d xform = xform_ref;

      //turn on backface culling
      glEnable(GL_CULL_FACE);
      glCullFace(GL_BACK);

      //Create a quadric for cylinders (for bone) and spheres (joints)
      GLUquadricObj* obj = gluNewQuadric();
      gluQuadricNormals(obj, GLU_FLAT);

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

      glPushMatrix();
      //joints have names. so find one that looks like "head"
      //and if so, use that as a bottom center of a sphere. 
      //(outer white w/ normals to in and inner black w/ normals out)
      //looks like most of them have a "Head" and some of them have a "head_end" (do what there?)
      if(skeleton.name.compare("Head") == 0 || skeleton.name.compare("head") == 0 
        || skeleton.name.compare("HEAD") == 0){
          //draw a head.
          glTranslatef(0, 3, 0);
          if(!disable_color) glColor4f(1., 1., 1., alpha);
          gluQuadricOrientation (obj, GLU_INSIDE);
          gluSphere(obj, 6, 20, 20);
          if(!disable_color) glColor4f(0., 0., 0., alpha);
          gluQuadricOrientation (obj, GLU_OUTSIDE);
          gluSphere(obj, 5, 20, 20);
      } else {
        //Because cylinders draw along the z axis, we need to rotate our coordinate system
        double height = sqrt((base_pt(0) * base_pt(0)) + (base_pt(1) * base_pt(1)) + 
          (base_pt(2) * base_pt(2)));
        float vx = base_pt(0);
        float vy = base_pt(1);
        float vz = base_pt(2);
        if(vz == 0) {
          vz = .00000001;
        }
        float ax = (180./M_PI)*acos( vz/height );
        if ( vz < 0.0 ) {
          ax = -ax;
        }
        // yang - does this fix the rotation bug with swing examples?
        // weird bug that only shows up in Linux; Windows version works fine
        // no idea why though
        if ( vz < 0. ) vz = -1.; else vz = 1.;
        float rx = -vy*vz;
        float ry = vx*vz;
        if(rx == 0. && ry == 0.) ry = 1.; 
        if(rx == 0. && ry == 0.) ry = 1.;

        glRotatef(ax, rx, ry, 0.0);

        //Draw bone as a cylinder 
        //(outer in black w/ normals facing in, inner in white w/ normals facing out)
        //(obj, baseRadius, topRadius, height, slices, stacks)
        if(!disable_color) glColor4f(0., 0., 0., alpha);
        gluQuadricOrientation (obj, GLU_INSIDE);
        gluCylinder(obj, .7, .7, height, 20, 20);
        if(!disable_color) glColor4f(1., 1., 1., alpha);
        gluQuadricOrientation (obj, GLU_OUTSIDE);
        gluCylinder(obj, .5, .5, height, 20, 20);

        //This is just me dicking around
        /*if(!disable_color) glColor4f(0., 1., 0., alpha);
        gluQuadricOrientation (obj, GLU_INSIDE);
        gluCylinder(obj, .9, .9, height, 20, 20);
        if(!disable_color) glColor4f(0., 0., 0., alpha);
        gluQuadricOrientation (obj, GLU_OUTSIDE);
        gluCylinder(obj, .5, .5, height, 20, 20);
        */

        //Draw point for joint location
        //Draw a sphere at 0 for a joint
        if(!disable_color) glColor4f(1., 1., 1., alpha);
        gluSphere(obj, 0.5, 20, 20);
      }
      glPopMatrix(); 
      //ok, so right now he has a neck in his head. otherwise, this is BRILLIANT.
      //TODO: add a top hat, and possibly a top hat button.

      //Undo OGL transform
      glPopMatrix();
      gluDeleteQuadric(obj);

      /*
      for(int i=0; i<skeleton.children.size(); i++)
      _draw_stick_skeleton_impl(xform, skeleton.children[i], pose_begin, pose_end);
      */

      if(skeleton.name.compare("Head") == 0 || skeleton.name.compare("head") == 0 || skeleton.name.compare("HEAD") == 0){
        for(int i=0; i<skeleton.children.size(); i++) {
          if(skeleton.children[i].name.compare("NONAME") == 0){
            Transform3d base_xform2 = *(pose_begin++);
            xform.translate(skeleton.children[i].offset);
            xform = xform * base_xform2;
            for(int j=0; j<skeleton.children[i].children.size(); j++) {
              _draw_stick_skeleton_impl(xform, skeleton.children[i].children[j], 
                pose_begin, pose_end, disable_color, alpha);
            }
          } else if (skeleton.children[i].name.compare("Head_end") == 0) {
            Transform3d base_xform2 = *(pose_begin++);
            xform.translate(skeleton.children[i].offset);
            xform = xform * base_xform2;
            for(int j=0; j<skeleton.children[i].children.size(); j++) {
              _draw_stick_skeleton_impl(xform, skeleton.children[i].children[j], 
                pose_begin, pose_end, disable_color, alpha);
            }
          } else {
            _draw_stick_skeleton_impl(xform, skeleton.children[i], pose_begin, pose_end, disable_color, alpha);
          }
        }
      } else {
        for(int i=0; i<skeleton.children.size(); i++) {
          _draw_stick_skeleton_impl(xform, skeleton.children[i], pose_begin, pose_end, disable_color, alpha);
        }
      }
    }

  template<class XformIter>
    void draw_stick_skeleton(const Joint& skeleton, XformIter pose_begin, XformIter pose_end, 
      bool disable_color = false, float alpha = 1.0)
    {
      Transform3d xform;
      xform.setIdentity();
      _draw_stick_skeleton_impl(xform, skeleton, pose_begin, pose_end, disable_color, alpha);
    }




  //Draws the stick figure skeleton
  template<class XformIter>
    void _draw_stick_skeleton_impl2(Transform3d& xform_prev, const Joint& skeleton, XformIter& pose_begin, XformIter pose_end, 
      bool disable_color, float alpha, float bounding_box_radius, int show_face, bool head_found = false)
    {
      assert(pose_begin != pose_end);

      
      float thickness = bounding_box_radius / 30.;

      //turn on backface culling
      glEnable(GL_CULL_FACE);
      //glCullFace(GL_BACK);

      //Create a quadric for cylinders (for bone) and spheres (joints)
      GLUquadricObj* obj = gluNewQuadric();
      gluQuadricNormals(obj, GLU_FLAT);

      //Construct joint transform
      Transform3d xform = xform_prev;
      Transform3d base_xform = *(pose_begin++);

      if(skeleton.name.compare("Head") == 0 || skeleton.name.compare("head") == 0 
          || skeleton.name.compare("HEAD") == 0)
      {
        xform.translate(skeleton.offset);
        xform = xform * base_xform;
      }
      else
      {
        xform.translate(skeleton.offset);
        xform = xform * base_xform;
      }

      //Set up OGL transform
      glPushMatrix();
      Matrix4d tr = xform.matrix();
      glMultMatrixd(tr.data());

      //Start point for bone
      Vector4d base_pt =  base_xform.matrix().inverse() * 
        Vector4d(-skeleton.offset(0), -skeleton.offset(1), -skeleton.offset(2), 0);

      if(!head_found)
      {
        glPushMatrix();
        //joints have names. so find one that looks like "head"
        //and if so, use that as a bottom center of a sphere. 
        //(outer white w/ normals to in and inner black w/ normals out)
        //looks like most of them have a "Head" and some of them have a "head_end" (do what there?)
        if(skeleton.name.compare("Head") == 0 || skeleton.name.compare("head") == 0 
          || skeleton.name.compare("HEAD") == 0){
            

            // compute the head size radius
            float radius = bounding_box_radius / 5.;
 
            
            //draw a head as a cylinder
            glPushMatrix();
            glTranslatef(0, radius, 0);
            if(!disable_color) glColor4f(1., 1., 0., alpha);
            glutSolidTorus(thickness, radius, 20, 20);
            glPopMatrix(); 

            if(show_face == 2)
            {
              glPushMatrix();
              glTranslatef(0, radius, 0);
              //if(!disable_color) glColor4f(1., 1., 0., alpha);
              
              glPushAttrib(GL_ALL_ATTRIB_BITS);
              glEnable(GL_BLEND);
              glDepthMask(GL_FALSE);
              glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
              glEnable(GL_TEXTURE_2D);
              glBegin(GL_QUADS);
              glTexCoord2f(1.0f, 1.0f); glVertex3f(-radius, -radius, 0);	
              glTexCoord2f(0.0f, 1.0f); glVertex3f(radius, -radius, 0);	
              glTexCoord2f(0.0f, 0.0f); glVertex3f(radius, radius, 0);	
              glTexCoord2f(1.0f, 0.0f); glVertex3f(-radius, radius, 0);
              glEnd();
              glDisable(GL_TEXTURE_2D);
              glDisable(GL_BLEND);
              glPopAttrib();

              glPopMatrix(); 
            }

            if(show_face == 1)
            {
              float face_thickness = thickness / 1.5;

              //draw one eye
              glPushMatrix();
              glTranslatef(radius / 3., radius + radius / 3., 0.);
              if(!disable_color) glColor4f(1., 1., 0., alpha);
              gluSphere(obj, face_thickness, 20, 20);
              glPopMatrix(); 

              
              //draw the other eye
              glPushMatrix();
              glTranslatef(-radius / 3., radius + radius / 3., 0.);
              if(!disable_color) glColor4f(1., 1., 0., alpha);
              gluSphere(obj, face_thickness, 20, 20);
              glPopMatrix(); 

              
              //draw the nose
              glPushMatrix();
              glTranslatef(0., radius, 0.);
              glRotatef(90., 1., 0., 0.);
              if(!disable_color) glColor4f(1., 1., 0., alpha);
              gluCylinder(obj, face_thickness, face_thickness, radius / 6., 20, 20);
              glPopMatrix(); 

              glPushMatrix();
              glTranslatef(0., radius, 0.);
              if(!disable_color) glColor4f(1., 1., 0., alpha);
              gluSphere(obj, face_thickness, 20, 20);
              glPopMatrix(); 

              glPushMatrix();
              glTranslatef(0., radius - radius / 6., 0.);
              if(!disable_color) glColor4f(1., 1., 0., alpha);
              gluSphere(obj, face_thickness, 20, 20);
              glPopMatrix(); 

              //draw the mouth
              glPushMatrix();
              glTranslatef(-radius / 3., radius / 2, 0.);
              glRotatef(90., 0., 1., 0.);
              if(!disable_color) glColor4f(1., 1., 0., alpha);
              gluCylinder(obj, face_thickness, face_thickness, radius * 2. / 3., 20, 20);
              glPopMatrix(); 

              glPushMatrix();
              glTranslatef(-radius / 3., radius / 2, 0.);
              if(!disable_color) glColor4f(1., 1., 0., alpha);
              gluSphere(obj, face_thickness, 20, 20);
              glPopMatrix(); 

              glPushMatrix();
              glTranslatef(radius / 3., radius / 2, 0.);
              if(!disable_color) glColor4f(1., 1., 0., alpha);
              gluSphere(obj, face_thickness, 20, 20);
              glPopMatrix(); 
            
            }

        } 


        //Because cylinders draw along the z axis, we need to rotate our coordinate system
        double height = sqrt((base_pt(0) * base_pt(0)) + (base_pt(1) * base_pt(1)) + 
          (base_pt(2) * base_pt(2)));
        float vx = base_pt(0);
        float vy = base_pt(1);
        float vz = base_pt(2);
        if(vz == 0) {
          vz = .00000001;
        }
        float ax = (180./M_PI)*acos( vz/height );
        if ( vz < 0.0 ) {
          ax = -ax;
        }
        // yang - does this fix the rotation bug with swing examples?
        // weird bug that only shows up in Linux; Windows version works fine
        // no idea why though
        if ( vz < 0. ) vz = -1.; else vz = 1.;
        float rx = -vy*vz;
        float ry = vx*vz;
        if(rx == 0. && ry == 0.) ry = 1.; 
        if(rx == 0. && ry == 0.) ry = 1.;
        glRotatef(ax, rx, ry, 0.0);

        //Draw bone as a cylinder 
        if(!disable_color) glColor4f(1., 1., 0., alpha);
        gluCylinder(obj, thickness, thickness, height, 20, 20);

        //Draw a sphere at 0 for a joint
        if(!disable_color) glColor4f(1., 1., 0., alpha);
        gluSphere(obj, thickness, 20, 20);
      
        glPopMatrix(); 
      }

      
      //ok, so right now he has a neck in his head. otherwise, this is BRILLIANT.
      //TODO: add a top hat, and possibly a top hat button.

      //Undo OGL transform
      glPopMatrix();
      gluDeleteQuadric(obj);


      if(skeleton.name.compare("Head") == 0 || skeleton.name.compare("head") == 0 || skeleton.name.compare("HEAD") == 0){
        for(int i=0; i<skeleton.children.size(); i++) {
          _draw_stick_skeleton_impl2(xform, 
            skeleton.children[i], 
            pose_begin, 
            pose_end, 
            disable_color, 
            alpha, 
            bounding_box_radius, 
            show_face,
            true);
        }
      } else {
        for(int i=0; i<skeleton.children.size(); i++) {
          _draw_stick_skeleton_impl2(xform, 
            skeleton.children[i], 
            pose_begin, 
            pose_end, 
            disable_color, 
            alpha, 
            bounding_box_radius,
            show_face);
        }
      }   
    }

  template<class XformIter>
    void draw_stick_skeleton2(const Joint& skeleton, XformIter pose_begin, XformIter pose_end, 
      bool disable_color = false, float alpha = 1.0, float bounding_box_radius = 0.5, int show_face = true, bool show_edges = false)
    {

      XformIter pose_begin1 = pose_begin,
                pose_begin2 = pose_begin,
                pose_end1 = pose_end,
                pose_end2 = pose_end;

      Transform3d xform;
      xform.setIdentity();
      _draw_stick_skeleton_impl2(xform, skeleton, pose_begin1, pose_end1, disable_color, alpha, bounding_box_radius, show_face);

      
      if(show_edges && !disable_color && alpha == 1.0)
      {
        

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        glCullFace(GL_FRONT);
	      glDepthFunc(GL_LEQUAL);
        glPolygonMode(GL_BACK, GL_LINE);
        GLfloat lineWidth = 3.0f;
        glLineWidth(lineWidth);
        glColor4f(0., 0., 0., 1.);
        _draw_stick_skeleton_impl2(xform, skeleton, pose_begin2, pose_end2, true, alpha, bounding_box_radius, show_face);
        
		    //glPolygonMode(GL_FRONT, GL_FILL);
        //glCullFace(GL_BACK);

        glPopAttrib();
      }
      

      
    }



  //Draws the line skeleton
  template<class XformIter>
    bool _compute_end_effector_pos_impl(Transform3d& xform_ref, const Joint& skeleton, XformIter& pose_begin, XformIter pose_end, 
      const Joint& end_effector, Vector3d &position)
    {
      assert(pose_begin != pose_end);

      //Construct joint transform
      Transform3d xform = xform_ref;
      Transform3d base_xform = *(pose_begin++);
      xform.translate(skeleton.offset);
      xform = xform * base_xform;
    
      if(&skeleton == &end_effector) 
      {
        position = xform.translation();
        return true;
      }

      for(int i=0; i<skeleton.children.size(); i++)
      {
        if(_compute_end_effector_pos_impl(xform, skeleton.children[i], pose_begin, pose_end, end_effector, position) == true)
          return true;
      }

      return false;
    }

  template<class XformIter>
    bool compute_end_effector_pos(const Joint& skeleton, XformIter pose_begin, XformIter pose_end, 
      const Joint& end_effector, Vector3d &position)
    {
      Transform3d xform;
      xform.setIdentity();
      return _compute_end_effector_pos_impl(xform, skeleton, pose_begin, pose_end, end_effector, position);
    }




};

#endif
