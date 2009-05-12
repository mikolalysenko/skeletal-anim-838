
#include <FL/fl_ask.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Image.H>
#include <FL/fl_draw.H>

//STL includes
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

//Eigen includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

//OpenGL
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


#ifndef WIN32
#include <sys/time.h>
#endif

#include <misc.hpp>
#include <skeleton.hpp>
#include <skin.hpp>
#include <mograph.hpp>

#include "gl_view.h"


#ifdef WIN32
#  include <io.h>
#    define access _access
#    define chdir _chdir
#    define getcwd _getcwd
#else
#  include <unistd.h>
#endif



#define PAN_RATE        100.

//Window parameters
//int     width = 800,
//        height = 800;

//Perspective matrix
double  fov = 45., 
        z_near = 0.1,
        z_far = 10000.;

double colors[10][4] =
{
        {1., 0., 0., 0.5},
        {.2, .2, .2, 0.5},
        {0., 0., 1., 0.5},
        {1., 1., 0., 0.5},
        {0., 1., 1., 0.5},
        {1., .5, 1., 0.5},
        {1., 1., .5, 0.5},
        {.5, 1., 1., 0.5},
        {1., 1., 1., 0.5},
        {0., 0., 0., 0.5}
};

static float floorHeight = 0.0;
static float floorLength = 200;
static GLfloat floorVertex[4][3] = {
  { -floorLength, floorHeight,  floorLength },
  {  floorLength, floorHeight,  floorLength },
  {  floorLength, floorHeight, -floorLength },
  { -floorLength, floorHeight, -floorLength },
};

//Camera transformation
Vector3d    object_center;
Quaterniond camera_rot;


//Spline stuff
aligned<Vector3f>::vector control_points;
double t_max;

//Arcball parameters
bool moving_arcball = false, zoom_camera = false, pan_camera = false;
int rotation_pos[2], pan_base[2], zoom_base[2];
Quaterniond previous_rotation;


// project shadow matrix calculation function taken from
// http://www.bluevoid.com/opengl/sig00/advanced00/notes/node199.html
static void
projectShadowMatrix(float ground[4], float light[4])
{
  float  dot;
  float  shadowMat[4][4];

  dot = ground[0] * light[0] +
        ground[1] * light[1] +
        ground[2] * light[2] +
        ground[3] * light[3];
  
  shadowMat[0][0] = dot - light[0] * ground[0];
  shadowMat[1][0] = 0.0 - light[0] * ground[1];
  shadowMat[2][0] = 0.0 - light[0] * ground[2];
  shadowMat[3][0] = 0.0 - light[0] * ground[3];
  
  shadowMat[0][1] = 0.0 - light[1] * ground[0];
  shadowMat[1][1] = dot - light[1] * ground[1];
  shadowMat[2][1] = 0.0 - light[1] * ground[2];
  shadowMat[3][1] = 0.0 - light[1] * ground[3];
  
  shadowMat[0][2] = 0.0 - light[2] * ground[0];
  shadowMat[1][2] = 0.0 - light[2] * ground[1];
  shadowMat[2][2] = dot - light[2] * ground[2];
  shadowMat[3][2] = 0.0 - light[2] * ground[3];
  
  shadowMat[0][3] = 0.0 - light[3] * ground[0];
  shadowMat[1][3] = 0.0 - light[3] * ground[1];
  shadowMat[2][3] = 0.0 - light[3] * ground[2];
  shadowMat[3][3] = dot - light[3] * ground[3];

  glMultMatrixf((const GLfloat*)shadowMat);
}

// find the plane equation given 3 points
// below are links that explains the math
// http://www.jtaylor1142001.net/calcjat/Solutions/VPlanes/VP3Pts.htm
// http://easyweb.easynet.co.uk/~mrmeanie/plane/planes.htm
void findPlane(GLfloat plane[4], GLfloat vertex1[3], GLfloat vertex2[3], GLfloat vertex3[3])
{
  GLfloat vector1[3], vector2[3];

  // get the vector differences
  vector1[0] = vertex2[0] - vertex1[0];
  vector1[1] = vertex2[1] - vertex1[1];
  vector1[2] = vertex2[2] - vertex1[2];

  vector2[0] = vertex3[0] - vertex1[0];
  vector2[1] = vertex3[1] - vertex1[1];
  vector2[2] = vertex3[2] - vertex1[2];

  // compute A, B, C by computing the cross product
  plane[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
  plane[1] = -(vector1[0] * vector2[2] - vector1[2] * vector2[0]);
  plane[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];

  // compute D
  plane[3] = -(plane[0] * vertex1[0] + plane[1] * vertex1[1] + plane[2] * vertex1[2]);
}

void glView::print_skeleton(const Joint& skel, int t)
{
  for(int i=0; i<t; i++)
    cout << '\t';
          
  cout << skel.name << ":";
  for(int i=0; i<skel.channels.size(); i++)
    cout << skel.channels[i] << ", ";
  cout << endl;

  for(int i=0; i<skel.children.size(); i++)
    print_skeleton(skel.children[i], t+1);
}


void mainWindow_callback(Fl_Widget* widget, void*) 
{
  exit(0);
}
void childWindow_callback(Fl_Widget* widget, void*) 
{
  // do nothing
  ((Fl_Double_Window*)widget)->iconize();
}


void glView::initScene()
{
  m_ui->mainWindow->callback(mainWindow_callback);
  m_ui->motionGraphWindow->callback(childWindow_callback);
  m_ui->pathFindingWindow->callback(childWindow_callback);
  m_ui->inputCloudTreshold->value("10.0");
  m_ui->inputRandomFrames->value("2000"); 
  m_ui->inputDistanceSpline->value("10.0");
  m_ui->boxPointCloudMap->view = this;
  m_ui->viewPointCloud->view = this;
  m_ui->editorSpline->buffer(textSpline);
  m_ui->inputStartX->value("0.0");
  m_ui->inputStartY->value("0.0");
  m_ui->inputEndX->value("50.0");
  m_ui->inputEndY->value("0.0");
  reset_mg_map();

  // camera position
  object_center = Vector3d(0, 50, 400);
  camera_rot.setIdentity();

  // light position from above the origin
  lightPosition[0] = 0;
  lightPosition[1] = 200;
  lightPosition[2] = 0;
  lightPosition[3] = 0;

  
  glEnable(GL_DEPTH_TEST);

  Fl_Image* img = m_ui->boxImageLogo->image();
  idFloor = 0;
  if(img)
  {
    glGenTextures(1, &idFloor);
    glBindTexture(GL_TEXTURE_2D, idFloor);
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, img->w(), img->h(), 0, GL_RGBA, GL_UNSIGNED_BYTE, img->data()[0]);
    //img->release();
  }


  
  Fl_Shared_Image* imgFace = Fl_Shared_Image::get("face.png");
  idFace = 0;
  if(imgFace)
  {
    glGenTextures(1, &idFace);
    glBindTexture(GL_TEXTURE_2D, idFace);
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, imgFace->w(), imgFace->h(), 0, GL_RGBA, GL_UNSIGNED_BYTE, imgFace->data()[0]);
    imgFace->release();
  }


}

void glView::draw_skeleton(double t, bool disable_color, float alpha)
{
  if(!mocap_selected) return;

  glPushAttrib(GL_ALL_ATTRIB_BITS);

  //Figure interpolate current pose
  Frame c_frame = mocap_selected->get_frame(t, true);

  //Extract matrices
  aligned<Transform3d>::vector xform = c_frame.local_xform(mocap_selected->skeleton);
  
  switch(m_draw_style)
  {
  case STYLE_STICK:
    draw_stick_skeleton(mocap_selected->skeleton, xform.begin(), xform.end(), disable_color, alpha);
    break;

  case STYLE_STICK2:
    draw_stick_skeleton2(mocap_selected->skeleton, xform.begin(), xform.end(), disable_color, alpha, mocap_selected->bound_sphere_radius_skeleton());
    break;

  case STYLE_STICK2_NO_FACE:
    draw_stick_skeleton2(mocap_selected->skeleton, xform.begin(), xform.end(), disable_color, alpha, mocap_selected->bound_sphere_radius_skeleton(), 0);
    break;

  case STYLE_STICK2_EDGES:
    draw_stick_skeleton2(mocap_selected->skeleton, xform.begin(), xform.end(), disable_color, alpha, mocap_selected->bound_sphere_radius_skeleton(), 1, true);
    break;

  case STYLE_STICK2_CUSTOM_FACE:
    glBindTexture(GL_TEXTURE_2D, idFace);
    draw_stick_skeleton2(mocap_selected->skeleton, xform.begin(), xform.end(), disable_color, alpha, mocap_selected->bound_sphere_radius_skeleton(), 2);
    break;

  case STYLE_LINES:
    draw_line_skeleton(mocap_selected->skeleton, xform.begin(), xform.end(), disable_color, alpha);
    break;

  case STYLE_ELLIPSOIDS:
    draw_ellipsoid_skeleton(mocap_selected->skeleton, xform.begin(), xform.end(), disable_color, alpha);
    break;

  default:
    break;
  }


  glPopAttrib();


/*
  //display the bounding sphere
  Transform3d xform2;
  xform2.setIdentity();
  Transform3d base_xform = xform[0];
  xform2.translate(mocap_selected->skeleton.offset);
  xform2 = xform2 * base_xform;
  glPushMatrix();
  Matrix4d tr = xform2.matrix();
  glMultMatrixd(tr.data());
  glDisable(GL_LIGHTING);
  glColor3f(1., 1., 0.);
  glutWireSphere(mocap_selected->bound_sphere_radius_skeleton(), 8, 8);
  glPopMatrix();
*/

/*
  //display the bounding box
  glDisable(GL_LIGHTING);
  Vector3d minPt;
  Vector3d maxPt;
  minPt = mocap_selected->bound_box_min;
  maxPt = mocap_selected->bound_box_max;
  glColor3f(1., 0., 0.);
  glBegin(GL_QUADS);							// Start Drawing Quads
  // Bottom Face
  glTexCoord2f(1.0f, 1.0f); glVertex3f(minPt[0], minPt[1], minPt[2]);	// Top Right Of The Texture and Quad
  glTexCoord2f(0.0f, 1.0f); glVertex3f(maxPt[0], minPt[1], minPt[2]);	// Top Left Of The Texture and Quad
  glTexCoord2f(0.0f, 0.0f); glVertex3f(maxPt[0], minPt[1], maxPt[2]);	// Bottom Left Of The Texture and Quad
  glTexCoord2f(1.0f, 0.0f); glVertex3f(minPt[0], minPt[1], maxPt[2]);	// Bottom Right Of The Texture and Quad
  // Front Face
  glTexCoord2f(0.0f, 0.0f); glVertex3f(minPt[0], minPt[1], maxPt[2]);	// Bottom Left Of The Texture and Quad
  glTexCoord2f(1.0f, 0.0f); glVertex3f(maxPt[0], minPt[1], maxPt[2]);	// Bottom Right Of The Texture and Quad
  glTexCoord2f(1.0f, 1.0f); glVertex3f(maxPt[0], maxPt[1], maxPt[2]);	// Top Right Of The Texture and Quad
  glTexCoord2f(0.0f, 1.0f); glVertex3f(minPt[0], maxPt[1], maxPt[2]);	// Top Left Of The Texture and Quad
  // Back Face
  glTexCoord2f(1.0f, 0.0f); glVertex3f(minPt[0], minPt[1], minPt[2]);	// Bottom Right Of The Texture and Quad
  glTexCoord2f(1.0f, 1.0f); glVertex3f(minPt[0], maxPt[1], minPt[2]);	// Top Right Of The Texture and Quad
  glTexCoord2f(0.0f, 1.0f); glVertex3f(maxPt[0], maxPt[1], minPt[2]);	// Top Left Of The Texture and Quad
  glTexCoord2f(0.0f, 0.0f); glVertex3f(maxPt[0], minPt[1], minPt[2]);	// Bottom Left Of The Texture and Quad
  // Right face
  glTexCoord2f(1.0f, 0.0f); glVertex3f(maxPt[0], minPt[1], minPt[2]);	// Bottom Right Of The Texture and Quad
  glTexCoord2f(1.0f, 1.0f); glVertex3f(maxPt[0], maxPt[1], minPt[2]);	// Top Right Of The Texture and Quad
  glTexCoord2f(0.0f, 1.0f); glVertex3f(maxPt[0], maxPt[1], maxPt[2]);	// Top Left Of The Texture and Quad
  glTexCoord2f(0.0f, 0.0f); glVertex3f(maxPt[0], minPt[1], maxPt[2]);	// Bottom Left Of The Texture and Quad
  // Left Face
  glTexCoord2f(0.0f, 0.0f); glVertex3f(minPt[0], minPt[1], minPt[2]);	// Bottom Left Of The Texture and Quad
  glTexCoord2f(1.0f, 0.0f); glVertex3f(minPt[0], minPt[1], maxPt[2]);	// Bottom Right Of The Texture and Quad
  glTexCoord2f(1.0f, 1.0f); glVertex3f(minPt[0], maxPt[1], maxPt[2]);	// Top Right Of The Texture and Quad
  glTexCoord2f(0.0f, 1.0f); glVertex3f(minPt[0], maxPt[1], minPt[2]);	// Top Left Of The Texture and Quad
  glEnd();	
*/
 

}


void glView::draw_frame(int f)
{
  if(!mocap_selected) return;

  //Figure interpolate current pose
  Frame c_frame = mocap_selected->frames[f];

  //Extract matrices
  aligned<Transform3d>::vector xform = c_frame.local_xform(mocap_selected->skeleton);

  //Draw the line skeleton
  draw_ellipsoid_skeleton(mocap_selected->skeleton, xform.begin(), xform.end());
}




//*************************************************************************
//
// the two colors for the floor for the check board
//
//*************************************************************************
float floorColor1[3] = { .9f, .9f, .9f }; // Light color
float floorColor2[3] = { .3f, .3f, 1.0f }; // Dark color

//*************************************************************************
//
// Draw the check board floor without texturing it
// Taken from the CS559 train sample code
//===============================================================================
void drawCheckeredFloor(float size, int nSquares)
//===============================================================================
{
  // parameters:
  float maxX = size/2, maxY = size/2;
  float minX = -size/2, minY = -size/2;

  int x,y,v[3],i;
  float xp,yp,xd,yd;
  v[2] = 0;
  xd = (maxX - minX) / ((float) nSquares);
  yd = (maxY - minY) / ((float) nSquares);
  glBegin(GL_QUADS);
  for(x=0,xp=minX; x<nSquares; x++,xp+=xd) {
    for(y=0,yp=minY,i=x; y<nSquares; y++,i++,yp+=yd) {
      glColor4fv(i%2==1 ? floorColor1:floorColor2);
      glNormal3f(0, 1, 0); 
      glVertex3d(xp,      floorHeight, yp);
      glVertex3d(xp,      floorHeight, yp + yd);
      glVertex3d(xp + xd, floorHeight, yp + yd);
      glVertex3d(xp + xd, floorHeight, yp);

    } // end of for j
  }// end of for i
  glEnd();
}


void glView::drawFloor(bool backface)
{

  
  glEnable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);

  // draw the checkered floor
  drawCheckeredFloor(abs(floorVertex[0][0]-floorVertex[1][0]),20);

  // draw the uw logo
  if(idFloor && !backface)
  {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_BLEND);
    glDepthMask(GL_FALSE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, idFloor);
    glColor3f(1.,1.,1.);
    glBegin(GL_QUADS);
      glNormal3f(0, 1, 0); 
      glTexCoord2f(0, 0); glVertex3fv(floorVertex[0]);
      glTexCoord2f(1, 0); glVertex3fv(floorVertex[1]);
      glTexCoord2f(1, 1); glVertex3fv(floorVertex[2]);
      glTexCoord2f(0, 1); glVertex3fv(floorVertex[3]);
    glEnd();
    glDisable(GL_TEXTURE_2D);

    glDisable(GL_NORMALIZE);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
  }

  glEnable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
}

void glView::drawScene()
{


  // return if no animation is selected
  if(!mocap_selected) 
  {

    // draw the back of the floor
    glFrontFace(GL_CW);
    drawFloor(true);
    glFrontFace(GL_CCW);

    
    glEnable(GL_DEPTH_TEST);
    drawFloor();

    draw_spline();

    return;
  }


#ifdef WIN32
  // update the time if necessary
  static unsigned long lastClock = clock();
  if(m_play)
  {
    float elapse_time = (float)(clock() - lastClock) / (float)CLOCKS_PER_SEC;
    m_time += elapse_time * (mocap_selected->frame_time / (1. / m_play_fps));
  }
  lastClock = clock();
#else
  static struct timeval start, end;
  long mtime, seconds, useconds;    
  gettimeofday(&end, NULL);
  seconds  = end.tv_sec  - start.tv_sec;
  useconds = end.tv_usec - start.tv_usec;
  mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;


  // update the time if necessary
  static unsigned long lastClock = clock();
  if(m_play)
  {
    float elapse_time = (float)mtime / 1000.0;
    m_time += elapse_time * (mocap_selected->frame_time / (1. / m_play_fps));
  }
  start = end;
#endif

  // check if the repeat mode is on
  if(m_time >= mocap_selected->duration())
  {
    if(!m_repeat)
    {
      m_time = 0.;
      m_frame_num = 0;
      m_play = false;
      m_ui->btn_play->clear();
    }
    else
    {
      m_time -= mocap_selected->duration();
    }
  }
  

  // if we need to draw the reflection
  if(m_draw_reflection) 
  {

    // setup the stencil buffer to only draw on the floor
    glEnable(GL_STENCIL_TEST);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    glStencilFunc(GL_ALWAYS, 3, 0xffffffff);

    // draw the floor with depth test off so we can blend the floor
    // with the reflection skeleton
    glDisable(GL_DEPTH_TEST);
    drawFloor();

    // only draw on the floor
    glStencilFunc(GL_LESS, 2, 0xffffffff); 
    if(m_draw_style == STYLE_STICK)
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    else
      glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    glEnable(GL_DEPTH_TEST);


    // reflect the scene on the y axis so that we can draw the skeleton upside down
    glPushMatrix();
    glScalef(1.0, -1.0, 1.0);
    glTranslatef(0.0, -floorHeight * 2., 0.0);

    glEnable(GL_NORMALIZE);
    //glCullFace(GL_FRONT);
    glFrontFace(GL_CW);

    // blend with the ground
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // draw the skeleton
    glDisable(GL_LIGHTING);
    //glDisable(GL_DEPTH_TEST);
    //glColor4f(.2, 1., .2, .25);
    draw_skeleton(m_time, false, .25);

    glDisable(GL_BLEND);
    glDisable(GL_NORMALIZE);
    //glCullFace(GL_BACK);
    glFrontFace(GL_CCW);


    glPopMatrix();

    glDisable(GL_STENCIL_TEST);


  }

  // setup the stencil buffer for the projection shadow
  if(m_draw_shadow)
  {
    // set the floor object stencil buffer location to 3
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 3, 0xffffffff);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
  }
 
  
 
  // draw the floor
  // if reflection is on, only update the depth buffer since we already drew the floor
  if(m_draw_reflection) glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  drawFloor();
  if(m_draw_reflection) glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glDisable(GL_STENCIL_TEST);


  glDisable(GL_LIGHTING);
  // if we need to draw the colored preview
  if(m_draw_preview)
  {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    for(int i=0; i<6; i++)
    {
      glColor4dv(colors[i]);
      //draw_frame(i * mocap_selected->frames.size() / 6);
      draw_skeleton(i * mocap_selected->frames.size() / 6 * mocap_selected->frame_time, true);
    }
    glDisable(GL_BLEND);
  }


  // if we need to draw the trailing motion
  if(m_draw_trailing_motion)
  {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    for(int i=0; i<6; i++)
    {
      double frame_num_tmp;
      double time_tmp = m_time - (float)i * (1. / (m_play_fps / 6.));
      //double time_tmp = m_time - (float)i * mocap_selected->frame_time;
      modf(time_tmp / mocap_selected->frame_time, &frame_num_tmp);
      int frame_num = (int)frame_num_tmp % mocap_selected->frames.size();
      if(frame_num < m_frame_num)
        draw_skeleton(m_time - (float)i * mocap_selected->frame_time, false, 1. - (float)(i+1) * 1./7.);
    }
    glDisable(GL_BLEND);
  }

  /*
  // this code is not very efficient
  if(m_draw_end_effectors)
  {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // draw the end effectors forward
    Vector3d end_effector_pos;
    Vector3d end_effector_pos_prev;
    int test = mocap_selected->list_end_effectors.size();
    string str = mocap_selected->list_end_effectors[0]->name;
    for(int i = 0; i < mocap_selected->list_end_effectors.size(); i++)
    {
      for(int j=0; j<20; j++)
      {
        double frame_num_tmp;
        double time_tmp = m_time + (float)j * .1;
        modf(time_tmp / mocap_selected->frame_time, &frame_num_tmp);
        int frame_num = (int)frame_num_tmp % mocap_selected->frames.size();

        
        if(i < 10) 
          glColor4dv(colors[i]);
        else
          glColor4f(.2, .2, .8, .5);

        // draw the end effector
        glBegin(GL_LINES);
        if(frame_num >= 0)
        {
          //Figure interpolate current pose
          Frame c_frame = mocap_selected->get_frame(time_tmp, false);

          //Extract matrices
          aligned<Transform3d>::vector xform( mocap_selected->skeleton.size() );
          mocap_selected->skeleton.interpret_pose(
                  xform.begin(), 
                  c_frame.pose.begin(),
                  c_frame.pose.end());

          bool found = compute_end_effector_pos(mocap_selected->skeleton,
            xform.begin(), xform.end(),
            *mocap_selected->list_end_effectors[i],
            end_effector_pos);

          if(j != 0) 
          {
            glVertex3f(end_effector_pos_prev[0], end_effector_pos_prev[1], end_effector_pos_prev[2]);
            glVertex3f(end_effector_pos[0], end_effector_pos[1], end_effector_pos[2]);
          }
          end_effector_pos_prev = end_effector_pos;
          
          
          // *
          if(found)
          {
            glEnable(GL_POINT_SMOOTH);
            glPointSize(5);
            glBegin(GL_POINTS);
              glVertex3f(end_effector_pos[0], end_effector_pos[1], end_effector_pos[2]);
            glEnd();
          }
          // *
        }
        glEnd();

        
      }

      
    }

    glDisable(GL_BLEND);
  }
  */

  if(m_draw_shadow)
  {
    glPushMatrix();


    // if 2 < stencil value, draw so only draw on the floor
    // always replace the stencil value so we don't redraw more than once
    glStencilFunc(GL_LESS, 2, 0xffffffff);  /* draw if ==1 */
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);


    // use polygon offset to resolve any z buffer fighting issues
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0, -1.0);


    // set the color of the shadow to be 50% black
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);  
    glColor4f(0.0, 0.0, 0.0, 0.5);

    // project the object to the ground plane and draw
    findPlane(floorPlaneEquation, floorVertex[0], floorVertex[1], floorVertex[2]);
    projectShadowMatrix(floorPlaneEquation, lightPosition);
    glDisable(GL_DEPTH_TEST);
    draw_skeleton(m_time, true);


    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_STENCIL_TEST);
    glPopMatrix();
  }
     
  
  
  // draw the back of the floor
  glFrontFace(GL_CW);
  glEnable(GL_DEPTH_TEST);
  drawFloor(true);
  glFrontFace(GL_CCW);

  // draw the spline
  draw_spline();

  // draw the skeleton
  //glColor4f(.2, 1., .2, 1.);
  glDisable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  draw_skeleton(m_time);





  // update the slider
  double frame_num;
  double tau = modf(m_time / mocap_selected->frame_time, &frame_num);
  m_ui->m_slider->value(frame_num);
  m_frame_num = (unsigned int)frame_num;


  


}


void glView::display()
{
  //Set window bounds
  glViewport(0, 0, w(), h());

  

  //Clear buffer
  glClearColor(0., 0., 0., 0.);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  //Set up camera
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fov, (double)w() / (double)h(), z_near, z_far);

  //Create base camer matrix from mouse controls
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(-object_center(0), -object_center(1), -object_center(2));
  Matrix4d tr = Transform3d(camera_rot).matrix().transpose();
  glMultMatrixd(tr.data());


  //Do actual drawing code
  drawScene();


}





glView::glView(int x,int y,int w,int h,const char *l)
: Fl_Gl_Window(x,y,w,h,l)

{
  m_play = false;
  m_repeat = false;
  m_frame_num = 0;
  m_ui = NULL;
  m_time = 0.;
  m_play_fps = 30.;
  mocap_list.clear();
  mocap_selected = NULL;
  m_drawing = false;
  m_draw_reflection = true;
  m_draw_shadow = true;
  m_draw_preview = false;
  m_draw_trailing_motion = false;
  m_draw_end_effectors = false;
  m_draw_fps = false;
  m_camera_mode = CAMERA_FREE;
  m_draw_style = STYLE_LINES;
  imgMap = NULL;

  fl_register_images();

  mode(FL_RGB | FL_DOUBLE | FL_DEPTH | FL_STENCIL);



  
  // Make the file chooser...
  Fl_File_Icon::load_system_icons();
  file_chooser = new Fl_File_Chooser(".", "*", Fl_File_Chooser::SINGLE, "Select Motion File");

  

}



void glView::draw() 
{
  // initialize stuff
  static bool firstTime = true;
  if(firstTime)
  {
    initScene();
    firstTime = false;
    reload_concat_times();
  }

  m_drawing = true;


  display();


  time_t curtime;
  char buf[255];
  static time_t fpstime = 0;
  static int fpscount = 0;
  static int fps = 0;

  // draw the fps if necessary
  if(m_draw_fps)
  {
    // get ready to draw FPS
    glDisable(GL_LIGHTING);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, w(), 0.0, h());

    // draw FPS
    sprintf(buf, "FPS=%d", fps);
    glColor3f(1.0f, 0.0f, 0.0f);
    gl_font(FL_HELVETICA_BOLD, 10);
    gl_draw(buf, 10, 10);
    glEnable(GL_LIGHTING);
  }

  
  // Use glFinish() instead of glFlush() to avoid getting many frames
  // ahead of the display (problem with some Linux OpenGL implementations...)
  //
  glFinish();

  // update the camera if necessary
  updateAutoCamera();

  // Update frames-per-second
  fpscount ++;
  curtime = time(NULL);
  if ((curtime - fpstime) >= 2)
  {
    fps      = (fps + fpscount / (curtime - fpstime)) / 2;
    fpstime  = curtime;
    fpscount = 0;
  }

  m_drawing = false;
}


// FlTk Event handler for the window
int glView::handle(int event)
{

  // remember what button was used
  static int last_push;


  switch(event) {
    // handle the mouse down event
    case FL_PUSH:
      if(m_camera_mode != CAMERA_AUTO)
      {
        last_push = Fl::event_button();
        if(last_push == 1)
        {
          moving_arcball = true;
          previous_rotation.setIdentity();
          rotation_pos[0] = Fl::event_x_root();
          rotation_pos[1] = Fl::event_y_root();
          damage(1);
          return 1;
        }
        if(last_push == 3)
        {
          pan_camera = true;
          pan_base[0] = Fl::event_x_root();
          pan_base[1] = Fl::event_y_root();
          damage(1);
          return 1;
        }
        if(last_push == 2)
        {
          zoom_camera = true;
          zoom_base[0] = Fl::event_x_root();
          zoom_base[1] = Fl::event_y_root();
          damage(1);
          return 1;
        }
      }
      break;
    
    // handle the mouse up event
    case FL_RELEASE:
      
      if(last_push == 1)
      {
        moving_arcball = false;
      }
      if(last_push == 3)
      {
        pan_camera = false;
      }
      if(last_push == 2)
      {
        zoom_camera = false;
      }
      damage(1);
      last_push=0;
      return 1;

    // handle the mouse move event
    case FL_DRAG:
      if(moving_arcball)
      {
        int x = Fl::event_x_root(),
            y = Fl::event_y_root();
        camera_rot *= previous_rotation.inverse().normalized();
        
        Vector3d A = Vector3d(
                        -0.5 + (double)x / (double)w(), 
                         0.5 - (double)y / (double)h(), 0.5),
                 B = Vector3d(
                        -0.5 + (double)rotation_pos[0] / (double)w(), 
                         0.5 - (double)rotation_pos[1] / (double)h(), 0.5);
                        
        Quaterniond next;
        next.setFromTwoVectors(A, B).normalize();
        camera_rot *= next;
                
        previous_rotation = next;
        damage(1);
        return 1;
      }
      
      if(pan_camera)
      {
        int x = Fl::event_x_root(),
            y = Fl::event_y_root();
        Vector3d delta = 
          Vector3d((double)(pan_base[0] - x) / w(), 
                    -(double)(pan_base[1] - y) / (double)h(),
                    0.0);
        
        object_center += delta * PAN_RATE;
        
        pan_base[0] = x;
        pan_base[1] = y;
        damage(1);
        return 1;
      }
      
      if(zoom_camera)
      {
        int x = Fl::event_x_root(),
            y = Fl::event_y_root();
        Vector3d delta = 
          Vector3d((double)(zoom_base[0] - x) / w(), 
                    0.,
                    -(double)(zoom_base[1] - y) / (double)h());
        
        object_center += delta * PAN_RATE;
        
        zoom_base[0] = x;
        zoom_base[1] = y;
        damage(1);
        return 1;
      }

      break;

    // in order to get keyboard events, we need to accept focus
    case FL_FOCUS:
      return 1;

    case FL_ENTER:	// every time the mouse enters this window, aggressively take focus
      focus(this);
      break;

    case FL_KEYBOARD:
      int k = Fl::event_key();
      int ks = Fl::event_state();
      break;
  }

  return Fl_Gl_Window::handle(event);
}


void glView::move_frame_to(int value)
{
  if(!mocap_selected) return;

  m_frame_num = value;
  m_play = false;
  m_ui->btn_play->clear();
  m_time = (float)m_frame_num * mocap_selected->frame_time;

  redraw();
}



void glView::load_file()
{

  int	i;			            // Looping var
  int	count;			        // Number of files selected
  char	relative[1024];		// Relative filename


  // apply filter
  const char filters[] = 
    "BVH Files (*.bvh)\t"
    //"PostScript Files (*.ps)\t"
    //"Image Files (*.{bmp,gif,jpg,png})\t"
    //"C/C++ Source Files (*.{c,C,cc,cpp,cxx})"
  ;
  file_chooser->filter(filters);

  // allow multiple files
  file_chooser->type(Fl_File_Chooser::MULTI);
  file_chooser->label("Select Motion File");

  // display dialog until it is closed
  file_chooser->show();
  while (file_chooser->visible()) {
    Fl::wait();
  }

  // process selected files
  bool found = false;
  int index = -1;
  count = file_chooser->count();
  if (count > 0)
  {

    for (i = 1; i <= count; i ++)
    {
      if (!file_chooser->value(i))
        break;

      // get the relative path
      fl_filename_relative(relative, sizeof(relative), file_chooser->value(i));


      // Parse out motion capture data
      ifstream c_in(relative);
      Motion* mocap;
      try
      {
         mocap = new Motion(parseBVH(c_in).convert_quat());
        //FIXME: Need to reimplement using new end_effector code
//         find_end_effectors(mocap->skeleton, mocap->list_end_effectors);
      }
      catch(...)
      {
        
        fl_message("Error: An error occured trying to read the file %s", relative);
        continue;
      }
      mocap_list.push_back(mocap); 

      
      // add the filename to the list
      m_ui->m_browser_file->add(fl_filename_name(file_chooser->value(i)));

      // save  the first selected file index
      if(!found)
      {
        found = true;
        index = mocap_list.size()-1;
      }

    }

  }

  // process the first selected file as the active motion
  if(index != -1)
  {
    select_animation(index);

    // select item
    m_ui->m_browser_file->select(index + 1);
  }
  

  redraw();
}

void glView::remove_file()
{
  // make sure if we are not still drawing
  while(m_drawing)
  {
#ifdef WIN32
    Sleep(10);
#else 
    usleep(10000);
#endif
  }

  
  int index = m_ui->m_browser_file->value() - 1;
  if(index < 0) return;

  // remove the active motion and deselect selection
  mocap_selected = NULL;
  m_ui->m_browser_file->select(0);


  // delete the motion from the list of multiple motion
  Motion *ptrMotion;
  for(int i=1; i<m_ui->m_browser_file_comb->size(); i++)
  {
    ptrMotion = (Motion*)m_ui->m_browser_file_comb->data(i+1);
    if(ptrMotion == mocap_list[index])
    {
      m_ui->m_browser_file_comb->remove(i+1);
      i = 0;
    }
  }
  
  // update the multiple mode if necessary
  if(m_ui->radio_mutliple->value() == 1)
  {
    mode_multiple();
  }

  // delete the motion
  ptrMotion = mocap_list[index];
  mocap_list.erase(mocap_list.begin() + index);
  m_ui->m_browser_file->remove(index + 1);
  delete ptrMotion;

  redraw();
}

void glView::update_selection()
{

  if(m_ui->radio_single->value() != 1) return;
  int index = m_ui->m_browser_file->value() - 1;
  select_animation(index);

  redraw();
}


void glView::select_animation(int index)
{
  if(index < 0 || index >= mocap_list.size()) return;

  // set the active motion capture object
  mocap_selected = mocap_list[index];

  // display some info about the active object
  char text[256];
  m_ui->m_slider->range(0., (double)mocap_selected->frames.size()-1.);
  m_ui->lbl_name->copy_label(m_ui->m_browser_file->text(index+1));
  sprintf(text, "%i", mocap_selected->skeleton.size());
  m_ui->lbl_joints->copy_label(text);
  sprintf(text, "%i", mocap_selected->frames.size());
  m_ui->lbl_frames->copy_label(text);
  sprintf(text, "%i", (int)(1. / mocap_selected->frame_time + .5));
  m_ui->lbl_fps->copy_label(text);
  m_ui->mainWindow->redraw();
  //m_ui->edit_desired_fps->value((int)(1. / mocap_selected->frame_time + .5));


  // stop playing
  m_time = 0.;
  m_frame_num = 0;
  m_play = false;
  m_ui->btn_play->clear();

  // update the default camera position
  updateCamera();
}


void glView::add_animation_list()
{
  // make sure index is valid
  int index = m_ui->m_browser_file->value() - 1;
  if(index < 0 || index >= mocap_list.size()) return;

  // validate motion have same number of joints
  if(m_ui->m_browser_file_comb->size() >= 2)
  {
    Motion* ptrMotion = (Motion*)m_ui->m_browser_file_comb->data(2);
    if( !ptrMotion || (ptrMotion->skeleton.size() != mocap_list[index]->skeleton.size()) )
    {
      fl_message("Error: The number of joints in the motion did not match.");
      return;
    }
  }
  
  // add the item to the list of animation to contatenate
  char text[256];
  sprintf(text, 
    "%s\t0\t%i\t0",
    m_ui->m_browser_file->text(index + 1),
    mocap_list[index]->frames.size()-1);
  m_ui->m_browser_file_comb->add(
    text,
    mocap_list[index]
    );

  // update the multiple mode if necessary
  if(m_ui->radio_mutliple->value() == 1)
  {
    mode_multiple();
  }

  reload_concat_times();
}


void glView::del_animation_list()
{
  int index = m_ui->m_browser_file_comb->value();
  if(index <= 1) return;

  // delete the item from the list
  m_ui->m_browser_file_comb->remove(index);

  // update the multiple mode if necessary
  if(m_ui->radio_mutliple->value() == 1)
  {
    mode_multiple();
  }

  reload_concat_times();
}



void glView::mode_single()
{
  // stop playing
  m_time = 0.;
  m_frame_num = 0;
  m_play = false;
  m_ui->btn_play->clear();

  // make sure if we are not still drawing
  while(m_drawing)
  {
#ifdef WIN32
    Sleep(10);
#else 
    usleep(10000);
#endif
  }

  // active the selected file 
  mocap_selected = NULL;
  update_selection();
}



void glView::mode_multiple()
{
  // stop playing
  m_time = 0.;
  m_frame_num = 0;
  m_play = false;
  m_ui->btn_play->clear();

  // make sure the number of files is valid
  int num_files = m_ui->m_browser_file_comb->size();
  if(num_files < 1) return; // TODO add error message
  
  // make sure if we are not still drawing
  while(m_drawing)
  {
#ifdef WIN32
    Sleep(10);
#else 
    usleep(10000);
#endif
  }

  // active the selected file 
  mocap_selected = NULL;


  // set the first item to be the start of the motion
  Motion*  ptrMotion = (Motion*)m_ui->m_browser_file_comb->data(2);
  if(ptrMotion == NULL) return;  // TODO add error message


  string str;
  char *word;
  double next_duration;
  double a_start, a_end, b_start, b_end;
  double duration;

  // parse the string text from the row for first motion
  Motion* a = (Motion*)m_ui->m_browser_file_comb->data(2);
  str = m_ui->m_browser_file_comb->text(2);
  word = strtok((char*)str.c_str(), "\t");
  word = strtok(NULL, "\t");
  a_start = atof(word) * a->frame_time;
  word = strtok(NULL, "\t");
  a_end = atof(word) * a->frame_time;
  word = strtok(NULL, "\t");
  duration = atof(word) * a->frame_time;

  // copy the first motion section
  mocap_combine.skeleton = ptrMotion->skeleton;
  mocap_combine.frame_time = ptrMotion->frame_time;
  mocap_combine.frames.resize(0);
  for(double t = a_start; t<=a_end; t+=mocap_combine.frame_time)
    mocap_combine.frames.push_back(ptrMotion->get_frame(t));



  // loop through the list of animation to contatenate
  for(int i = 2; i < num_files; i++)
  {

    Motion* a = &mocap_combine;
    Motion* b = (Motion*)m_ui->m_browser_file_comb->data(i+1);
    if(b == NULL) return;  // TODO add error message


    a_start = 0.;
    a_end = (a->frames.size()) * a->frame_time;


    // parse the string text from the row for second motion
    str = m_ui->m_browser_file_comb->text(i+1);
    word = strtok((char*)str.c_str(), "\t");
    word = strtok(NULL, "\t");
    b_start = atof(word) * b->frame_time;
    word = strtok(NULL, "\t");
    b_end = atof(word) * b->frame_time;
    word = strtok(NULL, "\t");
    next_duration = atof(word) * b->frame_time;

                                            
    
    mocap_combine = combine_motions(*a, *b, 
                                    relative_xform(a->skeleton, 
                                                   a->get_frame(a_end - duration), 
                                                   b->get_frame(b_start)),
                                    a_start, a_end,
                                    b_start, b_end,
                                    duration, 1);

    duration = next_duration;
  }

  
  mocap_selected = &mocap_combine;
  
  // display some info about the active object
  char text[256];
  m_ui->m_slider->range(0., (double)mocap_selected->frames.size()-1);
  m_ui->lbl_name->copy_label("Animation Contatenation");
  sprintf(text, "%i", mocap_selected->skeleton.size());
  m_ui->lbl_joints->copy_label(text);
  sprintf(text, "%i", mocap_selected->frames.size());
  m_ui->lbl_frames->copy_label(text);
  sprintf(text, "%i", (int)(1. / mocap_selected->frame_time + .5));
  m_ui->lbl_fps->copy_label(text);
  m_ui->mainWindow->redraw();

  
  /* //Not needed anymore I think -Mik
  
  // compute the bounding box for the skeleton
  Vector3d offset = mocap_selected->skeleton.offset;
  Vector3d min_pt = offset;
  Vector3d max_pt = offset;
  
  mocal_selected->bounding_box(min_pt, max_pt);
  
  compute_bounding_box(offset, 
                       mocap_selected->skeleton, 
                       min_pt, 
                       max_pt);

  // compute the bounding box for the motion
  compute_bounding_box(*mocap_selected,
                       mocap_selected->bound_box_min,
                       mocap_selected->bound_box_max);

  // compute the bounding sphere radius
  mocap_selected->bound_sphere_radius = sqrt(max(min_pt.dot(min_pt), max_pt.dot(max_pt)));
  */
  
  // update the default camera position
  updateCamera();


}

// update the default camera to a distance away from the motion's bounding sphere
void glView::updateCamera()
{

  if(!mocap_selected) return;


  //Figure interpolate current pose
  Frame c_frame = mocap_selected->frames[m_frame_num];

  //Extract matrices
  aligned<Transform3d>::vector xform = c_frame.local_xform(mocap_selected->skeleton);
  
  Transform3d xform_sphere;
  xform_sphere.setIdentity();


  if(m_camera_mode == CAMERA_AUTO)
  {
    // fix the center to be the skeleton center
    xform_sphere.translate(mocap_selected->skeleton.offset);
    xform_sphere = xform_sphere * xform[0];
    //xform_sphere.translate(Vector3d(0, 0, 0));

    // move the z back 5 times the bounding sphere
    Vector3d offset = xform_sphere.translation();
    
    offset[2] += mocap_selected->bound_sphere_radius_skeleton() * 5.;
    
    // update the object center and camera rotation
    object_center = offset;
    camera_rot.setIdentity();
  }
  else
  {
    Vector3d lo, hi;
    mocap_selected->bounding_box(lo, hi);
    
    // fix the center to be the center of the bounding box of the motion
    Vector3d mid_pt = (lo + hi) / 2.;
    xform_sphere.translate(mid_pt);

    // calculate the distance to move back
    Vector3d diag_box = (hi - lo) / 2.;
    float half_diag = sqrt(diag_box.dot(diag_box));
    float aspect_ratio = (float)w() / (float)h();
    float fov_x = fov * aspect_ratio;
    float dist = half_diag / tan(fov_x/2. * M_PI / 180.);
    xform_sphere.translate(Vector3d(0, 10, dist));

    // move the z back 5 times the bounding sphere + radius
    Vector3d offset = xform_sphere.translation();
    offset[2] += mocap_selected->bound_sphere_radius_skeleton() * 5.;
   
    // update the object center and camera rotation
    object_center = offset;
    camera_rot.setIdentity();

  }

  // update the floor height
  Vector3d min_pt = Vector3d(0., 0., 0.);
  Vector3d max_pt = Vector3d(0., 0., 0.);
  
  mocap_selected->bounding_box(min_pt, max_pt);

/*  
  Transform3d xformPose;
  xformPose.setIdentity();
  compute_bounding_box( xformPose, 
                        mocap_selected->skeleton, 
                        xform.begin(), 
                        xform.end(),
                        min_pt, 
                        max_pt);
*/
  
  floorHeight = min_pt[1];
  floorVertex[0][1] = floorHeight;
  floorVertex[1][1] = floorHeight;
  floorVertex[2][1] = floorHeight;
  floorVertex[3][1] = floorHeight;

}


// update the camera position automatically
void glView::updateAutoCamera()
{
  // make sure we need to update the camera
  if(m_camera_mode == CAMERA_FREE) return;
  if(!mocap_selected) return;
  
 

  if(m_camera_mode == CAMERA_AUTO)
  {

    //Figure interpolate current pose
    Frame c_frame = mocap_selected->frames[m_frame_num];

    //Extract matrices
    aligned<Transform3d>::vector xform = c_frame.local_xform(mocap_selected->skeleton);

    Transform3d xform_sphere;
    xform_sphere.setIdentity();

    
    // fix the center to be the skeleton center
    xform_sphere.translate(mocap_selected->skeleton.offset);
    xform_sphere = xform_sphere * xform[0];
    //xform_sphere.translate(Vector3d(0, 0, 0));

    // move the z back 5 times the bounding sphere
    Vector3d offset = xform_sphere.translation();
    offset[2] += mocap_selected->bound_sphere_radius_skeleton() * 5.;
    
    // update the object center and camera rotation
    Vector3d diff = offset - object_center;
    float diff_length = sqrt(diff.dot(diff));
    float radius = mocap_selected->bound_sphere_radius_skeleton();
    if(diff_length > radius)
    {
      // add the difference from the offset to the point on the sphere
      diff.normalize();
      diff *= radius;
      object_center += (offset - (object_center + diff));
    }
    camera_rot.setIdentity();
  }


}

void glView::reload_concat_times()
{

  if(!mocap_selected) return;
  int index = m_ui->m_browser_file_comb->value() - 1;
  if(index < 1)
  {
    m_ui->lbl_start_time->copy_label("");
    m_ui->lbl_end_time->copy_label("");
    m_ui->lbl_blend_time->copy_label("");
    m_ui->edit_num_blend_frames->value(0);
    m_ui->edit_start_frame->value(0);
    m_ui->edit_end_frame->value(0);
    m_ui->edit_num_blend_frames->deactivate();
    m_ui->edit_start_frame->deactivate();
    m_ui->edit_end_frame->deactivate();
  }
  else
  {
    // get the motion data
    Motion* ptrMotion = (Motion*)m_ui->m_browser_file_comb->data(index+1);
    if(!ptrMotion) return;

    
    // parse the string text from the row
    string str = m_ui->m_browser_file_comb->text(index + 1);
    string strFile, strStart, strEnd, strBlend;
    char *word;
    word = strtok((char*)str.c_str(), "\t");
    strFile = word;
    word = strtok(NULL, "\t");
    strStart = word;
    word = strtok(NULL, "\t");
    strEnd = word;
    word = strtok(NULL, "\t");
    strBlend = word;

    char text[256];
    m_ui->edit_start_frame->maximum(ptrMotion->frames.size()-1);
    m_ui->edit_end_frame->maximum(ptrMotion->frames.size()-1);
    m_ui->edit_num_blend_frames->maximum(ptrMotion->frames.size()-1);
    sprintf(text, "%.2f", (float)atoi(strBlend.c_str()) * ptrMotion->frame_time);
    m_ui->lbl_blend_time->copy_label(text);
    sprintf(text, "%.2f", (float)atoi(strStart.c_str()) * ptrMotion->frame_time);
    m_ui->lbl_start_time->copy_label(text);
    sprintf(text, "%.2f", (float)atoi(strEnd.c_str()) * ptrMotion->frame_time);
    m_ui->lbl_end_time->copy_label(text);
    m_ui->edit_num_blend_frames->value(atoi(strBlend.c_str()));
    m_ui->edit_start_frame->value(atoi(strStart.c_str()));
    m_ui->edit_end_frame->value(atoi(strEnd.c_str()));
    m_ui->edit_num_blend_frames->activate();
    m_ui->edit_start_frame->activate();
    m_ui->edit_end_frame->activate();
  }

  m_ui->mainWindow->redraw();

}

void glView::update_concat_times()
{
  // make sure index is valid
  if(!mocap_selected) return;
  int index = m_ui->m_browser_file_comb->value() - 1;
  if(index < 1) return;
  

  // parse the string text from the row
  string str = m_ui->m_browser_file_comb->text(index + 1);
  string strFile, strStart, strEnd, strBlend;
  char *word;
  word = strtok((char*)str.c_str(), "\t");
  strFile = word;
  word = strtok(NULL, "\t");
  strStart = word;
  word = strtok(NULL, "\t");
  strEnd = word;
  word = strtok(NULL, "\t");
  strBlend = word;

  
  // add the item to the list of animation to contatenate
  char text[256];
  sprintf(text, 
          "%s\t%i\t%i\t%i",
          strFile.c_str(),
          (int)m_ui->edit_start_frame->value(),
          (int)m_ui->edit_end_frame->value(),
          (int)m_ui->edit_num_blend_frames->value());
  m_ui->m_browser_file_comb->text(index + 1, text);

  // update the durations
  sprintf(text, "%.2f", m_ui->edit_num_blend_frames->value() * mocap_selected->frame_time);
  m_ui->lbl_blend_time->copy_label(text);
  sprintf(text, "%.2f", m_ui->edit_start_frame->value() * mocap_selected->frame_time);
  m_ui->lbl_start_time->copy_label(text);
  sprintf(text, "%.2f", m_ui->edit_end_frame->value() * mocap_selected->frame_time);
  m_ui->lbl_end_time->copy_label(text);
  m_ui->mainWindow->redraw();

  // recompute the motion
  mode_multiple();

}


void glView::save_file()
{
  if(!mocap_selected) return;

  // snippet take from fltk example
  const char *c = NULL;
  fl_file_chooser_ok_label("Save");
  c=fl_file_chooser("Save To:", "BVH Files (*.bvh)", c);
  fl_file_chooser_ok_label(NULL);
  if (!c) return;

  if (!access(c, 0))
  {
    const char *basename;
    if ((basename = strrchr(c, '/')) != NULL)
      basename ++;
#if defined(WIN32)
    if ((basename = strrchr(c, '\\')) != NULL)
      basename ++;
#endif 
    else
      basename = c;

    if (fl_choice("The file \"%s\" already exists.\n"
                  "Do you want to replace it?", "Cancel",
	    "Replace", NULL, basename) == 0) return;
  }

  // write the bvh file
  ofstream bvh_file(c);
  writeBVH(bvh_file, (const Motion&)*mocap_selected); 

}

//Windowing function
double hann_window(double t)
{
	return 0.5 * (1. - cos(M_PI * 2. * t));
}

void glView::debugFunction1()
{
  
}

void glView::debugFunction2()
{
  
}

void glView::debugFunction3()
{
}

void glView::debugFunction4()
{
}

void glView::debugFunction5()
{
}


void glView::mode_motion_graph()
{
    mocap_selected = NULL;
    m_time = 0.;
    m_frame_num = 0;
    m_play = false;
    m_ui->btn_play->clear();
    m_ui->motionGraphWindow->show();
}

void glView::mode_path_finding()
{
    mocap_selected = NULL;
    m_time = 0.;
    m_frame_num = 0;
    m_play = false;
    m_ui->btn_play->clear();
    m_ui->pathFindingWindow->show();
}

void glView::load_motion_graph()
{
  int	count;			        // Number of files selected
  char	relative[1024];		    // Relative filename


  // apply filter
  const char filters[] = "Motion Graph Files (*)\t";
  file_chooser->filter(filters);

  // allow single file
  file_chooser->type(Fl_File_Chooser::SINGLE);
  file_chooser->label("Select Motion Graph");

  // display dialog until it is closed
  file_chooser->show();
  while (file_chooser->visible()) {
    Fl::wait();
  }

  // process selected file
  int index = -1;
  count = file_chooser->count();
  if (count > 0)
  {

      // get the relative path
      fl_filename_relative(relative, sizeof(relative), file_chooser->value());

      // Parse out motion graph
      ifstream c_in(relative);
      try
      {
         motion_graph = parseMotionGraph(c_in);
         reset_mg_map();
         update_mg_info();
      }
      catch(...)
      {
        fl_message("Error: An error occured trying to read the file %s", relative);
      }

  }

}

void glView::save_motion_graph()
{

  // snippet take from fltk example
  const char *c = NULL;
  fl_file_chooser_ok_label("Save");
  c=fl_file_chooser("Save To:", "Motion Graph Files (*)\t", c);
  fl_file_chooser_ok_label(NULL);
  if (!c) return;

  if (!access(c, 0))
  {
    const char *basename;
    if ((basename = strrchr(c, '/')) != NULL)
      basename ++;
#if defined(WIN32)
    if ((basename = strrchr(c, '\\')) != NULL)
      basename ++;
#endif 
    else
      basename = c;

    if (fl_choice("The file \"%s\" already exists.\n"
                  "Do you want to replace it?", "Cancel",
	    "Replace", NULL, basename) == 0) return;
  }

  // write the motion graph file
  ofstream bvh_file(c);
  writeMotionGraph(bvh_file, motion_graph); 


}

void glView::reset_mg_map()
{
    imgCloudMapData.resize(1 * 1 * 3);
    imgCloudMapData[0] = 0;
    imgCloudMapData[1] = 0;
    imgCloudMapData[2] = 0;
    Fl_RGB_Image* imgCloudMap = new Fl_RGB_Image(&imgCloudMapData[0], 1, 1);
    imgMap = imgCloudMap->copy(400, 400);
    m_ui->boxPointCloudMap->image(imgMap);
    delete imgCloudMap;
    
    m_ui->lbl_delta->copy_label("");
    m_ui->motionGraphWindow->redraw();
    m_ui->viewPointCloud->redraw();
}

void glView::recompute_mg()
{

    

    m_ui->boxPointCloudMap->image(NULL);
    if(imgMap)
    {
      delete imgMap;
      imgMap = NULL;
    }

    if(motion_graph.frames.size() == 0) 
    {
        reset_mg_map();
        return;
    }
    double max_distance;
    motion_graph.create_point_cloud_map(dataCloudMap, max_distance, motion_graph.frame_time * 5.,  5, hann_window);

    int rows = dataCloudMap.rows(),
        cols = dataCloudMap.cols();
    imgCloudMapData.resize(rows * cols * 3);



    long index = 0;
    unsigned char value;
    for(int i = 0; i < rows; i++)
    {
        index = (rows - 1 - i) * cols * 3;
        for(int j = 0; j < cols; j++)
        {
            unsigned char value = (unsigned char)(dataCloudMap(i,j) / max_distance * 255.);
            imgCloudMapData[index++] = value;
            imgCloudMapData[index++] = value;
            imgCloudMapData[index++] = value;
        }
    }

    Fl_RGB_Image* imgCloudMap = new Fl_RGB_Image(&imgCloudMapData[0], rows, cols);
    imgMap = imgCloudMap->copy(400, 400);
    m_ui->boxPointCloudMap->image(imgMap);

    delete imgCloudMap;
    m_ui->motionGraphWindow->redraw();

}

void glView::insert_motion_mg()
{

  int	i;			            // Looping var
  int	count;			        // Number of files selected
  char	relative[1024];		// Relative filename


  // apply filter
  const char filters[] = 
    "BVH Files (*.bvh)\t"
    //"PostScript Files (*.ps)\t"
    //"Image Files (*.{bmp,gif,jpg,png})\t"
    //"C/C++ Source Files (*.{c,C,cc,cpp,cxx})"
  ;
  file_chooser->filter(filters);

  // allow multiple files
  file_chooser->type(Fl_File_Chooser::MULTI);
  file_chooser->label("Select Motion File");

  // display dialog until it is closed
  file_chooser->show();
  while (file_chooser->visible()) {
    Fl::wait();
  }

  double treshold = atof(m_ui->inputCloudTreshold->value());

  // process selected files
  bool found = false;
  int index = -1;
  count = file_chooser->count();
  if (count > 0)
  {

    for (i = 1; i <= count; i ++)
    {
      if (!file_chooser->value(i))
        break;

      // get the relative path
      fl_filename_relative(relative, sizeof(relative), file_chooser->value(i));


      // Parse out motion capture data
      ifstream c_in(relative);
      try
      {
         Motion mocap = parseBVH(c_in).convert_quat();

         if(motion_graph.frames.size() == 0)
         {
            motion_graph.skeleton = mocap.skeleton;
	        motion_graph.frame_time = mocap.frame_time;
         }

         if(m_ui->checkGradientDescent->value() == 1)
           motion_graph.insert_motion_fast(mocap, treshold, motion_graph.frame_time * 5.,  5, hann_window);
         else
           motion_graph.insert_motion(mocap, treshold, motion_graph.frame_time * 5.,  5, hann_window);

      }
      catch(...)
      {
        
        fl_message("Error: An error occured trying to read the file %s", relative);
        continue;
      }

    }

    
    reset_mg_map();
    update_mg_info();
  }


  
}

void glView::mg_extract_ssc()
{
    if(motion_graph.frames.size() == 0) return;

    // use the motion graph with the most frames
    vector<MotionGraph> motion_list = motion_graph.extract_scc();
    if(motion_list.size() > 0)
    {
      	int max_graph = 0;
	
	      for(int i=1; i<motion_list.size(); i++)
		      if(motion_list[i].frames.size() > motion_list[max_graph].frames.size())
			      max_graph = i;

        motion_graph = motion_list[max_graph];
        reset_mg_map();
        update_mg_info();
    }
}

void glView::synthesize_motion()
{
    if(motion_graph.frames.size() > 0)
    {
        mocap_random = motion_graph.random_motion(atoi(m_ui->inputRandomFrames->value()));
        mocap_selected = &mocap_random;

        
        // display some info about the active object
        char text[256];
        m_ui->m_slider->range(0., (double)mocap_selected->frames.size()-1.);
        m_ui->lbl_name->copy_label("RANDOM MOTION");
        sprintf(text, "%i", mocap_selected->skeleton.size());
        m_ui->lbl_joints->copy_label(text);
        sprintf(text, "%i", mocap_selected->frames.size());
        m_ui->lbl_frames->copy_label(text);
        sprintf(text, "%i", (int)(1. / mocap_selected->frame_time + .5));
        m_ui->lbl_fps->copy_label(text);
        m_ui->mainWindow->redraw();
    }
}

void glView::create_mg_files()
{
    if(mocap_list.size() == 0) return;
    double treshold = atof(m_ui->inputCloudTreshold->value());
   
    MotionGraph graph(mocap_list[0]->skeleton);
	  graph.frame_time = mocap_list[0]->frame_time;

    for(int i = 0; i < mocap_list.size(); i++)
    {
        if(mocap_list[0]->skeleton.size() == mocap_list[i]->skeleton.size())
        {
          if(m_ui->checkGradientDescent->value() == 1)
            graph.insert_motion_fast(*mocap_list[i], treshold, graph.frame_time * 5.,  5, hann_window);
          else
            graph.insert_motion(*mocap_list[i], treshold, graph.frame_time * 5.,  5, hann_window);
        }

    }

    //vector<MotionGraph> graph_list = graph.extract_scc();
    //graph = graphlist[0];
    motion_graph = graph;

    reset_mg_map();
    update_mg_info();
    
}

void glView::update_mg_info()
{

    // display some info about the motion graph
    char text[256];
    sprintf(text, "%i", motion_graph.skeleton.size());
    m_ui->lbl_joints_mg->copy_label(text);
    sprintf(text, "%i", motion_graph.frames.size());
    m_ui->lbl_frames_mg->copy_label(text);
    sprintf(text, "%i", motion_graph.frames.size() == 0 ? 0 : (int)(1. / motion_graph.frame_time + .5));
    m_ui->lbl_fps_mg->copy_label(text);

    // update the camera pos
    if(motion_graph.frames.size() > 0)
    {
      // compute the camera position to be 5 * radius of bounding sphere of skeleton
      Transform3d base_pose;
      base_pose.setIdentity();
      Frame frame = motion_graph.frames[0].apply_transform(motion_graph.skeleton, base_pose);
      aligned<Transform3d>::vector xform = frame.local_xform(motion_graph.skeleton);
      base_pose.translate(motion_graph.skeleton.offset);
      base_pose = base_pose * xform[0];
      Vector3d offset = base_pose.translation();
      offset[2] += motion_graph.frames[0].bound_sphere_radius(motion_graph.skeleton) * 5.;
      
      // update the object center and camera rotation
      m_ui->viewPointCloud->m_object_center = offset;
      m_ui->viewPointCloud->m_camera_rot.setIdentity();
    }

    m_ui->edit_x_frame->value(0);
    m_ui->edit_y_frame->value(0);
    m_ui->edit_x_frame->maximum(motion_graph.frames.size()-1);
    m_ui->edit_y_frame->maximum(motion_graph.frames.size()-1);
    m_ui->boxPointCloudMap->selectPointCloud();
    
    m_ui->motionGraphWindow->redraw();
 
}

void glView::clear_motion_graph()
{
    MotionGraph graph;
    motion_graph = graph;
    reset_mg_map();
    update_mg_info();
}


//Spline stuff
void read_control_points(istream& data)
{
	int n_points;
  aligned<Vector3f>::vector control_points_tmp;
	if(!(data >> n_points) || n_points <= 0)
		throw "Invalid number of control points for spline curve";
		
	control_points_tmp.resize(n_points);
	for(int i=0; i<n_points; i++)
	{
		if(!(data >> control_points_tmp[i].x() >> control_points_tmp[i].y() >> control_points_tmp[i].z()) )
			throw "Invalid control point";
	}
	
	//Validate data (assert that path is monotonic)
	assert(control_points_tmp[0].z() == 0.);
	for(int i=1; i<n_points; i++)
		assert(control_points_tmp[i].z() >= control_points_tmp[i-1].z());
		
  
  control_points = control_points_tmp;
	t_max = control_points[control_points.size() - 1].z();
}


//deCasteljau's algorithm
Vector3f eval_spline(float t)
{
	aligned<Vector3f>::vector tmp(control_points.size());
	copy(control_points.begin(), control_points.end(), tmp.begin());
	
	for(int i=control_points.size()-1; i>0; i--)
	for(int j=0; j<i; j++)
	{
		tmp[j] = tmp[j] * (1. - t) + tmp[j+1] * t;
	}

	return tmp[0];
}

//Evaluates the spline at some point in time
Vector2f eval_time_spline(double t)
{
	assert(t >= 0. && t <= t_max);
	
	//Do a binary search to find t coordinate for time value
	float lo = 0., hi = 1.;
	Vector3f pt;
	while(abs(lo - hi) > 1e-6)
	{
		float m = lo + (hi - lo) * .5;
		pt = eval_spline(m);
		
		if(pt.z() > t)
			hi = m;
		else
			lo = m;
	}
	
	//Return the value of the spline at this time
	return Vector2f(pt.x(), pt.y());
}



void glView::load_motion_graph_path()
{
  int	count;			        // Number of files selected
  char	relative[1024];		    // Relative filename


  // apply filter
  const char filters[] = "Motion Graph Files (*)\t";
  file_chooser->filter(filters);

  // allow single file
  file_chooser->type(Fl_File_Chooser::SINGLE);
  file_chooser->label("Select Motion Graph");

  // display dialog until it is closed
  file_chooser->show();
  while (file_chooser->visible()) {
    Fl::wait();
  }

  // process selected file
  int index = -1;
  count = file_chooser->count();
  if (count > 0)
  {

      // get the relative path
      fl_filename_relative(relative, sizeof(relative), file_chooser->value());

      // Parse out motion graph
      ifstream c_in(relative);
      try
      {
         motion_graph_path = parseMotionGraph(c_in);

         // display some info about the motion graph
         char text[256];
         m_ui->lbl_file_spline->copy_label(fl_filename_name(file_chooser->value()));
         sprintf(text, "%i", motion_graph_path.skeleton.size());
         m_ui->lbl_joints_spline->copy_label(text);
         sprintf(text, "%i", motion_graph_path.frames.size());
         m_ui->lbl_frames_spline->copy_label(text);
         sprintf(text, "%i", motion_graph_path.frames.size() == 0 ? 0 : (int)(1. / motion_graph_path.frame_time + .5));
         m_ui->lbl_fps_spline->copy_label(text);
      }
      catch(...)
      {
        fl_message("Error: An error occured trying to read the file %s", relative);
      }

  }

}

bool glView::apply_changes_spline()
{
  // try to read in the changes
  string text(textSpline.text());
  istringstream iss(text,istringstream::in);

  try
  {
     read_control_points(iss);
     m_ui->pathFindingWindow->redraw();
  }
  catch(...)
  {
    fl_message("Error: The spline file is invalid.");
    return false;
  }
  
  return true;
}

void glView::save_spline_file()
{
  // make sure the spline file is valid before saving
  bool valid = apply_changes_spline();
  if(!valid) return;


  // snippet take from fltk example
  const char *c = NULL;
  fl_file_chooser_ok_label("Save");
  c=fl_file_chooser("Save To:", "Motion Spline Files (*)\t", c);
  fl_file_chooser_ok_label(NULL);
  if (!c) return;

  if (!access(c, 0))
  {
    const char *basename;
    if ((basename = strrchr(c, '/')) != NULL)
      basename ++;
#if defined(WIN32)
    if ((basename = strrchr(c, '\\')) != NULL)
      basename ++;
#endif 
    else
      basename = c;

    if (fl_choice("The file \"%s\" already exists.\n"
                  "Do you want to replace it?", "Cancel",
	    "Replace", NULL, basename) == 0) return;
  }

  // save the spline file
  textSpline.savefile(c);


}

void glView::load_spline_file()
{

  int	count;			        // Number of files selected
  char	relative[1024];		    // Relative filename


  // apply filter
  const char filters[] = "Spline Files (*)\t";
  file_chooser->filter(filters);

  // allow single file
  file_chooser->type(Fl_File_Chooser::SINGLE);
  file_chooser->label("Select Spline File");

  // display dialog until it is closed
  file_chooser->show();
  while (file_chooser->visible()) {
    Fl::wait();
  }

  // process selected file
  int index = -1;
  count = file_chooser->count();
  if (count > 0)
  {

      // get the relative path
      fl_filename_relative(relative, sizeof(relative), file_chooser->value());

      // Parse out spline file
      ifstream c_in(relative);
      try
      {
         read_control_points(c_in);
         textSpline.loadfile(relative);
         m_ui->pathFindingWindow->redraw();
      }
      catch(...)
      {
        fl_message("Error: An error occured trying to read the file %s", relative);
      }

  }


}

void glView::draw_spline()
{
  if(control_points.size() == 0) return;
  float height = floorHeight + 1.;

  
  
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glDisable(GL_LIGHTING);
  glColor3f(1., 1., 0.);
  glLineWidth(5.);

  // draw the path
  glBegin(GL_LINES);
  float interval = 1. / 30.;
  Vector2f last_pos, pos;
  last_pos = eval_time_spline(0);
  for(float time = interval; time < t_max; time += interval)
  {
    pos = eval_time_spline(time);
    glVertex3f(last_pos[0], height, last_pos[1]);
    glVertex3f(pos[0], height, pos[1]);
    last_pos = pos;
  }
  glEnd();

  /* 
  // draw the control points
  glColor3f(0., 1., 1.);
  glEnable(GL_POINT_SMOOTH);
  glPointSize(7.);
  for(int i = 0; i < control_points.size(); i++)
  {
    glBegin(GL_POINTS);
      glVertex3d(control_points[i].x(), height, control_points[i].y());
    glEnd();
  }
  */
 
  glPopAttrib();
}

void glView::clear_spline()
{
  control_points.clear();
  textSpline.text("");
}

void glView::follow_path()
{

    if(control_points.size() == 0) return;
    if(motion_graph_path.frames.size() == 0) return;

    double max_d = atof(m_ui->inputDistanceSpline->value());
	mocap_path = motion_graph_path.follow_path(&eval_time_spline, max_d, t_max);

    if(mocap_path.frames.size() == 0)
    {
        fl_message("No path could be generated.");
        return;
    }
    mocap_selected = &mocap_path;

    
    // display some info about the active object
    char text[256];
    m_ui->m_slider->range(0., (double)mocap_selected->frames.size()-1.);
    m_ui->lbl_name->copy_label("PATH FINDING MOTION");
    sprintf(text, "%i", mocap_selected->skeleton.size());
    m_ui->lbl_joints->copy_label(text);
    sprintf(text, "%i", mocap_selected->frames.size());
    m_ui->lbl_frames->copy_label(text);
    sprintf(text, "%i", (int)(1. / mocap_selected->frame_time + .5));
    m_ui->lbl_fps->copy_label(text);
    m_ui->mainWindow->redraw();
}


void glView::follow_path_segment()
{
    
    if(motion_graph_path.frames.size() == 0) return;

    Vector2f start = Vector2f(atof(m_ui->inputStartX->value()), atof(m_ui->inputStartY->value())),
             finish = Vector2f(atof(m_ui->inputEndX->value()), atof(m_ui->inputEndY->value()));
	
	
	Transform2f xform;
	xform.setIdentity();
	mocap_path = motion_graph_path.path_segment(xform.translate(start), finish);


    if(mocap_path.frames.size() == 0)
    {
        fl_message("No path could be generated.");
        return;
    }
    mocap_selected = &mocap_path;

    
    // display some info about the active object
    char text[256];
    m_ui->m_slider->range(0., (double)mocap_selected->frames.size()-1.);
    m_ui->lbl_name->copy_label("PATH FINDING MOTION");
    sprintf(text, "%i", mocap_selected->skeleton.size());
    m_ui->lbl_joints->copy_label(text);
    sprintf(text, "%i", mocap_selected->frames.size());
    m_ui->lbl_frames->copy_label(text);
    sprintf(text, "%i", (int)(1. / mocap_selected->frame_time + .5));
    m_ui->lbl_fps->copy_label(text);
    m_ui->mainWindow->redraw();
}


// ***** PointCloudMap ******
void PointCloudMap::draw()
{
    Fl_Box::draw();
    fl_color(FL_RED); 
    fl_rect(this->x() + pt_x - 3,
            this->y() + pt_y - 3,
            5,
            5);
    //fl_line(
}

// FlTk Event handler for the window
int PointCloudMap::handle(int event)
{



  switch(event) {
    // handle the mouse down event
    case FL_PUSH:
        take_focus ();
        if(Fl::event_button() == FL_LEFT_MOUSE)
        {
            pt_x = Fl::event_x()-this->x();
            pt_y = Fl::event_y()-this->y();
            updatePointCloud();
        } 
        return 1;
    
    // handle the mouse up event
    case FL_RELEASE:
        break;

    // handle the mouse move event
    case FL_DRAG:
        if(Fl::event_button() == FL_LEFT_MOUSE)
        {
            pt_x = Fl::event_x()-this->x();
            pt_y = Fl::event_y()-this->y();
            updatePointCloud();
        }
        return 1;


    case FL_FOCUS :
    case FL_UNFOCUS :
        return 1;


    case FL_KEYBOARD:
       {
          switch (Fl::event_key()) 
          {
              case FL_Up:
                  {
                      int value = min((int)view->m_ui->edit_y_frame->value() + 1,(int)view->m_ui->edit_y_frame->maximum());
                      view->m_ui->edit_y_frame->value(value);
                  }
                  break;

              case FL_Down:
                  {
                      int value = max((int)view->m_ui->edit_y_frame->value() - 1,0);
                      view->m_ui->edit_y_frame->value(value);
                  }
                  break;

              case FL_Left:
                  {
                      int value = max((int)view->m_ui->edit_x_frame->value() - 1,0);
                      view->m_ui->edit_x_frame->value(value);
                  }
                  break;

              case FL_Right:
                  {
                      int value = min((int)view->m_ui->edit_x_frame->value() + 1,(int)view->m_ui->edit_x_frame->maximum());
                      view->m_ui->edit_x_frame->value(value);
                  }
                  break;
          }
          selectPointCloud();
       }
       return 1;


  }

  return Fl_Box::handle(event);
}

void PointCloudMap::updateDelta()
{
    int x_frame = view->m_ui->edit_x_frame->value(),
        y_frame = view->m_ui->edit_y_frame->value();

    // compute the delta
    aligned<Vector4d>::vector cloud = 
        view->motion_graph.point_cloud(x_frame, view->motion_graph.frame_time * 5.,  5, hann_window); 
    aligned<Vector4d>::vector dest_cloud = 
        view->motion_graph.point_cloud(y_frame, view->motion_graph.frame_time * 5.,  5, hann_window);
    Transform3d rel = relative_xform(dest_cloud, cloud);
    double delta = cloud_distance(cloud, dest_cloud, rel);
    char text[256];
    sprintf(text, "%.2f", delta);
    view->m_ui->lbl_delta->copy_label(text);
}

void PointCloudMap::selectPointCloud()
{
    if(view == NULL) return;
    if(view->motion_graph.frames.size() == 0) return;

    // compute the equivalent point position
    float x_scale = (float)w() / (float)view->motion_graph.frames.size(),
          y_scale = (float)h() / (float)view->motion_graph.frames.size();
    int x_frame = view->m_ui->edit_x_frame->value(),
        y_frame = view->m_ui->edit_y_frame->value();
    pt_x = (int)((float)x_frame * x_scale);
    pt_y = this->h() - (int)((float)y_frame * y_scale) - 1;
 

    updateDelta();


    
    view->m_ui->motionGraphWindow->redraw();
    view->m_ui->viewPointCloud->redraw();
}

void PointCloudMap::updatePointCloud()
{
    if(view == NULL) return;
    
    // draw the current position of the red square
    pt_y = max(0, pt_y);
    pt_y = min(h()-1, pt_y);
    pt_x = max(0, pt_x);
    pt_x = min(w()-1, pt_x);
    view->m_ui->motionGraphWindow->redraw();
    
    if(view->motion_graph.frames.size() == 0) return;
    

    // compute the frame index
    int x_index = pt_x,
        y_index = this->h() - pt_y - 1;
    float x_scale = (float)w() / (float)view->motion_graph.frames.size(),
          y_scale = (float)h() / (float)view->motion_graph.frames.size();
    x_index = (int)((float)x_index / x_scale);
    y_index = (int)((float)y_index / y_scale);

    // update the cloud info
    view->m_ui->edit_x_frame->value(x_index);
    view->m_ui->edit_y_frame->value(y_index);
    /*
    if(x_index < view->dataCloudMap.rows() && y_index < view->dataCloudMap.cols())
    {
        sprintf(text, "%.2f", (float)view->dataCloudMap(x_index, y_index));
        view->m_ui->lbl_delta->copy_label(text);
    }
    */
    updateDelta();

    view->m_ui->viewPointCloud->redraw();
}


// ***** glPointCloud ******
int glPointCloud::handle(int event)
{

  // remember what button was used
  static int last_push;


  switch(event) {
    // handle the mouse down event
    case FL_PUSH:

        last_push = Fl::event_button();
        if(last_push == 1)
        {
          moving_arcball = true;
          previous_rotation.setIdentity();
          rotation_pos[0] = Fl::event_x_root();
          rotation_pos[1] = Fl::event_y_root();
          damage(1);
          return 1;
        }
        if(last_push == 3)
        {
          pan_camera = true;
          pan_base[0] = Fl::event_x_root();
          pan_base[1] = Fl::event_y_root();
          damage(1);
          return 1;
        }
        if(last_push == 2)
        {
          zoom_camera = true;
          zoom_base[0] = Fl::event_x_root();
          zoom_base[1] = Fl::event_y_root();
          damage(1);
          return 1;
        }

      break;
    
    // handle the mouse up event
    case FL_RELEASE:
      
      if(last_push == 1)
      {
        moving_arcball = false;
      }
      if(last_push == 3)
      {
        pan_camera = false;
      }
      if(last_push == 2)
      {
        zoom_camera = false;
      }
      damage(1);
      last_push=0;
      return 1;

    // handle the mouse move event
    case FL_DRAG:
      if(moving_arcball)
      {
        int x = Fl::event_x_root(),
            y = Fl::event_y_root();
        m_camera_rot *= previous_rotation.inverse().normalized();
        
        Vector3d A = Vector3d(
                        -0.5 + (double)x / (double)w(), 
                         0.5 - (double)y / (double)h(), 0.5),
                 B = Vector3d(
                        -0.5 + (double)rotation_pos[0] / (double)w(), 
                         0.5 - (double)rotation_pos[1] / (double)h(), 0.5);
                        
        Quaterniond next;
        next.setFromTwoVectors(A, B).normalize();
        m_camera_rot *= next;
                
        previous_rotation = next;
        damage(1);
        return 1;
      }
      
      if(pan_camera)
      {
        int x = Fl::event_x_root(),
            y = Fl::event_y_root();
        Vector3d delta = 
          Vector3d((double)(pan_base[0] - x) / w(), 
                    -(double)(pan_base[1] - y) / (double)h(),
                    0.0);
        
        m_object_center += delta * PAN_RATE;
        
        pan_base[0] = x;
        pan_base[1] = y;
        damage(1);
        return 1;
      }
      
      if(zoom_camera)
      {
        int x = Fl::event_x_root(),
            y = Fl::event_y_root();
        Vector3d delta = 
          Vector3d((double)(zoom_base[0] - x) / w(), 
                    0.,
                    -(double)(zoom_base[1] - y) / (double)h());
        
        m_object_center += delta * PAN_RATE;
        
        zoom_base[0] = x;
        zoom_base[1] = y;
        damage(1);
        return 1;
      }

      break;

    // in order to get keyboard events, we need to accept focus
    case FL_FOCUS:
      return 1;

    case FL_ENTER:	// every time the mouse enters this window, aggressively take focus
      focus(this);
      break;

  }

  return Fl_Gl_Window::handle(event);
}
void glPointCloud::draw()
{
    //Set window bounds
    glViewport(0, 0, w(), h());



    //Clear buffer
    glClearColor(0., 0., 0., 0.);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    //Set up camera
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (double)w() / (double)h(), z_near, z_far);

    //Create base camer matrix from mouse controls
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(-m_object_center(0), -m_object_center(1), -m_object_center(2));
    Matrix4d tr = Transform3d(m_camera_rot).matrix().transpose();
    glMultMatrixd(tr.data());

    draw_skeleton();
    glFinish();
}


void glPointCloud::draw_skeleton()
{
    
    if(view == NULL) return;
    if(view->motion_graph.frames.size() == 0) return;

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    
    // update the cloud info
    int x_index = view->m_ui->edit_x_frame->value(),
        y_index = view->m_ui->edit_y_frame->value();
    

    Transform3d base_pose;
    base_pose.setIdentity();


    Frame frame_a = view->motion_graph.frames[x_index].apply_transform(view->motion_graph.skeleton, base_pose);
	
	Transform3d pxform = base_pose;
	

	//Compute relative xform
	aligned<Vector4d>::vector
		pcloud = view->motion_graph.frames[x_index].point_cloudw(view->motion_graph.skeleton),
		ncloud = view->motion_graph.frames[y_index].point_cloudw(view->motion_graph.skeleton);
	Transform3d rel = relative_xform(pcloud, ncloud);

	Frame frame_b = view->motion_graph.frames[y_index].apply_transform(view->motion_graph.skeleton, rel);

    //Extract matrices
    aligned<Transform3d>::vector xform_a = frame_a.local_xform(view->motion_graph.skeleton);
    aligned<Transform3d>::vector xform_b = frame_b.local_xform(view->motion_graph.skeleton);

    
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor4f(1., 0., 0., .5);
    draw_line_skeleton(view->motion_graph.skeleton, xform_a.begin(), xform_a.end(), true);
    
    glColor4f(0., 0., 1., .5);
    draw_line_skeleton(view->motion_graph.skeleton, xform_b.begin(), xform_b.end(), true);
   


    glPopAttrib();


}

