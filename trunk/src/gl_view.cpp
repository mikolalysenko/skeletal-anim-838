

//STL includes
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

//Eigen includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

//OpenGL
#include <GL/glut.h>

#ifndef WIN32
#include <sys/time.h>
#endif

#include "gl_view.h"




#define PAN_RATE        100.

//Window parameters
//int     width = 800,
//        height = 800;

//Perspective matrix
double  fov = 45., 
        z_near = 0.1,
        z_far = 10000.;

double colors[6][4] =
{
        {1., 0., 0., 0.5},
        {.2, .2, .2, 0.5},
        {0., 0., 1., 0.5},
        {1., 1., 0., 0.5},
        {0., 1., 1., 0.5},
        {1., 0., 1., 0.5},
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

//Motion capture data
//Motion mocap_quat;

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


void glView::initScene()
{
  // camera position
  object_center = Vector3d(0, 50, 400);
  camera_rot.setIdentity();

  // light position from above the origin
  lightPosition[0] = 0;
  lightPosition[1] = 200;
  lightPosition[2] = 0;
  lightPosition[3] = 0;

  
  //Fl_Shared_Image* img = Fl_Shared_Image::get("/afs/cs.wisc.edu/u/y/a/yangk/Desktop/p2/a/uw_logo.png");
//Fl_Shared_Image* img = Fl_Shared_Image::get((const char*)m_ui->boxImageLogo->image()->data(),
//  m_ui->boxImageLogo->image()->w(),
//m_ui->boxImageLogo->image()->h());
Fl_Image* img = m_ui->boxImageLogo->image();
  idFloor = 0;
  if(img)
  {
//exit(0);
    //::MessageBox(0 , "hsdf", "sk", 0);
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

  glEnable(GL_DEPTH_TEST);

 // if(mode( FL_STENCIL) == true)
 //   ::MessageBox(0, "stencil", "", 0);
}

void glView::draw_skeleton(double t)
{
  if(!mocap_selected) return;

  //Figure interpolate current pose
  Frame c_frame = mocap_selected->get_frame(t, true);

  //Extract matrices
  vector<Transform3d> xform( mocap_selected->skeleton.size() );
  mocap_selected->skeleton.interpret_pose(
          xform.begin(), 
          c_frame.pose.begin(),
          c_frame.pose.end());

  //Draw the line skeleton
  draw_ellipsoid_skeleton(mocap_selected->skeleton, xform.begin(), xform.end());




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
  glutWireSphere(mocap_selected->bound_sphere_radius, 8, 8);
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
  vector<Transform3d> xform( mocap_selected->skeleton.size() );
  mocap_selected->skeleton.interpret_pose(
          xform.begin(), 
          c_frame.pose.begin(),
          c_frame.pose.end());

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


void glView::drawFloor()
{

  /*
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
  */

  



	//glEnable(GL_COLOR_MATERIAL);
  glDisable(GL_LIGHTING);

  // draw the checkered floor
  drawCheckeredFloor(abs(floorVertex[0][0]-floorVertex[1][0]),20);

  // draw the uw logo
  if(idFloor)
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
}

void glView::drawScene()
{


  // return if no animation is selected
  if(!mocap_selected) 
  {
    drawFloor();
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
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    glEnable(GL_DEPTH_TEST);


    // reflect the scene on the y axis so that we can draw the skeleton upside down
    glPushMatrix();
    glScalef(1.0, -1.0, 1.0);
    glTranslatef(0.0, -floorHeight * 2., 0.0);

    glEnable(GL_NORMALIZE);
    glCullFace(GL_FRONT);

    // blend with the ground
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // draw the skeleton
    glDisable(GL_LIGHTING);
    //glDisable(GL_DEPTH_TEST);
    glColor4f(.2, 1., .2, .25);
    draw_skeleton(m_time);

    glDisable(GL_BLEND);
    glDisable(GL_NORMALIZE);
    glCullFace(GL_BACK);

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
      draw_frame(i * mocap_selected->frames.size() / 6);
    }
    glDisable(GL_BLEND);
  }

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
    draw_skeleton(m_time);


    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_STENCIL_TEST);
    glPopMatrix();
  }
     

  // draw the skeleton
  glColor4f(.2, 1., .2, 1.);
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
  m_draw_fps = false;
  m_camera_mode = CAMERA_FREE;

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
  file_chooser->type(file_chooser->type() ^ Fl_File_Chooser::MULTI);

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

      // add the filename to the list
      m_ui->m_browser_file->add(fl_filename_name(file_chooser->value(i)));

      // Parse out motion capture data
      ifstream c_in(relative);
      Motion mocap = parseBVH(c_in).convert_quat();
      mocap_list.push_back(mocap); 

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

  // remove the active motion and deselect selection
  mocap_selected = NULL;
  m_ui->m_browser_file->select(0);

  // delete the motion
  int index = m_ui->m_browser_file->value() - 1;
  mocap_list.erase(mocap_list.begin() + index);
  m_ui->m_browser_file->remove(index + 1);

  redraw();
}

void glView::update_selection()
{
  int index = m_ui->m_browser_file->value() - 1;
  select_animation(index);

  redraw();
}


void glView::select_animation(int index)
{
  if(index < 0 || index >= mocap_list.size()) return;

  // set the active motion capture object
  mocap_selected = &mocap_list[index];

  // display some info about the active object
  char text[256];
  m_ui->m_slider->range(0., (double)mocap_selected->frames.size()-1.);
  m_ui->lbl_name->copy_label(m_ui->m_browser_file->text(index+1));
  sprintf(text, "%i", mocap_selected->skeleton.size());
  m_ui->lbl_joints->copy_label(text);
  sprintf(text, "%i", mocap_selected->frames.size());
  m_ui->lbl_frames->copy_label(text);
  sprintf(text, "%.1f", 1. / mocap_selected->frame_time);
  m_ui->lbl_fps->copy_label(text);
  m_ui->mainWindow->redraw();
  m_ui->edit_desired_fps->value((int)(1. / mocap_selected->frame_time + .5));


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
  
  // add the item to the list of animation to contatenate
  m_ui->m_browser_file_comb->add(
    m_ui->m_browser_file->text(index + 1),
    &mocap_list[index]
    );

  // update the multiple mode if necessary
  if(m_ui->radio_mutliple->value() == 1)
  {
    mode_multiple();
  }
}


void glView::del_animation_list()
{
  // delete the item from the list
  m_ui->m_browser_file_comb->remove(m_ui->m_browser_file_comb->value());

  // update the multiple mode if necessary
  if(m_ui->radio_mutliple->value() == 1)
  {
    mode_multiple();
  }
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


//Fixes the y=0 plane to a constant -Mik
Transform3d constrain_xform(const Transform3d& xform)
{
	Matrix3d m = xform.rotation(), r;
	
	//Zero out y part
	for(int i=0; i<3; i++)
		m(i,1) = m(1,i) = 0.;
	m(1,1) = 1.	;
	
	//Compute closest orthonormal matrix
	r = m.svd().matrixU() * m.svd().matrixV().transpose();
	
	//Reconstruct final matrix
	Matrix4d v = Matrix4d::Identity();
	v.block(0,0,3,3) = r;
	v.block(0,3,3,1) = xform.translation();
	
	//Cancel y translation
	v(1,3) = 0.;
	
	return Transform3d(v);
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
 
  Motion* ptrMotion;

  // set the first item to be the start of the motion
  ptrMotion = (Motion*)m_ui->m_browser_file_comb->data(1);
  if(ptrMotion == NULL) return;  // TODO add error message
  mocap_combine = *ptrMotion;

  
  double a_start = 2.,
         b_start = 0.,
         duration = 0.2;

  duration = m_ui->edit_num_blend_frames->value() * mocap_combine.frame_time;
  

  //ifstream a_in("walkLoop.bvh");
  //ifstream b_in("PETE.bvh");

  //Motion a = parseBVH(a_in).convert_quat(),
  //       b = parseBVH(b_in).convert_quat();

  //b.frame_time = 0.5;

  
  // loop through the list of animation to contatenate
  for(int i = 1; i < num_files; i++)
  {

    Motion* a = &mocap_combine;
    Motion* b = (Motion*)m_ui->m_browser_file_comb->data(i+1);
    if(b == NULL) return;  // TODO add error message

    a_start = a->duration() - duration;

    vector<Transform3d> xform_a( a->skeleton.size() ),
                        xform_b( b->skeleton.size() );
                                            
    Frame fa = a->get_frame(a_start),
          fb = b->get_frame(b_start);
    
    
    a->skeleton.interpret_pose(
            xform_a.begin(), 
            fa.pose.begin(),
            fb.pose.end());

    b->skeleton.interpret_pose(
            xform_b.begin(), 
            fb.pose.begin(),
            fb.pose.end());
    
            
    
    Transform3d relative_xform(xform_b[0].inverse());
    relative_xform = xform_a[0] * relative_xform;
    
    //Add constraint to fix y=0 plane (necessary to avoid the silly walking on air bug) -Mik
    relative_xform = constrain_xform(relative_xform);
    
    mocap_combine = combine_motions(*a, *b, relative_xform, a_start, b_start, duration, 1);
  }

  
  mocap_selected = &mocap_combine;
  
  // display some info about the active object
  char text[256];
  m_ui->m_slider->range(0., (double)mocap_selected->frames.size());
  m_ui->lbl_name->copy_label("Animation Contatenation");
  sprintf(text, "%i", mocap_selected->skeleton.size());
  m_ui->lbl_joints->copy_label(text);
  sprintf(text, "%i", mocap_selected->frames.size());
  m_ui->lbl_frames->copy_label(text);
  sprintf(text, "%.1f", 1. / mocap_selected->frame_time);
  m_ui->lbl_fps->copy_label(text);
  sprintf(text, "%.2f", duration);
  m_ui->lbl_blend_time->copy_label(text);
  m_ui->mainWindow->redraw();
  m_ui->edit_desired_fps->value((int)(1. / mocap_selected->frame_time + .5));

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
  vector<Transform3d> xform( mocap_selected->skeleton.size() );
  mocap_selected->skeleton.interpret_pose(
          xform.begin(), 
          c_frame.pose.begin(),
          c_frame.pose.end());

  
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
    offset[2] += mocap_selected->bound_sphere_radius * 5.;
    
    // update the object center and camera rotation
    object_center = offset;
    camera_rot.setIdentity();
  }
  else
  {
    // fix the center to be the center of the bounding box of the motion
    Vector3d mid_pt = (mocap_selected->bound_box_max + mocap_selected->bound_box_min) / 2.;
    xform_sphere.translate(mid_pt);

    // calculate the radius of half the diagonal of the bounding box
    Vector3d diag_box = (mocap_selected->bound_box_max - mocap_selected->bound_box_min) / 2.;
    double radius = sqrt(diag_box.dot(diag_box));
    xform_sphere.translate(Vector3d(0, 10, radius));

    // move the z back 5 times the bounding sphere + radius
    Vector3d offset = xform_sphere.translation();
    offset[2] += mocap_selected->bound_sphere_radius * 5.;
    
    // update the object center and camera rotation
    object_center = offset;
    camera_rot.setIdentity();

  }

  // update the floor height
  Vector3d min_pt = Vector3d(0., 0., 0.);
  Vector3d max_pt = Vector3d(0., 0., 0.);
  Transform3d xformPose;
	xformPose.setIdentity();
  compute_bounding_box( xformPose, 
                        mocap_selected->skeleton, 
                        xform.begin(), 
                        xform.end(),
                        min_pt, 
                        max_pt);
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
    vector<Transform3d> xform( mocap_selected->skeleton.size() );
    mocap_selected->skeleton.interpret_pose(
            xform.begin(), 
            c_frame.pose.begin(),
            c_frame.pose.end());

    Transform3d xform_sphere;
    xform_sphere.setIdentity();

    
    // fix the center to be the skeleton center
    xform_sphere.translate(mocap_selected->skeleton.offset);
    xform_sphere = xform_sphere * xform[0];
    //xform_sphere.translate(Vector3d(0, 0, 0));

    // move the z back 5 times the bounding sphere
    Vector3d offset = xform_sphere.translation();
    offset[2] += mocap_selected->bound_sphere_radius * 5.;
    
    // update the object center and camera rotation
    Vector3d diff = offset - object_center;
    float diff_length = sqrt(diff.dot(diff));
    float radius = mocap_selected->bound_sphere_radius;
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


