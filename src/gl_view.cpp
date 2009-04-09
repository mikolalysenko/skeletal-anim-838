

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

//OpenGL
#include <GL/glut.h>

#ifndef WIN32
#include <sys/time.h>
#endif

#include "gl_view.h"




#define PAN_RATE        100.

//Window parameters
int     width = 800,
        height = 800;

//Perspective matrix
double  fov = 45., 
        z_near = 0.1,
        z_far = 10000.;

double colors[6][4] =
{
        {1., 0., 0., 0.5},
        {0., 1., 0., 0.5},
        {0., 0., 1., 0.5},
        {1., 1., 0., 0.5},
        {0., 1., 1., 0.5},
        {1., 0., 1., 0.5},
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
  //Clear camera rotation
  object_center = Vector3d(30, 150, 500);
  camera_rot.setIdentity();


  /*
  ifstream a_in("walkLoop.bvh");

  //Parse out motion capture data
  mocap_quat = parseBVH(a_in).convert_quat();
  //print_skeleton(mocap.skeleton);
  
  mocap_quat = mocap_quat.convert_quat();
  */

/*
  
  double a_start = 2.,
         b_start = 0.,
         duration = 0.2;

  ifstream a_in("walkLoop.bvh");
  ifstream b_in("PETE.bvh");

  Motion a = parseBVH(a_in).convert_quat(),
         b = parseBVH(b_in).convert_quat();

  b.frame_time = 0.5;

  vector<Transform3d> xform_a( a.skeleton.size() ),
                      xform_b( b.skeleton.size() );
                                          
  Frame fa = a.get_frame(a_start),
        fb = b.get_frame(b_start);
  
  
  a.skeleton.interpret_pose(
          xform_a.begin(), 
          fa.pose.begin(),
          fb.pose.end());

  b.skeleton.interpret_pose(
          xform_b.begin(), 
          fb.pose.begin(),
          fb.pose.end());
          
  Transform3d relative_xform(xform_b[0].inverse());
  relative_xform = xform_a[0] * relative_xform;
  
  mocap_quat = combine_motions(a, b, relative_xform, a_start, b_start, duration, 1);
  */
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

void glView::drawScene()
{
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

  
  if(!mocap_selected) return;


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
      m_play = false;
      m_ui->btn_play->clear();
    }
    else
    {
      m_time -= mocap_selected->duration();
    }
  }
  

  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  
  for(int i=0; i<6; i++)
  {
    glColor4dv(colors[i]);
    //draw_frame(i * mocap_selected->frames.size() / 6);
  }
          
  glColor4f(1., 1., 1., 1.);
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
  drawScene();

  
  //glDisable(GL_DEPTH_TEST);
  //glLoadIdentity();
  //gl_draw(true ? "Cube: wire" : "Cube: flat", -4.5f, -4.5f );
  //glEnable(GL_DEPTH_TEST);

  //Finish up drawing
  //glFlush();
  //glutSwapBuffers();
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


  initScene(); 

  
  // Make the file chooser...
#ifdef WIN32
  Fl::scheme(NULL);
  Fl_File_Icon::load_system_icons();
#endif
  file_chooser = new Fl_File_Chooser(".", "*", Fl_File_Chooser::SINGLE, "Select Motion File");


}



void glView::draw() 
{
  m_drawing = true;


  display();


  time_t curtime;
  char buf[255];
  static time_t fpstime = 0;
  static int fpscount = 0;
  static int fps = 0;

  // get ready to draw FPS
  glLoadIdentity();
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0.0, w(), 0.0, h());

  // draw FPS
  sprintf(buf, "FPS=%d", fps);
  glColor3f(1.0f, 1.0f, 0.0f);
  gl_font(FL_HELVETICA, 12);
  gl_draw(buf, 10, 10);


  
  // Use glFinish() instead of glFlush() to avoid getting many frames
  // ahead of the display (problem with some Linux OpenGL implementations...)
  //
  glFinish();

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
				damage(1);
				return 1;
      }
      
      if(pan_camera)
      {
        int x = Fl::event_x_root(),
            y = Fl::event_y_root();
        Vector3d delta = 
          Vector3d((double)(pan_base[0] - x) / width, 
                    -(double)(pan_base[1] - y) / (double)height,
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
          Vector3d((double)(zoom_base[0] - x) / width, 
                    0.,
                    -(double)(zoom_base[1] - y) / (double)height);
        
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
  m_ui->m_slider->range(0., (double)mocap_selected->frames.size());
  m_ui->lbl_name->copy_label(m_ui->m_browser_file->text(index+1));
  sprintf(text, "%i", mocap_selected->skeleton.size());
  m_ui->lbl_joints->copy_label(text);
  sprintf(text, "%i", mocap_selected->frames.size());
  m_ui->lbl_frames->copy_label(text);
  sprintf(text, "%.1f", 1. / mocap_selected->frame_time);
  m_ui->lbl_fps->copy_label(text);
  m_ui->mainWindow->redraw();
  m_ui->edit_desired_fps->value((int)(1. / mocap_selected->frame_time));


  // stop playing
  m_time = 0.;
  m_play = false;
  m_ui->btn_play->clear();
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
  m_ui->edit_desired_fps->value((int)(1. / mocap_selected->frame_time));


/*

  Motion* a = (Motion*)m_ui->m_browser_file_comb->data(1);
  Motion* b = (Motion*)m_ui->m_browser_file_comb->data(2);

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
  
  mocap_combine = combine_motions(*a, *b, relative_xform, a_start, b_start, duration, 1);
*/

}
