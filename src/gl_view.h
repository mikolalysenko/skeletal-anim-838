#ifndef gl_view_h
#define gl_view_h

#define NOMINMAX
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h>
#include <FL/Fl_File_Icon.H>
#include <FL/Fl_File_Chooser.H>
#include <stdlib.h>
#include <stdio.h>
#include <vector>


class glView;
#include "ui.h"

//Project includes
#include <skeleton.hpp>
#include <skin.hpp>

using namespace std;
using namespace Eigen;
using namespace Skeletal;

class UserInterface;
class glView : public Fl_Gl_Window 
{

public:

  glView(int x,int y,int w,int h,const char *l=0);

  // overrides of important window things
	virtual int handle(int);
  virtual void draw();  
  void display();
  void drawScene();
  void initScene();
  void draw_skeleton(double t);
  void draw_frame(int f);

  void set_ui(UserInterface* ui){m_ui = ui;};
  void select_animation(int index);

  // callback functions
  void toggle_play(int value){m_play = value;redraw();};
  void toggle_repeat(int value){m_repeat = value;redraw();};
  void move_frame_to(int value);
  void set_desired_fps(int value){m_play_fps = value; redraw();};
  void load_file();
  void remove_file();
  void update_selection();
  void print_skeleton(const Joint& skel, int t = 0);
  void add_animation_list();
  void del_animation_list();
  void mode_single();
  void mode_multiple();

private:
  bool m_drawing;
  bool m_play;
  bool m_repeat;
  unsigned int m_frame_num;
  UserInterface* m_ui;
  float m_time;
  float m_play_fps;

  Fl_File_Chooser* file_chooser; 
  vector<Motion> mocap_list;
  Motion* mocap_selected;
  Motion mocap_combine;

};


#endif