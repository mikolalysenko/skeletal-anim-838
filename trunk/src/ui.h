// generated by Fast Light User Interface Designer (fluid) version 1.0109

#ifndef ui_h
#define ui_h
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Group.H>
#include "gl_view.h"
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Spinner.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Browser.H>

class UserInterface {
public:
  UserInterface();
  Fl_Double_Window *mainWindow;
  glView *view;
  static Fl_Menu_Item menu_[];
  static Fl_Menu_Item *menuShowShadow;
private:
  void cb_menuShowShadow_i(Fl_Menu_*, void*);
  static void cb_menuShowShadow(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuShowReflection;
private:
  void cb_menuShowReflection_i(Fl_Menu_*, void*);
  static void cb_menuShowReflection(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuShowPreview;
private:
  void cb_menuShowPreview_i(Fl_Menu_*, void*);
  static void cb_menuShowPreview(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuShowFPS;
private:
  void cb_menuShowFPS_i(Fl_Menu_*, void*);
  static void cb_menuShowFPS(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuShowTrailingMotion;
private:
  void cb_menuShowTrailingMotion_i(Fl_Menu_*, void*);
  static void cb_menuShowTrailingMotion(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuShowEndEffectors;
private:
  void cb_menuShowEndEffectors_i(Fl_Menu_*, void*);
  static void cb_menuShowEndEffectors(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuCameraFree;
private:
  void cb_menuCameraFree_i(Fl_Menu_*, void*);
  static void cb_menuCameraFree(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuCameraAuto;
private:
  void cb_menuCameraAuto_i(Fl_Menu_*, void*);
  static void cb_menuCameraAuto(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuSkeletonLines;
private:
  void cb_menuSkeletonLines_i(Fl_Menu_*, void*);
  static void cb_menuSkeletonLines(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuSkeletonEllipsoids;
private:
  void cb_menuSkeletonEllipsoids_i(Fl_Menu_*, void*);
  static void cb_menuSkeletonEllipsoids(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuSkeletonStick;
private:
  void cb_menuSkeletonStick_i(Fl_Menu_*, void*);
  static void cb_menuSkeletonStick(Fl_Menu_*, void*);
  void cb_Stick_i(Fl_Menu_*, void*);
  static void cb_Stick(Fl_Menu_*, void*);
  void cb_Stick1_i(Fl_Menu_*, void*);
  static void cb_Stick1(Fl_Menu_*, void*);
  void cb_Stick2_i(Fl_Menu_*, void*);
  static void cb_Stick2(Fl_Menu_*, void*);
  void cb_Stick3_i(Fl_Menu_*, void*);
  static void cb_Stick3(Fl_Menu_*, void*);
  void cb_DebugFunction1_i(Fl_Menu_*, void*);
  static void cb_DebugFunction1(Fl_Menu_*, void*);
  void cb_DebugFunction2_i(Fl_Menu_*, void*);
  static void cb_DebugFunction2(Fl_Menu_*, void*);
  void cb_DebugFunction3_i(Fl_Menu_*, void*);
  static void cb_DebugFunction3(Fl_Menu_*, void*);
  void cb_DebugFunction4_i(Fl_Menu_*, void*);
  static void cb_DebugFunction4(Fl_Menu_*, void*);
  void cb_DebugFunction5_i(Fl_Menu_*, void*);
  static void cb_DebugFunction5(Fl_Menu_*, void*);
public:
  Fl_Button *btn_play;
private:
  void cb_btn_play_i(Fl_Button*, void*);
  static void cb_btn_play(Fl_Button*, void*);
public:
  Fl_Value_Slider *m_slider;
private:
  void cb_m_slider_i(Fl_Value_Slider*, void*);
  static void cb_m_slider(Fl_Value_Slider*, void*);
  void cb_1refresh_i(Fl_Button*, void*);
  static void cb_1refresh(Fl_Button*, void*);
public:
  Fl_Spinner *edit_desired_fps;
private:
  void cb_edit_desired_fps_i(Fl_Spinner*, void*);
  static void cb_edit_desired_fps(Fl_Spinner*, void*);
public:
  Fl_Browser *m_browser_file;
private:
  void cb_m_browser_file_i(Fl_Browser*, void*);
  static void cb_m_browser_file(Fl_Browser*, void*);
  void cb_2_i(Fl_Button*, void*);
  static void cb_2(Fl_Button*, void*);
  void cb_Remove_i(Fl_Button*, void*);
  static void cb_Remove(Fl_Button*, void*);
public:
  Fl_Browser *m_browser_file_comb;
private:
  void cb_m_browser_file_comb_i(Fl_Browser*, void*);
  static void cb_m_browser_file_comb(Fl_Browser*, void*);
  void cb__i(Fl_Button*, void*);
  static void cb_(Fl_Button*, void*);
  void cb_1_i(Fl_Button*, void*);
  static void cb_1(Fl_Button*, void*);
public:
  Fl_Button *radio_single;
private:
  void cb_radio_single_i(Fl_Button*, void*);
  static void cb_radio_single(Fl_Button*, void*);
public:
  Fl_Button *radio_mutliple;
private:
  void cb_radio_mutliple_i(Fl_Button*, void*);
  static void cb_radio_mutliple(Fl_Button*, void*);
public:
  Fl_Spinner *edit_start_frame;
private:
  void cb_edit_start_frame_i(Fl_Spinner*, void*);
  static void cb_edit_start_frame(Fl_Spinner*, void*);
public:
  Fl_Box *lbl_start_time;
  Fl_Spinner *edit_end_frame;
private:
  void cb_edit_end_frame_i(Fl_Spinner*, void*);
  static void cb_edit_end_frame(Fl_Spinner*, void*);
public:
  Fl_Box *lbl_end_time;
  Fl_Spinner *edit_num_blend_frames;
private:
  void cb_edit_num_blend_frames_i(Fl_Spinner*, void*);
  static void cb_edit_num_blend_frames(Fl_Spinner*, void*);
public:
  Fl_Box *lbl_blend_time;
private:
  void cb_Save_i(Fl_Button*, void*);
  static void cb_Save(Fl_Button*, void*);
public:
  Fl_Box *lbl_name;
  Fl_Box *lbl_joints;
  Fl_Box *lbl_frames;
  Fl_Box *lbl_fps;
  Fl_Box *boxImageLogo;
  void show(int argc, char **argv);
  static void idleCB(UserInterface* gv);
};
#endif
