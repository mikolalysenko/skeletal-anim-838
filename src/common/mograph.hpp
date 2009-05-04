#ifndef MOGRAPH_H
#define MOGRAPH_H


//Vector arithmetic
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

//STL
#include <iostream>
#include <string>
#include <vector>
#include <cassert>

//Project
#include <misc.hpp>
#include <skeleton.hpp>

namespace Skeletal
{
  using namespace std;
  using namespace Eigen;

  //Point cloud distance metrics
  double cloud_distance(const aligned<Vector4d>::vector& a, const aligned<Vector4d>::vector& b, const Transform3d& x);
  

  //A motion graph data structure
  struct MotionGraph
  {
    Joint           skeleton;
    double          frame_time;
    vector<Frame>   frames;
    vector< vector< int > > graph;
    
    //Ctors
    MotionGraph() {}
    MotionGraph(const Joint& skeleton) :
      skeleton(skeleton) {}        
    MotionGraph(const MotionGraph& other) :
      skeleton(other.skeleton),
      frame_time(other.frame_time),
      frames(other.frames),
      graph(other.graph) {}
      
    //Assignment
    MotionGraph operator=(const MotionGraph& other)
    {
      skeleton = other.skeleton;
      frame_time = other.frame_time;
      frames = other.frames;
      graph = other.graph;
      return *this;
    }
    
    //Inserts a motion into the motion graph
    void insert_motion(const Motion& motion, 
      double threshold,
      double window_size, 
      int window_res, 
      double (*window_func)(double));
      
    void insert_motion2(const Motion& motion, 
      double threshold,
      double window_size, 
      int window_res, 
      double (*window_func)(double)) { assert(false); }
    
    //Removes all dead ends from the motion graph
    vector<MotionGraph> extract_scc() const;
    
    //Synthesize a random motion with l frames
    Motion random_motion(int l) const;
    Motion random_motion2(int l) const { assert(false); }
    
    //Synthesizes a motion which follows a path
    Motion follow_path(Vector2d (*path_func)(double), double max_d) const;
    
    //Extracts a linear submotion from the motion graph
    Motion submotion(int start, int end) const;    
    
    //Pulls out a local window frame about the frame f
    aligned<Vector4d>::vector point_cloud(int f, double w, int n, double (*wind_func)(double)) const;
  };
 
  //Serialization
  MotionGraph parseMotionGraph(istream& is);
  void writeMotionGraph(ostream& os, const MotionGraph g);
  };

#endif
