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
      
    //Removes all dead ends from the motion graph, returning a list of strongly connected components
    vector<MotionGraph> extract_scc() const;
    
    //Synthesizes a motion
    Motion synthesize_motion(const vector<int> frame_seq, const Transform3d& base_pose) const;
    
    //Synthesize a random motion with l frames
    Motion random_motion(int l) const;
    
    //Synthesizes a motion which follows a path
    Motion follow_path(Vector2f (*path_func)(double), double max_d, double dur) const;
    
    //Extracts a linear submotion from the motion graph
    Motion submotion(int start, int end) const;    
    
    //Pulls out a local window frame about the frame f
    aligned<Vector4d>::vector point_cloud(int f, double w, int n, double (*wind_func)(double)) const;

    //Create point cloud map
    void create_point_cloud_map(MatrixXd& data,
      double& max_distance,
      double window_size,
      int window_res,
      double (*window_func)(double));
  };
  
  //Point cloud distance metric
  double cloud_distance(const aligned<Vector4d>::vector& a, const aligned<Vector4d>::vector& b, const Transform3d& x);

  //Point cloud alignment
  Transform3d relative_xform(const aligned<Vector4d>::vector& a, const aligned<Vector4d>::vector& b);
 
  //Serialization
  MotionGraph parseMotionGraph(istream& is);
  void writeMotionGraph(ostream& os, const MotionGraph& g);
  };

#endif
