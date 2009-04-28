#ifndef MOGRAPH_H
#define MOGRAPH_H


//Vector arithmetic
#include <Eigen/Core>
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
      graph(other.graph),
      local_clouds(other.local_clouds) {}
    MotionGraph operator=(const MotionGraph& other)
    {
      skeleton = other.skeleton;
      frame_time = other.frame_time;
      frames = other.frames;
      graph = other.graph;
      local_clouds = other.local_clouds;
      return *this;
    }
    
    //Inserts a motion into the motion graph
    void insert_motion(const Motion& motion, 
      double threshold,
      double window_size, 
      int window_res, 
      double (*window_func)(double));
    
    //Removes all dead ends from the motion graph
    MotionGraph extract_biconnected() const;
    
    //Synthesize a random motion with l frames
    Motion random_motion(int l) const;
    
    private:
      vector< aligned<Vector4d>::vector > local_clouds;
  };
 
  //Serialization
  MotionGraph parseMotionGraph(istream& is);
  void writeMotionGraph(ostream& os, const MotionGraph g);
  
  //Constructs a motion graph from a bunch of random motions
  MotionGraph construct_graph(const aligned<Motion>::vector& motions,
    double frame_time,
    double threshold,
    double window_size,
    int n_samples,
    double (*window_func)(double));
  
};

#endif
