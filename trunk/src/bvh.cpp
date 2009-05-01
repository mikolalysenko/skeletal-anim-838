//Eigen
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/QR>

//STL
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

//Project
#include <skeleton.hpp>

using namespace std;
using namespace Eigen;
using namespace Skeletal;

namespace Skeletal
{

void assert_token(istream& file, const string& tok)
{
  string str;
  if(!(file >> str) || str != tok)
    throw "Expected token " + tok + ", but got token " + str;
}


//Parses in a joint from the BVH file
Joint parseJoint(istream& bvh_file)
{
  string name;
  
  if(!(bvh_file >> name))
    throw string("Expected name for joint");
  
  //Handle blank name
  if(name == "{")
    name = "NONAME";
  else
    assert_token(bvh_file, "{");
  
  //Set initial offset to 0
  Vector3d offset(0., 0., 0.);
  vector<string> channels;
  vector<Joint> children;
  
  //Scan through tokens
  while(true)
  {
    string tok;
    if(!(bvh_file >> tok))
      throw string("Unexpected EOF while reading joint");
    
    if(tok == "OFFSET")
    {
      Vector3d v;
      if(!(bvh_file >> v(0) >> v(1) >> v(2)))
        throw string("Unexpected EOF while reading offset");
      offset += v;
    }
    else if(tok == "CHANNELS")
    {
      int nchannels;
      if(!(bvh_file >> nchannels))
        throw string("Unexpected EOF");
      if(nchannels < 0)
        throw string("Invalid number of channels");
      
      for(int i=0; i<nchannels; i++)
      {
        string chan_name;
        if(!(bvh_file >> chan_name))
          throw string("Expected channel name, got EOF");
        channels.push_back(chan_name);
      }
    }
    else if(tok == "JOINT")
    {
      children.push_back(parseJoint(bvh_file));
    }
    else if(tok == "End")
    {
      assert_token(bvh_file, "Site");
      children.push_back(parseJoint(bvh_file));
    }
    else if(tok == "}")
    {
      return Joint(name, offset, channels, children);
    }
    else
    {
      throw "Unknown token: " + tok;
    }
  }
}



//Parses a complete BVH file from some input stream
Motion parseBVH(istream& bvh_file)
{
  //Read in the hierarchy of the file
  assert_token(bvh_file, "HIERARCHY");
  assert_token(bvh_file, "ROOT");
  Joint skeleton = parseJoint(bvh_file);
  
  //Read in mocap data
  assert_token(bvh_file, "MOTION");
  
  //Read frame count
  assert_token(bvh_file, "Frames:");
  int num_frames;
  if(!(bvh_file >> num_frames))
    throw string("Unexpected EOF.");
  if(num_frames <= 0)
    throw string("Incorrect frame count");
  vector<Frame> frames(num_frames);
  
  //Read frame time
  assert_token(bvh_file, "Frame");
  assert_token(bvh_file, "Time:");
  double frame_time;
  if(!(bvh_file >> frame_time))
    throw string("Unexpected EOF.");
    
  //Get number of parameters
  int param_count = skeleton.num_parameters();
  
  //Read in frames
  for(int i=0; i<num_frames; i++)
  {
    frames[i].pose.resize(param_count);
    for(int j=0; j<param_count; j++)
    {
      if(!(bvh_file >> frames[i].pose[j]))
        throw string("Invalid number of parameters");
    }
  }

  return Motion(frame_time, frames, skeleton);
}

//Writes a skeleton to file
void writeJoint(ostream& bvh_file, const Joint& skel, string tabs = "")
{
  if(skel.name != "NONAME")
    bvh_file << skel.name << endl;
  
  bvh_file << tabs << "{" << endl
       << tabs << "\tOFFSET " << skel.offset[0] << " " << skel.offset[1] << " " << skel.offset[2] << endl
       << tabs << "\tCHANNELS " << skel.channels.size() << " ";

  for(int i=0; i<skel.channels.size(); i++)
      bvh_file << skel.channels[i] << " ";
  bvh_file << endl;
  
  for(int i=0; i<skel.children.size(); i++)
  {
    if(skel.children[i].children.size() > 0)
      bvh_file << "JOINT ";
    else
      bvh_file << "End Site" << endl;
    writeJoint(bvh_file, skel.children[i], tabs + "\t");
  }
  
  bvh_file << tabs << "}" << endl;
}

//Writes a BVH to file (untested)
void writeBVH(ostream& bvh_file, const Motion& motion)
{
  bvh_file << "HIERARCHY" << endl
           << "ROOT ";
  writeJoint(bvh_file, motion.skeleton);
  
  bvh_file << "MOTION" << endl
           << "Frames: " << motion.frames.size() << endl
           << "Frame Time: " << motion.frame_time << endl;
  
  for(int i=0; i<motion.frames.size(); i++)
  {
    for(int j=0; j<motion.frames[i].pose.size(); j++)
      bvh_file << motion.frames[i].pose[j] << " ";
    bvh_file << endl;
  }
}

}
