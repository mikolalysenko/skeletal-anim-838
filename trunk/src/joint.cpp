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

//Counts the number of parameters for this joint
int Joint::num_parameters() const
{
  int s = 0;
  
  //Need to handle special case for quaternions
  for(int i=0; i<channels.size(); i++)
  {	if(channels[i] == "Quaternion")
      s += 4;
    else
      s++;
  }
  
  for(int i=0; i<children.size(); i++)
    s += children[i].num_parameters();
  return s;
}

//Count size of joint
int Joint::size() const
{
  int s = 1;
  for(int i=0; i<children.size(); i++)
    s += children[i].size();
  return s;
}

//Changes parameterization to quaternion
Joint Joint::convert_quat() const
{
  Joint res;
  res.name = name;
  res.offset = offset;

  //Scan through channels, factor out groups of rotations
  res.channels.resize(0);
  int last_rot = 0;
  for(int i=0; i<channels.size(); i++)
  {
    if( channels[i] != "Xrotation" &&
      channels[i] != "Yrotation" &&
      channels[i] != "Zrotation" )
    {
      //Got non-rotation channel
      if(last_rot < i)
        res.channels.push_back("Quaternion");
      last_rot = i + 1;
      res.channels.push_back(channels[i]);
    }
  }
  if(last_rot < channels.size())
    res.channels.push_back("Quaternion");
  
  //Convert children
  res.children.resize(children.size());
  for(int i=0; i<children.size(); i++)
    res.children[i] = children[i].convert_quat();
  
  return res;
}


void find_end_effectors_impl(
    const Joint& skeleton,
    vector<int> &list_end_effector,
    int& idx)
{
  // add the end effector
  if(skeleton.children.size() == 0) list_end_effector.push_back(idx);

  idx++;
  
  // loop through all the joints
  for(int i=0; i<skeleton.children.size(); i++)
    find_end_effectors_impl(skeleton.children[i], list_end_effector, idx);
}


//Gets a list of end effectors
vector<int> Joint::find_end_effectors() const
{
    vector<int> result;
    int idx = 0;
    find_end_effectors_impl(*this, result, idx);
    return result;
}



}