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


//Point cloud distance with windows
double cloud_distance(
    const vector<Vector4d>& a, 
    const vector<Vector4d>& b, 
    const Transform3d& alignment)
{
  assert(a.size() == b.size());
  
  double s = 0.;
  for(int i=0; i<a.size(); i++)
  {
    Vector3d da = a[i].block(1,3,0,0),
             db = b[i].block(1,3,0,0);
    
    da = alignment * da;
      
    s += (da - db).squaredNorm() * a[i].w() * b[i].w();
  }
  
  return sqrt(s);
}

    
}