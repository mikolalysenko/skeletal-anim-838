//STL
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

//Eigen
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/QR>

//Project
#include <skeleton.hpp>

using namespace std;
using namespace Eigen;
using namespace Skeletal;

namespace Skeletal
{


//Converts the motion into a quaternion parameterized motion, thus removing the dependency on Euler angles
Motion Motion::convert_quat() const
{
  Joint quat_skeleton = skeleton.convert_quat();
  vector<Frame> quat_frames(frames.size());
  
  for(int i=0; i<frames.size(); i++)
    quat_frames[i] = frames[i].reparameterize(skeleton, quat_skeleton);
  
  return Motion(frame_time, quat_frames, quat_skeleton);
}


//Retrieves an intermediate frame
Frame Motion::get_frame(double t, bool loop) const
{
  int start_frame, end_frame;

  //Get int/fract parts of the frame time
  double frame_num;
  double tau = modf(t / frame_time, &frame_num);
  
  if(loop)
  {
    //Handle looping case
    start_frame = frame_num;
    if(start_frame < 0)
      start_frame = frames.size() - ((-start_frame) % frames.size());
    start_frame %= frames.size();
    end_frame = (start_frame + 1) % frames.size();
  }
  else
  {
    //For non-looping, just clamp end points
    if(t < 0)
    {
      start_frame = end_frame = tau = 0.;
    }
    else if(frame_num >= frames.size() - 1) 
    {
      start_frame = end_frame = frames.size() - 1;
      tau = 0.;
    }
    else
    {
      start_frame = frame_num;
      end_frame = (start_frame + 1) % frames.size();
    }	
  }
  
  assert(start_frame >= 0 && start_frame < frames.size());
  assert(end_frame >= 0 && end_frame < frames.size());
  
  
  //Interpolate frames
  return interpolate_frames(skeleton, frames[start_frame], frames[end_frame], tau);
}


//The interpolation function
double interp_func(double t, int deg)
{
  return t; //TODO: Implement spline steps
}

//Combines two sections of motions together
Motion combine_motions(
  const Motion& a, 
  const Motion& b, 
  const Transform3d& relative_xform, 
  double a_start, 
  double a_end,
  double b_start, 
  double b_end, 
  double duration, 
  int deg)
{
  //assert(a_end > a_start);
  //assert(b_end > b_start);
  //assert(a_start >= 0. && a_end <= a.duration());
  //assert(b_start >= 0. && b_end <= b.duration());
  assert(a.skeleton.size() == b.skeleton.size());
  assert(a.skeleton.num_parameters() == b.skeleton.num_parameters());

  Motion result;
  result.skeleton = a.skeleton;
  result.frame_time = min(a.frame_time, b.frame_time);
  result.frames.resize(0);
  
  for(double t = a_start; t<=a_end - duration; t+=result.frame_time)
    result.frames.push_back(a.get_frame(t));
  
  for(double t = 0.; t<duration; t+=result.frame_time)
  {
    Frame fa = a.get_frame(a_end - duration + t),
        fb = b.get_frame(b_start + t).apply_transform(result.skeleton, relative_xform);
    result.frames.push_back(interpolate_frames(a.skeleton, fa, fb, interp_func(t / duration, deg)));
  }

  for(double t=b_start+duration; t<=b_end; t+=result.frame_time)
    result.frames.push_back(b.get_frame(t).apply_transform(result.skeleton, relative_xform));
  
  return result;	
}


//Recovers the bounding sphere radius
double Motion::bound_sphere_radius() const
{
  Vector3d avg(0.,0.,0.);
  int n = 0;
  
  for(int i=0; i<frames.size(); i++)
  {
    aligned<Vector3d>::vector cloud = frames[i].point_cloud(skeleton);
    for(int j=0; j<cloud.size(); j++)
    {
      avg += cloud[j];
      n++;
    }
  }
  
  avg /= n;
  
  
  double r = 0.;
  for(int i=0; i<frames.size(); i++)
  {
    aligned<Vector3d>::vector cloud = frames[i].point_cloud(skeleton);
    for(int j=0; j<cloud.size(); j++)
    {
      r = max(r, (avg - cloud[j]).norm());
    }
  }
  
  cout << "r = " << r << endl;
  
  return r;
}

//Computes the bounding box for this frame sequence
void Motion::bounding_box(Vector3d& lo, Vector3d& hi) const
{
  lo = Vector3d(1e30, 1e30, 1e30);
  hi = Vector3d(-1e30, -1e30, -1e30);
  
  for(int i=0; i<frames.size(); i++)
  {
    aligned<Vector3d>::vector cloud = frames[i].point_cloud(skeleton);
    for(int j=0; j<cloud.size(); j++)
    {
	    for(int d=0; d<3; d++)
	    {
	      lo[d] = min(lo[d], cloud[j][d]);
	      hi[d] = max(hi[d], cloud[j][d]);
	    }
	    cout << "cloud[" << j << "] = " << cloud[j] << endl;
	 }
  }
  
  cout << "box = " << lo << "," << hi << endl;
}

/*
//Compute a window about the point cloud
aligned<Vector4d>::vector Motion::point_cloud_window(double orig, double extent, int n_samples, double (*window)(double)) const
{
  assert(n_samples % 2 == 1); // Number of samples must be odd
  
  aligned<Vector4d>::vector result;
  
  double frame_t = orig - extent/2.,
         delta_t = extent / (double)n_samples,
         t = -1.;
  for(int i=0; i<n_samples; i++, frame_t += delta_t, t += 2. / (double)n_samples)
  {
    aligned<Vector3d>::vector cloud = get_frame(frame_t).point_cloud(skeleton);
    for(int j=0; j<cloud.size(); j++)
      result.push_back(Vector4d(cloud[j].x(), cloud[j].y(), cloud[j].z(), window(t)));
  }
  
  return result;
}
*/



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


//Computes the relative alignment of two motions
Transform3d relative_xform(const Joint& skel, 
  const Frame& base_frame, 
  const Frame& target_frame)
{
  aligned<Transform3d>::vector	xform_a = base_frame.local_xform(skel),
                      			xform_b = target_frame.local_xform(skel);
          
  Transform3d result(xform_b[0].inverse());
  result = xform_a[0] * result;
  result = constrain_xform(result);

  return result;
}


}
