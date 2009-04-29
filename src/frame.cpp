
//Eigen
#include <Eigen/Core>
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


//Performs the actual reparameterization from pose base to target
//Currently assumes consistent topology
void reparameterize_impl(
  vector<double>& n_params, 
  Transform3d*& cur_xform,
  const Joint& target)
{
  Transform3d xform = *(cur_xform++);
  
  //Parse out parameters
  for(int i=0; i<target.channels.size(); i++)
  {
    string chan = target.channels[i];
    
    if(chan == "Quaternion")
    {
      Matrix3d rot = xform.rotation();
      xform = xform.rotate(rot.inverse());
      
      Quaterniond q(rot);
      n_params.push_back(q.w());
      n_params.push_back(q.x());
      n_params.push_back(q.y());
      n_params.push_back(q.z());
    }
    else if(chan == "Xrotation" ||
            chan == "Yrotation" ||
            chan == "Zrotation")
    {
      //TODO: Implement this
      assert(false);
    }
    else if(chan == "Xposition" ||
            chan == "Yposition" ||
            chan == "Zposition")
    {
      int d = chan[0] - 'X';
      Vector3d v(0.,0.,0.);
      v[d] = xform.translation()[d];
      xform.pretranslate(v);
      n_params.push_back(v[d]);
    }
    else assert(false);
  }
  
  //Recursively adjust children
  for(int i=0; i<target.children.size(); i++)
    reparameterize_impl(n_params, cur_xform, target.children[i]);	
}

//Converts the parameterization of the pose from base to target
Frame Frame::reparameterize(const Joint& base, const Joint& target) const
{
  //Check sizes
  assert(base.size() == target.size());

  //Interpret pose
  aligned<Transform3d>::vector target_xform = local_xform(base);

  //Construct result
  Frame result;
  Transform3d* ptr = &target_xform[0];
  reparameterize_impl(result.pose, ptr, target);
  return result;
}

//Recursive implementation of interpolate
void _interpolate_impl(
  double*& result, 
  const Joint& skel,
  const double*& pa,
  const double*& pb,
  double t)
{
  for(int i=0; i<skel.channels.size(); i++)
  {
    string pname = skel.channels[i];
    
    if(pname == "Quaternion")
    {
      Quaterniond qa(pa[0], pa[1], pa[2], pa[3]),
            qb(pb[0], pb[1], pb[2], pb[3]);
            
      pa += 4;
      pb += 4;
      
      Quaterniond qr = qa.slerp(t, qb);
      
      *(result++) = qr.w();
      *(result++) = qr.x();
      *(result++) = qr.y();
      *(result++) = qr.z();
    }
    else if(pname == "Xrotation" ||
        pname == "Yrotation" ||
        pname == "Zrotation")
    {
      //TODO: Implement this
    }
    else if(pname == "Xposition" ||
        pname == "Yposition" ||
        pname == "Zposition")
    {
      *(result++) = (1. - t) * (*(pa++)) + t * (*(pb++));
    } else assert(false);
  }
  
  for(int i=0; i<skel.children.size(); i++)
    _interpolate_impl(result, skel.children[i], pa, pb, t);
}

//Interpolate two poses
Frame interpolate_frames(const Joint& skel, const Frame& a, const Frame& b, double t)
{
  assert(a.pose.size() == b.pose.size());
  assert(skel.num_parameters() == a.pose.size());

  Frame result;
  result.pose.resize(a.pose.size());
  const double *pa = &a.pose[0], *pb = &b.pose[0];
  double *pr = &result.pose[0];
  
  _interpolate_impl(pr, skel, pa, pb, t);
  
  return result;
}

//Applies a transformation to this pose
void apply_transform_impl(
  const Joint& skeleton,
  const Transform3d& xform,
  const double*& params,
  double *& result)
{
  Transform3d xform_tmp = xform;
  for(int i=0; i<skeleton.channels.size(); i++)
  {
    string chan = skeleton.channels[i];
    
    if(chan == "Quaternion")
    {
      Matrix3d rot = xform.rotation();
      xform_tmp = xform_tmp.rotate(rot.inverse());
      
      Quaterniond q(params[0], params[1], params[2], params[3]);
      q = Quaterniond(rot) * q;
      
      result[0] = q.w();
      result[1] = q.x();
      result[2] = q.y();
      result[3] = q.z();
      
      result += 4;
      params += 4;
    }
    else if(chan == "Xrotation" ||
        chan == "Yrotation" ||
        chan == "Zrotation")
    {
      //TODO: Implement this
      *(result++) = *(params++);
    }
    else if(chan == "Xposition" ||
        chan == "Yposition" ||
        chan == "Zposition")
    {
      //HACK: Need to use a 3-vector to represent position, not enough DOF to do it componentwise
      assert(skeleton.channels.size() >= i + 3);
      assert(skeleton.channels[i] == "Xposition");
      assert(skeleton.channels[i+1] == "Yposition");
      assert(skeleton.channels[i+2] == "Zposition");
    
      Vector3d t = Vector3d(params[0], params[1], params[2]);
      Vector3d p = xform * t;
      xform_tmp.pretranslate(-xform_tmp.translation());
      
      result[0] = p[0];
      result[1] = p[1];
      result[2] = p[2];
      
      //Skip ahead
      params += 3;
      result += 3;
      i += 3;
    } else assert(false);
  }

  //Recurse
  for(int i=0; i<skeleton.children.size(); i++)
  {
    apply_transform_impl(skeleton.children[i], xform_tmp, params, result);
  }
}


//Applies a transformation to a frame/skeleton pair
Frame Frame::apply_transform(const Joint& skeleton, const Transform3d& xform) const
{
  Frame result;
  result.pose.resize(pose.size());
  
  const double *pa = &pose[0];
  double *pr = &result.pose[0];
  
  apply_transform_impl(skeleton, xform, pa, pr);
  return result;
}

#ifdef WIN32
//Interprets the pose
void Frame::local_pose_impl(
    const Joint joint,
    Transform3d*& result, 
    const double*& pbegin, 
    const double* pend,
    bool skip_transform) const
#else
void local_pose_impl(
    const Joint joint,
    Transform3d*& result, 
    const double*& pbegin, 
    const double* pend,
    bool skip_transform = false)
#endif
{
  Transform3d xform;
  xform.setIdentity();
  
  //Interpret parameters
  for(int i=0; i<joint.channels.size(); i++)
  {
    //Make sure we haven't walked past end of parameters
    assert(pbegin != pend);
    
    double p = *(pbegin++);
    
    string chan = joint.channels[i];
    
    //Interpret parameter
    if( chan == "Xrotation" ||
        chan == "Yrotation" ||
        chan == "Zrotation" )
    {
      Vector3d v(0.,0.,0.);
      v[chan[0] - 'X'] = 1.;
      xform.rotate(AngleAxisd(p * M_PI / 180., v));
    }
    else if(chan == "Xposition" ||
            chan == "Yposition" ||
            chan == "Zposition" )
    {
      Vector3d v(0.,0.,0.);
      v[chan[0] - 'X'] = p;
        
      if(!skip_transform)       //HACK:  Should this be here?
        xform.pretranslate(v);
    }
    else if(chan == "Quaternion")
    {
      Quaterniond quat(p, 0., 0., 0.);
      assert(pbegin != pend);
      quat.x() = *(pbegin++);
      assert(pbegin != pend);
      quat.y() = *(pbegin++);
      assert(pbegin != pend);
      quat.z() = *(pbegin++);
      xform.rotate(quat);
    }
    else assert(false);

  }
  
  //Store result
  *(result++) = xform;
  
  //Compute transforms for children
  for(int i=0; i<joint.children.size(); i++)
    local_pose_impl(joint.children[i], result, pbegin, pend, true);
}


#ifdef WIN32


void Frame::global_xform_impl(const Joint& joint, Transform3d*& xform, Transform3d& root_ref) const
{
    //Update transform matrix
    Transform3d root = root_ref; // Windows complains that 'formal parameter with __declspec(align('16')) won't be aligned'
    root.translate(joint.offset);
    root = root * (*xform);
    *(xform++) = root;
    
    for(int i=0; i<joint.children.size(); i++)
        global_xform_impl(joint.children[i], xform, root);
}

#else


//Retrieves a local transform vector for the skeleton
aligned<Transform3d>::vector Frame::local_xform(const Joint& skel) const
{
    aligned<Transform3d>::vector xform(skel.num_parameters());
    Transform3d *xptr = &xform[0];
    const double *ptr = &pose[0];
    local_pose_impl(skel, xptr, ptr, &pose[pose.size()]);
    return xform;
}


void global_xform_impl(const Joint& joint, Transform3d*& xform, Transform3d root)
{
    //Update transform matrix
    root.translate(joint.offset);
    root = root * (*xform);
    *(xform++) = root;
    
    for(int i=0; i<joint.children.size(); i++)
        global_xform_impl(joint.children[i], xform, root);
}

//Retrieves a global pose vector for the skeleton
aligned<Transform3d>::vector Frame::global_xform(const Joint& skel) const
{
    aligned<Transform3d>::vector xform = local_xform(skel);
    Transform3d root, *xptr = &xform[0];
    root.setIdentity();
    global_xform_impl(skel, xptr, root);
  
    return xform;
}

//Retrieves a point cloud for this frame/skeleton pair
aligned<Vector3d>::vector Frame::point_cloud(const Joint& skel) const
{
  aligned<Transform3d>::vector xform = global_xform(skel);
  aligned<Vector3d>::vector result(xform.size());
  for(int i=0; i<xform.size(); i++)
  {
    result[i] = xform[i].translation();
  }
  return result;
}


#endif

}
