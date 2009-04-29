#ifndef SKELETON_H
#define SKELETON_H

//STL
#include <iostream>
#include <string>
#include <vector>
#include <cassert>

//Vector arithmetic
#include <Eigen/Core>
#include <Eigen/Geometry>



#include <misc.hpp>

namespace Skeletal
{
  using namespace std;
  using namespace Eigen;

  //A node in the parse tree
  struct Joint
  {
    //Joint data
    string			name;
    Vector3d		offset;
    vector<string>	channels;
	vector<Joint>	children;
    
    //Constructors/assignment
    Joint() : name("NONAME"), offset(0,0,0) {}

    Joint(const string& name_,
        const Vector3d& offset_,
        const vector<string>& channels_,
        const vector<Joint>& children_) :
        name(name_),
        offset(offset_),
        channels(channels_),
        children(children_) {}
          
    Joint(const Joint& other) :
        name(other.name),
        offset(other.offset),
        channels(other.channels),
        children(other.children) {}
            
    Joint& operator=(const Joint& other)
    {
        name = other.name;
        offset = other.offset;
        channels = other.channels;
        children = other.children;
        return *this;
    }
      
    //Count the number of parameters in a frame for this skeleton
    int num_parameters() const;
    
    //Count number of joints in skeleton
    int size() const;
    
    //Reparameterizes a skeleton in terms of quaternions
    Joint convert_quat() const;
    
    //Recovers end effectors
    vector<int> find_end_effectors() const;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  };
  
  //A single fame in the animation
  struct Frame
  {
    vector<double>	pose;
    
    //Constructors
    Frame() {}
    Frame(int n) : pose(n) {}
    Frame(const vector<double>& pose_) : pose(pose_) {}
    Frame(const Frame& other) : pose(other.pose) {}
    Frame operator=(const Frame& other)
    {
        pose = other.pose;
        return *this;
    }
        
    //Reparemterizes the pose from base to target
    Frame reparameterize(const Joint& base, const Joint& target) const;
    
    //Apply a transformation to this pose
    Frame apply_transform(const Joint& skeleton, const Transform3d& xform) const;

#ifdef WIN32
    void local_pose_impl(const Joint joint,
                         Transform3d*& result, 
                         const double*& pbegin, 
                         const double* pend,
                         bool skip_transform = false) const;

    //Retrieves a local transform vector for the skeleton
    aligned<Transform3d>::vector local_xform(const Joint& skel) const
    {
        aligned<Transform3d>::vector xform(skel.num_parameters());
        Transform3d *xptr = &xform[0];
        const double *ptr = &pose[0];
        local_pose_impl(skel, xptr, ptr, NULL);//&pose[pose.size()]);
        return xform;
    }
    
    void global_xform_impl(const Joint& joint, Transform3d*& xform, Transform3d& root_ref) const;

    //Retrieves a global pose vector for the skeleton
    aligned<Transform3d>::vector global_xform(const Joint& skel) const
    {
        aligned<Transform3d>::vector xform = local_xform(skel);
        Transform3d root, *xptr = &xform[0];
        root.setIdentity();
        global_xform_impl(skel, xptr, root);
      
        return xform;
    }
    
    //Extracts a point cloud for this skeleton
    aligned<Vector3d>::vector point_cloud(const Joint& skel) const
    {
      aligned<Transform3d>::vector xform = global_xform(skel);
      aligned<Vector3d>::vector result(xform.size());
      for(int i=0; i<xform.size(); i++)
      {
        result[i] = xform[i].translation();
      }
      return result;
    }
  };
#else

    //Retrieves a local transform vector for the skeleton
    aligned<Transform3d>::vector local_xform(const Joint& skel) const;
    
    //Retrieves a global pose vector for the skeleton
    aligned<Transform3d>::vector global_xform(const Joint& skel) const;
    
    //Extracts a point cloud for this skeleton
    aligned<Vector3d>::vector point_cloud(const Joint& skeleton) const;

#endif
  //Interpolate two poses
  Frame interpolate_frames(const Joint& skel, const Frame& a, const Frame& b, double t);
  
  //Motion capture data structure
  struct Motion
  {
    double 			frame_time;
    vector<Frame>	frames;
    Joint			skeleton;
      
    Motion() : 
        frame_time(0.),
        frames(0) {}
    
    Motion(const Motion& other) :
        frame_time(other.frame_time),
        frames(other.frames),
        skeleton(other.skeleton) {}
    
    Motion(double frame_time_, const vector<Frame>& frames_, const Joint& skeleton_) :
        frame_time(frame_time_),
        frames(frames_),
        skeleton(skeleton_) {}
    
    Motion operator=(const Motion& other)
    {
        frame_time = other.frame_time;
        frames = other.frames;
        skeleton = other.skeleton;
        return *this;
    }
    
    //Returns the total duration of the animation
    double duration() const
    {
      return frames.size() * frame_time;
    }
    
    //Converts the motion to a quaternion parameterized motion
    Motion convert_quat() const;
    
    //Returns the frame at time t, with optional looping
    Frame get_frame(double t, bool loop = false) const;
    
    //Computes bounding sphere radius
    double bound_sphere_radius() const;
    
    //Constructs the bounding box
    void bounding_box(Vector3d& lo, Vector3d& hi) const;


#ifdef WIN32
    //Returns a window of point clouds
    // orig is the time about which to sample, extent is the radius of the window in +/- delta_t
    // n_samples are the number of times to sample the point cloud
    aligned<Vector4d>::vector point_cloud_window(double orig, double extent, int n_samples, double (*window)(double)) const
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
#else
    
    //Returns a window of point clouds
    // orig is the time about which to sample, extent is the radius of the window in +/- delta_t
    // n_samples are the number of times to sample the point cloud
    aligned<Vector4d>::vector point_cloud_window(double orig, double extent, int n_samples, double (*window)(double)) const;
    
#endif
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };
  
  //Constrains a transformation such that the y=0 plane is fixed
  Transform3d constrain_xform(const Transform3d& xform);
  
  //Computes the relative alignment of two motions
  Transform3d relative_xform(const Joint& skel, const Frame& base_frame, const Frame& target_frame);
  
  //Parses a BVH file from some input stream
  Motion parseBVH(istream& bvh_file);
  
  //Writes a BVH file to disk
  void writeBVH(ostream& bvh_file, const Motion& motion);
  
  //Combines two sections of motions together
  Motion combine_motions(
    const Motion& a, const Motion& b, 
    const Transform3d& relative_xform, 
    double a_start, double a_end,
    double b_start, double b_end, 
    double duration, 
    int deg);
    
  //Windowing functions
  double constant_window(double t);
};

#endif
