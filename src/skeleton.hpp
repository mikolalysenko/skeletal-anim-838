#ifndef SKELETON_H
#define SKELETON_H


//Vector arithmetic
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>


//STL
#include <iostream>
#include <string>
#include <vector>
#include <cassert>

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
		
		//Count the number of parameters in a frame for this skeleton
		int num_parameters() const;
		
		//Count number of joints in skeleton
		int size() const;
		
		//Reparameterizes a skeleton in terms of quaternions
		Joint convert_quat() const;
		
		//Constructs a pose from parameters
		template<class XformIter, class ParamIter>
			void interpret_pose(XformIter result, ParamIter pbegin, ParamIter pend) const
		{
			_interpret_pose_impl(result, pbegin, pend);
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
	private:
	
		//Interprets the pose
		template<class XformIter, class ParamIter>
			void _interpret_pose_impl(XformIter& result, ParamIter& pbegin, ParamIter pend) const
		{
			Transform3d xform;
			xform.setIdentity();
			
			//Interpret parameters
			for(int i=0; i<channels.size(); i++)
			{
				//Make sure we haven't walked past end of parameters
				assert(pbegin != pend);
				
				double p = *(pbegin++);
				
				//Interpret parameter
				if(channels[i] == "Xrotation")
					xform.rotate(AngleAxisd(p * M_PI / 180., Vector3d::UnitX()));
				else if(channels[i] == "Yrotation")
					xform.rotate(AngleAxisd(p * M_PI / 180., Vector3d::UnitY()));
				else if(channels[i] == "Zrotation")
					xform.rotate(AngleAxisd(p * M_PI / 180., Vector3d::UnitZ()));
				else if(channels[i] == "Xposition")
					xform.translate(Vector3d(p, 0, 0));
				else if(channels[i] == "Yposition")
					xform.translate(Vector3d(0, p, 0));
				else if(channels[i] == "Zposition")
					xform.translate(Vector3d(0, 0, p));
				else if(channels[i] == "Quaternion")
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
			for(int i=0; i<children.size(); i++)
				children[i]._interpret_pose_impl(result, pbegin, pend);
		}
	};
	
	//A single fame in the animation
	struct Frame
	{
		vector<double>	pose;
		
		//Reparemterizes the pose from base to target
		Frame reparameterize(const Joint& base, const Joint& target) const;
		
		//Apply a transformation to this pose
		Frame apply_transform(const Joint& skeleton, const Transform3d& xform) const;
	};

	//Interpolate two poses
	Frame interpolate_frames(const Joint& skel, const Frame& a, const Frame& b, double t);
	
	//Motion capture data structure
	struct Motion
	{
		double 			frame_time;
		vector<Frame>	frames;
		Joint			skeleton;
    double bound_sphere_radius;
    Vector3d bound_box_min;
    Vector3d bound_box_max;
		
		//Returns the total duration of the animation
		double duration() const
		{
			return frames.size() * frame_time;
		}
		
		//Converts the motion to a quaternion parameterized motion
		Motion convert_quat() const;
		
		//Returns the frame at time t, with optional looping
		Frame get_frame(double t, bool loop = false) const;
	};
	
	//Parses a BVH file from some input stream
	Motion parseBVH(istream& bvh_file);
	
	//Combines two motions together
	Motion combine_motions(
		const Motion& a, const Motion& b, 
		const Transform3d& relative_xform, 
		double a_start, double b_start, double duration, 
		int deg);

	//Combines two sections of motions together
	Motion combine_motions(
		const Motion& a, const Motion& b, 
		const Transform3d& relative_xform, 
		double a_start, double a_end,
    double b_start, double b_end, 
    double duration, 
		int deg);

  // compute the bounding box for a skeleton
  void compute_bounding_box(
    Vector3d offset, 
    const Joint &skeleton, 
    Vector3d &min_pt, 
    Vector3d &max_pt);

  
  // compute the bounding box for a motion
  void compute_bounding_box(
    const Motion& motion, 
    Vector3d &min_pt, 
    Vector3d &max_pt);

  // compute the bounding box for a pose
  template<class ParamIter>
  void compute_bounding_box( 
    Transform3d& xform_ref, 
    const Joint& skeleton, 
    ParamIter pose_begin, 
    ParamIter pose_end,
    Vector3d &min_pt, 
    Vector3d &max_pt)
  {
	  assert(pose_begin != pose_end);

    Transform3d xform = xform_ref;

	  //Construct joint transform
	  Transform3d base_xform = *(pose_begin++);
	  xform.translate(skeleton.offset);
	  xform = xform * base_xform;

    Vector3d offset = xform.translation();
    if ( offset[0] < min_pt[0] ) min_pt[0] = offset[0];
    if ( offset[1] < min_pt[1] ) min_pt[1] = offset[1];
    if ( offset[2] < min_pt[2] ) min_pt[2] = offset[2];
    if ( offset[0] > max_pt[0] ) max_pt[0] = offset[0];
    if ( offset[1] > max_pt[1] ) max_pt[1] = offset[1];
    if ( offset[2] > max_pt[2] ) max_pt[2] = offset[2];
  		
    for(int i=0; i<skeleton.children.size(); i++)
	    compute_bounding_box(xform, skeleton.children[i], pose_begin, pose_end, min_pt, max_pt);
  };
  


};

#endif

