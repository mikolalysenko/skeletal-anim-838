
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
	Joint result;
	if(!(bvh_file >> result.name))
		throw string("Expected name for joint");
	
	//Handle blank name
	if(result.name == "{")
		result.name = "NONAME";
	else
		assert_token(bvh_file, "{");
	
	//Set initial offset to 0
	result.offset = Vector3d(0., 0., 0.);
	
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
			result.offset += v;
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
				result.channels.push_back(chan_name);
			}
		}
		else if(tok == "JOINT")
		{
			result.children.push_back(parseJoint(bvh_file));
		}
		else if(tok == "End")
		{
			assert_token(bvh_file, "Site");
			result.children.push_back(parseJoint(bvh_file));
		}
		else if(tok == "}")
		{
			return result;
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
	Motion result;

	//Read in the hierarchy of the file
	assert_token(bvh_file, "HIERARCHY");
	assert_token(bvh_file, "ROOT");
	result.skeleton = parseJoint(bvh_file);
	
	//Read in mocap data
	assert_token(bvh_file, "MOTION");
	
	//Read frame count
	assert_token(bvh_file, "Frames:");
	int num_frames;
	if(!(bvh_file >> num_frames))
		throw string("Unexpected EOF.");
	if(num_frames <= 0)
		throw string("Incorrect frame count");
	result.frames.resize(num_frames);
	
	//Read frame time
	assert_token(bvh_file, "Frame");
	assert_token(bvh_file, "Time:");
	double frame_time;
	if(!(bvh_file >> result.frame_time))
		throw string("Unexpected EOF.");
		
	//Get number of parameters
	int param_count = result.skeleton.num_parameters();
	
	//Read in frames
	for(int i=0; i<num_frames; i++)
	{
		result.frames[i].pose.resize(param_count);
		for(int j=0; j<param_count; j++)
		{
			if(!(bvh_file >> result.frames[i].pose[j]))
				throw string("Invalid number of parameters");
		}
	}

  // compute the bounding box for the skeleton
  Vector3d offset = result.skeleton.offset;
  Vector3d min_pt = offset;
  Vector3d max_pt = offset;
  compute_bounding_box(offset, 
                       result.skeleton, 
                       min_pt, 
                       max_pt);

  // compute the bounding box for the motion
  compute_bounding_box(result,
                       result.bound_box_min,
                       result.bound_box_max);

  // compute the bounding sphere radius
  result.bound_sphere_radius = sqrt(max(min_pt.dot(min_pt), max_pt.dot(max_pt)));


	return result;
}


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
		else if(chan == "Xrotation")
		{
			//TODO: Implement this
		}
		else if(chan == "Yrotation")
		{
			//TODO: Implement this
		}
		else if(chan == "Zrotation")
		{
			//TODO: Implement this
		}
		else if(chan == "Xposition")
		{
			double x = xform.translation().x();
			xform.pretranslate(Vector3d(x, 0., 0.));
			n_params.push_back(x);
		}
		else if(chan == "Yposition")
		{
			double x = xform.translation().y();
			xform.pretranslate(Vector3d(0., x, 0.));
			n_params.push_back(x);
		}
		else if(chan == "Zposition")
		{
			double x = xform.translation().z();
			xform.pretranslate(Vector3d(0., 0., x));
			n_params.push_back(x);
		}
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
	vector<Transform3d> target_xform(base.size());
	base.interpret_pose(target_xform.begin(), pose.begin(), pose.end());

	//Construct result
	Frame result;
	Transform3d* ptr = &target_xform[0];
	reparameterize_impl(result.pose, ptr, target);
	return result;
}

//Converts the motion into a quaternion parameterized motion, thus removing the dependency on Euler angles
Motion Motion::convert_quat() const
{
	Motion result;
	result.frame_time = frame_time;
	result.skeleton = skeleton.convert_quat();
	result.frames.resize(frames.size());
  result.bound_sphere_radius = bound_sphere_radius;
  result.bound_box_min = bound_box_min;
  result.bound_box_max = bound_box_max;

	for(int i=0; i<frames.size(); i++)
		result.frames[i] = frames[i].reparameterize(skeleton, result.skeleton);
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
		}
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
	return t;
}

//Combines two motions to synthesize a new motion
Motion combine_motions(
	const Motion& a, 
	const Motion& b, 
	const Transform3d& relative_xform, 
	double a_start, 
	double b_start, 
	double duration, 
	int deg)
{
	assert(a_start >= 0. && a_start + duration <= a.duration());
	assert(b_start >= 0. && b_start + duration <= b.duration());
	assert(a.skeleton.size() == b.skeleton.size());
	assert(a.skeleton.num_parameters() == b.skeleton.num_parameters());

	Motion result;
	result.skeleton = a.skeleton;
	result.frame_time = min(a.frame_time, b.frame_time);
	result.frames.resize(0);
	
	for(double t = 0.; t<=a_start; t+=result.frame_time)
		result.frames.push_back(a.get_frame(t));
	
	for(double t = 0.; t<=duration; t+=result.frame_time)
	{
		Frame fa = a.get_frame(a_start + t),
			  fb = b.get_frame(b_start + t).apply_transform(result.skeleton, relative_xform);
		result.frames.push_back(interpolate_frames(a.skeleton, fa, fb, interp_func(t / duration, deg)));
	}

	for(double t=b_start+duration; t<=b.duration(); t+=result.frame_time)
		result.frames.push_back(b.get_frame(t).apply_transform(result.skeleton, relative_xform));
	
	return result;	
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
  assert(a_end > a_start);
  assert(b_end > b_start);
	assert(a_end >= 0. && a_end <= a.duration());
	assert(b_start >= 0. && b_start <= b.duration());
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
		Frame fa = a.get_frame(a_end + t),
			  fb = b.get_frame(b_start + t).apply_transform(result.skeleton, relative_xform);
		result.frames.push_back(interpolate_frames(a.skeleton, fa, fb, interp_func(t / duration, deg)));
	}

	for(double t=b_start+duration; t<=b_end; t+=result.frame_time)
		result.frames.push_back(b.get_frame(t).apply_transform(result.skeleton, relative_xform));
	
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
		}
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


// compute the bounding box for a skeleton
void compute_bounding_box(Vector3d offset, 
                          const Joint &skeleton, 
                          Vector3d &min_pt, 
                          Vector3d &max_pt)
{
  
  // update the min and max point if necessary
  offset += skeleton.offset;
  if ( offset[0] < min_pt[0] ) min_pt[0] = offset[0];
  if ( offset[1] < min_pt[1] ) min_pt[1] = offset[1];
  if ( offset[2] < min_pt[2] ) min_pt[2] = offset[2];
  if ( offset[0] > max_pt[0] ) max_pt[0] = offset[0];
  if ( offset[1] > max_pt[1] ) max_pt[1] = offset[1];
  if ( offset[2] > max_pt[2] ) max_pt[2] = offset[2];

  // loop through all the joints
  for(int i=0; i<skeleton.children.size(); i++)
		compute_bounding_box(offset, 
                         skeleton.children[i], 
                         min_pt, 
                         max_pt);

}
	

// compute the bounding box for a motion
void compute_bounding_box(const Motion& motion, 
                          Vector3d &min_pt, 
                          Vector3d &max_pt)
{
  Vector3d offset;
  min_pt[0] = motion.frames[0].pose[0];
  min_pt[1] = motion.frames[0].pose[1];
  min_pt[2] = motion.frames[0].pose[2];
  max_pt[0] = motion.frames[0].pose[0];
  max_pt[1] = motion.frames[0].pose[1];
  max_pt[2] = motion.frames[0].pose[2];

  // loop through all the joints
  for(int i=0; i<motion.frames.size(); i++)
  {
    offset[0] = motion.frames[i].pose[0];
    offset[1] = motion.frames[i].pose[1];
    offset[2] = motion.frames[i].pose[2];
    if ( offset[0] < min_pt[0] ) min_pt[0] = offset[0];
    if ( offset[1] < min_pt[1] ) min_pt[1] = offset[1];
    if ( offset[2] < min_pt[2] ) min_pt[2] = offset[2];
    if ( offset[0] > max_pt[0] ) max_pt[0] = offset[0];
    if ( offset[1] > max_pt[1] ) max_pt[1] = offset[1];
    if ( offset[2] > max_pt[2] ) max_pt[2] = offset[2];
  }
}

  

}
