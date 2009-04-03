//STL
#include <iostream>
#include <vector>
#include <string>

//Eigen
#include <Eigen/Core>
#include <Eigen/StdVector>
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
			xform.translate(Vector3d(-x, 0., 0.));
			n_params.push_back(x);
		}
		else if(chan == "Yposition")
		{
			double x = xform.translation().y();
			xform.translate(Vector3d(0., -x, 0.));
			n_params.push_back(x);
		}
		else if(chan == "Zposition")
		{
			double x = xform.translation().z();
			xform.translate(Vector3d(0., 0., -x));
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
		else if(pname == "Xrotation")
		{
			//TODO: Implement this
		}
		else if(pname == "Yrotation")
		{
			//TODO: Implement this
		}
		else if(pname == "Zrotation")
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


}
