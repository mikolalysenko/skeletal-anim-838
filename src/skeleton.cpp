//STL
#include <iostream>
#include <vector>
#include <string>

//Eigen
#include <Eigen/Core>
#include <Eigen/StdVector>

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
	int s = channels.size();
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


}
