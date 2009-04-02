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
		
		//Interprets the pose
		template<class XformIter, class ParamIter>
			void _interpret_pose_impl(Transform3d xform, XformIter& result, ParamIter& pbegin, ParamIter pend) const
		{
			xform.translate(offset);
			
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
				else assert(false);
			}
			
			//Store result
			*(result++) = xform;
			
			//Compute transforms for children
			for(int i=0; i<children.size(); i++)
				children[i]._interpret_pose_impl(xform, result, pbegin, pend);
		}
		
		
		template<class XformIter, class ParamIter>
			void interpret_pose(XformIter result, ParamIter pbegin, ParamIter pend) const
		{
			Transform3d xform;
			xform.setIdentity();
			_interpret_pose_impl(xform, result, pbegin, pend);
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	
	//A single fame in the animation
	struct Frame
	{
		vector<double>	pose;
	};
	
	//Motion capture data structure
	struct Motion
	{
		double 			frame_time;
		vector<Frame>	frames;
		Joint			skeleton;
	};
	
	//Parses a BVH file from some input stream
	Motion parseBVH(istream& bvh_file);
};

#endif

