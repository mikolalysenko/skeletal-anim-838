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
#include <misc.hpp>
#include <skeleton.hpp>
#include <mograph.hpp>

using namespace std;
using namespace Eigen;
using namespace Skeletal;

namespace Skeletal
{

//Point cloud distance with windows
double cloud_distance(const aligned<Vector4d>::vector& a, const aligned<Vector4d>::vector& b, const Transform3d& x)
{
  assert(a.size() == b.size());
  
  double s = 0.;
  for(int i=0; i<a.size(); i++)
  {
    Vector3d da = a[i].block(0,0,3,1),
             db = b[i].block(0,0,3,1);
    
    da = x * da;
    
    /*
    cout << "da = " << da << endl;
    cout << "db = " << db << endl;
    cout << "(aw,bw) = " << a[i].w() << "," << b[i].w() << endl;
      */
      
    double d = (da - db).squaredNorm() * a[i].w() * b[i].w();
    s += d;
    //cout << "\t\td = " << d << endl;
  }
  
  //cout << "\t\t\t\ts = " << s <<endl;
  
  return sqrt(s);
}

//Extracts a motion subsequence
Motion MotionGraph::submotion(int start, int end) const
{
	assert(0 <= start && start <= end && end <= frames.size());

	vector<Frame> tmp(end - start);
	copy(frames.begin() + start, frames.begin() + end, tmp.begin());
	return Motion(frame_time, tmp, skeleton);
}

//Extracts a point cloud
aligned<Vector4d>::vector MotionGraph::point_cloud(int f, double w, int n, double (*wind_func)(double)) const
{
	int start = max(f - (int)(.5 * w / frame_time), 0),
		end = 	min(f + 1 + (int)(.5 * w / frame_time), (int)frames.size());
	return submotion(start, end).point_cloud_window(.5 * w, w, n, wind_func);
}


//Inserts a motion into the MotionGraph
void MotionGraph::insert_motion(const Motion& motion, double threshold, double w, int n, double (*window_func)(double))
{
    for(double t=0.; t<motion.duration(); t+=frame_time)
    {
        //Extract window about i
        aligned<Vector4d>::vector cloud = motion.point_cloud_window(t, w, n, window_func);  
        vector<int> edges;
        /*
        if(t+frame_time<motion.duration())
            edges.push_back(frames.size());
        */
        
        for(int j=0; j<frames.size(); j++)
        {
            //Compute distance metric
            Transform3d rel = relative_xform(skeleton, frames[j], motion.get_frame(t));
            double dist = cloud_distance(cloud, point_cloud(j, w, n, window_func), rel);
            
            if(dist < threshold)
                edges.push_back(j+1);
        }
        
        //Add frame to graph
        frames.push_back(motion.get_frame(t));
        graph.push_back(edges);
    }
}

//Extracts all biconnected components
MotionGraph MotionGraph::extract_biconnected() const
{
	//Not yet implemented
    return MotionGraph(*this);
}

//Do a random walk on the motion graph of length at most l, stop if you get stuck
Motion MotionGraph::random_motion(int l) const
{
    vector<Frame> random_frames;
    
    int c = rand()%frames.size();
    random_frames.push_back(frames[c]);
    
    for(int i=0; i<l; i++)
    {
        if(graph[c].size() == 0)
            break;
        
        int n = graph[c][rand() % graph[c].size()];
        Frame next = frames[n];
        Transform3d rel = relative_xform(skeleton, random_frames[random_frames.size()-1], next);
        random_frames.push_back(next.apply_transform(skeleton, rel));
        
        c = n;
    }

    return Motion(frame_time, random_frames, skeleton);
}


extern void assert_token(istream& file, const string& tok);

//Parses a motion graph from the input stream
MotionGraph parseMotionGraph(istream& is)
{
	MotionGraph g;
	assert_token(is, "MOGRAPH");
	g.skeleton = parseJoint(is);

	assert_token(is, "Frames:");
	int n_frames;
	if(!(is >> n_frames) || n_frames < 0)
		throw "Invalid frame count";
	
	assert_token(is, "Frame");
	assert_token(is, "Time:");
	if(!(is >> g.frame_time) || g.frame_time <= 0.)
		throw "Invalid frame time";
	
	//Read in pose state
	assert_token(is, "POSE");
	g.frames.resize(n_frames);
	
	int n_params = g.skeleton.num_parameters();
	for(int i=0; i<g.frames.size(); i++)
	{
		g.frames[i].pose.resize(n_params);
		for(int j=0; j<n_params; j++)
			if(!(is >> g.frames[i].pose[j]))
				throw "Unexpected EOF in pose";
				
	}
	
	//Read in graph
	assert_token(is, "EDGES");
	
	for(int i=0; i<n_frames; i++)
	{
		int deg;
		if(!(is >> deg) || deg < 0)
			throw "Invalid degree";
		
		vector<int> edge_list(deg);
		
		for(int j=0; j<deg; j++)
			if(!(is >> edge_list[j]) || edge_list[j] < 0 || edge_list[j] >= n_frames)
				throw "Invalid edge";

		g.graph.push_back(edge_list);
	}
	
	return g;
}

//Writes a motion graph to the output stream
void writeMotionGraph(ostream& os, const MotionGraph g)
{
	os << "MOGRAPH" << endl;
	writeJoint(os, g.skeleton);
	
	os	<< "Frames: " << g.frames.size() << endl
		<< "Frame Time: " << g.frame_time << endl;
	
	os << "POSE" << endl;
	
	for(int i=0; i<g.frames.size(); i++)
	{
		for(int j=0; j<g.frames[i].pose.size(); j++)
			os << g.frames[i].pose[j] << ' ';
		os << endl;
	}
	
	os << "EDGES" << endl;
	for(int i=0; i<g.graph.size(); i++)
	{
		os << g.graph[i].size();
		for(int j=0; j<g.graph[i].size(); j++)
			os << ' ' << g.graph[i][j];
		os << endl;
	}
}

}
