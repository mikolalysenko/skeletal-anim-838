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

//Size of the local min window for frame matching
const int WINDOW_SIZE = 5;


//Computes a relative transformation
Transform3d relative_xform(
	const aligned<Vector4d>::vector& a, 
	const aligned<Vector4d>::vector& b)
{
	assert(a.size() == b.size());

	double ax = 0., az = 0., aw = 0.,
			bx = 0., bz = 0., bw = 0.;
	
	for(int i=0; i<a.size(); i++)
	{
		ax += a[i].w() * a[i].x();
		az += a[i].w() * a[i].z();
		aw += a[i].w();
		
		bx += b[i].w() * b[i].x();
		bz += b[i].w() * b[i].z();
		bw += b[i].w();
	}
	
	ax /= aw;
	az /= aw;
	bx /= bw;
	bz /= bw;
	
	cerr << "abar = " << ax << ',' << az << endl
		 << "bbar = " << bx << ',' << bz << endl;
	
	double dx = 0., dz = 0., tw = 0.;
	
	for(int i=0; i<a.size(); i++)
	{
		double wt = a[i].w() * b[i].w();
		tw += wt;
		dx += wt * (a[i].x() * b[i].z() - b[i].x() * a[i].z());
		dz += wt * (a[i].x() * b[i].x() + a[i].z() * b[i].z());
	}
	
	dx /= tw;
	dz /= tw;
	dx -= ax * bz - bx * az;
	dz -= ax * bx + bz * az;
	
	double d = sqrt(dx * dx + dz * dz);
	assert(abs(d) >= 1e-8);
	
	dx /= d;
	dz /= d;

	Matrix3d rot;
	rot.setZero();
	
	rot(0, 0) = dz;
	rot(0, 2) = dx;
	rot(2, 0) = -dx;
	rot(2, 2) = dz;
	rot(1, 1) = 1.;
	
	Transform3d result;
	result.setIdentity();
	result = result.translate(Vector3d(ax, 0., az))
				   .rotate(rot)
					.translate(Vector3d(-bx, 0., -bz))
				   
				   ;
	
	return result;
}


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
    cerr << "da = " << da << endl;
    cerr << "db = " << db << endl;
    cerr << "(aw,bw) = " << a[i].w() << "," << b[i].w() << endl;
    */
    
    double d = (da - db).squaredNorm() * a[i].w() * b[i].w();
    s += d;
    //cerr << "\t\td = " << d << endl;
  }
  
  s = sqrt(s);
  assert(!isnan(s));
  assert(-1e10 <= s && s <= 1e10);
  //cout << "\t\t\t\ts = " << s <<endl;
  
  return s;
}

//Extracts a motion subsequence
Motion MotionGraph::submotion(int start, int end) const
{
	assert(0 <= start && start <= end && end <= frames.size());

	//cerr << "Getting sub motion : " << start << "," << end << endl;

	vector<Frame> tmp(end - start);
	copy(frames.begin() + start, frames.begin() + end, tmp.begin());
	
	
	return Motion(frame_time, tmp, skeleton);
}

//Extracts a point cloud
aligned<Vector4d>::vector MotionGraph::point_cloud(int f, double w, int n, double (*wind_func)(double)) const
{
	int start = max(f - (int)(.5 * w / frame_time), 0),
		end = 	min(f + 1 + (int)(.5 * w / frame_time), (int)frames.size());

	aligned<Vector4d>::vector result = submotion(start, end).point_cloud_window(.5 * w, w, n, wind_func);
	
	return result;	
}


//Inserts a motion into the MotionGraph
void MotionGraph::insert_motion(const Motion& motion, double threshold, double w, int n, double (*window_func)(double))
{
	//Get frame sizes
	int n_frames = motion.duration() / frame_time;
	int o_frames = frames.size();
	
	//Allocate data
	MatrixXd data(n_frames, o_frames + n_frames);

	int i = 0;
    for(double t=0., i=0; i<n_frames; i++, t+=frame_time)
    {
        //Extract window about t
        aligned<Vector4d>::vector cloud = motion.point_cloud_window(t, w, n, window_func);  
        
        //Add frame to frame set
        frames.push_back(motion.get_frame(t));
        
        //Add edges to graph
        vector<int> edges;
        if(i < n_frames-1)
        	edges.push_back(frames.size());
        graph.push_back(edges);
        
        //Add row to matrix
        for(int j=0; j<frames.size(); j++)
        {
        	aligned<Vector4d>::vector dest_cloud = point_cloud(j, w, n, window_func);
            Transform3d rel = relative_xform(dest_cloud, cloud);
        	data(i, j) = cloud_distance(cloud, dest_cloud, rel);
        	
        	if(j >= o_frames)
        		data(j - o_frames, i) = data(i, j);
        }
    }
    
    //Print out matrix for matlab analysis
    //cerr << data << endl;
    
    for(int i=0; i<n_frames; i++)
    for(int j=0; j<o_frames + n_frames; j++)
    {
    	//Check if i,j is a local minima
    	
    	double val = data(i, j);
    	
    	for(int dx=-2; dx<=2; dx++)
    	for(int dy=-2; dy<=2; dy++)
		{
			int nx = dx + i, ny = dy + j;
			
			if( nx < 0 || nx >= n_frames ||
				ny < 0 || ny >= n_frames + o_frames ||
				(dx == 0 && dy == 0))
				continue;
			
			if(val >= data(nx, ny))
				goto skip;
		}
		
		if(i != j - o_frames)
		{
			cerr << "Adding edge: " << i << ',' << j << endl;
			graph[i + o_frames].push_back(j);
		}
    	
	skip: continue;
    }
    
    /*
    //assert(false);
    exit(1);
    */
}


//Extracts all biconnected components
MotionGraph MotionGraph::extract_biconnected() const
{
	//TODO: Not yet implemented
    return MotionGraph(*this);
}

//Constant window function
double const_func(double t)
{
	return 1.;
}


//Do a random walk on the motion graph of length at most l, stop if you get stuck
Motion MotionGraph::random_motion(int l) const
{
    vector<Frame> random_frames;
    
    //Get initial frame
    int c = rand()%frames.size();
    random_frames.push_back(frames[c]);
    
    
    for(int i=0; i<l; i++)
    {
        if(graph[c].size() == 0)
            break;
        
        int r  = rand() % graph[c].size();
        int n = graph[c][r];
        
        //cerr << "n = " << n << endl;
        Frame next = frames[n];
        
        //Get local point clouds
        aligned<Vector4d>::vector 
        	pcloud = random_frames[random_frames.size()-1].point_cloudw(skeleton),
        	ncloud = frames[max(0, n-1)].point_cloudw(skeleton);
        
        //Compute relative transformation
        Transform3d rel = relative_xform(pcloud, ncloud);
			
		cerr << "rel_xform = " << rel.matrix() << endl;
        	
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
