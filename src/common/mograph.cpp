//Eigen
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/QR>

//STL
#include <iostream>
#include <vector>
#include <stack>
#include <string>
#include <algorithm>
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
					.translate(Vector3d(-bx, 0., -bz));
	
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
    
    double d = (da - db).squaredNorm() * a[i].w() * b[i].w();
    s += d;
  }
  
  s = sqrt(s);
  
  assert(!isnan(s));
  assert(-1e10 <= s && s <= 1e10);
  
  return s;
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

	aligned<Vector4d>::vector result = submotion(start, end).point_cloud_window(.5 * w, w, n, wind_func);
	
	return result;	
}

//New idea: Building motion graphs is too slow.  Try using Newton's method to find local minima instead of brute force search
struct NewtonSearch
{
	const MotionGraph * mog;
	const Motion * motion;
	double threshold, window_size;
	double (*window_func)(double);
	int n_samples, n_frames, o_frames;
	
	float eval(const Vector2f& pt) const
	{
        //Extract window about t
        aligned<Vector4d>::vector 
        	pcloud = mog->point_cloud((int)pt.x(), window_size, n_samples, window_func),
        	ncloud = mog->point_cloud((int)pt.y(), window_size, n_samples, window_func);
        	
        Transform3d rel = relative_xform(ncloud, pcloud);
        
		return cloud_distance(pcloud, ncloud, rel);
	}
	
	Vector2f grad(const Vector2f& pt) const
	{
		Vector2f sum = Vector2f(0., 0.);
		float g0 = eval(pt);
		
		for(int dx=-2; dx<=2; dx++)
		for(int dy=-2; dy<=2; dy++)
		{
			if(dx == 0 && dy == 0) continue;
			
			Vector2f delta(dx, dy);
			sum += delta * ((g0 - eval(pt + delta)) / delta.norm());
		}
		
		return sum;
	}

	//Finds a local min	
	bool find_min(Vector2f& x0) const
	{
		float val = eval(x0);
		
		for(int i=0; i<=100; i++)
		{
			Vector2f delta = grad(x0);
			Vector2f x1 = x0 - val * Vector2f(1. / delta.x(), 1. / delta.y());
			
			//Enforce boundary constraints
			x1.x() = max(min(x1.x(), (float)(o_frames + n_frames - 2)), 0.f);
			x1.y() = max(min(x1.y(), (float)(o_frames + n_frames - 2)), (float)o_frames);
			
			float n_val = eval(x1);
			
			if(n_val >= val)
				return val <= threshold;
			
			//Update and continue
			val = n_val;
			x0 = x1;
		}
		
		return false;
	}
};

//Do a fast insertion using Newton's method to find local minima (approx O(n) instead of O(n^2))
void MotionGraph::insert_motion_fast(const Motion& motion, double threshold, double w, int n, double (*window_func)(double))
{
	//Get frame sizes
	int n_frames = motion.duration() / frame_time;
	int o_frames = frames.size();
	
	int i = 0;
    for(double t=0., i=0; i<n_frames; i++, t+=frame_time)
    {
        //Add frame to frame set
        frames.push_back(motion.get_frame(t));
        
        //Add edges to graph
        vector<int> edges;
        if(i < n_frames-1)
        	edges.push_back(frames.size());
        graph.push_back(edges);
    }
    
    //Do a fast gradient descent method
    NewtonSearch search;
    search.mog = this;
	search.motion = &motion;
	search.threshold = threshold;
	search.window_size = w;
	search.window_func = window_func;
	search.n_samples = n;
	search.n_frames = n_frames;
	search.o_frames = o_frames;
    
    //Do about O(n) iterations
    for(int i=0; i<motion.frames.size() * 2; i++)
    {
    	Vector2f start = Vector2f(rand() % (n_frames + o_frames - 1), (rand() % (n_frames - 1)) +  o_frames);
    	
    	if(!search.find_min(start))
    		continue;
    	
    	//Check if edge already exists
    	int u = start.x(), v = start.y();
    	if(abs(u - v) <= 3)
    		continue;
    	
		for(int i=0; i<graph[u].size(); i++)
			if(graph[u][i] == v)
				goto skip;
    	
    	//cerr << "Adding edge: " << u << "," << v << endl;
    	
    	//Add edge to graph
    	graph[u].push_back(v + 1);
		graph[v].push_back(u + 1);

		skip: continue;
    }
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
    
    //Print out matrix for debugging
    //cerr << data << endl;
    
    for(int i=0; i<n_frames - 1; i++)
    for(int j=0; j<o_frames + i; j++)
    {
    	//Check if i,j is a local minima
    	
    	double val = data(i, j);
    	
    	if(val >= threshold) continue;
    	
    	//Check
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
		
		//Add edges to graph
		graph[j].push_back(i + o_frames + 1);
		graph[i + o_frames].push_back(j + 1);
		
		//cerr << "Adding edge: " << j << "," << (i + o_frames) << endl;
    	
	skip: continue;
    }
}

//Computes finish times for each node
void dfs(
	int node,
	int& finish_time,
	vector<int>& finish,
	const vector< vector<int> >& graph)
{
	if(finish[node] != -1)
		return;
	
	finish[node] = finish_time++;
	
	for(int i=0; i<graph[node].size(); i++)
		dfs(graph[node][i], finish_time, finish, graph);
		
	finish[node] = finish_time++;
}

//Extracts all biconnected components
vector<MotionGraph> MotionGraph::extract_scc() const
{
	//Initialize finish times
	vector<int> finish(frames.size());
	fill(finish.begin(), finish.end(), -1);
	
	//Compute finish times for forward DFS
	for(int i=0, finish_time = 0; i<frames.size(); i++)
	if(finish[i] == -1)
		dfs(i, finish_time, finish, graph);

	//Construct G^T, event order
	vector< vector<int> > graphT(frames.size());
	vector< pair<int, int> > ordered(frames.size());
	for(int i=0; i<graph.size(); i++)	
	{
		for(int j=0; j<graph[i].size(); j++)
			graphT[graph[i][j]].push_back(i);
		ordered[i] = make_pair(finish[i], i);
	}	
	sort(ordered.begin(), ordered.end(), greater< pair<int,int> >());
	
	//Extract each connected component
	vector< MotionGraph > result;

	//Initialize visit buffer
	vector<bool> visited(frames.size());
	fill(visited.begin(), visited.end(), false);

	//Visit frames in reverse order
	for(int i=0; i<frames.size(); i++)
	{
		int n = ordered[i].second;
		if(visited[n]) continue;
		
		//Extract subgraph
		vector<int> subgraph;
		stack<int> to_visit;
		to_visit.push(n);
		
		//Extract tree rooted at this DFS point
		while(!to_visit.empty())
		{
			int v = to_visit.top();
			to_visit.pop();
			
			if(visited[v])
				continue;
			visited[v] = true;
			
			subgraph.push_back(v);
			
			for(int i=0; i<graphT[v].size(); i++)
				to_visit.push(graphT[v][i]);
		}
		
		//Throw out 1 node motion graphs because they are stupid
		if(subgraph.size() == 1)
			continue;
		sort(subgraph.begin(), subgraph.end());
		
		//Build motion graph
		MotionGraph mog(skeleton);
		mog.frame_time = frame_time;
		
		//Put frames back into motion graph
		map<int, int> frame_indexes;
		for(int i=0; i<subgraph.size(); i++)
		{
			int v = subgraph[i];
			
			//Needed for continuitity
			if(v > 0 && (i == 0) || (subgraph[i-1] != v-1))
				mog.frames.push_back(frames[v-1]);
			
			//Add frame index
			frame_indexes[v] = mog.frames.size();
			mog.frames.push_back(frames[v]);
			
			if(v < frames.size() && (i == subgraph.size() - 1) || (subgraph[i+1] != v+1))
				mog.frames.push_back(frames[v+1]);
		}
		
		//Put edges back into motion graph
		mog.graph.resize(mog.frames.size());
		for(int i=0; i<mog.graph.size(); i++)
			mog.graph[i].resize(0);
		
		for(int i=0; i<subgraph.size(); i++)
		{
			int v = subgraph[i];
			int x = frame_indexes[v];
			
			for(int j=0; j<graph[v].size(); j++)
			{
				int u = graph[v][j];
			
				//Check that edge u,v is contained in subgraph
				if(frame_indexes.find(u) == frame_indexes.end())
					continue;
				mog.graph[x].push_back(frame_indexes[u]);
			}
		}
		
		result.push_back(mog);
	}

    return result;
}



//Synthesizes a motion
Motion MotionGraph::synthesize_motion(const vector<int> frame_seq, const Transform3d& base_pose) const
{
	//Initialize frame result
	vector<Frame> result;
	result.push_back(frames[frame_seq[0]].apply_transform(skeleton, base_pose));
	
	Transform3d pxform = base_pose;
	
	for(int i=1; i<frame_seq.size(); i++)
	{
		//Extract point clouds
		aligned<Vector4d>::vector
			pcloud = result[result.size() - 1].point_cloudw(skeleton),
			ncloud = frames[max(frame_seq[i]-1, 0)].point_cloudw(skeleton);
			
		//Compute relative xform
		Transform3d rel = relative_xform(pcloud, ncloud);
		Frame next = frames[frame_seq[i]].apply_transform(skeleton, rel);
		
		//Smooth animation slightly
		result[i-1] = interpolate_frames(skeleton, result[i-1], frames[frame_seq[i]-1].apply_transform(skeleton, rel), 0.5);
		
		//Store result
		result.push_back(next);
		pxform = rel;
	}
	
	//Return the final motion sequence
	return Motion(frame_time, result, skeleton);
}


//Do a random walk on the motion graph of length at most l, stop if you get stuck
Motion MotionGraph::random_motion(int l) const
{
	vector<int> seq;

    //Get initial frame
    int c = rand()%frames.size();
    while(graph[c].size() == 0)
    	c = rand() % frames.size();
    seq.push_back(c);
    
    //Generate a bunch of random frames
    for(int i=0; i<l; i++)
    {
        if(graph[c].size() == 0)
            break;
        
        seq.push_back( c = graph[c][rand() % graph[c].size()] );
    }

	//Synthesize motion from frame sequence
    Transform3d start_pose;
    start_pose.setIdentity();
    return synthesize_motion(seq, start_pose);
}

//Performs the path DFS
struct Traversal
{
	Vector2f (*path_func)(double t);
	vector<int> visit_frames;
	const MotionGraph* mog;
	double max_d;
	
	Transform2f rel2D(int a, int b)
	{
		//Extract point clouds
		aligned<Vector4d>::vector
			pcloud = mog->frames[a].point_cloudw(mog->skeleton),
			ncloud = mog->frames[b].point_cloudw(mog->skeleton);
		
		//Compute relative xform
		Matrix4d x3d = relative_xform(pcloud, ncloud).matrix();
		
		//Extract x-z component
		Matrix3f mat;
		mat.setIdentity();
		mat(0,0) = x3d(0,0);
		mat(0,1) = x3d(0,2);
		mat(0,2) = x3d(0,3);
		mat(1,0) = x3d(2,0);
		mat(1,1) = x3d(2,2);
		mat(1,2) = x3d(2,3);
		
		return Transform2f(mat);
	}

	//Extracts the base point for the anim	
	Vector2f base_pt(int frame, const Transform2f& pose) const
	{
		aligned<Vector4d>::vector pcloud = mog->frames[frame].point_cloudw(mog->skeleton);
		Vector2f pt = Vector2f(pcloud[0].x(), pcloud[0].z());
		return pose * pt;
	}
	
	Vector2f vel(int frame, const Transform2f& pose) const
	{
		return base_pt(frame, pose) - base_pt(max(frame-1, 0), pose);
	}
	
	bool dfs(int frame, int t, Transform2f pose)
	{
		//Success!
		if(t >= visit_frames.size())
			return true;
	
		//Check we are still in bounds
		double t0 = t * mog->frame_time;
		Vector2f tpos = path_func(t0),
				 cpos = base_pt(frame, pose);
		
		if((tpos - cpos).squaredNorm() > max_d)
			return false;
		
		//assert(frame < 100);
		
		//Mark frame as visited
		visit_frames[t] = frame;
		
		//Generate successor states
		for(int i=0; i<mog->graph[frame].size(); i++)
		{
			int n = mog->graph[frame][i];
			Transform2f rel_xform = rel2D(frame, max(n-1, 0)) * pose;
			
			if(dfs(n, t+1, rel_xform))
				return true;
		}
		
		//fail
		return false;
	}
};


//Synthesizes a motion following along the path function
Motion MotionGraph::follow_path(Vector2f (*path_func)(double t), double max_d, double dur) const
{
	Vector2f start_pt = path_func(0.);
	Vector2f heading = (path_func(frame_time) - start_pt).normalized();
	double base_theta = atan2(heading.y(), heading.x());
	
	//Allocate data for traversal
	Traversal trav;
	trav.path_func = path_func;
	trav.visit_frames.resize(dur / frame_time);
	trav.mog = this;
	trav.max_d = max_d * max_d;

	//Initialize traversal queue	
	for(int i=1; i<frames.size() - 1; i++)
	{
		//Construct target pose vector
		aligned<Transform3d>::vector target = frames[i].local_xform(skeleton);
		Transform3d root3d = constrain_xform(Transform3d(target[0].inverse()) );
		
		//Convert root3d into 2d 
		Matrix4d rot3 = root3d.matrix();
		Matrix3f rot2;
		rot2.setIdentity();
		rot2(0,0) = rot3(0,0);
		rot2(0,1) = rot3(0,2);
		rot2(0,2) = rot3(0,3);
		rot2(1,0) = rot3(2,0);
		rot2(1,1) = rot3(2,2);
		rot2(1,2) = rot3(2,3);
		Transform2f root(rot2);
	
		//Set up initial transform
		Vector2f mesh_vel = trav.vel(i, root);
		double mesh_theta = atan2(mesh_vel.y(), mesh_vel.x());
		if(mesh_vel.squaredNorm() <= 1e-6)
			mesh_theta = 0.;
			
		/*
		cerr << "Heading = " << heading << endl;
		cerr << "Base theta = " << base_theta << endl;
		cerr << "Mesh vel = " << mesh_vel << endl;
		cerr << "Mesh theta = " << mesh_theta << endl;
		*/
		
		double ang = mesh_theta - base_theta;
		
		Matrix2f rot;
		rot(0,0) = cos(ang);
		rot(0,1) = sin(ang);
		rot(1,0) = -sin(ang);
		rot(1,1) = cos(ang);
	
		//Set up matrix
		Transform2f rel_target;
		rel_target.setIdentity();
		rel_target = rel_target.translate(start_pt).rotate(rot) * root;
		
		//cerr << "Initial velocity: " << trav.vel(i, rel_target) << endl;
		
		//Do DFS
		if(trav.dfs(i, 0, rel_target))
		{
			//Convert pose to 3D
			Matrix3f rel = rel_target.matrix();
			Matrix4d tmp = Matrix4d::Identity();
			tmp(0,0) = rel(0,0);
			tmp(0,2) = rel(0,1);
			tmp(0,3) = rel(0,2);
			tmp(2,0) = rel(1,0);
			tmp(2,2) = rel(1,1);
			tmp(2,3) = rel(1,2);
		
			return synthesize_motion(trav.visit_frames, Transform3d(tmp));
		}
	}
	
	//Did not find, motion return failure
	return Motion(frame_time, vector<Frame>(0), skeleton);
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
void writeMotionGraph(ostream& os, const MotionGraph& g)
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

//Create point cloud map
void MotionGraph::create_point_cloud_map(MatrixXd& data,
                                         double& max_distance,
                                         double w, 
                                         int n, 
                                         double (*window_func)(double))
{
    //Get frame sizes
    int num_frames = frames.size();
   
    //Allocate data
    data = MatrixXd::Zero(num_frames, num_frames);
    max_distance = 0.;

    for(int i=0; i<num_frames; i++)
    {
        aligned<Vector4d>::vector cloud = point_cloud(i, w, n, window_func); 
       
        for(int j=0; j<num_frames; j++)
        {
            aligned<Vector4d>::vector dest_cloud = point_cloud(j, w, n, window_func);
            Transform3d rel = relative_xform(dest_cloud, cloud);
            data(i, j) = cloud_distance(cloud, dest_cloud, rel);
            if(data(i, j) > max_distance) max_distance = data(i, j);
        }
    }


}

}
