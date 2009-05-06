#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/StdVector>

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>

#include <misc.hpp>
#include <skeleton.hpp>
#include <mograph.hpp>

using namespace std;
using namespace Eigen;
using namespace Skeletal;


//Spline stuff
aligned<Vector3f>::vector control_points;
double t_max;

void read_control_points(istream& data)
{
	int n_points;
	if(!(data >> n_points) || n_points <= 0)
		throw "Invalid number of control points for spline curve";
		
	control_points.resize(n_points);
	for(int i=0; i<n_points; i++)
	{
		if(!(data >> control_points[i].x() >> control_points[i].y() >> control_points[i].z()) )
			throw "Invalid control point";
	}
	
	//Validate data (assert that path is monotonic)
	assert(control_points[0].z() == 0.);
	for(int i=1; i<n_points; i++)
		assert(control_points[i].z() >= control_points[i-1].z());
		
	t_max = control_points[control_points.size() - 1].z();
}


//deCasteljau's algorithm
Vector3f eval_spline(float t)
{
	aligned<Vector3f>::vector tmp(control_points.size());
	copy(control_points.begin(), control_points.end(), tmp.begin());
	
	for(int i=control_points.size()-1; i>0; i--)
	for(int j=0; j<i; j++)
	{
		tmp[j] = tmp[j] * (1. - t) + tmp[j+1] * t;
	}

	return tmp[0];
}

//Evaluates the spline at some point in time
Vector2f eval_time_spline(double t)
{
	assert(t >= 0. && t <= t_max);
	
	//Do a binary search to find t coordinate for time value
	float lo = 0., hi = 1.;
	Vector3f pt;
	while(abs(lo - hi) > 1e-6)
	{
		float m = lo + (hi - lo) * .5;
		pt = eval_spline(m);
		
		if(pt.z() > t)
			hi = m;
		else
			lo = m;
	}
	
	//Return the value of the spline at this time
	return Vector2f(pt.x(), pt.y());
}



//File IO helpers
Motion read_motion(const string& file_name)
{
	ifstream tmp(file_name.c_str());
	return parseBVH(tmp).convert_quat();
}

MotionGraph read_graph(const string& file_name)
{
	ifstream tmp(file_name.c_str());
	return parseMotionGraph(tmp);
}

//Windowing function
double hann_window(double t)
{
	return 0.5 * (1. - cos(M_PI * 2. * t));
}

//Creates a motion graph from a motion file
void create_graph(const string& mo_file)
{
	Motion motion = read_motion(mo_file);
	MotionGraph graph(motion.skeleton);
	graph.frame_time = motion.frame_time;
	writeMotionGraph(cout, graph);
}

//Adds a motion to a motion graph
void add_motion(const string& mo_file, const string& graph_file)
{
	Motion motion = read_motion(mo_file);
	MotionGraph graph = read_graph(graph_file);
	graph.insert_motion(motion, 15., motion.frame_time * 5.,  5, hann_window);
	writeMotionGraph(cout, graph);
}

//Synthesizes a new motion
void synthesize_motion(int l, const string& graph_file)
{
	MotionGraph graph = read_graph(graph_file);
	Motion motion = graph.random_motion(l);
	writeBVH(cout, motion);
}

//Extract strongly connected components
void extract_scc(const string& graph_file)
{
	MotionGraph graph = read_graph(graph_file);
	vector<MotionGraph> scc = graph.extract_scc();
	
	//Validate size constraint
	assert(scc.size() > 0);
	
	int max_graph = 0;
	
	for(int i=1; i<scc.size(); i++)
		if(scc[i].frames.size() > scc[max_graph].frames.size())
			max_graph = i;
	
	writeMotionGraph(cout, scc[max_graph]);
}

void process_spline(double max_d, const string& spline_file, const string& graph_file)
{
	ifstream sin(spline_file.c_str());
	read_control_points(sin);
	
	MotionGraph graph = read_graph(graph_file);
	Motion motion = graph.follow_path(&eval_time_spline, max_d, t_max);
	writeBVH(cout, motion);
}


//Prints out a helpful message
void print_help()
{
	cerr << endl
		 << "motool : Motion Graph Edit Tool" << endl
		 << "-------------------------------------------------" << endl
		 << "This program processes BVH format character motions and motion graphs." << endl
		 << "All results are written to stdout because I am a lazy programmer." << endl
		 << endl
		 << "Use Cases:" << endl
		 << "  Create a motion graph:" << endl
		 << "     motool -c <base_motion.bvh>" << endl
		 << endl
		 << "  Add a motion to a graph:" << endl
		 << "     motool -a <motion.bvh> <graph.mog>" << endl
		 << endl
		 << "  Extract strongly connected components:" << endl
		 << "     motool -scc <graph.mog>" << endl
		 << endl
		 << "  Synthesize a random motion:" << endl
		 << "     motool -s <length> <graph.mog>" << endl
		 << endl
		 << "  Walk along a path:" << endl
		 << "     motool -path <max_distance> <path_file.spline> <graph.mog>" << endl
		 << endl
		 << endl;
}

//Program start point
int main(int argc, char** argv)
{
	if(argc < 2)
	{
		cerr << "Incorrect arguments" << endl;
		print_help();
		exit(1);
	}
	
	string command = argv[1];
	
	try
	{
	if(command == "-c")
	{
		assert(argc == 3);
		string motion_file(argv[2]);
		create_graph(motion_file);
	}
	else if(command == "-a")
	{
		assert(argc == 4);
		string motion_file(argv[2]), graph_file(argv[3]);
		add_motion(motion_file, graph_file);
	}
	else if(command == "-s")
	{
		assert(argc == 4);
		int l = atoi(argv[2]);
		string graph_file(argv[3]);
		synthesize_motion(l, graph_file);
	}
	else if(command == "-scc")
	{
		assert(argc == 3);
		extract_scc(argv[2]);
	}
	else if(command == "-path")
	{
		assert(argc == 5);
		process_spline(atof(argv[2]), string(argv[3]), string(argv[4]));
	}
	else
	{
		cerr << "Unknown command: " << command << endl;
		print_help();
		exit(1);
	}
	}
	catch(string str)
	{
		cerr << "Exception: " << str << endl;
		exit(1);
	}

	return 0;
}

