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

//Creates a motion graph from a motion file
void create_graph(const string& mo_file)
{
}

//Adds a motion to a motion graph
void add_motion(const string& mo_file, const string& graph_file)
{
}


//Synthesizes a new motion
void synthesize_motion(int l, const string& graph_file)
{
}


//Prints out a helpful message
void print_help()
{
	cout << endl
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
		 << "  Synthesize a random motion:" << endl
		 << "     motool -s <length> <graph.mog>" << endl
		 << endl
		 << endl;
}

//Program start point
int main(int argc, char** argv)
{
	if(argc < 2)
	{
		cout << "Incorrect arguments" << endl;
		print_help();
		exit(1);
	}
	
	string command = argv[1];
	
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
	else
	{
		cout << "Unknown command: " << command << endl;
		print_help();
		exit(1);
	}

	return 0;
}

