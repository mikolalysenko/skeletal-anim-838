#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/StdVector>

#include <iostream>
#include <vector>

#include <misc.hpp>
#include <skeleton.hpp>
#include <mograph.hpp>

using namespace std;
using namespace Eigen;
using namespace Skeletal;


void print_help()
{
	cout << endl
		 << "motool : Motion Graph Edit Tool" << endl
		 << "-------------------------------------------------" << endl
		 << "This program processes BVH format character motions and motion graphs." << endl
		 << endl
		 << "Use Cases:" << endl
		 << "  Create a motion graph:" << endl
		 << "     motool -c <base_motion.bvh> <graph.mog>" << endl
		 << endl
		 << "  Add a motion to a graph:" << endl
		 << "     motool -a <motion.bvh> <graph.mog>" << endl
		 << endl
		 << "  Synthesize a random motion:" << endl
		 << "     motool -s <length> <graph.mog>" << endl
		 << endl
		 << endl;
}

int main(int argc, char** argv)
{
	print_help();

	return 0;
}

