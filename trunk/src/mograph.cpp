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
#include <mograph.hpp>

using namespace std;
using namespace Eigen;
using namespace Skeletal;

namespace Skeletal
{

//Constructs a motion graph
MotionGraph construct_graph(const vector<Motion>& motions, double ft, double thresh, double w, int n, double (*wind_func)(double))
{
    MotionGraph result(motions[0].skeleton);
    result.frame_time = ft;
    
    for(int i=0; i<motions.size(); i++)
        result.insert_motion(motions[i], thresh, w, n, wind_func);
    return result;
}

//Inserts a motion into the MotionGraph
void MotionGraph::insert_motion(const Motion& motion, double threshold, double w, int n, double (*window_func)(double))
{
    for(double t=0.; t<motion.duration(); t+=frame_time)
    {
        //Extract window about i
        vector<Vector4d> cloud = motion.point_cloud_window(t, w, n, window_func);
        
        vector<int> edges;
        if(t+frame_time<motion.duration())
            edges.push_back(frames.size());
        
        for(int j=0; j<frames.size(); j++)
        {
            //Compute distance metric
            Transform3d rel = relative_xform(skeleton, frames[j], motion.get_frame(t));
            double dist = cloud_distance(cloud, local_clouds[j], rel);
            
            if(dist < threshold)
                edges.push_back(j);
        }
        
        //Add frame to graph
        frames.push_back(motion.get_frame(t));
        graph.push_back(edges);
        local_clouds.push_back(cloud);
    }
}

//Extracts all biconnected components
MotionGraph MotionGraph::extract_biconnected() const
{
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


}