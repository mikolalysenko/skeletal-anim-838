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
    Vector3d da = a[i].block(1,3,0,0),
             db = b[i].block(1,3,0,0);
    
    da = x * da;
      
    s += (da - db).squaredNorm() * a[i].w() * b[i].w();
  }
  
  return sqrt(s);
}


//Constructs a motion graph
MotionGraph construct_graph(const aligned<Motion>::vector& motions, double ft, double thresh, double w, int n, double (*wind_func)(double))
{
    MotionGraph result(motions[0].skeleton);
    result.frame_time = ft;
    
    for(int i=0; i<motions.size(); i++)
        result.insert_motion(motions[i], thresh, w, n, wind_func);
    return result;
}



//Compute a window about the point cloud
void wtf( aligned<Vector4d>::vector& tmptest)
{

  aligned<Vector4d>::vector cloud;
  aligned<Vector4d>::vector result;
  /*
  double frame_t = orig - extent/2.,
         delta_t = extent / (double)n_samples,
         t = -1.;
  for(int i=0; i<n_samples; i++, frame_t += delta_t, t += 2. / (double)n_samples)
  {
    aligned<Vector3d>::vector cloud = get_frame(frame_t).point_cloud(skeleton);
    for(int j=0; j<cloud.size(); j++)
      result.push_back(Vector4d(cloud[j].x(), cloud[j].y(), cloud[j].z(), window(t)));
  }
  */
  //return result;
}

//Inserts a motion into the MotionGraph
void MotionGraph::insert_motion(const Motion& motion, double threshold, double w, int n, double (*window_func)(double))
{
    for(double t=0.; t<motion.duration(); t+=frame_time)
    {
        //Extract window about i
        aligned<Vector4d>::vector cloud;// = motion.point_cloud_window(t, w, n, window_func);
 motion.point_cloud_window(t, w, n, window_func);   
        wtf(cloud);
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
