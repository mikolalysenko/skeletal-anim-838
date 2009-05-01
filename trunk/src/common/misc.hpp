//Has aliases for aligned types

#ifndef MISC_H
#define MISC_H

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>
#include <map>
#include <list>
#include <deque>
#include <queue>

//Aligned types
//Eigen decided to change the way stdvector worked, so you now have to type Eigen::aligned_allocator<T> for each thing *groan*
//This hack saves some keystrokes, for example to get a vector you type in:
//
//	aligned<Vector3d>::vector x
//
// Which is kind of annoying, but what are you gonna do? This is the best I could come up with within the limitations of C++
//
template<typename T, typename V=int> struct aligned
{
	typedef std::vector<T, Eigen::aligned_allocator<T> > vector;
	typedef std::list<T, Eigen::aligned_allocator<T> > list;
	typedef std::deque<T, Eigen::aligned_allocator<T> > deque;
	typedef std::priority_queue<T, vector> priority_queue;
	typedef std::map< T, V, std::less<int>, Eigen::aligned_allocator<V> > map;
};

#endif

