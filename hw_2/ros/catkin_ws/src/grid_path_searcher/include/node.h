#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <eigen3/Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id; // 1--> open set, -1 --> closed set, 0 --> not expanded
    Eigen::Vector3d coord; // 全局坐标系下的坐标
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index; // 栅格的 index
	
    double gScore, fScore; // 节点的 path cost, 注：fScore = gScore + hScore
    GridNodePtr cameFrom;  // 指向父节点的指针
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
    {  
		id = 0;
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();

		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};


#endif
