#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <eigen3/Eigen/Eigen>
#include "backward.hpp"
#include "node.h"

class AstarPathFinder
{	
	private:

	protected:
		uint8_t * data;
		GridNodePtr *** GridNodeMap;
		Eigen::Vector3i goalIdx;  // 列向量，含有 3 个 int 型元素。
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;

		GridNodePtr terminatePtr;
		std::multimap<double, GridNodePtr> openSet; // 用 multimap 构造一个 open list。
        // STL 中的 multimap，允许重复元素，会根据 key 值对 键值对自动进行排序。- 底层是红黑树实现。

		double getHeu(GridNodePtr node1, GridNodePtr node2); // 启发式函数，作业内容
        // 拓展节点函数，作业内容
		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets,
                         std::vector<double> & edgeCostSets);		

    	bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;
		
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index); // 通过栅格的 index，计算栅格中心的全局坐标
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt); // 通过节点的全局坐标，计算所在栅格的 index

	public:
		AstarPathFinder(){};
		~AstarPathFinder(){};
        // A* 搜索算法函数，作业内容
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		void resetGrid(GridNodePtr ptr);
		void resetUsedGrids();

		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u,
                         int max_x_id, int max_y_id, int max_z_id);
		void setObs(const double coord_x, const double coord_y, const double coord_z);

		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		std::vector<Eigen::Vector3d> getPath();
		std::vector<Eigen::Vector3d> getVisitedNodes();
};

#endif