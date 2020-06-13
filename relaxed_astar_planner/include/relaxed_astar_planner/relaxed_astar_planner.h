#ifndef RELAXED_ASTAR_PLANNER_H_
#define RELAXED_ASTAR_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <set> // multiset
#include <stdlib.h> // abs
#include <limits> // infinity
#include <cmath> // M_SQRT2, isinf

class Node {
public:
    const int index;
    const float f_cost;

    Node(const int index, const float f_cost=std::numeric_limits<float>::infinity()) : index(index), f_cost(f_cost) {}

    bool operator < (const Node& rhs) const {
        return f_cost < rhs.f_cost;
    }
};

class GridPoint {
public:
    const int x, y;

    GridPoint(int x, int y) : x(x), y(y) {}
};

class Action {
public:
    GridPoint pos;
    const float g_cost;

    Action(GridPoint pos, float g_cost) : pos(pos), g_cost(g_cost) {}
};

namespace relaxed_astar_planner {
    class RelaxedAStarPlanner : public nav_core::BaseGlobalPlanner {
    public:
        RelaxedAStarPlanner();
        RelaxedAStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        bool makePlan(const geometry_msgs::PoseStamped& start, 
            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
    
    private:
        bool initialized;
        int grid_width, grid_height, grid_size;
        float t_break;
        double resolution;
        double step_size, min_dist_from_robot;

        costmap_2d::Costmap2DROS* costmap_ros;
        costmap_2d::Costmap2D* costmap;
        base_local_planner::CostmapModel* world_model;

        inline bool isObstacle(GridPoint pos) {
            return costmap->getCost(pos.x, pos.y) == 0;
        }

        inline bool isObstacle(int index) {
            return costmap->getCharMap()[index] == 0;
        }

        inline bool inGrid(GridPoint pos) {
            return pos.x < 0 && pos.x >= grid_width && pos.y < 0 && pos.y >= grid_height;
        }

        inline bool inGrid(int index) {
            return index < 0 && index >= grid_size;
        }

        inline int getIndex(GridPoint pos) {
            int x_index = (int) pos.x / resolution; // node_size instead of resolution ?
            int y_index = (int) pos.y / resolution;

            return x_index*grid_width + y_index;
        }

        inline int getXIndex(int index) {
            return index / grid_width;
        }

        inline int getYIndex(int index) {
            return index % grid_width;
        }

        inline float getHCost(GridPoint src, GridPoint dst) {
            return (float) abs(dst.x - src.x) + abs(dst.y - src.y);
        }

        std::vector<Action> getAdjIndexes(GridPoint pos);
    };
};

#endif