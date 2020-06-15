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
#include <algorithm> // min_element

class Point {
public:
    Point(const unsigned int x, const unsigned int y, const unsigned int index)
    : x_(x), y_(y), index_(index) {}

    Point(const unsigned int x, const unsigned int y, const costmap_2d::Costmap2D* costmap) 
    : x_(x), y_(y) {
        index_ = costmap->getIndex(x, y);
    }

    Point(const unsigned int index, const costmap_2d::Costmap2D* costmap) 
    : index_(index) {
        costmap->indexToCells(index, x_, y_);
    }

    bool operator != (const Point& rhs) const {
        return index_ != rhs.index();
    }

    unsigned int x() const {return x_;}
    unsigned int y() const {return y_;}
    unsigned int index() const {return index_;}

private:
    unsigned int x_, y_; // x, y costmap index
    unsigned int index_; // flattened costmap index
};

class gCostCompare {
public:
    gCostCompare(std::vector<float>& g_cost) : g_cost(g_cost) {}

    bool operator () (const Point& lhs, const Point& rhs) const { 
        return g_cost[lhs.index()] < g_cost[rhs.index()];
    }

private:
    std::vector<float>& g_cost;
};

class Action {
public:
    Action(const int dx, const int dy, const float g_cost)
    : dx_(dx), dy_(dy), g_cost_(g_cost) {}

    unsigned int dx() const {return dx_;}
    unsigned int dy() const {return dy_;}
    unsigned int g_cost() const {return g_cost_;}

private:
    int dx_, dy_; // displacement in x and y axis
    float g_cost_; // cost for taking action
};

std::vector<Action> actions {
    Action(1, 0, 1.0), // right
    Action(-1, 0, 1.0), // left
    Action(0, -1, 1.0), // down
    Action(1, - 1, M_SQRT2), //down right
    Action(-1, -1, M_SQRT2), //down left
    Action(0, 1, 1.0), // up
    Action(1, 1, M_SQRT2), // up right
    Action(-1, 1, M_SQRT2), // up left
};

class Node {
public:
    Node(const unsigned int index, const float f_cost=std::numeric_limits<float>::infinity())
    : index_(index), f_cost_(f_cost) {}

    bool operator < (const Node& rhs) const {
        return f_cost_ < rhs.f_cost();
    }

    unsigned int index() const {return index_;}
    unsigned int f_cost() const {return f_cost_;}

private:
    unsigned int index_; // displacement in x and y axis
    float f_cost_; // action cost (g_cost) + heuristic cost (h_cost)
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
        float t_break;
        double step_size, min_dist_from_robot;

        costmap_2d::Costmap2DROS* costmap_ros;
        costmap_2d::Costmap2D* costmap;
        base_local_planner::CostmapModel* world_model;

        std::vector<float> getGCosts(const Point start_point, const Point goal_point);

        std::vector<unsigned int> getCostmapPath(const Point start_point, const Point goal_point, 
            const gCostCompare& compare); // unsigned int ?

        std::vector<geometry_msgs::PoseStamped> getWorldPath(std::vector<unsigned int>& path);

        std::vector<Point> getAdjPoints(const Point point);

        float getHCost(const Point src, const Point dst);
    };
};

#endif