#include <relaxed_astar_planner/relaxed_astar_planner.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(relaxed_astar_planner::RelaxedAStarPlanner, nav_core::BaseGlobalPlanner)

namespace relaxed_astar_planner {
    RelaxedAStarPlanner::RelaxedAStarPlanner()
    : costmap_ros(NULL), initialized(false) {}

    RelaxedAStarPlanner::RelaxedAStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros(NULL), initialized(false) {
        initialize(name, costmap_ros);
    }

    void RelaxedAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized) {
            this->costmap_ros = costmap_ros;
            costmap = this->costmap_ros->getCostmap();

            t_break = 1 + 1/(costmap->getSizeInCellsX()+costmap->getSizeInCellsY());

            ros::NodeHandle nh("~/" + name);
            nh.param("step_size", step_size, costmap->getResolution());
            nh.param("min_dist_from_robot", min_dist_from_robot, 0.10);

            world_model = new base_local_planner::CostmapModel(*costmap);

            initialized = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    bool RelaxedAStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {

        // check if plan can be made
        if (!initialized) {
            ROS_ERROR("The planner is not initialized. Call initialize() to use planner");
            return false;
        }

        if (goal.header.frame_id != costmap_ros->getGlobalFrameID()) {
            ROS_ERROR("The goal pose is in the %s frame, not the %s frame", goal.header.frame_id.c_str(), costmap_ros->getGlobalFrameID().c_str());

            return false;
        }

        if (start.header.frame_id != costmap_ros->getGlobalFrameID()) {
            ROS_ERROR("The start pose is in the %s frame, not the %s frame", goal.header.frame_id.c_str(), costmap_ros->getGlobalFrameID().c_str());

            return false;
        }
        
        plan.clear(); // clear plan in case it contains old data

        costmap = costmap_ros->getCostmap(); // get costmap again in case it has updated ?

        // convert world coordinates to costmap indexes
        unsigned int start_x, start_y;
        if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
            ROS_WARN("The start pose is not in bounds");
            return false;
        }
        Point start_point(start_x, start_y, costmap);

        unsigned int goal_x, goal_y;
        costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
        Point goal_point(goal_x, goal_y, costmap);

        std::vector<float> g_cost = getGCosts(start_point, goal_point); // move constructor used ?

        gCostCompare compare(g_cost);
        std::vector<unsigned int> path = getCostmapPath(start_point, goal_point, compare);

        // reverse path and convert path indexes to world coordinates
        plan = getWorldPath(path);
        return true;
    }

    std::vector<float> RelaxedAStarPlanner::getGCosts(const Point start_point, const Point goal_point) {
        const unsigned int costmap_size = costmap->getSizeInCellsX() * costmap->getSizeInCellsY(); 
        std::vector<float> g_cost(costmap_size, std::numeric_limits<float>::infinity());
        g_cost[start_point.index()] = 0.0;

        Node start_node(start_point.index(), getHCost(start_point, goal_point));
        std::multiset<Node> open_set {start_node};

        while (!open_set.empty() && std::isinf(g_cost[goal_point.index()])) {
            Point curr_point(open_set.begin()->index(), costmap);
            open_set.erase(open_set.begin());

            for (Action action : actions) {
                Point adj_point(curr_point.x() + action.dx(), curr_point.y() + action.dy(), costmap);

                g_cost[adj_point.index()] = g_cost[curr_point.index()] + action.g_cost();
                float f_cost = g_cost[adj_point.index()] + t_break*getHCost(adj_point, goal_point);
    
                Node adj_node(adj_point.index(), f_cost);
                open_set.insert(adj_node);
            }
        }

        return g_cost;
    }

    std::vector<unsigned int> RelaxedAStarPlanner::getCostmapPath(const Point start_point, 
        const Point goal_point, const gCostCompare& compare) {

        std::vector<unsigned int> path;

        // find path from goal to start
        Point curr_point = goal_point;
        path.push_back(curr_point.index());

        while (curr_point != start_point) {
            std::vector<Point> adj_points = getAdjPoints(curr_point);
            curr_point = *std::min_element(adj_points.begin(), adj_points.end(), compare);
            path.push_back(curr_point.index());
        }

        return path;
    }

    std::vector<geometry_msgs::PoseStamped> RelaxedAStarPlanner::getWorldPath(std::vector<unsigned int>& path) {
        std::vector<geometry_msgs::PoseStamped> world_path(path.size());

        for (int i = path.size()-1; i >= 0; i--) {
            Point point(path[i], costmap);
            geometry_msgs::PoseStamped pose;

            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0;

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            world_path[i] = pose;
        }

        return world_path;
    }

    std::vector<Point> RelaxedAStarPlanner::getAdjPoints(const Point point) {
        std::vector<Point> points;
        points.reserve(actions.size());

        for (Action action : actions)
            points.push_back(Point(point.x() + action.dx(), point.y() + action.dy(), costmap));

        return points;
    }

    float RelaxedAStarPlanner::getHCost(const Point src, const Point dst) {
        unsigned int dx = dst.x() > src.x() ? dst.x() - src.x() : src.x() - dst.x();
        unsigned int dy = dst.y() > src.y() ? dst.y() - src.y() : src.y() - dst.y();
        return (float) (dx + dy);
    }
};

