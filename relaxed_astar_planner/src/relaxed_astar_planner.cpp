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

            resolution = costmap->getResolution();
            grid_width = costmap->getSizeInCellsX();
            grid_height = costmap->getSizeInCellsY();
            grid_size = grid_width * grid_height;
            t_break = 1 + 1/(grid_width+grid_height);

            ros::NodeHandle nh("~/" + name);
            nh.param("step_size", step_size, resolution);
            nh.param("min_dist_from_robot", min_dist_from_robot, 0.10);

            world_model = new base_local_planner::CostmapModel(*costmap);

            initialized = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    bool RelaxedAStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    
        // TODO: function to check if start, end are valid ?

        if (!initialized) {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        if (goal.header.frame_id != costmap_ros->getGlobalFrameID()) {
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", costmap_ros->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());

            return false;
        }

        plan.clear();
        costmap = costmap_ros->getCostmap();

        GridPoint goal_point(goal.pose.position.x, goal.pose.position.y);
        int goal_index = getIndex(goal_point);

        GridPoint start_point(start.pose.position.x, start.pose.position.y);
        Node start_node(getIndex(start_point), getHCost(start_point, goal_point));

        if (isObstacle(start_point) || !inGrid(start_point) || isObstacle(goal_point) || !inGrid(goal_point)) {
            ROS_WARN("Not valid start or goal");
            return false;
        }

        std::multiset<Node> open_set {start_node};

        std::vector<float> g_cost(grid_size, std::numeric_limits<float>::infinity());
        g_cost[start_node.index] = 0.0;

        while (!open_set.empty() && std::isinf(g_cost[goal_index])) {
            int curr_index = open_set.begin()->index;
            open_set.erase(open_set.begin());

            GridPoint curr_point(getXIndex(curr_index), getYIndex(curr_index));
            for (Action action : getAdjIndexes(curr_point)) {
                int new_index = getIndex(action.pos);
                if (std::isinf(g_cost[new_index])) {
                    g_cost[new_index] = g_cost[curr_index] + action.g_cost;
                    float f_cost = g_cost[new_index] + t_break*getHCost(action.pos, goal_point);

                    Node adj_node(new_index, f_cost);
                    open_set.insert(adj_node);
                }
            }
        }

        // ...
    }

    std::vector<Action> RelaxedAStarPlanner::getAdjIndexes(GridPoint pos) {
        std::vector<Action> possible_actions {
            Action(GridPoint(pos.x + 1, pos.y), 1.0), // right
            Action(GridPoint(pos.x - 1, pos.y), 1.0), // left
            Action(GridPoint(pos.x, pos.y - 1), 1.0), // down
            Action(GridPoint(pos.x + 1, pos.y - 1), M_SQRT2), //down right
            Action(GridPoint(pos.x - 1, pos.y - 1), M_SQRT2), //down left
            Action(GridPoint(pos.x, pos.y + 1), 1.0), // up
            Action(GridPoint(pos.x + 1, pos.y + 1), M_SQRT2), // up right
            Action(GridPoint(pos.x - 1, pos.y + 1), M_SQRT2), // up left
        };

        std::vector<Action> valid_actions;
        valid_actions.reserve(8);

        for (Action action : possible_actions) {
            if (!isObstacle(action.pos) && inGrid(action.pos))  
                valid_actions.push_back(action);
        }

        return valid_actions;
    }
};