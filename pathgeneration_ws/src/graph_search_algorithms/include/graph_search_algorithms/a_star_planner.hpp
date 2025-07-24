#ifndef A_STAR_PLANNER_H
#define A_STAR_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
/*  This includes the TransformListener class from ROS 2’s tf2_ros library.
    TransformListener subscribes to the /tf topic.
    It stores transform data (between coordinate frames like odom, base_link, map) into a tf2_ros::Buffer.
*/
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


#include <memory>


struct GraphNode {
    int x;
    int y;
    int cost;
    double heuristic;
    std::shared_ptr<GraphNode> prev_ ;

    /*Parameter overloading the constructor*/
    GraphNode() : GraphNode(0,0) {
        /*Nothing*/
    }
    GraphNode(int c_x , int c_y) : x(c_x) , y(c_y) , cost(0) {
        /*Nothing*/
    }
    
    /* Overloading the "==" operator  to compare only x , y variables */
    bool operator==(const GraphNode & RHS) const{
        return (x == RHS.x && y == RHS.y);
    }

    /* Overloading the "+" operator to add (x's & y's) , this is done to help with exploring the map*/
    GraphNode operator+(const std::pair<int,int> & RHS) {
        GraphNode Copy_Solution( x + RHS.first , y + RHS.second);
        return Copy_Solution;
    }

    /* Overloading the ">" operator to compare nodes cost*/
    bool operator>(const GraphNode & RHS) const {
        return (cost + heuristic > RHS.cost + RHS.heuristic) ;
    }
    
};
    
class A_StarPlanner : public rclcpp::Node {
    public :
        A_StarPlanner();

    private :
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_ ;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        
        nav_msgs::msg::OccupancyGrid visited_map_;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;    
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

        nav_msgs::msg::Path plan(geometry_msgs::msg::Pose & start,  geometry_msgs::msg::Pose & goal);

        GraphNode PoseToGrid (geometry_msgs::msg::Pose & Copy_Pose);
        int GridToOGIndex(GraphNode & Copy_point);
        bool PoseOnMap(const GraphNode & Copy_Node);

        geometry_msgs::msg::Pose GridToWorld(const GraphNode & node);


        double manhattanDistance(const GraphNode &node, const GraphNode &goal_node);
};


#endif