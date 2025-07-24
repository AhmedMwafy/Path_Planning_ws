#include "../include/graph_search_algorithms/dijkstra_planner.hpp"
#include <vector>
#include "rmw/qos_profiles.h"
#include <queue>


DijksrtaPlanner::DijksrtaPlanner() : Node("dijkstra_planner")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /*To be compatable with RVIZ2*/
    rclcpp::QoS map_qos(10);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    map_sub_=  create_subscription<nav_msgs::msg::OccupancyGrid>("/map" , map_qos , std::bind(&DijksrtaPlanner::mapCallback , this , std::placeholders::_1));
    pose_sub_= create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose" , 10 , std::bind(&DijksrtaPlanner::goalCallback , this , std::placeholders::_1));
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/dijkstra/visitedmap" , 10) ;
    path_pub_  = create_publisher<nav_msgs::msg::Path>("/dijkstra/path" , 10) ;
}



void DijksrtaPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map){
    if(!map){
        RCLCPP_ERROR(get_logger(), "map callback issue !");
        return;
    }
    map_ = map ;
    visited_map_.header.frame_id = map->header.frame_id ;
    visited_map_.info = map->info ;
    /*Setting up the map & initialize it to unknown*/
    visited_map_.data = std::vector<int8_t>( visited_map_.info.height * visited_map_.info.width , -1 ) ;
}
void DijksrtaPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose){
    if(!pose){
        return ;
    }
    if(!map_){
        RCLCPP_ERROR(get_logger(), "No map received !");
        return;
    }
/* Initialize the visited map with -1 (unvisited) for each cell.
   The total number of cells is height * width.
   Using int8_t because nav_msgs::msg::OccupancyGrid uses it for cell values.
   Values: -1 = unknown/unvisited, 0 = free, 100 = occupied */
visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1);

/* Create a TransformStamped object to store the transformation 
   from the map frame to the base_footprint frame */
geometry_msgs::msg::TransformStamped map_to_base_tf;

try {
    /* Lookup the latest transform between the map and base_footprint frames.
       This is necessary to know where the robot is in the global map frame.
       tf2::TimePointZero means "latest available transform". */
    map_to_base_tf = tf_buffer_->lookupTransform(
        map_->header.frame_id,    /* Target frame (e.g., "map") */
        "base_footprint",         /* Source frame (robot base frame) */
        tf2::TimePointZero);      /* Use latest transform available */
} catch (const tf2::TransformException & ex) {
    /* If transform is not found, log error and return early */
    RCLCPP_ERROR(get_logger(), "Could not transform from map to base_footprint");
    return;
}

/* Convert the transform to a simple Pose structure.
   Pose contains only position (x, y) and orientation (quaternion).
   This is used for path planning as the robot's current position in the map. */
geometry_msgs::msg::Pose map_to_base_pose;
map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

/* Call the path planning function.
   Input: current pose (start), goal pose.
   Output: path as a nav_msgs::msg::Path (sequence of waypoints). */
auto path = plan(map_to_base_pose, pose->pose);

/* If the path is not empty, publish it to a topic so it can be followed.
   Otherwise, log a warning indicating planning failed. */
if (!path.poses.empty()) {
    RCLCPP_INFO(this->get_logger(), "Shortest path found!");
    path_pub_->publish(path);
} else {
    RCLCPP_WARN(this->get_logger(), "No path found to the goal.");
}

}
nav_msgs::msg::Path DijksrtaPlanner::plan(geometry_msgs::msg::Pose & start,  geometry_msgs::msg::Pose & goal){
    /*This is used to access the surrounding nodes*/
    std::vector<std::pair<int, int>> explore_directions = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };
    std::priority_queue<GraphNode , std::vector<GraphNode> , std::greater<GraphNode>> Possible_Options ;
  
    
    std::vector<GraphNode> visited_nodes;
    /* The First pop option is with the least cost -> greater */
    Possible_Options.push(PoseToGrid(start));
    GraphNode Current_Node;
    while(!Possible_Options.empty() && rclcpp::ok()){
        Current_Node = Possible_Options.top();
        Possible_Options.pop();

        if(PoseToGrid(goal) == Current_Node){
            break;
        }
        for(const auto & dir : explore_directions){
            GraphNode New_Node = Current_Node + dir ;
            if( PoseOnMap(New_Node) && (std::find(visited_nodes.begin(),visited_nodes.end(),New_Node) == visited_nodes.end()) 
            && map_->data.at(GridToOGIndex(New_Node)) == 0 ){
                New_Node.cost = 1 + Current_Node.cost ;
                New_Node.prev_ = std::make_shared<GraphNode>(Current_Node) ;
                Possible_Options.push(New_Node);
                visited_nodes.push_back(New_Node);
            }
        }
        visited_map_.data.at(GridToOGIndex(Current_Node)) = 10 ;    
        map_pub_->publish(visited_map_);
    }
    
    nav_msgs::msg::Path path;
    path.header.frame_id = map_->header.frame_id;
    
    while(Current_Node.prev_ && rclcpp::ok()){
        geometry_msgs::msg::Pose the_pose = GridToWorld(Current_Node);
        geometry_msgs::msg::PoseStamped pose__ ;
        pose__.header.frame_id = map_->header.frame_id ;
        pose__.pose = the_pose ;
        path.poses.push_back(pose__);
        Current_Node = *Current_Node.prev_;
    }

    std::reverse(path.poses.begin() , path.poses.end());
    return path ;

}

GraphNode DijksrtaPlanner::PoseToGrid (geometry_msgs::msg::Pose & Copy_Pose){
            int Grid_x = static_cast<int> (Copy_Pose.position.x - map_->info.origin.position.x) / map_->info.resolution ;
            int Grid_y = static_cast<int> (Copy_Pose.position.y - map_->info.origin.position.y) / map_->info.resolution ;
            /* This is a return by value */
            return GraphNode(Grid_x , Grid_y);
        }
int DijksrtaPlanner::GridToOGIndex(GraphNode & Copy_point){
    return (Copy_point.y * map_->info.width + Copy_point.x ) ;
}
        
bool DijksrtaPlanner::PoseOnMap(const GraphNode & Copy_Node){
    
    return Copy_Node.x < static_cast<int>(map_->info.width) && Copy_Node.x >= 0 &&
        Copy_Node.y < static_cast<int>(map_->info.height) && Copy_Node.y >= 0;
}

geometry_msgs::msg::Pose DijksrtaPlanner::GridToWorld(const GraphNode & node)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * map_->info.resolution + map_->info.origin.position.x;
    pose.position.y = node.y * map_->info.resolution + map_->info.origin.position.y;
    return pose;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DijksrtaPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}