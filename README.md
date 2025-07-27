## General Overview: A* vs. Dijkstra

### 🔹 Dijkstra’s Algorithm

* **Purpose** : Finds the shortest path from a source node to all other nodes in a graph.
* **Key Feature** : Explores nodes in order of increasing cost (distance from the start).
* **Optimality** : Always finds the shortest path.
* **Speed** : Can be slow on large graphs because it doesn’t prioritize toward the goal.

### 🔹 A* (A-Star) Algorithm

* **Purpose** : Finds the shortest path from a start node to a goal node.
* **Key Feature** : Uses both actual cost from start (`g`) and estimated cost to goal (`h`, a heuristic).
* **Heuristic** : Commonly `Euclidean` or `Manhattan` distance.
* **Optimality** : Finds the optimal path if the heuristic is admissible (never overestimates).
* **Speed** : Usually faster than Dijkstra because it directs the search toward the goal.
* 

---

## General Documentation: A* and Dijkstra ROS 2 Planners

These two files implement **global path planning algorithms** — A* and Dijkstra — in ROS 2, using a shared architecture for subscribing to maps and goals, transforming robot poses, and publishing results. Both are intended to run within the ROS 2 Navigation (Nav2) ecosystem and provide real-time path computation over a static occupancy grid.

---

### 🧠 Common Structure (Both Files)

#### ✅ Node Definition

Each class (`A_StarPlanner` or `DijkstraPlanner`) is a subclass of `rclcpp::Node`. They initialize ROS 2 interfaces for subscribing and publishing:

* `/map`: Subscribes to `nav_msgs::msg::OccupancyGrid`, representing the static environment.
* `/goal_pose`: Subscribes to `geometry_msgs::msg::PoseStamped`, representing the user-defined goal.
* `/<algorithm>/path`: Publishes the computed path (`nav_msgs::msg::Path`).
* `/<algorithm>/visitedmap`: Publishes a visual representation of visited cells.

#### ✅ TF Integration

Each planner uses `tf2_ros::Buffer` and `TransformListener` to look up the robot's current position in the map frame, typically from `"base_footprint"` to `"map"`.

#### ✅ Map Management

They store the map and create a separate occupancy grid to track visited cells (for visualization in RViz). Unvisited cells are initialized to `-1`, matching ROS occupancy grid conventions.

---

### 🔍 Algorithmic Differences

#### 🔷 **DijkstraPlanner**

* Implements the classic  **Dijkstra algorithm** , which explores the entire graph in order of increasing cost from the start.
* It  **does not use a heuristic** . It guarantees the shortest path but can be  **slow** , especially on large maps.
* Every reachable cell is expanded based on total accumulated cost, regardless of how far it is from the goal.

#### 🔷 **A_StarPlanner**

* Uses  **A*** , an informed search algorithm that combines:
  * `g(x)`: the cost from start to current node
  * `h(x)`: estimated cost from current node to goal (heuristic)
* This allows it to  **prioritize nodes likely to reach the goal faster** , usually reducing computation time significantly.
* Common heuristics are Euclidean or Manhattan distance — likely hardcoded in your implementation.

---

### 🗺️ Output

Both planners:

* Compute a path from the robot’s current location to the goal.
* Publish the final path as a sequence of poses (`nav_msgs::msg::Path`).
* Optionally visualize explored areas using a secondary occupancy grid.

---

### 🛠️ Usage Context

* These planners are typically launched in simulation or mapped environments for mobile robots.
* They support visualization and debugging in RViz via custom topics.
* The decision between A* and Dijkstra should be based on:
  * **A*** : Preferred when real-time speed is important.
  * **Dijkstra** : Useful when path optimality is critical, and planning time is less constrained.
