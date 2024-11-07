#include "../include/planner.h"
#include <math.h>
#include <stdio.h>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <limits>
#include <chrono>
#include <iostream>
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
double epsilon; // Initial epsilon
bool initialized = false; // Whether the planner is initialized

std::chrono::time_point<std::chrono::steady_clock> startTime;
std::unordered_set<int> closed_list; // Closed list
std::unordered_map<int, double> g_map; // Best g-values for nodes
std::unordered_map<int, double> v_map; // Best v-values for nodes
// Struct for Node
struct Node {
    int x, y;
    int mapIdx;
    double h;
    bool valid;
    
    Node* parent;
    
    Node(int x, int y, int mapIdx, double h, Node* parent = nullptr, bool valid = true)
        : x(x), y(y), mapIdx(mapIdx), h(h), parent(parent), valid(true) {}
    
    double get_g() const {
        return g_map[mapIdx];  // Assuming g_map is accessible globally or passed in appropriately
    }
};

// Struct for comparing nodes in priority queue
struct Compare {
    double e;  // Weight for the heuristic (epsilon)
    Compare(double epsilon) : e(epsilon) {}
    bool operator()(const Node* a, const Node* b) const {
        return a->get_g() + e * (a->h) > b->get_g() + e * (b->h);
    }
};

// Define global structures and variables
int s_start;
int s_goal;
Node* start_node = nullptr;
Node* goal_node = nullptr;
std::priority_queue<Node*, std::vector<Node*>, Compare> open_list(Compare(3.0)); // Open list with priority queue
std::unordered_map<int, Node*> inconsistent_list; // Inconsistent list with epsilon
std::unordered_map<int, int> open_list_map; // Map for open list
std::unordered_map<int, Node*> deleted_list; // Map for deleted nodes
std::unordered_map<int, Node*> node_map;  // Map to store all nodes by their map index

int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};


// Heuristic function (Manhattan distance)
double heuristic(int x, int y, int goalX, int goalY) {
    return std::hypot(abs(x - goalX), abs(y - goalY));
}
void registerOpenList(Node* node) {
    open_list.push(node);
    open_list_map[node->mapIdx] = node->get_g();
}
void removeFromOpenList(Node* node) {
    open_list_map.erase(open_list.top()->mapIdx);
    open_list.pop();
}


Node* getValidTopNode(std::priority_queue<Node*, std::vector<Node*>, Compare>& list) {
    while (!list.empty()) {
        Node* topNode = list.top();
        if (!topNode->valid) {
            // Node is invalid, pop and discard
            list.pop();
        } else {
            return topNode;  // Return valid node
        }
    }
    return nullptr;  // No valid nodes left
}
// Placeholder function for ImprovePath() (to be implemented later)
bool ImprovePath(Node* goal_node, int* map, int x_size, int y_size, int collision_thresh) {
    while (goal_node->get_g() > getValidTopNode(open_list)->get_g() + epsilon * heuristic(goal_node->x, goal_node->y, getValidTopNode(open_list)->x, getValidTopNode(open_list)->y)) {
        Node* node = getValidTopNode(open_list);
        removeFromOpenList(node);

        closed_list.insert(node->mapIdx);
        v_map[node->mapIdx] = node->get_g();

        std::cout << "Expanding node (" << node->x << ", " << node->y << ") with g = " << node->get_g() << std::endl;
        std::cout << "Goal no g value: " << goal_node->get_g() << std::endl;

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int newx = node->x + dX[dir];
            int newy = node->y + dY[dir];
            int newIdx = GETMAPINDEX(newx, newy, x_size, y_size);
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
                int cost = map[newIdx];
                if (cost >= 0 && cost < collision_thresh) {
                    double tentative_g = v_map[node->mapIdx] + cost;

                    if (g_map[newIdx] > tentative_g) {
                        g_map[newIdx] = tentative_g;
                        Node* new_node = new Node(newx, newy, newIdx, heuristic(newx, newy, goal_node->x, goal_node->y), node, true);
                        if (newx == goal_node->x && newy == goal_node->y) {
                            goal_node->parent = node;
                        }

                        // std::cout << new_node->x << " " << new_node->y << std::endl;
                        if (closed_list.find(newIdx) == closed_list.end()) {
                            if (inconsistent_list.find(newIdx) == inconsistent_list.end() 
                                && open_list_map.find(newIdx) == open_list_map.end()) {
                                
                                registerOpenList(new_node);
                            }
                            else {
                                inconsistent_list.insert(std::make_pair(newIdx, new_node));
                                closed_list.erase(newIdx);
                            }
                        }
                    }
                }
            }
        }
    }
    if (g_map[goal_node->mapIdx] == std::numeric_limits<double>::infinity()) {
        std::cout << "Goal node is unreachable!" << std::endl;

        return false;
    }
    else return true;
    
}

// Placeholder function for Step1 (to be implemented)
bool ComputePath(Node* goal_node, int* map, int x_size, int y_size, int collision_thresh) {
    while(true) {
        bool improved_path = ImprovePath(goal_node, map, x_size, y_size, collision_thresh);
        if (!improved_path) return false;
        else if (epsilon == 1.0 || (int) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count() > 700.0) return true;
        for(const auto& pair : inconsistent_list) {
            open_list.push(pair.second);
        }
        inconsistent_list.clear();

        closed_list.clear();

        epsilon = MAX(1.0, epsilon * 0.9);
    }
}
void traverseSubtree(Node* root, std::unordered_set<Node*>& subtree, const std::unordered_map<int, Node*>& node_map) {
    if (root == nullptr) return;
    
    // Add the root node to the subtree
    subtree.insert(root);

    // Traverse through the node_map to find nodes that have this root as their parent
    for (const auto& pair : node_map) {
        Node* node = pair.second;
        if (node->parent == root) {
            traverseSubtree(node, subtree, node_map);  // Recursive call for children
        }
    }
}

// Helper function to compare two subtrees
void compareSubtrees(const std::unordered_set<Node*>& previous_subtree,
                     const std::unordered_set<Node*>& current_subtree) {
    for (Node* node : previous_subtree) {
        // If a node is in previous_subtree but not in current_subtree
        if (current_subtree.find(node) == current_subtree.end()) {
            // Handle the node as needed, e.g., mark it for deletion
            g_map[node->mapIdx] = std::numeric_limits<double>::infinity();
            v_map[node->mapIdx] = std::numeric_limits<double>::infinity();
            node->parent = nullptr;

            // Remove the node from open/inconsistent lists, mark it as deleted, etc.
        }
    }
}
// Main planner function for Iterative ARA*

// TODO: At second iteration, the next move isnt found, which means that the goal_node doesnt become connected to the start_node
// TODO: The goal_node is not being connected to the start_node
// TODO: improved path and compute path is still working, so there is a path, but just that the goal node isnt connected.

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    // Initialize variables and structures on the first call
    // Initialize epsilon, g_map, v_map, and parent pointers for all nodes.
    targetposeX = target_traj[curr_time + 1];
    targetposeY = target_traj[curr_time + 1 + target_steps];
    if (!initialized) {
        startTime = std::chrono::steady_clock::now();

        for (int i = 0; i < x_size * y_size; ++i) {
            g_map[i] = std::numeric_limits<double>::infinity();
            v_map[i] = std::numeric_limits<double>::infinity();
        }
        epsilon = 10.0;  // Set initial epsilon value

        // targetposeX = target_traj[curr_time + 1]; // Set target position as one in front
        // targetposeY = target_traj[curr_time + 1 + target_steps];
        s_start = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);  // Set start position of the robot
        s_goal = GETMAPINDEX(targetposeX, targetposeY, x_size, y_size); // Set initial goal position

        while(!open_list.empty()) open_list.pop(); // Clear the open list
        open_list_map.clear();
        closed_list.clear();
        inconsistent_list.clear();
        deleted_list.clear();
        g_map[s_start] = 0; // Set the g-value of the start node to 0
        start_node = new Node(robotposeX, robotposeY, s_start, heuristic(robotposeX, robotposeY, targetposeX, targetposeY)); // Create start node
        goal_node = new Node(targetposeX, targetposeY, s_goal, heuristic(targetposeX, targetposeY, targetposeX, targetposeY)); // Create goal node
        open_list.push(start_node); // Add start node to open list
        std::cout << "Robot position: (" << robotposeX << ", " << robotposeY << ")" << std::endl;
        initialized = true;
    }

    if (s_start != s_goal) {
        std::cout << "current time: " << curr_time << std::endl;
        bool path_found = ComputePath(goal_node, map, x_size, y_size, collision_thresh);
        // printf("Path found: %d\n", path_found);
        if (!path_found) {
            std::cout << "path not found due to computePath returning false" << std::endl;
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        }

        Node* previous_node = start_node;  // Store the current start position

        // std::cout << "Path length " << path.size() << std::endl;

        // // Identify the path from start to goal using parent pointers
        // if (start_node->mapIdx != s_goal && path.size() > 1 ) {
        //     action_ptr[0] = path[1]->x;
        //     action_ptr[1] = path[1]->y;
        //     std::cout << "Action: (" << action_ptr[0] << ", " << action_ptr[1] << ")" << std::endl;
        // } else {
        //     action_ptr[0] = robotposeX;
        //     action_ptr[1] = robotposeY;
        //     return;
        // }
        // If a path to the target was found, backtrack to get the next move
        if (goal_node->parent == nullptr) std::cout << "goal node parent is null" << std::endl;
        if (goal_node == nullptr) std::cout << "goal node is null" << std::endl;

        if (start_node->mapIdx != goal_node->mapIdx && goal_node != nullptr) {
            Node* backtrack = goal_node;
            while (backtrack->parent != nullptr && backtrack->parent->parent != nullptr) {
                backtrack = backtrack->parent;
            }
            std::cout << "moving to grid x y" << backtrack->x << " " << backtrack->y << std::endl;
            action_ptr[0] = backtrack->x;
            action_ptr[1] = backtrack->y;
        } else {
            // If no path found, stay in place
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        }
        s_start = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
        start_node = new Node(robotposeX, robotposeY, s_start, heuristic(robotposeX, robotposeY, targetposeX, targetposeY));

        s_goal = GETMAPINDEX(targetposeX, targetposeY, x_size, y_size);
        goal_node = new Node(targetposeX, targetposeY, s_goal, heuristic(targetposeX, targetposeY, targetposeX, targetposeY));

        // Step 1: Handle start state updates
        if (g_map[start_node->mapIdx] != v_map[start_node->mapIdx]) {
            g_map[start_node->mapIdx] = v_map[start_node->mapIdx];
            if (inconsistent_list.find(start_node->mapIdx) != inconsistent_list.end()) {
                start_node->valid = false;
                inconsistent_list.erase(start_node->mapIdx);
            }

            if (open_list_map.find(start_node->mapIdx) != open_list_map.end()) {
                start_node->valid = false;
                open_list_map.erase(start_node->mapIdx);
            }
        }

        // Step 2: Handle state changes in the search tree
        if (start_node != previous_node) {
            start_node->parent = nullptr;
                // Set of nodes in the previous start node's subtree
            std::unordered_set<Node*> previous_subtree;
            // Set of nodes in the current start node's subtree
            std::unordered_set<Node*> current_subtree;

            traverseSubtree(previous_node, previous_subtree, node_map);
            traverseSubtree(start_node, current_subtree, node_map);

            for (Node* node : previous_subtree) {
                if (current_subtree.find(node) == current_subtree.end()) {
                    g_map[node->mapIdx] = std::numeric_limits<double>::infinity();
                    v_map[node->mapIdx] = std::numeric_limits<double>::infinity();
                    node->parent = nullptr;

                    deleted_list[node->mapIdx] = node;

                    node->valid = false;                    
                    open_list_map.erase(node->mapIdx);
                    inconsistent_list.erase(node->mapIdx);
                }
            }
        }

        // Step 3: Handle inconsistent nodes
            for (const auto& pair : deleted_list) {
                Node* node = pair.second;  // Get the Node* from the map

                for (int dir = 0; dir < NUMOFDIRS; dir++) {
                int newx = node->x + dX[dir];
                int newy = node->y + dY[dir];
                int newIdx = GETMAPINDEX(newx, newy, x_size, y_size);
                if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
                    int cost = map[newIdx];
                    if (cost >= 0 && cost < collision_thresh) {
                        double tentative_g = v_map[newIdx] + cost;
                        if (g_map[node->mapIdx] > tentative_g) {
                            g_map[node->mapIdx] = tentative_g;
                            Node* new_node = new Node(newx, newy, newIdx, heuristic(newx, newy, goal_node->x, goal_node->y), node);
                            node->parent = new_node;
                            
                        }
                    }
                }
            }
            if (g_map[node->mapIdx] != std::numeric_limits<double>::infinity()) {
                registerOpenList(node);
            }

        }
        for(const auto& pair : inconsistent_list) {
            open_list.push(pair.second);
            open_list_map[pair.second->mapIdx];
        }
        inconsistent_list.clear();

        closed_list.clear();
        deleted_list.clear();

        // Step 4: Adjust epsilon and prepare for the next iteration
        if (goal_node->get_g() > getValidTopNode(open_list)->get_g() + epsilon * heuristic(goal_node->x, goal_node->y, getValidTopNode(open_list)->x, getValidTopNode(open_list)->y)) {
            epsilon = 4.0;
        }
        else {
            epsilon = MAX(1.0, epsilon * 0.9);
        }
        

    }
    // End of planner function
}
