#include "../include/planner.h"
#include <math.h>
#include <stdio.h>
#include <queue>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
Node* goal_node = nullptr;

struct Node {
    int x, y;
    double g, h;
    Node* parent;

    Node(int x, int y, double g, double h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}
};

struct Compare {
    double e = 1.0;  // Weight for the heuristic
    bool operator()(const Node* a, const Node* b) {
        return a->g + e * (a->h) > b->g + e * (b->h);
    }
};

// Manhattan distance heuristic
double heuristic(int x, int y, int goalX, int goalY) {
    return MAX(abs(x - goalX), abs(y - goalY));
}

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
    if (curr_time == target_steps - 1) {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }
    targetposeX = target_traj[curr_time + 1];
    targetposeY = target_traj[curr_time + 1 + target_steps];
    // 8-connected grid movement directions
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // The target's current position is given by targetposeX and targetposeY
    printf("robot: %d %d;\n", robotposeX, robotposeY);
    printf("target: %d %d;\n", targetposeX, targetposeY);

    // Priority queue for open list
    std::priority_queue<Node*, std::vector<Node*>, Compare> open_list;

    // g_map keeps track of the lowest g-values (cost to reach a node)
    std::vector<std::vector<double>> g_map(x_size+1, std::vector<double>(y_size+1, std::numeric_limits<double>::infinity()));

    // Add the start node (robot's position) to the open list
    Node* start_node = new Node(robotposeX, robotposeY, 0, heuristic(robotposeX, robotposeY, targetposeX, targetposeY));
    open_list.push(start_node);
    g_map[robotposeX][robotposeY] = 0;


    // A* search loop
    while (!open_list.empty()) {
        Node* current = open_list.top();
        open_list.pop();

        // Check if we've reached the target's current position
        if (current->x == targetposeX && current->y == targetposeY) {
            goal_node = current;
            break;
        }
        else if ( map[GETMAPINDEX(current->x, current->y, x_size, y_size)] >= collision_thresh) {
            break;
        }

        // Expand the current node's neighbors
        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int newx = current->x + dX[dir];
            int newy = current->y + dY[dir];

            // Ensure the new position is within bounds
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
                int cell_cost = map[GETMAPINDEX(newx, newy, x_size, y_size)];
                if (cell_cost >= 0 && cell_cost < collision_thresh) {
                    double tentative_g = current->g + cell_cost;

                    // If the new g-value is lower, update the neighbor's cost
                    if (tentative_g < g_map[newx][newy]) {
                        g_map[newx][newy] = tentative_g;
                        Node* new_node = new Node(newx, newy, tentative_g, heuristic(newx, newy, targetposeX, targetposeY), current);
                        open_list.push(new_node);
                    }
                }
            }
        }
    }

    // If a path to the target was found, backtrack to get the next move
    if (goal_node != nullptr) {
        Node* backtrack = goal_node;
        while (backtrack->parent != nullptr && backtrack->parent->parent != nullptr) {
            backtrack = backtrack->parent;
        }
        action_ptr[0] = backtrack->x;
        action_ptr[1] = backtrack->y;
    } else {
        // If no path found, stay in place
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    // Clean up allocated memory
    delete start_node;
    while (!open_list.empty()) {
        delete open_list.top();
        open_list.pop();
    }

    return;
}