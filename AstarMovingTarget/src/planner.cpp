#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <memory>
#include <chrono>
#include <iostream>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 9 // [0, 0] action added

int inf = std::numeric_limits<double>::infinity();

struct Node 
{
    int idx;
    int g;
    int h;
    int f;
    int t;
    Node* parent;

    Node(int idx, int h, int t, int f = inf, Node* parent = nullptr)
        : idx(idx), g(inf) , h(h), t(t), f(inf), parent(parent)
    {}
};
struct HeuristicData {
    int h;      // optimal g or heuristic value
    int time;   // time at which the heuristic is calculated
};

struct Compare 
{
    bool operator()(Node* a, Node* b) const {
        return a->f > b->f;
    }
};

// Global Variables
std::chrono::time_point<std::chrono::steady_clock> s_time; // start time of each iteration

std::unordered_map<int, Node* > node_list;
std::priority_queue<Node*, std::vector<Node* >, Compare> open_list; // Open list for both Dijsktra and A*

std::unordered_map<int, HeuristicData> h_map;  // idx, h (optimal g), time
std::unordered_set<int> closed_list;

std::unordered_map<int, int> goal_traj;
std::vector<int> computed_path;

bool initialized = false;
int x_robot, y_robot;

// Since 3D, node_list requires unique index to account for time
int cantorPairing(int idx, int t) 
{
    return ((idx + t) * (idx + t + 1)) / 2 + t;
}

bool inBound(int x, int y, int x_size, int y_size) 
{
    return (x >= 1 && x <= x_size && y >= 1 && y <= y_size);
}

void updateNode( 
        int new_idx, 
        int t, 
        int cost, 
        int collision_thresh,
        Node* curr_node,
        bool astar= false,
        int elapsed_time = 0) 
{
    if (cost >= 0 && cost < collision_thresh)
    {
        int h;
        double eps = (astar) ? 2.0 : 1.0;
        int unique_idx = (astar) ? cantorPairing(new_idx, t) : new_idx;
        if (node_list.find(unique_idx) == node_list.end()) 
        {
            double lambda = 1.0;
            if (astar) 
            {
                int total_time = elapsed_time + t;
                h =  h_map[new_idx].h + lambda * abs(h_map[new_idx].time - total_time);  // Accessing fields of the HeuristicData struct
            }
            else h = 0;
            Node* _new_node = new Node(new_idx, h, t);
            node_list[unique_idx] = _new_node;
        }

        if (node_list[unique_idx]->g > curr_node->g + cost) 
        {
            node_list[unique_idx]->g = curr_node->g + cost;
            node_list[unique_idx]->f = node_list[unique_idx]->g + eps * node_list[unique_idx]->h;
            node_list[unique_idx]->parent = curr_node;

            open_list.push(node_list[unique_idx]);
        }
    }
}

void expandNeighbor(Node* curr_node, int x_size, int y_size, int* map, int collision_thresh, int time_elapsed = 0, bool astar = false) 
{
    // Additional move of staying in the same place
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

    int t = curr_node->t + 1; // neighbor time
    int curr_x = curr_node->idx % x_size + 1;
    int curr_y = curr_node->idx / x_size + 1;
    for(int dir = 0; dir < (NUMOFDIRS); ++dir)
    {
        int newx = curr_x + dX[dir];
        int newy = curr_y + dY[dir];
        int new_idx = GETMAPINDEX(newx, newy, x_size, y_size);
        int unique_idx = cantorPairing(new_idx, t);

        int cost = map[new_idx];
        int t = curr_node->t + 1;
        // std::cout << "time of neighbor: " << t << std::endl;
        if (inBound(newx, newy, x_size, y_size))
        {
            if (astar && closed_list.find(unique_idx) == closed_list.end()) updateNode(new_idx, t, cost, collision_thresh, curr_node, true, time_elapsed);
            else if (!astar && h_map.find(new_idx) == h_map.end()) updateNode(new_idx, t, cost, collision_thresh, curr_node);
        }
    }
}

void heuristicsDijsktra(int x_size, int y_size, int* map, int collision_thresh)
{
    while(!open_list.empty())
    {
        Node* curr_node = open_list.top();
        open_list.pop();

        if(h_map.find(curr_node->idx) != h_map.end()) continue; // skip if already computed

        h_map[curr_node->idx] = HeuristicData{curr_node->g, curr_node->t}; //optimal g is the heuristic
        expandNeighbor(curr_node, x_size, y_size, map, collision_thresh, 0, false);
    }
}

void computeAstarPath(int x_size, int y_size, int* map, int collision_thresh, int target_steps)
{

    while(!open_list.empty())
    {
        int time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - s_time).count();
        // std::cout << " ASTAR" << std::endl;
        Node* curr_node = open_list.top();
        open_list.pop();

        // int x = (int)(curr_node->idx % x_size) + 1;
        // int y = (int)(curr_node->idx / x_size) + 1;
        // std::cout << "Expanding node: (" << x << ", " << y << ") at time: " << curr_node->t << std::endl;
        int unique_idx = cantorPairing(curr_node->idx, curr_node->t); 
        if(closed_list.find(unique_idx) != closed_list.end()) continue; //skip if in closed list
        closed_list.insert(unique_idx);

        if(goal_traj.find(curr_node->idx) != goal_traj.end() 
          && curr_node->t == (goal_traj[curr_node->idx] - time_elapsed))
        {
            while(curr_node->parent != nullptr) //backtracking
            {
                computed_path.push_back(curr_node->idx);
                curr_node = curr_node->parent;
                std::cout << "Computed path: " << curr_node->idx << std::endl;
            }
            std::reverse(computed_path.begin(), computed_path.end());
            return;
        }

        std::cout << "target_steps" << target_steps << std::endl;
        std::cout << "time elapsed: " << time_elapsed << std::endl;
        expandNeighbor(curr_node, x_size, y_size, map, collision_thresh, time_elapsed, true);
    }
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
    targetposeX = target_traj[curr_time + 1];
    targetposeY = target_traj[curr_time + 1 + target_steps];

    if(!initialized) // if initialized for the first time
    {
        s_time = std::chrono::steady_clock::now();
        initialized = true; // set initialized to true to skip in future calls

        for(int i = 0; i < target_steps; ++i) // Multi Goal Map for 3D A*
        {
            int target_idx = GETMAPINDEX(target_traj[i], target_traj[target_steps + i], x_size, y_size);
            goal_traj[target_idx] = i; //construct target trajectory map wrt timestep

            if(i > (target_steps / 2)) // only add nodes for the second half of the trajectory
            {
                Node* node = new Node(target_idx, 0, i);
                node->g = 0;
                node->f = node->g + node->h;
                node_list[target_idx] = node;
                open_list.push(node);
            }
        }
        std::cout << "computing heuristics" << std::endl;

        heuristicsDijsktra(x_size, y_size, map, collision_thresh);
        node_list.clear();

        int index = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
        int h = h_map[index].h;
        Node* node = new Node(index, h, 0);
        node->f = node->h; // g_start is 0

        node_list[index] = node;
        open_list.push(node);
        std::cout << "computing path" << std::endl;

        computeAstarPath(x_size, y_size, map, collision_thresh, target_steps);
    }

    static int path_idx = 0;
    if(path_idx < computed_path.size())
    {
        int next_cell_idx = computed_path[path_idx];
        x_robot = (next_cell_idx % x_size) + 1;
        y_robot = (next_cell_idx / x_size) + 1;
        path_idx++;
    }

    action_ptr[0] = x_robot;
    action_ptr[1] = y_robot;
    
    if (x_robot == targetposeX && y_robot == targetposeY) {
        node_list.clear();
        open_list = std::priority_queue<Node*, std::vector<Node*>, Compare>(Compare());
        closed_list.clear();
        h_map.clear();
        computed_path.clear();
    }
    std::cout << "Robot command: (" << x_robot << ", " << y_robot << ")" << std::endl;
    return;
}