#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <stack>
#include <memory>
#include <chrono>
#include <climits>
#include <iostream>

using namespace std;

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 9

#define INF INT_MAX
struct Node {
    int idx;
    int g;
    int h;
    int f;
    int t;
    Node* parent;

    Node(int idx, int h, int t, int f = INF, Node* parent = nullptr)
        : idx(idx), g(INF) , h(h), t(t), f(INF), parent(parent)
    {}
};


struct Compare {
    bool operator()(Node* a, Node* b) const {
        return a->f > b->f;
    }
};

// Global Variables
bool initialized = false;
std::unordered_map<int, Node* > node_list;
std::unordered_map<int, int> goal_traj;
std::unordered_map<int, std::pair<int, int> > heuristics;
std::unordered_set<int> closed;
std::priority_queue<Node*, std::vector<Node* >, Compare> open_list;
std::stack<int> actionStack;
std::chrono::time_point<std::chrono::steady_clock> startTime;

int prevX, prevY;

void computeHeuristics(
        int x_size,
        int y_size,
        int dX[],
        int dY[],
        int* map,
        int collision_thresh
        )
{
    while(!open_list.empty())
    {
        Node* curr_node = open_list.top();
        open_list.pop();
        if(heuristics.find(curr_node->idx) != heuristics.end()) continue; // skip repetitions in open list
        heuristics[curr_node->idx] = std::pair<int, int>(curr_node->g, curr_node->t); // closed list. stores optimal g-vals

        int rX = (int)(curr_node->idx % x_size) + 1;
        int rY = (int)(curr_node->idx / x_size) + 1;

        for(int dir = 0; dir < NUMOFDIRS; ++dir)
        {
            int newx = rX + dX[dir];
            int newy = rY + dY[dir];
            int newIndex = (int) GETMAPINDEX(newx, newy, x_size, y_size);

            if(newx >= 1 and newx <= x_size and newy >= 1 and newy <= y_size and heuristics.find(newIndex) == heuristics.end())
            {
                int cost = (int) map[newIndex];
                if((cost >= 0) and (cost < collision_thresh)) // cell is free
                {
                    if(node_list.find(newIndex) == node_list.end()) // create a new Node, if it does not exist
                    {
                        Node* _new_node = new Node(newIndex, 0, curr_node->t);
                        node_list[newIndex] = _new_node;
                    }
                    if(node_list[newIndex]->g > curr_node->g + cost) // compare g values and cost, update parent if needed
                    {
                        node_list[newIndex]->g = curr_node->g + cost;
                        node_list[newIndex]->f = node_list[newIndex]->g + node_list[newIndex]->h;
                        node_list[newIndex]->parent = curr_node;
                        open_list.push(node_list[newIndex]);
                    }
                }
            }
        }
    }
}

/*
    working for all maps except 7
*/
void computePath(
        int x_size,
        int y_size,
        int dX[],
        int dY[],
        int* map,
        int collision_thresh,
        int* target_traj,
        int target_steps,
        int robotposeX,
        int robotposeY,
        int targetposeX,
        int targetposeY
        )
{

    while(!open_list.empty())
    {

        Node* curr_node = open_list.top();
        open_list.pop();

        int rX = (int)(curr_node->idx % x_size) + 1;
        int rY = (int)(curr_node->idx / x_size) + 1;

        int digits = (curr_node->t == 0) ? 0 : (int)(std::log10(curr_node->t) + 1);
        int newIndexForMap = curr_node->idx * ((int)std::pow(10, digits)) + curr_node->t; // concatenate t value to the end of index for unique key
        
        if(closed.find(newIndexForMap) != closed.end()) continue; // skip repetitions in open list
        closed.insert(newIndexForMap);
        int timeElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();

        if(goal_traj.find(curr_node->idx) != goal_traj.end() && curr_node->t == (goal_traj[curr_node->idx] - timeElapsed))
        {
            while(curr_node)
            {
                actionStack.push(curr_node->idx);
                curr_node = curr_node->parent;
            }
            actionStack.pop(); // remove start Node
            return;
        }






        int t = curr_node->t + 1;
        if(t > target_steps)
        {
            continue;
        }
        for(int dir = 0; dir < (NUMOFDIRS + 1); ++dir)
        {
            int newx = rX + dX[dir];
            int newy = rY + dY[dir];
            int newIndex = (int) GETMAPINDEX(newx, newy, x_size, y_size);
            digits = (t == 0) ? 0 : (int)(std::log10(t) + 1);
            newIndexForMap = newIndex * ((int)std::pow(10, digits)) + t; // concatenate t value to the end of index for unique key
            
            if(newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
            {
                int newIndex = (int) GETMAPINDEX(newx, newy, x_size, y_size);
                digits = (t == 0) ? 0 : (int)(std::log10(t) + 1);
                newIndexForMap = newIndex * ((int)std::pow(10, digits)) + t; // concatenate t value to the end of index for unique key
                int cost = map[newIndex];

                if (closed.find(newIndexForMap) == closed.end())
                {
                    int cost = map[newIndex];

                    // Modify cost handling
                    if (cost < collision_thresh) // cell is free
                    {
                        // Normal pathfinding behavior
                        if(node_list.find(newIndexForMap) == node_list.end()) // create a new Node, if it does not exist
                        {
                            int totalTime = timeElapsed + t;
                            int h = (goal_traj.find(newIndex) != goal_traj.end() && totalTime <= goal_traj[newIndex]) ? cost * (goal_traj[newIndex] - totalTime) : heuristics[newIndex].first + abs(heuristics[newIndex].second - totalTime);
                            Node* _new_node = new Node(newIndex, h, t);
                            node_list[newIndexForMap] = _new_node;
                        }

                        if(node_list[newIndexForMap]->g > curr_node->g + cost) // compare g values and cost, update parent if needed
                        {
                            node_list[newIndexForMap]->g = curr_node->g + cost;
                            node_list[newIndexForMap]->f = node_list[newIndexForMap]->g + 1.8 * node_list[newIndexForMap]->h; // weighted A*
                            node_list[newIndexForMap]->parent = curr_node;
                            open_list.push(node_list[newIndexForMap]);
                        }
                    }
                }
            }
        }
    }
}

void planner(
        int*	map,
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
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};
    

    prevX = robotposeX;
    prevY = robotposeY;

    if(not initialized) // init s_start, g(start) = 0, add to the open set, and Node map
    {
        startTime = std::chrono::steady_clock::now();
        initialized = true;

        for(int i = 0; i < target_steps; ++i) // setup multi-goal map
        {
            int target_idx = GETMAPINDEX((int) target_traj[i], (int) target_traj[target_steps + i], x_size, y_size);
            // atleast 1 second will be skipped after execution (ceiling function). so subtract 1 sec from goal times
            goal_traj[target_idx] = i;

            if(i > (target_steps/2))
            {
                // all node_list are initalised with 0
                Node* a = new Node(target_idx, 0, i);
                a->g = 0;
                a->f = a->g + a->h;
                node_list[target_idx] = a;
                open_list.push(a);
            }
        }

        computeHeuristics(x_size, y_size, dX, dY, map, collision_thresh);
        node_list.clear();

        int index = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
        int h = heuristics[index].first;
        Node* b = new Node(index, h, 0);
        b->g = 0;
        b->f = b->g + b->h;
        node_list[index] = b;
        open_list.push(b);
        // call compute path
        computePath(x_size, y_size, dX, dY, map, collision_thresh, target_traj, target_steps, robotposeX, robotposeY, targetposeX, targetposeY);
    }
    if(!actionStack.empty())
    {
        int nextIndex = actionStack.top();
        actionStack.pop();
        prevX = (nextIndex % x_size) + 1;
        prevY = (nextIndex / x_size) + 1;
    }
    targetposeX = target_traj[curr_time + 1];
    targetposeY = target_traj[curr_time + 1 + target_steps];
    if (prevX == targetposeX && prevY == targetposeY) {
        node_list.clear();
        open_list = std::priority_queue<Node*, std::vector<Node*>, Compare>(Compare());
        closed.clear();
        heuristics.clear();
        actionStack = std::stack<int>();
    }
    action_ptr[0] = prevX;
    action_ptr[1] = prevY;

    // End
    
    return;
}