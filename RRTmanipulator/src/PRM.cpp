#include "PRM.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <memory>
#include <queue>
#include <vector>
#include <unordered_set>
// Constructor
PRMBase::PRMBase(int numofDOFs, double neighborhood_size) : numofDOFs(numofDOFs), neighborhood_size(neighborhood_size) {}

// Generates a random configuration
std::unique_ptr<double[]> PRMBase::generateRandomConfig() {
    auto config = std::make_unique<double[]>(numofDOFs);
    for (int i = 0; i < numofDOFs; ++i) {
        config[i] = ((double)rand() / RAND_MAX) * 2 * M_PI;
    }
    return config;
}

// Interpolates between two joint angle configurations
std::unique_ptr<double[]> PRMBase::createInterpolatedConfig(double* start, double* end, double alpha) {
    auto intermediateConfig = std::make_unique<double[]>(numofDOFs);
    for (int i = 0; i < numofDOFs; ++i) {
        intermediateConfig[i] = (1 - alpha) * start[i] + alpha * end[i];
    }
    return intermediateConfig;
}

// Checks validity of an edge (path between two configurations)
bool PRMBase::validatePath(double* start, double* end, int steps, double* map, int x_size, int y_size) {
    for (int i = 0; i <= steps; ++i) {
        double alpha = static_cast<double>(i) / steps;
        auto point = createInterpolatedConfig(start, end, alpha);
        if (!IsValidArmConfiguration(point.get(), numofDOFs, map, x_size, y_size)) {
            return false;
        }
    }
    return true;
}

// Computes Euclidean distance between two joint configurations
double PRMBase::calculateDistance(double* config1, double* config2) {
    double sum = 0.0;
    for (int i = 0; i < numofDOFs; ++i) {
        double diff = config1[i] - config2[i];
        diff = std::abs(diff);
        // Adjust for shortest path considering wrap-around at 2π
        if (diff > M_PI) diff = 2 * M_PI - diff;
        
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

// Adds an undirected edge between two nodes in the graph
void PRMBase::insertEdge(std::vector<std::vector<int>>& connectionGraph, int nodeA, int nodeB) {
    if (nodeA != nodeB) {
        if (std::find(connectionGraph[nodeA].begin(), connectionGraph[nodeA].end(), nodeB) == connectionGraph[nodeA].end()) {
            connectionGraph[nodeA].push_back(nodeB);
            connectionGraph[nodeB].push_back(nodeA);
        }
    }
}

// Connects the closest node to the given configuration
void PRMBase::linkClosestNode(double* config, std::vector<std::vector<int>>& connectionGraph, std::vector<std::unique_ptr<double[]>>& configMap, int newNodeIdx) {
    int closestNodeIdx = -1;
    double minDistance = std::numeric_limits<double>::max();
    for (size_t nodeIdx = 0; nodeIdx < configMap.size(); ++nodeIdx) {
        double distance = calculateDistance(config, configMap[nodeIdx].get());
        if (distance < minDistance) {
            minDistance = distance;
            closestNodeIdx = static_cast<int>(nodeIdx);
        }
    }
    if (closestNodeIdx != -1) {
        insertEdge(connectionGraph, newNodeIdx, closestNodeIdx);
        configMap[newNodeIdx] = std::unique_ptr<double[]>(config);
    }
}

// A* search to find path in PRM graph
std::vector<int> PRMBase::aStarSearch(int startNodeIdx, int goalNodeIdx, const std::vector<std::vector<int>>& connectionGraph, const std::vector<std::unique_ptr<double[]>>& configMap) {
    using NodeTuple = std::tuple<double, int, int>; // (f-score, node index, parent index)

    // Comparator for f-scores in priority queue
    auto comparator = [](const NodeTuple& n1, const NodeTuple& n2) {
        return std::get<0>(n1) > std::get<0>(n2); // Min-heap based on f-score
    };
    std::priority_queue<NodeTuple, std::vector<NodeTuple>, decltype(comparator)> openList(comparator);

    std::unordered_set<int> closedSet;  // Keeps track of fully processed nodes
    std::unordered_map<int, int> cameFrom;  // For path reconstruction
    std::unordered_map<int, double> gScore;  // Best known cost to each node
    gScore[startNodeIdx] = 0.0;

    double startToGoalDist = calculateDistance(configMap[startNodeIdx].get(), configMap[goalNodeIdx].get());
    openList.emplace(startToGoalDist, startNodeIdx, -1);

    while (!openList.empty()) {
        auto [fScore, currentNodeIdx, parentIdx] = openList.top();
        openList.pop();

        // Skip processing if already in closed set
        if (closedSet.find(currentNodeIdx) != closedSet.end()) continue;

        // Mark the node as visited
        cameFrom[currentNodeIdx] = parentIdx;

        // Check if we reached the goal
        if (currentNodeIdx == goalNodeIdx) {
            std::vector<int> path;
            while (currentNodeIdx != -1) {
                path.push_back(currentNodeIdx);
                currentNodeIdx = cameFrom[currentNodeIdx];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Add to closed set to prevent revisiting
        closedSet.insert(currentNodeIdx);

        // Explore neighbors
        for (int neighborIdx : connectionGraph[currentNodeIdx]) {
            if (closedSet.find(neighborIdx) != closedSet.end()) continue;

            double tentativeGScore = gScore[currentNodeIdx] + calculateDistance(configMap[currentNodeIdx].get(), configMap[neighborIdx].get());

            // Only consider this path if it's better than any previously recorded one
            if (gScore.find(neighborIdx) == gScore.end() || tentativeGScore < gScore[neighborIdx]) {
                gScore[neighborIdx] = tentativeGScore;
                double fScoreNeighbor = tentativeGScore + calculateDistance(configMap[neighborIdx].get(), configMap[goalNodeIdx].get());
                openList.emplace(fScoreNeighbor, neighborIdx, currentNodeIdx);
            }
        }
    }

    return {}; // Return empty path if no solution found
}