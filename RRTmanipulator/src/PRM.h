#ifndef PRM_H
#define PRM_H

#include <vector>
#include <cmath>
#include <queue>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>

// Checks if a given configuration is valid (no collision).
int IsValidArmConfiguration(double* angles, int numofDOFs, double* map, int x_size, int y_size);

class PRMBase {
public:
    PRMBase(int numofDOFs, double neighborhood_size);  // Constructor

    // Generates a random joint angle configuration within valid bounds.
    std::unique_ptr<double[]> generateRandomConfig();

    // Interpolates between two joint angle configurations.
    std::unique_ptr<double[]> createInterpolatedConfig(double* start, double* end, double alpha);

    // Checks if a path between two configurations is collision-free.
    bool validatePath(double* start, double* end, int steps, double* map, int x_size, int y_size);

    // Computes the distance between two configurations in joint space.
    double calculateDistance(double* config1, double* config2);

    // Adds an undirected edge between two nodes in the graph.
    void insertEdge(std::vector<std::vector<int>>& connectionGraph, int nodeA, int nodeB);

    // Connects the closest node to a given configuration and updates the graph.
    void linkClosestNode(double* config, std::vector<std::vector<int>>& connectionGraph, std::vector<std::unique_ptr<double[]>>& configMap, int newNodeIdx);

    // Performs A* search to find a path from the start to the goal node in the PRM graph.
    std::vector<int> aStarSearch(int startNodeIdx, int goalNodeIdx, const std::vector<std::vector<int>>& connectionGraph, const std::vector<std::unique_ptr<double[]>>& configMap);

private:
    int numofDOFs;
    double neighborhood_size;
    std::vector<std::vector<int>> connectionGraph;
    std::vector<std::unique_ptr<double[]>> configMap;
};

#endif // PRM_H