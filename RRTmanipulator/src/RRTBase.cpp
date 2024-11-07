#include "RRTBase.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
// Constructor with corrected parameter types and member initializers
RRTBase::RRTBase(
    const std::vector<double>& start, 
    const std::vector<double>& goal, 
    int maxIterations, 
    double stepSize, 
    double* map, 
    int numofDOFs, 
    int x_size, 
    int y_size)
    : start(start), goal(goal), maxIterations(maxIterations), stepSize(stepSize), map(map), 
      numofDOFs(numofDOFs), x_size(x_size), y_size(y_size) {}

std::shared_ptr<Node> RRTBase::addNodeToTree(std::vector<std::shared_ptr<Node>>& rrtree, std::shared_ptr<Node> parent, const std::vector<double>& config) {
    auto newNode = std::make_shared<Node>();
    newNode->joint_configs = config;
    newNode->parent = parent;

    rrtree.push_back(newNode);

    return newNode;  // Return the new node
}


// Find the Norm between two vectors
double RRTBase::computeNorm(const std::vector<double>& joint_configs_2, const std::vector<double>& joint_configs_1) const {
    double norm = 0.0;
    for (size_t i = 0; i < numofDOFs; i++) {
        norm += std::pow(joint_configs_2[i] - joint_configs_1[i], 2);
    }
    return std::sqrt(norm);
}

std::shared_ptr<Node> RRTBase::getNearest(const std::vector<double>& randomConfig, const std::vector<std::shared_ptr<Node>>& rrtree) {
    std::shared_ptr<Node> nearestNode = nullptr;
    double minDist = std::numeric_limits<double>::max();

    for (const auto& node : rrtree) {
        double dist = computeNorm(node->joint_configs, randomConfig);
        if (dist < minDist) {
            minDist = dist;
            nearestNode = node;
        }
    }
    return nearestNode;
}

bool RRTBase::createJointConfig(const std::vector<double>& random_state, const std::vector<double>& neighbor_state, std::vector<double>& new_state) {
    double maxStepDistance = this->stepSize;  // Max distance for each step

    // Compute the Euclidean distance between neighbor_state and random_state
    double dist = computeNorm(neighbor_state, random_state);

    // If random_state is within the maxStepDistance, just use it directly
    double scale = (dist <= maxStepDistance) ? 1.0 : maxStepDistance / dist;

    std::vector<double> midpoint_config(numofDOFs);     // Midpoint configuration

    // Move from neighbor_state toward random_state by the scaled step size
    for (size_t i = 0; i < numofDOFs; i++) {
        new_state[i] = neighbor_state[i] + scale * (random_state[i] - neighbor_state[i]);
        midpoint_config[i] = (neighbor_state[i] + new_state[i]) / 2.0;

    }

    // Ensure both the midpoint and new_state are collision-free
    if (dist < maxStepDistance * 0.5) {  // Only check midpoint if dist is significant
        return IsValidArmConfiguration(new_state.data(), numofDOFs, map, x_size, y_size);
    } else {
        return IsValidArmConfiguration(midpoint_config.data(), numofDOFs, map, x_size, y_size) &&
           IsValidArmConfiguration(new_state.data(), numofDOFs, map, x_size, y_size);
    }
}


// Generate a random configuration within the robot’s joint limits
std::vector<double> RRTBase::generateRandomNode() {
    std::vector<double> randomConfig(numofDOFs);
    for (int i = 0; i < numofDOFs; i++) {
        randomConfig[i] = ((double) rand() / RAND_MAX) * 2 * M_PI;  // Random value between 0 and 2*pi
    }
    return randomConfig;
}

// Build the RRT tree
// Build the RRT tree and extend it in each iteration
std::shared_ptr<Node> RRTBase::buildRRT() {
    addNodeToTree(tree, nullptr, start);  // Initialize tree with start node

    for (int i = 0; i < maxIterations; i++) {
        // Generate a random configuration, with 10% goal bias
        std::vector<double> random_config = (rand() / static_cast<double>(RAND_MAX) <= 0.1) ? goal : generateRandomNode();
        
        // Find the nearest node in the tree to the random configuration
        std::shared_ptr<Node> nearestNode = getNearest(random_config, tree);
        std::vector<double> newConfig(numofDOFs, 0);

        // Attempt to create a new configuration closer to the random configuration
        if (createJointConfig(random_config, nearestNode->joint_configs, newConfig)) {
            // Add the new node to the tree
            std::shared_ptr<Node> newNode = addNodeToTree(tree, nearestNode, newConfig);

            // Check if the new configuration is close enough to the goal
            if (computeNorm(newNode->joint_configs, goal) <= goalThreshold) {
                return newNode;  // Return the goal node if reached
            }
        }
    }
    return nullptr;  // Return null if no path is found
}
// Retrace the path from goal to start by traversing from goal node to start node
void RRTBase::backtrackRRT(std::shared_ptr<Node> result, double*** plan, int* planlength) {
    int pathLength = 0;
    std::shared_ptr<Node> currentNode = result;

    while (currentNode != nullptr) {
        pathLength++;
        currentNode = currentNode->parent;
    }

    *plan = (double**)malloc(pathLength * sizeof(double*));
    *planlength = pathLength;

    currentNode = result;
    for (int i = pathLength - 1; i >= 0; i--) {
        (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            (*plan)[i][j] = currentNode->joint_configs[j];
        }
        currentNode = currentNode->parent;
    }
}
//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

// Attempt to connect to a target configuration by extending the tree iteratively
std::shared_ptr<Node> RRTBase::connect(const std::vector<double>& targetConfig, std::vector<std::shared_ptr<Node>>& rrtree) {
    while (true) {
        // Find the nearest node in the tree to the target configuration
        std::shared_ptr<Node> nearestNode = getNearest(targetConfig, rrtree);
        std::vector<double> newConfig(numofDOFs, 0);

        // Create a new configuration moving from nearestNode towards targetConfig
        if (createJointConfig(targetConfig, nearestNode->joint_configs, newConfig)) {
            // Add the new node to the current tree
            std::shared_ptr<Node> newNode = addNodeToTree(rrtree, nearestNode, newConfig);

            // Check if the new node is within the goal threshold norm of the target
            if (computeNorm(newNode->joint_configs, targetConfig) <= goalThreshold) {
                return newNode;  // Reached target configuration
            }
        } else {
            return nullptr;  // Cannot extend further towards the target
        }
    }
}

std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>> RRTBase::buildRRTConnect() {
    addNodeToTree(tree, nullptr, start);         // Initialize start tree
    addNodeToTree(tree_connect, nullptr, goal);   // Initialize goal tree

    for (int k = 0; k < maxIterations; k++) {
        // Determine which tree to expand in this iteration
        auto& currentTree = (k % 2 == 0) ? tree : tree_connect;
        auto& oppositeTree = (k % 2 == 0) ? tree_connect : tree;

        // Generate random configuration with a goal bias
        std::vector<double> random_config = generateRandomNode();
        if (static_cast<double>(rand()) / RAND_MAX <= 0.1) {
            random_config = (k % 2 == 0) ? goal : start;  // Alternate goal bias based on the current tree
        }

        // Find the nearest node in the current tree
        std::shared_ptr<Node> nearestNode = getNearest(random_config, currentTree);
        std::vector<double> newConfig(numofDOFs, 0);

        // Extend current tree towards the random configuration
        if (createJointConfig(random_config, nearestNode->joint_configs, newConfig)) {
            std::shared_ptr<Node> newNode = addNodeToTree(currentTree, nearestNode, newConfig);

            // Try to connect the newly added node to the opposite tree
            std::shared_ptr<Node> connectNode = connect(newNode->joint_configs, oppositeTree);
            if (connectNode != nullptr) {
                return (k % 2 == 0) ? std::make_pair(newNode, connectNode) : std::make_pair(connectNode, newNode);
            }
        }
    }
    return {nullptr, nullptr};  // Return null pair if no path is found
}

// Extract path from the connected nodes in RRTConnect
void RRTBase::extractPathConnect(std::shared_ptr<Node> startNode, std::shared_ptr<Node> goalNode, double*** plan, int* planlength) {
    std::vector<std::shared_ptr<Node>> path1, path2;

    std::shared_ptr<Node> current = startNode;
    while (current != nullptr) {
        path1.push_back(current);
        current = current->parent;
    }

    current = goalNode;
    while (current != nullptr) {
        path2.push_back(current);
        current = current->parent;
    }

    *planlength = path1.size() + path2.size();
    *plan = (double**)malloc(*planlength * sizeof(double*));

    int index = 0;
    for (int i = path1.size() - 1; i >= 0; i--) {
        (*plan)[index] = (double*)malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            (*plan)[index][j] = path1[i]->joint_configs[j];
        }
        index++;
    }

    for (auto& node : path2) {
        (*plan)[index] = (double*)malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            (*plan)[index][j] = node->joint_configs[j];
        }
        index++;
    }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

double RRTBase::calculateCost(const std::shared_ptr<Node>& node) const {
    double cost = 0.0;
    std::shared_ptr<Node> current = node;
    while (current->parent) {
        cost += computeNorm(current->joint_configs, current->parent->joint_configs);
        current = current->parent;
    }
    return cost;
}

void RRTBase::rewire(const std::shared_ptr<Node>& newnNode, const std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    for (const std::shared_ptr<Node>& node : nearbyNodes) {
        double potentialNewCost = calculateCost(newnNode) + computeNorm(newnNode->joint_configs, node->joint_configs);
        if (potentialNewCost < calculateCost(node)) {
            node->parent = newnNode; // Assign using shared_ptr
        }
    }
}

std::shared_ptr<Node> RRTBase::extendRRTStar(const std::vector<double>& randomConfig) {
    std::shared_ptr<Node> nearestNode = getNearest(randomConfig, tree);
    std::vector<double> newConfig(numofDOFs, 0);

    if (createJointConfig(randomConfig, nearestNode->joint_configs, newConfig)) {
        // Create and add the new node
        std::shared_ptr<Node> newNode = std::make_shared<Node>(newConfig);
        newNode->parent = nearestNode;

        // Radius decreases over time
        double radius = 0.6;  // Set this to any fixed value that suits your needs
        std::vector<std::shared_ptr<Node>> nearbyNodes;
        for (const std::shared_ptr<Node>& node : tree) {
            if (computeNorm(node->joint_configs, newConfig) < radius) {
                nearbyNodes.push_back(node);
            }
        }
        // Choose the node in nearbyNodes with the lowest cost to set as the parent
        double minCost = calculateCost(newNode);
        std::shared_ptr<Node> minCostNode = nearestNode;

        for (const std::shared_ptr<Node>& neighbor : nearbyNodes) {
            double costThroughNeighbor = calculateCost(neighbor) + computeNorm(neighbor->joint_configs, newConfig);
            if (costThroughNeighbor < minCost) {
                minCost = costThroughNeighbor;
                minCostNode = neighbor;
            }
        }

        newNode->parent = minCostNode;
        tree.push_back(newNode);

        // Rewire other nearby nodes
        rewire(newNode, nearbyNodes);

        return newNode;
    }
    return nullptr;
}
std::shared_ptr<Node> RRTBase::buildRRTStar() {
    addNodeToTree(tree, nullptr, start);  // Add the start node to the tree

    for (int k = 0; k < maxIterations; ++k) {
        // Generate a random configuration
        std::vector<double> random_config = generateRandomNode();
        
        // Introduce a small goal bias to increase the chance of finding a path
        if (static_cast<double>(rand()) / RAND_MAX <= 0.1) random_config = goal; 

        // Extend the RRT* tree towards the random (or goal-biased) node
        std::shared_ptr<Node> newNode = extendRRTStar(random_config);
        
        // Check if the new node is close enough to the goal to consider the path complete
        if (newNode && computeNorm(newNode->joint_configs, goal) <= goalThreshold) {
            return newNode;  // Path to goal has been found
        }
    }
    
    // If we exhaust iterations without reaching the goal, return nullptr to indicate failure
    return nullptr;
}
