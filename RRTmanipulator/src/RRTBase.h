#ifndef RRTBASE_H
#define RRTBASE_H

#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>

int IsValidArmConfiguration(double* angles, int numofDOFs, double* map, int x_size, int y_size);

struct Node {
    std::vector<double> joint_configs;
    std::shared_ptr<Node> parent;

    // Default constructor
    Node() = default;

    // Constructor with joint angles
    Node(const std::vector<double>& joint_configs) : joint_configs(joint_configs) {}
};

class RRTBase {
public:
    RRTBase(
        const std::vector<double>& start, 
        const std::vector<double>& goal, 
        int maxIterations, 
        double stepSize, 
        double* map, 
        int numofDOFs, 
        int x_size, 
        int y_size
    );
    std::shared_ptr<Node> buildRRT();
    void backtrackRRT(std::shared_ptr<Node> result, double*** plan, int* planlength);

    std::shared_ptr<Node> connect(const std::vector<double>& targetConfig, std::vector<std::shared_ptr<Node>>& rrtree);    std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>> buildRRTConnect();
    void extractPathConnect(std::shared_ptr<Node> startNode, std::shared_ptr<Node> goalNode, double*** plan, int* planlength);
    
    std::shared_ptr<Node> buildRRTStar();
    std::shared_ptr<Node> extendRRTStar(const std::vector<double>& randomConfig);
    double calculateCost(const std::shared_ptr<Node>& node) const;
    void rewire(const std::shared_ptr<Node>& qNewNode, const std::vector<std::shared_ptr<Node>>& nearbyNodes);
    double computeNorm(const std::vector<double>& node, const std::vector<double>& q) const;
    std::shared_ptr<Node> getNearest(const std::vector<double>& randomConfig, const std::vector<std::shared_ptr<Node>>& rrtree);


private:
    std::shared_ptr<Node> addNodeToTree(std::vector<std::shared_ptr<Node>>& rrtree, std::shared_ptr<Node> parent, const std::vector<double>& config);
    bool createJointConfig(const std::vector<double>& qRand, const std::vector<double>& qNear, std::vector<double>& qNew);
    std::vector<double> generateRandomNode();

    // bool isCollisionFree(const std::vector<double>& config1, const std::vector<double>& config2, int steps);
    // void connectToNeighbors(int nodeIdx);
    // void reconstructPRMPath(int goalIdx, int startIdx, double*** plan, int* planlength, const std::unordered_map<int, int>& cameFrom);

    std::vector<double> start;
    std::vector<double> goal;
    int maxIterations;
    double stepSize;
    double* map;
    int numofDOFs;
    int x_size;
    int y_size;
    double goalThreshold = 1e-4;
    std::vector<std::shared_ptr<Node>> tree;
    std::vector<std::shared_ptr<Node>> tree_connect;
};

#endif // RRTBASE_H