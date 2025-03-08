#include <iostream>
#include <vector>
#include <cstdlib>  // For rand()
#include <cmath>    // For ceil()
using namespace std;

// Threshold for fault node energy level
#define MIN_ENERGY_THRESHOLD 5.0  // Minimum energy level
#define MIN_FAULT_TOLERANCE_PERCENTAGE 10  // Fault tolerance percentage (10% of total nodes)

// Sensor Node Structure
struct SensorNode {
    int id;
    double energy;  // Current energy level
    bool isFaulty;
};

// Cluster Head Structure
struct ClusterHead {
    int clusterId;
    vector<SensorNode*> nodes;  // Pointers to sensor nodes in the cluster
    int faultNodeCount;

    // Constructor
    ClusterHead(int id) : clusterId(id), faultNodeCount(0) {}

    // Function to check and update faulty nodes
    void UpdateFaultStatus() {
        faultNodeCount = 0;  // Reset count before checking

        for (auto &node : nodes) {
            if (node->energy <= MIN_ENERGY_THRESHOLD) {
                if (!node->isFaulty) {
                    node->isFaulty = true;
                    faultNodeCount++;
                    cout << "Node " << node->id << " declared as FAULTY in Cluster " << clusterId << endl;
                }
            }
        }

        // Check if the fault threshold is exceeded
        double totalNodes = nodes.size();
        double faultPercentage = (faultNodeCount / totalNodes) * 100.0;

        if (faultPercentage >= MIN_FAULT_TOLERANCE_PERCENTAGE) {
            cout << "[ALERT] Cluster " << clusterId << " exceeds " << MIN_FAULT_TOLERANCE_PERCENTAGE 
                 << "% faulty nodes! Initiating recovery...\n";
            InitiateRecovery();
        }
    }

    // Fault Recovery Mechanism
    void InitiateRecovery() {
        cout << "[Cluster " << clusterId << "] Recovering from faults...\n";
        // Options: Select a replacement node, trigger re-clustering, or notify base station
    }
};

// Simulating Energy Consumption & Fault Node Detection
void SimulateNetwork(int numClusters, int nodesPerCluster) {
    vector<ClusterHead> clusters;

    // Initialize clusters and sensor nodes
    for (int i = 0; i < numClusters; i++) {
        clusters.push_back(ClusterHead(i)); // Create Cluster Head
        for (int j = 0; j < nodesPerCluster; j++) {
            double initialEnergy = (rand() % 20) + 1; // Random energy between 1-20
            SensorNode* node = new SensorNode{j, initialEnergy, false};
            clusters[i].nodes.push_back(node);
        }
    }

    // Simulating multiple rounds of operation
    for (int round = 1; round <= 5; round++) {
        cout << "\n--- ROUND " << round << " ---\n";
        
        // Simulate energy drain
        for (auto &cluster : clusters) {
            for (auto &node : cluster.nodes) {
                if (!node->isFaulty) {
                    node->energy -= (rand() % 3) + 1; // Random energy consumption (1-3 units)
                }
            }
            cluster.UpdateFaultStatus(); // Check faults after energy drain
        }
    }
}

// Main Function
int main() {
    srand(time(0)); // Random seed
    int numClusters = 3;    // Number of clusters
    int nodesPerCluster = 10; // Nodes per cluster

    SimulateNetwork(numClusters, nodesPerCluster);
    return 0;
}
