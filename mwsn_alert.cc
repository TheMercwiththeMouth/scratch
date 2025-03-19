#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ClusterHeadAlert");

#define MIN_ENERGY_THRESHOLD 5.0  // Minimum energy before a node is faulty
#define FAULT_TOLERANCE_PERCENTAGE 10  // 10% of cluster nodes
#define CLUSTER_SIZE 5 // Nodes per cluster

struct SensorNode {
    int id;
    double energy;
    bool isFaulty;
};

struct ClusterHead {
    int clusterId;
    std::vector<SensorNode*> nodes;
    int faultNodeCount;
    Ptr<Socket> alertSocket; // Socket for sending alerts

    ClusterHead(int id, Ptr<Node> chNode) : clusterId(id), faultNodeCount(0) {
        alertSocket = Socket::CreateSocket(chNode, UdpSocketFactory::GetTypeId());
        alertSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9999));
        alertSocket->SetRecvCallback(MakeCallback(&ClusterHead::HandleAlert, this));
    }

    void CheckFaultNodes() {
        faultNodeCount = 0;
        for (auto &node : nodes) {
            if (node->energy <= MIN_ENERGY_THRESHOLD) {
                if (!node->isFaulty) {
                    node->isFaulty = true;
                    faultNodeCount++;
                    NS_LOG_INFO("Node " << node->id << " declared FAULTY in Cluster " << clusterId);
                }
            }
        }

        double faultPercentage = (double)faultNodeCount / nodes.size() * 100.0;
        if (faultPercentage >= FAULT_TOLERANCE_PERCENTAGE) {
            NS_LOG_WARN("[ALERT] Cluster " << clusterId << " exceeds " << FAULT_TOLERANCE_PERCENTAGE 
                         << "% faulty nodes! Sending HELP signal...");
            SendHelpSignal();
        }
    }

    void SendHelpSignal() {
        Ptr<Packet> packet = Create<Packet>(100); // Dummy packet
        alertSocket->SendTo(packet, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), 9999));
    }

    void HandleAlert(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            NS_LOG_WARN("[Cluster " << clusterId << "] Received HELP alert! Considering assistance...");
        }
    }
};

// Simulation Function
void Simulate(int numClusters, int nodesPerCluster) {
    NodeContainer clusterHeads;
    clusterHeads.Create(numClusters);
    InternetStackHelper stack;
    stack.Install(clusterHeads);

    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.255.0");

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(YansWifiChannelHelper().Create());

    Ipv4InterfaceContainer interfaces = address.Assign(WifiHelper().Install(wifiPhy, WifiMacHelper(), clusterHeads));

    //Ipv4InterfaceContainer interfaces = address.Assign(WifiHelper().Install(WifiMacHelper(), WifiPhyHelper(), clusterHeads));

    std::vector<ClusterHead> clusters;
    for (int i = 0; i < numClusters; i++) {
        clusters.emplace_back(i, clusterHeads.Get(i));

        for (int j = 0; j < nodesPerCluster; j++) {
            double initialEnergy = (rand() % 20) + 1;
            SensorNode* node = new SensorNode{j, initialEnergy, false};
            clusters[i].nodes.push_back(node);
        }
    }

    for (int round = 1; round <= 5; round++) {
        NS_LOG_INFO("\n--- ROUND " << round << " ---");
        for (auto &cluster : clusters) {
            for (auto &node : cluster.nodes) {
                if (!node->isFaulty) {
                    node->energy -= (rand() % 3) + 1;
                }
            }
            cluster.CheckFaultNodes();
        }
    }
}

// Main Function
int main(int argc, char *argv[]) {
    LogComponentEnable("ClusterHeadAlert", LOG_LEVEL_INFO);
    srand(time(0));
    
    int numClusters = 3;
    int nodesPerCluster = CLUSTER_SIZE;
    
    Simulate(numClusters, nodesPerCluster);
    Simulator::Run();
    Simulator::Destroy();
    
    return 0;
}

