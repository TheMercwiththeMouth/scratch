#include <iostream>
#include <vector>
#include <stdlib.h>
#include <nath.h>
#include <string.h>
#include <time.h>
#include <iterator>
#include <fstream>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wparentheses"

using namespace std;
using namespace ns3;
using std::vector;

#define MIN_ENERGY_THRESHOLD 5.0  // Minimum energy level
#define MIN_FAULT_TOLERANCE_PERCENTAGE 10  // Fault tolerance percentage (10% of total nodes)

int NUM_NODES = 50;    // number of nodes in the network    
                       // default is 50  
int NETWORK_X = 100;   // X-size of network  
                       // default is 100  
int NETWORK_Y = 100;   // Y-size of network  
                // default is 100  
double B_POWER = 0.5;   // initial battery power of sensors  
                // default is 0.75  
                  
                // the percentage of the nodes in the   
                // network that would ideally be cluster   
                // heads during any one round of the   
                // LEACH simulation, default is 0.05  
double CLUSTER_PERCENT = 0.3;  
      
                // the total rounds that the simulation  
                // should run for - the network lifetime  
                // default is 2000  
const int TOTAL_ROUNDS = 40000;  
  
                // the distance that the advertisement  
                // phase should broadcast to, in order   
                // to alert other nodes that there is a   
                // cluster head, default is 25.0  
double LEACH_AD_DISTANCE = 25;    
                // the message length of the advertisement  
                // that there is a cluster head  
                // default is 16  
int LEACH_AD_MESSAGE = 16;    
                // the distance for the cluster head to broadcast   
                // the schedule for transmission to the other nodes   
                // in the cluster, default is 25.0  
double SCHEDULE_DISTANCE = 25;    
                // the message length of the schedule which is   
                // sent to the nodes in the cluster during the   
                // scheduling phase of the LEACH simulation,   
                // default is 16.

int SCHEDULE_MESSAGE = 16;  

int BASE_STATION_X_DEFAULT = 300;

int BASE_STATION_Y_DEFAULT = 300;  

int DEAD_NODE = -2;

int MESSAGE_LENGTH = 8;

int TRIALS = 1;

string CHANGE = "Mod_CLUSTER_PERCENT0_3";
struct clusterHead;
  
struct sensor {
    int id;           // unique identifier for the sensor
    int clusterId;    // cluster identifier for the sensor

    short xLoc;        // X-location of sensor  
    short yLoc;        // Y-location of sensor  
    short lPeriods;        // number of periods the sensor  
            // has been in use for  
    double bCurrent;       // current battery power  
    double bPower;     // initial battery power 
    double pAverage;
    bool isFaulty;    // Flag to indicate if the node is faulty 

    // int round;         // the last round that the sensor   
    //         // served as a cluster head  
    
    clusterHead* head;  // stores the cluster head   
            // for the sensor to transmit to

    // int cluster_members;   // stores the total number of nodes in   
    //         // the cluster, applicable only for   
    //         // cluster head nodes  

    // int head_count;        // this contains the count of the   
    //                     // number of times a sensor has been   
    //                     // the head of a cluster, can be   
    //                     // removed for optimization later  

    double distance_BASE;
    double distance_current_head;

    // int V_round[TOTAL_ROUNDS];
    
    double V_bPower[TOTAL_ROUNDS];

    // int V_head[TOTAL_ROUNDS];

    int V_cluster_members[TOTAL_ROUNDS];

    // double V_distanceToHead[TOTAL_ROUNDS];

    sensor () {}

    void updateFaultStatus() {
        if (bCurrent <= MIN_ENERGY_THRESHOLD) {
            if (!isFaulty) {
                isFaulty = true;
                if (head) {
                    head->updateFaultNodeCount();
                }
                cout << "Node " << id << " declared as FAULTY in Cluster " << clusterId << endl;
            }
        }
    }
};

struct clusterHead : public sensor {

    vector<sensor*> members;
    int faultNodeCount;
    double totalNodes;
    double faultPercentage;

    clusterHead() {
        members = {};
        totalNodes = members.size();
        faultNodeCount = 0;
        faultPercentage = 0.0;
    }
    
    void updateFaultNodeCount() {
        
        faultNodeCount++;  
        faultPercentage = (faultNodeCount / totalNodes) * 100.0;
    }
}
  
  
struct sensor BASE_STATION;  

struct network_stats{
    int BASE_STATION_X;
    int BASE_STATION_Y;
    int NETWORK_X;
    int NETWORK_Y;
    int NUM_NODES;

    double network_comparison;
    int LEACH_ROUNDS;
    int LEACH_NEW_ROUNDS;
    int DIRECT_ROUNDS;
    double Improvement;
    double CLUSTER_PERCENT;

    //LEACH
    int LEACH_threshold;

 //LEACH_NEW
    int LEACH_NEW_rounds[TOTAL_ROUNDS];
    double LEACH_NEW_network_average_energy[TOTAL_ROUNDS];
    double LEACH_NEW_network_total_energy[TOTAL_ROUNDS];
    int LEACH_NEW_num_dead_node[TOTAL_ROUNDS];
    int LEACH_NEW_num_cluster_head[TOTAL_ROUNDS];
    double LEACH_NEW_percent_head[TOTAL_ROUNDS];
};