#include "ns3/netanim-module.h"
#include "ns3/node-container.h"
#include "ns3/mobility-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/animation-interface.h"
#include "ns3/flow-monitor-module.h"

#include "hleach.h"
#include "const.h" 

#include <vector>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <iterator>
#include <fstream>
#include <delaunator.hpp>

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

int NUM_NODES = 10;    // number of nodes in the network    
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

double Rs = 25;

int BASE_STATION_X_DEFAULT = 300;

int BASE_STATION_Y_DEFAULT = 300;  

int DEAD_NODE = -2;

int MESSAGE_LENGTH = 8;

int TRIALS = 1;

//utilit functions for coverage hole detection

double computeCircumradius(const pair<double, double>& A, 
                           const pair<double, double>& B, 
                           const pair<double, double>& C) {
    double a = hypot(B.first - C.first, B.second - C.second);
    double b = hypot(A.first - C.first, A.second - C.second);
    double c = hypot(A.first - B.first, A.second - B.second);
    double s = (a + b + c) / 2.0;
    double area = sqrt(s * (s - a) * (s - b) * (s - c));
    return (a * b * c) / (4.0 * area);
}

bool isObtuseTriangle(double a, double b, double c) {
    return (a * a + b * b < c * c) || (b * b + c * c < a * a) || (c * c + a * a < b * b);
}

double distance(double x1, double y1, double x2, double y2) {
    return hypot(x1 - x2, y1 - y2);
}

bool isFullyCovered(double x1, double y1, double x2, double y2, double x3, double y3, double rs) {
    double side[3];
    side[0] = distance(x1, y1, x2, y2); // Side between (x1,y1) and (x2,y2)
    side[1] = distance(x3, y3, x2, y2); // Side between (x3,y3) and (x2,y2)
    side[2] = distance(x1, y1, x3, y3); // Side between (x1,y1) and (x3,y3)

    // Find the index of the longest side
    int maxid = 0;
    if (side[1] > side[maxid]) maxid = 1;
    if (side[2] > side[maxid]) maxid = 2;

    // Calculate angles opposite the two shorter sides using the Law of Cosines
    double angle[2];
    angle[0] = acos((side[maxid] * side[maxid] + side[(maxid + 1) % 3] * side[(maxid + 1) % 3] - side[(maxid + 2) % 3] * side[(maxid + 2) % 3]) / (2 * side[maxid] * side[(maxid + 1) % 3]));
    angle[1] = acos((side[maxid] * side[maxid] + side[(maxid + 2) % 3] * side[(maxid + 2) % 3] - side[(maxid + 1) % 3] * side[(maxid + 1) % 3]) / (2 * side[maxid] * side[(maxid + 2) % 3]));

    // Calculate perpendicular bisector lengths
    double pbl[2];
    pbl[0] = tan(angle[0]) * (side[(maxid + 1) % 3] / 2);
    pbl[1] = tan(angle[1]) * (side[(maxid + 2) % 3] / 2);

    // Calculate lengths from midpoint to circumcenter
    double length[2];
    length[0] = sqrt(pbl[0] * pbl[0] + (side[(maxid + 1) % 3] / 2) * (side[(maxid + 1) % 3] / 2));
    length[1] = sqrt(pbl[1] * pbl[1] + (side[(maxid + 2) % 3] / 2) * (side[(maxid + 2) % 3] / 2));

    // Check if either length exceeds the radius rs
    if (length[0] > rs || length[1] > rs) {
        return true; // Triangle is NOT fully covered
    } else {
        return false; // Triangle is fully covered
    }
}



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
                    head->updateFaultNodeCount(this);
                }
                cout << "Node " << id << " declared as FAULTY in Cluster " << clusterId << endl;
            }
        }
    }
};

struct sensor BASE_STATION;  

struct clusterHead : public sensor {

    vector<sensor*> members;
    vector<sensor*> faulty;
    vector<vector<double>>hole_table;
    int faultNodeCount;
    double totalNodes;
    double faultPercentage;
    vector<double> coords;

    clusterHead() {}

    
    void updateFaultNodeCount(sensor* faultNode) {
        faulty.push_back(faultNode);
        faultNodeCount++;  
        faultPercentage = (faultNodeCount / totalNodes) * 100.0;
        if(faultNodeCount > 2) cout<<"Hole: " << checkForHole();
    }

    bool checkForHole() {
        

        if(!coords){
            for(auto &node : members){
                coords.push_back(node->xLoc);
                coords.push_back(node->yLoc);
            }
        }
        
        delaunator::Delaunator d(coords);
        vector<vector<double>> triangles;
        for(std::size_t i = 0; i < d.triangles.size(); i+=3) {
            vector<double>triangle;
            triangle.push_back(d.coords[2 * d.triangles[i]]);
            triangle.push_back(d.coords[2 * d.triangles[i] + 1]);
            triangle.push_back(d.coords[2 * d.triangles[i + 1]]);
            triangle.push_back(d.coords[2 * d.triangles[i + 1] + 1]);
            triangle.push_back(d.coords[2 * d.triangles[i + 2]]);
            triangle.push_back(d.coords[2 * d.triangles[i + 2] + 1]);
            triangles.push_back(triangle);
        }
        for(auto triangle: triangles){
            pair <double,double> A,B,C;
            double a, b c;
            double Rc;
            A = {triangle[0],triangle[1]};
            B = {triangle[2],triangle[3]};
            C = {triangle[4],triangle[5]};
            Rc = computeCircumradius(A, B, C);
            if(Rc > Rs){
                a = distance(B[0], B[1], C[0], C[1]);
                b = distance(A[0], A[1], C[0], C[1]);
                C = distance(A[0], A[1], B[0], B[1]);

                if(isObtuseTriangle(a,b,c)){
                    if(!isFullyCovered(A[0],A[1],B[0],B[1],C[0],C[1],Rs)){
                        hole_table.push_back(A[0],A[1],B[0],B[1],C[0],C[1]);
                    }
                }
                else{
                    hole_table.push_back(A[0],A[1],B[0],B[1],C[0],C[1]);
                }
            }

        
        }

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