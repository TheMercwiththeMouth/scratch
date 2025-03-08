#include <iostream>
#include <vector>
#include <stdlib.h>
#include <math.h>
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

    int round;         // the last round that the sensor   
            // served as a cluster head  
    int head;  // stores the index of the cluster head   
            // for the sensor to transmit to, set to -1 if the   
            // sensor is a cluster head  
    int cluster_members;   // stores the total number of nodes in   
            // the cluster, applicable only for   
            // cluster head nodes  
    int head_count;        // this contains the count of the   
                        // number of times a sensor has been   
                        // the head of a cluster, can be   
                        // removed for optimization later  
    double distance_BASE;
    double distance_current_head;

    int V_round[TOTAL_ROUNDS];
    double V_bPower[TOTAL_ROUNDS];
    int V_head[TOTAL_ROUNDS];
    int V_cluster_members[TOTAL_ROUNDS];
    double V_distanceToHead[TOTAL_ROUNDS];

    void updateFaultStatus() {
        if (bCurrent <= MIN_ENERGY_THRESHOLD) {
            if (!isFaulty) {
                isFaulty = true;
                cout << "Node " << id << " declared as FAULTY in Cluster " << clusterId << endl;
            }
        }
    }
};

struct clusterHead {
    int clusterId;
    vector<sensor*> members;
    int faultNodeCount;

    clusterHead(int id) : clusterId(id), faultNodeCount(0) {}

    void updateFaultNodeCount() {
        faultNodeCount = 0;  // Reset count before checking

        for (auto &node : members) {
            if(node->isFaulty) {
                faultNodeCount++;
            }
        }
    }

    void faultAreaDetection() {
        double totalNodes = members.size();
        double faultPercentage = (faultNodeCount / totalNodes) * 100.0;

        if (faultPercentage >= MIN_FAULT_TOLERANCE_PERCENTAGE) {
            cout << "[ALERT] Cluster " << clusterId << " exceeds " << MIN_FAULT_TOLERANCE_PERCENTAGE 
                 << "% faulty nodes! Initiating recovery...\n";
            //initiateRecovery();
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

class MobileWSN {
    public:
    struct sensor *network_LEACH;
    int closest = 0;

    int round = 0;                  // current round  
	int failed_transmit = 0;        // round where a failed transmission occurred  
	  
	int testing = 0;            // testing variable, TO BE REMOVED  
	int bits_transmitted = 0;       // count of bits transmitted  
	int power = FALSE;  
	int temp_cluster_members = 0;  
	  
	double average_energy = 0.0;  
	double distance_X_old = 0.0;  
	double distance_Y_old = 0.0;  
	double distance_old = 0.0;  
	double distance_X_new = 0.0;  
	double distance_Y_new = 0.0;  
	double distance_new = 0.0;  
	int recent_round = static_cast<int>(1.0 / CLUSTER_PERCENT);  
	double threshold = CLUSTER_PERCENT/(1-(CLUSTER_PERCENT*(round % recent_round)));
	double random_number;  
	int cluster_head_count = 0;  
	double percent_head = 0.0; 
	double mid_value = 0.0;
	bool flag = 1 ;
	int NUM_DEAD_NODE = 0;

	double AVE_ENERGY = 0;

	vector<double> dir;

    network_LEACH = new struct sensor[NUM_NODES];

    AVE_ENERGY = averageEnergy(network_LEACH);
	(*new_network_stats).LEACH_threshold = threshold;

    MobileWSN(const struct sensor network[]) {
        for(int i = 0; i < NUM_NODES; i++) {
            network_LEACH[i].bPower = network[i].bPower;      
            network_LEACH[i].xLoc = network[i].xLoc;  
            network_LEACH[i].cluster_members = 0;  
            network_LEACH[i].yLoc = network[i].yLoc;   
            network_LEACH[i].bCurrent = network[i].bCurrent;     

            //statistic
            network_LEACH[i].V_round[round] = 0;
            network_LEACH[i].V_bPower[round] = network[i].bPower;
            network_LEACH[i].V_cluster_members[round] = 0;
            network_LEACH[i].V_head[round] = -3;
            network_LEACH[i].V_distanceToHead[round] = 0;
        }
    }

    void clusterHeadSelection() {
        cluster_head_count = 0;
        NUM_DEAD_NODE = 0;  
        // advertisement phase  
        // we determine which nodes will be cluster heads
        //   
        for(i = 0; i < NUM_NODES; i++){  
                if((network_LEACH[i].round + recent_round < round) || (network_LEACH[i].round == 0)){  
                    if(network_LEACH[i].head != DEAD_NODE){  
                    random_number = .00001*(rand() % 100000);          
                    //cout << "random_number " << random_number << endl;
                        if(random_number <= threshold && network_LEACH[i].bCurrent > 0.03){  
                            // the random number selected is less   
                            // than the threshold so the node becomes   
                            // a cluster head for the round  
                            network_LEACH[i].head_count++;
                            // update the round variable   
                            // so we know that this sensor was   
                            // last a cluster head at round i  
                            network_LEACH[i].round = round;  
                            network_LEACH[i].head = -1;

                            // store the index of the node in the   
                            // cluster_heads array  
                            // increment the cluster_head_count  
                            cluster_head_count++; 
                        }  
                    }  
                }

        } 
            
        (*new_network_stats).LEACH_NEW_num_cluster_head[round] = cluster_head_count;
        (*new_network_stats).LEACH_NEW_percent_head[round] = ((double)cluster_head_count/(double)NUM_NODES);
        // now the cluster heads must transmit the fact that they   
        // are cluster heads to the network, this will be a constant   
        // transmit energy, during this period the other nodes must   
        // keep their receivers on - which has an energy cost, again   
        // this is constant  
        for(i = 0; i  < NUM_NODES; i++){  
            if(network_LEACH[i].head == -1){  
                network_LEACH[i].bCurrent -=   
                    computeEnergyTransmit(LEACH_AD_DISTANCE,  
                                LEACH_AD_MESSAGE);  
            }  
            else{  
                network_LEACH[i].bCurrent -=   
                    computeEnergyReceive(LEACH_AD_MESSAGE);  
            }  
        }  
    }

    void clusterSetUp() {
        for(i = 0; i  < NUM_NODES; i++){  
            closest = -1;
            if (network_LEACH[i].head < 0 || network_LEACH[i].head >= NUM_NODES) continue;    
            if((network_LEACH[i].head != -1) &&   
                network_LEACH[i].head != DEAD_NODE){  
                    // if the node's round is not equal to the    
                    // current round, the node is not a cluster  
                    // head and we must find a cluster head for  
                    // the node to transmit to  
            for(k = 0; k < NUM_NODES; k++){  
                        if(network_LEACH[k].head == -1 && closest != -1){  
                            distance_X_old = network_LEACH[i].xLoc - network_LEACH[closest].xLoc;  
                            distance_Y_old = network_LEACH[i].yLoc - network_LEACH[closest].yLoc;  
                            distance_old = sqrt(pow(distance_X_old, 2) + pow(distance_Y_old, 2));  
                            distance_X_new = network_LEACH[i].xLoc - network_LEACH[k].xLoc;  
                            distance_Y_new = network_LEACH[i].yLoc - network_LEACH[k].yLoc;  
                            distance_new = sqrt(pow(distance_X_new, 2) + pow(distance_Y_new, 2));  
                            if(distance_new < distance_old)  
                                closest = k;  
                        }  
                        else if(network_LEACH[k].head == -1 && closest == -1){  
                            closest = k;  
                        }  
            }  
            
            if (closest == -1) continue;  
            network_LEACH[i].head = closest;  
            network_LEACH[closest].cluster_members++;  
        }
        
    }
        for(i = 0; i < NUM_NODES; i++){ 
            network_LEACH[i].V_head[round] = network_LEACH[i].head;
            network_LEACH[i].V_cluster_members[round] = network_LEACH[i].cluster_members;  
        }
    }

    void scheduleCreation() {
        for(i = 0; i < NUM_NODES; i++){  
            if(network_LEACH[i].head == -1){  
                // if the node is going to be a cluster head, it transmits   
                // the schedule to the other nodes  
                network_LEACH[i].bCurrent -=   
                    computeEnergyTransmit(SCHEDULE_DISTANCE, SCHEDULE_MESSAGE);  
            }  
            else  
                network_LEACH[i].bCurrent -=   
                    computeEnergyReceive(SCHEDULE_MESSAGE);  
        }  
    }

    void dataTransmission() {
        for(i = 0; i < NUM_NODES; i++){ 

            network_LEACH[i].lPeriods++;

            if(network_LEACH[i].head >=0 && network_LEACH[i].head < NUM_NODES){ 

                    distance_X_new = network_LEACH[i].xLoc - network_LEACH[network_LEACH[i].head].xLoc;  
                    distance_Y_new = network_LEACH[i].yLoc - network_LEACH[network_LEACH[i].head].yLoc;  
                    distance_new = sqrt((pow(distance_X_new, 2) + pow(distance_Y_new, 2)));

                    network_LEACH[i].V_distanceToHead[round] = (distance_new);

                    if(network_LEACH[i].head != DEAD_NODE){
                        network_LEACH[i].bCurrent -= computeEnergyTransmit(distance_new, MESSAGE_LENGTH);  
                        network_LEACH[network_LEACH[i].head].bCurrent -= computeEnergyReceive(MESSAGE_LENGTH);

                        if(network_LEACH[i].bCurrent < 0.0 && network_LEACH[i].head != -1){  
                                network_LEACH[i].head = DEAD_NODE; 
                            } 
                    } 
            }
            else{
                network_LEACH[i].V_distanceToHead[round] = 0;
            }  
        }  
      
      
        for(i = 0; i < NUM_NODES; i++){  
            if (network_LEACH[i].head < 0 || network_LEACH[i].head >= NUM_NODES) continue;
            if(network_LEACH[i].head == -1){  

                double energy_enough = network_LEACH[i].bCurrent - computeEnergyTransmit(network_LEACH[i].distance_BASE, (MESSAGE_LENGTH * (network_LEACH[i].cluster_members+1)));
            
            //network_LEACH[i].bCurrent -= computeEnergyTransmit(distance_new,(MESSAGE_LENGTH * (network_LEACH[i].cluster_members+1)));  
            //if(network_LEACH[i].bCurrent > 0.0){  
                  if(energy_enough > 0.0){

                    network_LEACH[i].bCurrent -= computeEnergyTransmit(network_LEACH[i].distance_BASE,(MESSAGE_LENGTH * (network_LEACH[i].cluster_members+1)));
                    bits_transmitted += (MESSAGE_LENGTH * (network_LEACH[i].cluster_members+1));  
                 }  
                else
                {  
                    failed_transmit++;    
                }  
            }  
        }   
        // round has completed, increment the round count
    }

    void updateRoundStats() {
        for(i = 0; i < NUM_NODES; i++){  
            network_LEACH[i].V_bPower[round] = network_LEACH[i].bCurrent;
            network_LEACH[i].cluster_members = 0;  
           if(network_LEACH[i].bCurrent > 0.0)
           {
                   network_LEACH[i].head = -3;  
           }
           else
           {
                   network_LEACH[i].head = DEAD_NODE;
                   NUM_DEAD_NODE++;		       		
           }
          network_LEACH[i].V_round[round] = round;
        }

       
        (*new_network_stats).LEACH_NEW_rounds[round] = round;


        (*new_network_stats).LEACH_NEW_num_dead_node[round] = NUM_DEAD_NODE; 
        

        cluster_head_count = 0;  

        AVE_ENERGY = averageEnergy(network_LEACH);

        (*new_network_stats).LEACH_NEW_network_average_energy[round] = AVE_ENERGY;
        (*new_network_stats).LEACH_NEW_network_total_energy[round] = totalEnergy(network_LEACH);

        if(round % 5000 == 1 )
        {
            cout  << "network_pwr: " << AVE_ENERGY << endl;	
        }

        round += 1;
    }

};

double computeEnergyTransmit(float distance, int messageLength);    
  
double computeEnergyReceive(int messageLength);  
  
void initializeNetwork(struct sensor network[]);  
  
double averageEnergy(struct sensor network[]);  
  
double totalEnergy(struct sensor network[]);

int runLeachSimulation_New(const struct sensor network[], struct network_stats * new_network_stats); 

void initializeNetworkStats(struct network_stats network_stats_data[], int TRIALS);

int main(int argc, char * argv[])  
{  
    cout << "Running Main Function:" << endl;
	struct sensor *network = new struct sensor[NUM_NODES];
    cout << "Creating the Sensor Network " << endl;

    TRIALS = 1;

    struct network_stats * network_stats_data = new struct network_stats[TRIALS];
      
    int i = 0;  

    int rounds_LEACH_NEW = 0;

    BASE_STATION.xLoc = BASE_STATION_X_DEFAULT;  
    BASE_STATION.yLoc = BASE_STATION_Y_DEFAULT;

      
    initializeNetwork(network);  
    cout << "Intializing the Network: " << endl;    
    
	initializeNetworkStats(network_stats_data,TRIALS);
    cout << "Initializing Network Stats: " << endl;

  
    for(i = 0; i < TRIALS; i++){  
        rounds_LEACH_NEW = runLeachSimulation_New(network,&network_stats_data[i]);
        cout << "The LEACH Simulation has been COMPLETED." << endl;   
		 
        printf("\n");
        printf("The LEACH simulation was able to remain viable for %d rounds\n", rounds_LEACH_NEW);
        network_stats_data[i].BASE_STATION_X = BASE_STATION_X_DEFAULT;
        network_stats_data[i].BASE_STATION_Y = BASE_STATION_Y_DEFAULT;
        network_stats_data[i].NETWORK_X = NETWORK_X;
        network_stats_data[i].NETWORK_Y = NETWORK_Y;
        network_stats_data[i].NUM_NODES = NUM_NODES;

        network_stats_data[i].LEACH_ROUNDS = rounds_LEACH;
        network_stats_data[i].LEACH_NEW_ROUNDS = rounds_LEACH_NEW;
        network_stats_data[i].DIRECT_ROUNDS = rounds_DIRECT;
        network_stats_data[i].CLUSTER_PERCENT = CLUSTER_PERCENT;

        // if(i < TRIALS - 1){
        // 	initializeNetwork(network);  
        // }
        
    }
	ns3::Simulator::Run ();
	ns3::Simulator::Destroy ();

    return 0;    
}


void initializeNetwork(struct sensor network[]) {  
 
    int i = 0;  
    srand((unsigned int) time(0));
    double distance_X_new =0.0;
    double distance_Y_new =0.0;
    double distance_new =0.0;
            
    for(i = 0; i < NUM_NODES; i++) {  
        network[i].xLoc = rand() % NETWORK_X;  
        network[i].yLoc = rand() % NETWORK_Y;

        distance_X_new = network[i].xLoc - BASE_STATION.xLoc;  
        distance_Y_new = network[i].yLoc - BASE_STATION.yLoc;  
        distance_new = sqrt(pow(distance_X_new, 2) + pow(distance_Y_new, 2));  

        network[i].distance_BASE = distance_new;

        network[i].lPeriods = 0;  
        network[i].bCurrent = B_POWER;  
        network[i].bPower = B_POWER;  
        network[i].round = 0;  
        network[i].head = -3;  
    }  
}

void initializeNetworkStats(struct network_stats network_stats_data[],int TRIALS){
    
    int i = 0;
            
    for(i = 0; i < TRIALS; i++) {  
        network_stats_data[i].BASE_STATION_X = 0;
        network_stats_data[i].BASE_STATION_Y = 0;
        network_stats_data[i].NETWORK_X = 0;
        network_stats_data[i].NETWORK_Y = 0;
        network_stats_data[i].NUM_NODES = 0;

        network_stats_data[i].network_comparison = 0;
        network_stats_data[i].LEACH_ROUNDS = 0;
        network_stats_data[i].DIRECT_ROUNDS = 0;
        network_stats_data[i].Improvement = 0;
        network_stats_data[i].LEACH_threshold = 0;  

    }
} 
   
double computeEnergyTransmit(float distance, int messageLength){  
    float E_elec = 50 * pow(10,-9);  
    float epsilon_amp = 100 * pow(10,-12);  
    double EnergyUse = 0.00;  
            
    EnergyUse = (messageLength * E_elec) + (messageLength * epsilon_amp * pow(distance,2));   
        
    return EnergyUse;  
    
}           
              
double computeEnergyReceive(int messageLength) {  
 
    return (messageLength * (50 * pow(10,-9)));  
}           
    
double averageEnergy(struct sensor network[]) {  

    float average = 0.0;  
    float starting_power = 0.00;  
    float current_power = 0.00;  
    int i = 0;  
    
    for(i = 0; i < NUM_NODES; i++) {  
        starting_power += network[i].bPower;  
        current_power += network[i].bCurrent;  
    }  
    
    return current_power/starting_power;  

}          

double totalEnergy(struct sensor network[]) {  

    double total = 0.00;  
    int i = 0;  
    
    for(i = 0; i < NUM_NODES; i++) {  
        total += network[i].bCurrent;  
    }  
    
    return total;  
    
}