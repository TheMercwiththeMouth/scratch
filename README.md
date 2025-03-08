# Fina# Mobile WSNs Fault Node Replacement Algorithm - Final Year Project 2025

### Initial Setup
Upload all the files to the scratch folder of the ns-3.43 directory. Run the following commands:
```
./ns3 clean
./ns3 configure
```
### What to Run?
For the project, we are currently using leach_new for the implementation of the LEACH protocol. Navigate to the ns-3.43 directory and run the following command:
```
./ns3 run leach_new
```
### Current Output
```
Running Main Function:
sensor
network_stats
Intializing Network:
Initializing Network Stats:

Running the LEACH Transmission Simulation
network_pwr: 0.999987
network_pwr: 0.96607
network_pwr: 0.932116
network_pwr: 0.898193
network_pwr: 0.864247
network_pwr: 0.830285
network_pwr: 0.796347
network_pwr: 0.762371
LEACH: 40000
CHECK3:
The average energy of the network remained above 10% for 40000 tranmission periods
runLeachSimulation

Running the LEACH Transmission Simulation
network_pwr: 0.999989
network_pwr: 0.980839
network_pwr: 0.961673
network_pwr: 0.942526
network_pwr: 0.923365
network_pwr: 0.904198
network_pwr: 0.885032
network_pwr: 0.865882
LEACH_NEW: 40000
CHECK3:
The average energy of the network remained above 10% for 40000 tranmission periods
runLeachSimulation_NEW
runDirectSimulation

The LEACH simulation was able to remain viable for 40000 rounds
The direct transmission simulation was able to remain viable for 1 rounds
This is an improvement of 99.997500%
```
