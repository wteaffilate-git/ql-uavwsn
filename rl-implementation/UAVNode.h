#ifndef __UAVWSN_UAVNODE_H
#define __UAVWSN_UAVNODE_H
#include <omnetpp.h>
#include <unordered_set>
#include <vector>
#include <string>
#include "MetricsCollector.h"
#include "Location.h"

using namespace omnetpp;

class UAVNode : public omnetpp::cSimpleModule
{
  protected:
    // Position and mobility
    Location currentPos;
    Location networkCenter;     // Center of sensor field (250, 250)
    Location baseStationPos;
    
    // Parameters
    double uavHeight;
    double searchSpeed;
    double commRadius;
    double dataRate;
    double handshakeDelay;
    double roundDuration;                 // Duration of a sensor network round (align UAV cycles)
    double collectionWindow;              // Time budget the UAV spends in-network per round
    double discoveryInterval;             // Beacon transmission interval
    
    // Beacon and contact management
    struct DiscoveredCH {
        int id;
        Location location;
        int dataSize;
        double contactTime;    // Adaptive T_c calculated for this CH
        double priority;       // Priority score: (dataSize/1000) * contactTime
    };
    
    // Data buffering for BaseStation forwarding
    struct BufferedData {
        std::vector<int> packetIDs;
        int totalDataSize;
        int sourceNodeId;
    };
    std::vector<BufferedData> collectedData;  // Buffer all collected data for BS forwarding
    
    std::vector<DiscoveredCH> discoveredCHs;  // CHs found during beacon phase
    std::unordered_set<int> visitedSet;      // CHs already contacted in this round
    virtual ~UAVNode();
    int currentCHIndex;                        // Index in discoveredCHs being contacted
    
    // Adaptive T_c calculation methods
    void calculateAdaptivePriorities();        // Calculate T_c and priority for all CHs
    void sortCHsByPriority();                  // Sort CHs by priority (descending)
    
    // Random Waypoint Model parameters
    Location currentWaypoint;                  // Current target waypoint
    double areaMinX, areaMaxX;                 // Work area boundaries
    double areaMinY, areaMaxY;
    
    // Contact window timing
    simtime_t contactWindowStart;
    simtime_t contactWindowEnd;
    simtime_t roundEndTime;                     // When to leave network this round
    simtime_t roundStartTime;                   // Start of current round (aligned to sensors)
    simtime_t nextRoundStartTime;               // Scheduled start of next round
    bool inContactWindow;
    
    // State machine
    enum UAVMode { 
        WAIT_ROUND_START,    // Waiting for round to start
        ENTER_NETWORK,       // Flying to first waypoint
        RANDOM_WAYPOINT,     // Following random waypoint mobility
        IN_CONTACT,          // Hovering during CH contact
        LEAVE_NETWORK,       // Flying back to base
        RETURN_HOME          // Returning to base station
    };
    UAVMode currentMode;
    
    // Messages
    omnetpp::cMessage *modeTimer;
    omnetpp::cMessage *beaconMsg;
    omnetpp::cMessage *contactTimer;
    
    // Statistics
    int totalCHsVisited;
    int totalDataCollected;
    double totalFlightDistance;
    simtime_t simulationStartTime;
    int currentRound;
    
    // Metrics
    MetricsCollector* metrics;
    double intervalDataCollected;
    simtime_t lastThroughputUpdate;
    
    // RL Components for UAV Agent
    struct UAVState {
        int chQueue;     // Average queue size
        int chAge;       // Average packet age
        int chEnergy;    // Average energy level
        int chDistance;  // Average distance
    };
    
    enum UAVAction { SELECT_CH, SKIP_CH };
    
    std::map<std::tuple<int,int,int,int>, std::map<UAVAction, double>> qTableUAV;
    
    UAVState lastUAVState;
    UAVAction lastUAVAction;
    double lastUAVReward;
    
    // RL Parameters (from parameters-init.txt)
    double alpha_uav = 0.2;
    double gamma_uav = 0.9;
    double epsilon_uav = 0.3;
    double epsilon_min = 0.05;
    double epsilon_decay_uav = 0.001;
    
    // Helper functions for RL
    UAVState observeUAVState();
    UAVAction selectUAVAction(UAVState state);
    double computeUAVReward(UAVAction action);
    void updateUAVQTable(UAVState state, UAVAction action, double reward, UAVState nextState);
    
    // RL execution
    void performUAVRLScheduling();
    
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    virtual void finish() override;
    
    // Main sequence
    void startRound();                              // Begin network visit
    void enterNetwork();                            // Fly to first waypoint
    void generateRandomWaypoint();                  // Generate random waypoint in work area
    void executeRandomWaypoint();                   // Generate waypoint and schedule flight (Pure RWP)
    void arriveAtWaypoint();                        // Handle arrival at waypoint and generate next
    void sendBeacon();                              // Broadcast discovery beacon
    void handleCHResponse(int chId, Location chLoc, int dataSize);
    void contactCH(const DiscoveredCH& ch);         // Contact CH and collect data
    void contactNearestCH();                        // Select and contact nearest unvisited CH (greedy)
    void contactAllCHsInRange();                    // Contact ALL CHs in range at current waypoint (pure RWP)
    void leaveNetwork();                            // Exit and return home
    
    // Link Assessment
    double calculateContactTime(const Location& chLoc);  // Time to contact CH
    double calculateTransferTime(int dataSize);          // Time to transfer data
    bool assessLinkQuality(const Location& chLoc, int dataSize, bool& needsHover);
    
    // Routing-Specific Communication Model (First-Order Radio)
    double calculateSNR(double distance, double nodeEnergy);  // SNR based on distance and energy
    double calculatePacketErrorRate(double snr);              // PER from SNR (first-order radio)
    bool evaluateDistanceBasedFailure(double distance);       // Distance-based link failure
    bool evaluateEnergyBasedFailure(double nodeEnergy);       // Energy-based transmission failure
    double calculateActualContactTime_Tc(const Location& chLoc);  // Actual T_c for this specific CH
    bool evaluateChannelQuality(double distance, double nodeEnergy, int dataSize);  // Overall channel assessment
    
    // Utility
    void flyTo(const Location& target);
};

#endif
