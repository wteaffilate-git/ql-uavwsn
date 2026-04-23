#ifndef __UAVWSN_SENSORNODE_H
#define __UAVWSN_SENSORNODE_H

#include <omnetpp.h>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <string>
#include "MetricsCollector.h"
#include "Location.h"

using namespace omnetpp;

enum NodeStatus { MEMBER_NODE, CLUSTER_HEAD };

struct NeighborInfo {
    int id;
    Location location;
    int dataSize;
    double rssi;
};

class SensorNode : public omnetpp::cSimpleModule
{
  public:
    ~SensorNode();
    // Public getter for UAV data collection
    int getAggregatedDataSize() const { return aggregatedDataSize; }
    std::vector<int> getNeighborCHList() const { return neighborCHList; }  // For UAV breadcrumb collection
    Location getLocation() const { return myLocation; }  // For UAV to read sensor location
    bool isAggregationComplete() const { return aggregationComplete || aggregatedDataSize > 0; }
    double getEnergy() const { return energy; }  // For routing-specific channel quality assessment
    bool isAliveNode() const { return energy > 0.0; }  // For routing validation
    
  protected:
    // Energy parameters (First-Order Radio Model)
    double initialEnergy;
    double energy;
    double eElec;          // Electronic energy (50 nJ/bit)
    double eFreeSpace;     // Free space amplifier (10 pJ/bit/m^2)
    double eMultiPath;     // Multipath amplifier (0.0013 pJ/bit/m^4)
    double eDA;            // Data aggregation energy (5 nJ/bit)
    double idleListeningPower;  // Baseline idle power draw (W)
    double commRadius;
    
    // Clustering Protocol parameters
    double chProbability;
    double controlPacketSize;
    double dataPacketSize;
    Location myLocation;
    int lastEpoch;  // Track epoch per node to enforce LEACH fairness reset
    
    // Clustering timing parameters (configurable)
    double neighborDiscoveryDelay;   // CH beacon delay
    double clusteringPhaseDelay;     // TDMA schedule delay
    double joinAttemptDelay;         // Member join check delay
    double unclusteredTimeout;       // Unclustered node timeout
    double roundDuration;            // Total round duration
    int maxBufferSize;               // Maximum aggregation buffer bits
    
      // Duty-cycle parameters
      double chDutyCycle;  // CH nodes: 30% active
      double memberDutyCycle;  // Member nodes: 2% active
      double isolatedDutyCycle;  // Isolated nodes: 0.1% active
      bool isInSleepMode;  // Track if node is sleeping (isolated)
      omnetpp::cMessage *wakeUpMsg;  // Wake up signal when CH broadcasts ADV
    
    // Node state
    NodeStatus status;
    int roundNum;
    int chTarget;
    std::vector<int> joinedMembers;
    std::unordered_map<int, NeighborInfo> neighborMap;  // L_nbr for breadcrumbs (inter-CH neighbors)
    std::vector<int> neighborCHList;  // Simplified list of CH IDs for breadcrumb transmission
    std::unordered_set<int> recentCHRounds;  // Track rounds when node was CH (for G set)
    bool isDead;
    int aggregatedDataSize;
    // Join retry state
    bool joinScheduled;  // Flag to avoid duplicate scheduling of join attempts
    std::vector<int> candidateChs;     // Ordered by increasing distance
    std::map<int, double> chDistance;  // Cache distances for ordering
    size_t joinAttemptIndex;
    omnetpp::cMessage *joinAttemptMsg;
    bool receivedTDMA;  // Flag: member successfully received TDMA_SCHEDULE (prevent double-sending)
    bool tdmaFinalized;  // Flag: TDMA schedule built for this round
    bool tdmaRebuildScheduled;  // Flag: pending TDMA rebuild after late joins
    bool isCoordinatorNode;  // Tracks whether this node is driving metrics this round

    // Member packet tracking for aggregation completeness
    int expectedMemberPackets;
    std::unordered_set<int> receivedMemberSet;
    simtime_t tdmaEndTime;
    simtime_t aggregationDeadline;
    bool aggregationComplete;
    simtime_t roundStartTime;
    simtime_t nextRoundStartTime;
    
    // TDMA scheduling
    std::map<int, simtime_t> tdmaSchedule;
    int tdmaSlotDuration;
    
    // Buffer management (from code-analysis.txt Fix: Aggregation Timing)
    static const int MAX_BUFFER_SIZE = 50000;  // Maximum bits to buffer in CH (prevent overflow)
    
    // Messages
    omnetpp::cMessage *startRoundMsg;
    omnetpp::cMessage *sleepMsg;
    omnetpp::cMessage *tdmaSlotMsg;
    omnetpp::cMessage *dataCollectionMsg;
    
    // Statistics
    simtime_t firstNodeDeathTime;
    simtime_t lastNodeDeathTime;
    int controlPacketsSent;
    int dataPacketsSent;
    double totalEnergyConsumed;
    
    // Packet tracking for metrics
    std::map<int, simtime_t> generatedPackets;  // packetId -> generation time
    std::vector<int> aggregatedPacketIDs;  // Packet IDs aggregated by CH
    std::map<int, int> packetGenRounds;  // packetId -> generation round (for expiration)
    MetricsCollector* metrics;
    
    // RL Components for UDQL
    // Node Agent (Clustering Refinement)
    struct NodeState {
        int energyLevel;  // 0: LOW, 1: MEDIUM, 2: HIGH
        int distanceLevel; // 0: NEAR, 1: FAR
        int neighborDensity; // 0: SPARSE, 1: MODERATE, 2: DENSE
        int uavProximity; // 0: LOW, 1: HIGH
    };
    
    enum NodeAction { JOIN, SELF_CH, WAIT_UAV };
    
    std::map<std::tuple<int,int,int,int>, std::map<NodeAction, double>> qTableNode; // State -> Action -> Q-value
    
    NodeState lastNodeState;
    NodeAction lastNodeAction;
    double lastNodeReward;
    
    // CH Agent (Routing Decisions)
    struct CHState {
        int energyLevel;  // 0: LOW, 1: MEDIUM, 2: HIGH
        int queueLevel;   // 0: LOW, 1: MEDIUM, 2: HIGH
        int ageLevel;     // 0: FRESH, 1: AGING, 2: OLD
        int uavProximity; // 0: FAR, 1: NEAR
        int neighborDist; // 0: CLOSE, 1: FAR
    };
    
    enum CHAction { TRANSMIT_TO_UAV, BUFFER, FORWARD_TO_CH };
    
    std::map<std::tuple<int,int,int,int,int>, std::map<CHAction, double>> qTableCH; // State -> Action -> Q-value
    
    CHState lastCHState;
    CHAction lastCHAction;
    double lastCHReward;
    
    // RL Parameters (from parameters-init.txt)
    double alpha_node = 0.15;
    double gamma_node = 0.6;
    double epsilon_node = 0.2;
    double epsilon_min = 0.05;
    double epsilon_decay_node = 0.002;
    
    double alpha_ch = 0.1;
    double gamma_ch = 0.85;
    double epsilon_ch = 0.25;
    double epsilon_decay_ch = 0.0015;
    
    // Helper functions for RL
    NodeState observeNodeState();
    CHState observeCHState();
    NodeAction selectNodeAction(NodeState state);
    CHAction selectCHAction(CHState state);
    double computeNodeReward(NodeAction action);
    double computeCHReward(CHAction action);
    void updateNodeQTable(NodeState state, NodeAction action, double reward, NodeState nextState);
    void updateCHQTable(CHState state, CHAction action, double reward, CHState nextState);
    
    // RL execution
    void performNodeRLRefinement();
    void performCHRLRouting();
    std::vector<simtime_t> recentUAVBeacons;  // Timestamps of recent UAV beacons
    double uavProximityHistory;  // Moving average of UAV proximity (0-1)
    
    // Helper function to discard old packets
    void discardExpiredPackets();
    
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    virtual void startNewRound();
    virtual void electClusterHead();
    virtual double calculateThreshold();
    virtual void processAdvMsg(int senderId, Location senderLoc, double rssi);
    virtual void processJoinReq(int senderId, double senderX, double senderY);
    virtual void createTDMASchedule();
    virtual void collectDataFromMembers();
    virtual void aggregateData();
    virtual void sleepTillNextRound();
    virtual void interCHNeighborDiscovery();  // Algorithm 1 Phase 3: Breadcrumb mechanism
    virtual void processCHBeacon(int chId, Location chLoc);  // Process beacons from other CHs
    virtual void finish() override;
    
    // Energy consumption functions
    double consumeTxEnergy(double bits, double distance);  // TX energy (First-Order Radio Model)
    double consumeRxEnergy(double bits);                  // RX energy
    double consumeAggregationEnergy(double bits);         // Data aggregation energy
    double consumeIdleEnergy(double energyAmount);        // Idle listening/base load energy
    double calculateRSSI(double distance);
    bool checkAlive();
    
    // Message sending helpers
    void sendAdvMsg();
    void sendJoinReq(int chId);
    void sendDataPacket(int chId);
    void sendBeacon();
    void sendTDMASchedule();
    void sendAggregatedData();
};
#endif