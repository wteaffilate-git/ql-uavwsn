#ifndef __UAVWSN_METRICSCOLLECTOR_H
#define __UAVWSN_METRICSCOLLECTOR_H

#include <omnetpp.h>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>

using namespace omnetpp;

// Centralized metrics collector for the network
class MetricsCollector
{
private:
    static MetricsCollector* instance;
    std::string outputDir;
    
    // CSV file streams
    std::ofstream stabilityFile;
    std::ofstream energyFile;
    std::ofstream pdrFile;
    std::ofstream throughputFile;
    std::ofstream delayFile;
    std::ofstream contactFile;
    std::ofstream overheadFile;
    std::ofstream networkFile;
    std::ofstream clusteringFile;
    std::ofstream uavTrajectoryFile;
    std::ofstream rlRewardFile;
    std::ofstream rlActionFile;
    std::ofstream rlStateVisitFile;
    std::ofstream bufferOverflowFile;
    std::ofstream rlConvergenceFile;
    
    // RL convergence tracking
    int rlConvergenceRound;
    bool rlConverged;
    std::vector<double> recentRewards;
    std::vector<double> recentExplorationRates;
    static const int CONVERGENCE_WINDOW = 50;  // Rounds to check for convergence
    static constexpr double CONVERGENCE_THRESHOLD = 0.01;  // Reward stability threshold
    
    // Per-round RL tracking
    double roundTotalReward;
    int roundRewardCount;
    double roundTotalExplorationRate;
    int roundExplorationCount;
    
    // Metric data structures
    struct PacketInfo {        simtime_t genTime;
        simtime_t recvTime;
        int sourceNode;
        int dataSize;
        int genRound;  // Round in which packet was generated
    };
    
    std::map<int, PacketInfo> packetTracker;  // packet ID -> info
    int nextPacketId;
    
    // Per-round packet tracking for accurate PDR
    std::map<int, int> roundGeneratedMap;  // round -> packets generated
    std::map<int, int> roundReceivedMap;   // round -> packets received
    
    // Round-based metrics
    int currentRound;
    int totalNodes;
    int aliveNodes;
    int fndRound;
    int lndRound;
    bool fndRecorded;
    bool lndRecorded;
    int coordinatorId;
    std::map<int, int> aliveNodesAtRoundStart;  // Track alive count at round start for clustering math
    
    // Clustering metrics per round
    struct ClusterInfo {
        int chId;
        int memberCount;  // Number of member nodes (excluding CH itself)
        int expectedMembers;  // Members the CH scheduled
        int receivedMembers;  // Members that actually delivered
        bool deadlineHit;     // Aggregation cut off by deadline
    };
    std::vector<ClusterInfo> roundClusters;
    std::map<int, std::vector<ClusterInfo>> clustersByRound;  // Buffer clusters by round number
    std::map<int, std::unordered_map<int, int>> memberAssignmentsByRound;  // round -> memberId -> chId
    std::map<int, double> memberDistanceSumByRound;  // round -> sum of member-to-CH distances
    std::map<int, int> memberDistanceCountByRound;   // round -> count of member-to-CH distances
    
    // Per-round tracking
    double roundEnergyConsumption;
    int roundControlPackets;
    int roundDataPackets;
    int roundPacketsGenerated;
    int roundPacketsReceived;
    int roundBitsReceived;  // Track bits received for throughput
    std::vector<double> roundDelays;
    int totalExpiredPackets;  // Packets discarded due to age
    std::map<int,int> roundExpiredMap;
    std::map<int,double> roundRLRewardSum;
    std::map<int,int> roundRLRewardCount;
    std::map<int,int> roundRLActionCount;
    std::map<int,int> roundRLExplorationCount;
    std::unordered_map<std::string,int> rlStateVisitCounts;
    
    // Cumulative metrics
    int totalPacketsGenerated;
    int totalPacketsReceived;
    double totalEnergyConsumed;
    std::vector<double> allDelays;
    double totalThroughput;
    simtime_t simulationStartTime;
    double configuredRoundDuration;  // Seconds per round (from config)
    int fndCandidateRound;           // First observed death (before validation)
    
    // Contact period tracking
    struct ContactEvent {
        simtime_t startTime;
        simtime_t duration;
        int chId;
        bool successful;
    };
    std::vector<ContactEvent> contactEvents;
    
    // Energy tracking
    std::vector<double> nodeEnergies;
    
    MetricsCollector();
    
public:
    static MetricsCollector* getInstance();
    
    void initialize(int numNodes, simtime_t startTime);
    void finalize();
    void setRoundDuration(double seconds);
    void setCollectionWindow(double seconds);
    
    std::string getOutputDir() const { return outputDir; }
    
    // Round management
    bool claimCoordinator(int nodeId);
    void startNewRound(int roundNum);
    void endCurrentRound();
    
    // Packet tracking
    int generatePacket(int sourceNode, simtime_t genTime, int dataSize);
    void recordPacketReception(int packetId, simtime_t recvTime);
    void recordExpiredPacket(int packetId, int currentRound, int genRound);
    
    // Energy tracking
    void recordEnergyConsumption(int nodeId, double energyConsumed);
    void updateNodeEnergy(int nodeId, double residualEnergy);
    
    // Control overhead
    void recordControlPacket();
    void recordDataPacket();
    
    // Stability period
    void recordNodeDeath(int nodeId, int roundNum);
    
    // Contact period
    void recordContactStart(int chId, simtime_t startTime);
    void recordContactEnd(int chId, simtime_t endTime, bool successful);
    
    // Throughput
    void recordThroughput(double bitsReceived, simtime_t interval);
    
    // Clustering metrics
    void recordClusterFormation(int chId, int memberCount, int roundNum);
    void recordAggregationResult(int roundNum, int chId, int expectedMembers, int receivedMembers, bool deadlineHit);
    bool registerClusterMember(int roundNum, int memberId, int chId);
    void recordMemberDistance(int roundNum, double distance);
    void writeClusteringData(int roundNum);
    
        // UAV trajectory
        void recordUAVPosition(double x, double y, double z, simtime_t time, const char* event);
    
    // Write to CSV files
    void writeStabilityData(int roundNum, double timeStamp, int aliveNodes);
    void writeEnergyData(int roundNum, double energyConsumed, double avgResidualEnergy, double totalNetworkEnergy);
    void writePDRData(int roundNum, double pdr);
    void writeThroughputData(simtime_t time, double throughput);
    void writeDelayData(int packetId, double delay);
    void writeContactData(int instance, simtime_t duration, double probability);
    void writeOverheadData(int roundNum, int controlPkts, int dataPkts, double ratio);
    void writeNetworkData(int roundNum, double timeStamp, double totalEnergy, int alive, int dead);
    void recordRLReward(int roundNum, double reward);
    void recordRLAction(int roundNum, bool exploration);
    void recordRLStateVisit(const std::string& stateHash);
    
    // RL convergence detection
    void checkRLConvergence(double currentReward, double currentExplorationRate);
    bool isRLConverged();
    int getRLConvergenceRound();
    
    // Per-round RL accumulation
    void accumulateRoundReward(double reward);
    void accumulateRoundExplorationRate(double explorationRate);
    void resetRoundRLAccumulators();
    double getAverageRoundReward();
    double getAverageRoundExplorationRate();
    
    // Statistics
    double calculateAverageDelay();
    double calculatePDR();
    double calculateAverageResidualEnergy();
    double getTotalNetworkEnergy();
    
    // RL convergence data
    void writeRLReport();

    // Accessors for termination logic
    int getLndRound() const { return lndRecorded ? lndRound : -1; }
    int getFndRound() const { return fndRecorded ? fndRound : -1; }
    bool hasLndRecorded() const { return lndRecorded; }
    int getCurrentRound() const { return currentRound; }
    
    // Global synchronization functions
    simtime_t getRoundStartTime(int roundNum) const;
    simtime_t getClusteringPhaseEnd() const;
    simtime_t getUAVCollectionWindowStart(int roundNum) const;
    simtime_t getUAVCollectionWindowEnd(int roundNum) const;
    int getNodeUAVVisits(int nodeId) const;
    
private:
    // Global timing parameters
    double clusteringPhaseDuration;     // 8s
    double uavFlightToNetworkTime;      // 35s
    double uavCollectionWindowDuration; // 30s
    double uavReturnToBaseTime;         // 35s
    double uavBaseTransferTime;         // 1s
    
    // Per-node UAV visit tracking for adaptive expiration
    std::map<int, int> nodeUAVVisitCount;
};

#endif
