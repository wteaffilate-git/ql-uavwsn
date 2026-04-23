#include "BaseStation.h"
#include "MetricsCollector.h"

Define_Module(BaseStation);

void BaseStation::initialize()
{
    metrics = MetricsCollector::getInstance();
    totalPacketsReceived = 0;
    totalBitsReceived = 0;
}

void BaseStation::handleMessage(omnetpp::cMessage *msg)
{
    if (!strcmp(msg->getName(), "AGGREGATED_DATA") || !strcmp(msg->getName(), "UAV_DATA")) {
        // Data received from UAV or CH within range
        printf("BaseStation received %s at t=%.2f\n", msg->getName(), simTime().dbl());
        if (msg->hasPar("dataSize")) {
            int dataSize = msg->par("dataSize");
            totalBitsReceived += dataSize;
            totalPacketsReceived++;
            
            // Metric: Track packet reception for PDR and delay
            if (msg->hasPar("packetIDs")) {
                const char* packetIDsCStr = msg->par("packetIDs").stringValue();
                std::string packetIDsStr = std::string(packetIDsCStr);
                printf("BaseStation parsing packet IDs: '%s'\n", packetIDsStr.c_str());
                
                int parsedCount = 0;
                // Parse comma-separated packet IDs
                size_t pos = 0;
                while ((pos = packetIDsStr.find(',')) != std::string::npos) {
                    if (pos > 0) {
                        try {
                            int packetId = std::stoi(packetIDsStr.substr(0, pos));
                            printf("BaseStation recording packet %d\n", packetId);
                            metrics->recordPacketReception(packetId, simTime());
                            parsedCount++;
                        } catch (...) {
                            // Skip invalid IDs
                        }
                    }
                    packetIDsStr.erase(0, pos + 1);
                }
                // Process the last packet ID (no trailing comma)
                if (!packetIDsStr.empty()) {
                    try {
                        int packetId = std::stoi(packetIDsStr);
                        printf("BaseStation recording packet %d\n", packetId);
                        metrics->recordPacketReception(packetId, simTime());
                        parsedCount++;
                    } catch (...) {
                        // Skip invalid ID
                    }
                }
                EV << "BaseStation successfully parsed " << parsedCount << " packet IDs" << endl;
            } else {
                EV << "WARNING: BaseStation received message without packetIDs parameter!" << endl;
            }
            
            EV << "Base Station received " << dataSize << " bits at time " 
               << simTime() << endl;
        }
    }
    
    delete msg;
}

void BaseStation::finish()
{
    EV << "=== Base Station Final Statistics ===" << endl;
    EV << "Total Packets Received: " << totalPacketsReceived << endl;
    EV << "Total Data Received: " << totalBitsReceived << " bits" << endl;
    
    // Finalize all metrics and write summary
    if (metrics) {
        metrics->finalize();
        EV << "Overall PDR: " << metrics->calculatePDR() << endl;
        EV << "Average Delay: " << metrics->calculateAverageDelay() << " s" << endl;
    }
    
    recordScalar("totalPacketsReceived", totalPacketsReceived);
    recordScalar("totalBitsReceived", totalBitsReceived);
}
