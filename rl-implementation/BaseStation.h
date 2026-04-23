#ifndef __UAVWSN_BASESTATION_H
#define __UAVWSN_BASESTATION_H

#include <omnetpp.h>
#include "MetricsCollector.h"

class BaseStation : public omnetpp::cSimpleModule
{
  protected:
    MetricsCollector* metrics;
    int totalPacketsReceived;
    double totalBitsReceived;
    
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    virtual void finish() override;
};
#endif