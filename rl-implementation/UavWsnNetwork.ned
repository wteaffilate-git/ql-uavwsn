simple SensorNode
{
    parameters:
        @display("i=device/sensor");
        double initialEnergy @unit(J) = default(0.5J);
        double chProbability = default(0.1);
        double controlPacketSize @unit(bit) = default(200bit);
        double dataPacketSize @unit(bit) = default(2000bit);  // CONSISTENT: 2000 bits
        double eElec @unit(J) = default(50e-9J);
        double eFreeSpace @unit(J) = default(10e-12J);
        double eMultiPath @unit(J) = default(0.0013e-12J);
        double idleListeningPower @unit(W) = default(0W);
        double commRadius @unit(m) = default(100m);  // Extended range
        double chDutyCycle = default(0.3);  // CH nodes: 30% active per round
        double memberDutyCycle = default(0.02);  // Member nodes: 2% active per round
        double isolatedDutyCycle = default(0.001);  // Isolated nodes: 0.1% active per round
        // Clustering protocol timing parameters (GLOBAL SYNC)
        double neighborDiscoveryDelay @unit(s) = default(0.2s);  // CH beacon delay
        double clusteringPhaseDelay @unit(s) = default(2.0s);    // TDMA schedule delay
        double joinAttemptDelay @unit(s) = default(0.8s);        // Member join check delay
        double unclusteredTimeout @unit(s) = default(3.0s);      // Unclustered node timeout
        double roundDuration @unit(s) = default(100.0s);         // GLOBAL SYNC: 100s rounds
        int maxBufferSize @unit(bit) = default(200000bit);       // Maximum aggregation buffer
    gates:
        input directIn @directIn;
}

simple UAVNode
{
    parameters:
        @display("i=block/uav");
        double uavHeight @unit(m) = default(30m);  // Realistic altitude
        double minSpeed @unit(mps) = default(10mps);  // CONSISTENT: 10 m/s
        double maxSpeed @unit(mps) = default(10mps);  // CONSISTENT: 10 m/s
        double searchSpeed @unit(mps) = default(10mps);  // CONSISTENT: 10 m/s
        double discoveryInterval @unit(s) = default(5s);  // Beacon every 5s (reduced overhead)
        double commRadius @unit(m) = default(192m);  // 192m nominal (190m effective at 30m height)
        double dataRate @unit(bps) = default(2000000bps);  // 2 Mbps (benchmark)
        double handshakeDelay @unit(s) = default(0.05s);
        double roundDuration @unit(s) = default(200s);   // GLOBAL SYNC: 200s rounds (no overshoot)
        double collectionWindow @unit(s) = default(130s); // 130s collection (35-165s, ~3.71 waypoints)
        double rasterWidth @unit(m) = default(100m);  // ~2/3 of commRadius
    gates:
        input directIn @directIn;
}

simple BaseStation
{
    parameters:
        @display("i=block/app");
        double xPos @unit(m) = default(-100m);  // CONSISTENT: Outside field, west of network
        double yPos @unit(m) = default(250m);   // CONSISTENT: Mid-height of 500m field
        double commRadius @unit(m) = default(200m);  // BS comm range
    gates:
        input directIn @directIn;
}

network UavWsnNetwork
{
    parameters:
        int numNodes = default(100);
        double areaX @unit(m) = default(500m);  // Realistic area for discovery
        double areaY @unit(m) = default(500m);
    submodules:
        bs: BaseStation {
            @display("p=50,250");  // BS at (-100,250) in simulation coords
        }
        uav: UAVNode {
            @display("p=300,300");  // UAV starts near BS
        }
        node[numNodes]: SensorNode {
            @display("p=200+i*8,200+i*5");  // Distributed in 500m area
        }
}