# RL-Based UAV-Assisted WSN Implementation

This folder contains the RL-based implementation of UAV-assisted Wireless Sensor Network routing, based on the Unified Distributed Q-Learning (UDQL) framework.

## Overview

The implementation extends the baseline simulation with three RL agents:

1. **Node Agent**: Handles clustering refinement for unclustered nodes
   - Actions: JOIN (nearest CH), SELF_CH (become silent CH), WAIT_UAV (buffer for UAV)
   - State: Energy level, distance to CH, neighbor density, UAV proximity

2. **CH Agent**: Manages routing decisions for cluster heads
   - Actions: TRANSMIT_TO_UAV, BUFFER, FORWARD_TO_CH
   - State: Energy, queue size, packet age, UAV proximity, neighbor distance

3. **UAV Agent**: Controls scheduling and mobility
   - Actions: SELECT_CH (prioritize CH), SKIP_CH (ignore CH)
   - State: CH queue, age, energy, distance

## Files Modified

- `SensorNode.cc/h`: Added RL logic for nodes and CHs
- `UAVNode.cc/h`: Added RL logic for UAV scheduling
- `run_rl_until_lnd.sh`: Run script for RL simulation
- `generate_rl_plots.py`: Plotting script for RL results

## Key Features

- Epsilon-greedy Q-learning with configurable parameters
- State discretization for tabular Q-learning
- Reward functions based on delivery, delay, energy
- Integrated with existing OMNeT++ simulation framework

## Running the Simulation

```bash
./run_rl_until_lnd.sh
python3 generate_rl_plots.py
```

Results are stored in `results/scenarios/RL-Baseline/`

## Notes

- RL parameters (alpha, gamma, epsilon) are set to basic values
- State observations are simplified
- Q-table updates are implemented but rewards are basic
- This is a foundational implementation that can be extended with more sophisticated features</content>
<parameter name="filePath">/home/wte/uav-wsn-ql/rl-implementation/README.md