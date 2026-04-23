#!/bin/bash
#
# Baseline S0 Scenario Runner - Runs until LND (Last Node Death)
# Simple, focused script for baseline testing without parametric variations
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

PROJECT_DIR="$(pwd)"
cd "$PROJECT_DIR"

# Load OMNeT++ environment
echo -e "${BLUE}Loading OMNeT++ environment...${NC}"
source /home/wte/omnetpp/omnetpp-6.3.0/setenv

echo -e "${YELLOW}=========================================${NC}"
echo -e "${YELLOW}  UAV-WSN-BM: Baseline S0 Runner (LND)   ${NC}"
echo -e "${YELLOW}=========================================${NC}"

# Step 1: Rebuild project
echo -e "\n${BLUE}Step 1: Rebuilding project...${NC}"
make clean > /dev/null 2>&1
make MODE=release -j4 > /dev/null 2>&1

if [ ! -f "$PROJECT_DIR/uav-wsn-bm" ]; then
    echo -e "${RED}✗ Build failed!${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Build successful${NC}"

# Step 2: Run baseline S0 until LND
echo -e "\n${BLUE}Step 2: Running Baseline S0 until LND...${NC}"

RESULTS_DIR="results/scenarios/S0-Baseline"
mkdir -p "$RESULTS_DIR"

echo -e "${YELLOW}Starting simulation...${NC}"
echo "Parameters: Baseline configuration (100 nodes, CH=0.1, Energy=0.5J)"
echo "Seed: 1 (reproducible)"
echo "Output directory: $RESULTS_DIR"
echo ""

# Run simulation until LND (1161000s is ~1500 rounds at 774s per round)
timeout 7200 ./uav-wsn-bm -u Cmdenv -c General \
    omnetpp.ini \
    -n . \
    --cmdenv-express-mode=true \
    --cmdenv-performance-display=false \
    --sim-time-limit=1161000s \
    --seed-set=1 \
    > "$RESULTS_DIR/simulation.log" 2>&1 || {
    exit_code=$?
    if [ $exit_code -eq 124 ]; then
        echo -e "${YELLOW}Note: Timeout after 2 hours${NC}"
    else
        echo -e "${YELLOW}Simulation completed with code $exit_code${NC}"
    fi
}

# Step 3: Move result files
echo -e "\n${BLUE}Step 3: Moving result files...${NC}"

# Move CSV files
for csv in results/*.csv; do
    if [ -f "$csv" ]; then
        mv "$csv" "$RESULTS_DIR/" 2>/dev/null || true
    fi
done

# Move .sca files
for sca in results/*.sca; do
    if [ -f "$sca" ]; then
        mv "$sca" "$RESULTS_DIR/" 2>/dev/null || true
    fi
done

# Move summary.txt
if [ -f "results/summary.txt" ]; then
    mv "results/summary.txt" "$RESULTS_DIR/" 2>/dev/null || true
fi

# Check if simulation produced results
if [ -f "$RESULTS_DIR/stability.csv" ]; then
    echo -e "${GREEN}✓ Results saved to $RESULTS_DIR${NC}"
    
    # Step 4: Display summary
    echo -e "\n${BLUE}Step 4: Displaying summary...${NC}"
    
    if [ -f "$RESULTS_DIR/summary.txt" ]; then
        echo -e "${GREEN}✓ Summary available${NC}"
        
        # Display summary
        echo -e "\n${BLUE}Baseline S0 Results Summary:${NC}"
        echo "======================================"
        cat "$RESULTS_DIR/summary.txt"
    else
        echo -e "${YELLOW}⚠ Summary not found${NC}"
    fi
    
    # Step 5: Generate plots
    echo -e "\n${BLUE}Step 5: Generating plots...${NC}"
    
    python3 generate_s0_plots.py > /dev/null 2>&1
    
    if [ -f "plots/scenarios/S0-Baseline/fnd_lnd.png" ] 2>/dev/null || [ -f "plots/S0-Baseline/fnd_lnd.png" ] 2>/dev/null; then
        echo -e "${GREEN}✓ Plots generated${NC}"
    else
        echo -e "${YELLOW}⚠ Plot generation may need manual verification${NC}"
    fi
    
else
    echo -e "${RED}✗ No results generated!${NC}"
    echo "Check simulation.log for errors:"
    echo "---"
    tail -30 "$RESULTS_DIR/simulation.log"
    exit 1
fi

echo -e "\n${YELLOW}=========================================${NC}"
echo -e "${GREEN}✓ Baseline S0 execution complete!${NC}"
echo -e "${YELLOW}=========================================${NC}"
echo -e "Results: $RESULTS_DIR"
echo -e "Summary: $RESULTS_DIR/metrics_summary.csv"
echo ""
