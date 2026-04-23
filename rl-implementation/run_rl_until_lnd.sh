#!/bin/bash
#
# UAV-WSN-RL: RL-Based Runner (LND)
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
echo -e "${YELLOW}  UAV-WSN-RL: RL-Based Runner (LND)   ${NC}"
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

# Step 2: Run RL simulation until convergence or LND
echo -e "\n${BLUE}Step 2: Running RL simulation until convergence or LND...${NC}"

RESULTS_DIR="results/scenarios/RL-Baseline"
mkdir -p "$RESULTS_DIR"

echo -e "${YELLOW}Starting RL simulation...${NC}"
echo "Parameters: RL-based UAV-assisted WSN (100 nodes, CH=0.1, Energy=0.5J)"
echo "Seed: 1 (reproducible)"
echo "Output directory: $RESULTS_DIR"
echo "Monitoring for RL convergence..."
echo ""

MAX_ROUNDS=1500  # Maximum rounds to run
CONVERGENCE_CHECK_INTERVAL=50  # Check every 50 rounds
SIM_TIME_PER_CHECK=$((CONVERGENCE_CHECK_INTERVAL * 774))  # ~774s per round

round=0
converged=false

while [ $round -lt $MAX_ROUNDS ] && [ "$converged" = false ]; do
    echo -e "${BLUE}Running rounds $((round+1))-$((round+CONVERGENCE_CHECK_INTERVAL))...${NC}"
    
    # Run simulation for next batch of rounds
    timeout 3600 ./uav-wsn-bm -u Cmdenv -c General \
        omnetpp.ini \
        -n . \
        --cmdenv-express-mode=true \
        --cmdenv-performance-display=false \
        --sim-time-limit=${SIM_TIME_PER_CHECK}s \
        --seed-set=1 \
        >> "$RESULTS_DIR/simulation.log" 2>&1 || {
        exit_code=$?
        if [ $exit_code -eq 124 ]; then
            echo -e "${YELLOW}Batch timeout after 1 hour${NC}"
        else
            echo -e "${YELLOW}Batch completed with code $exit_code${NC}"
        fi
    }
    
    round=$((round + CONVERGENCE_CHECK_INTERVAL))
    
    # Check for convergence
    if [ -f "results/rl_convergence.csv" ]; then
        # Check if convergence was detected (look for "1" in the Converged column)
        if grep -q ",1," "results/rl_convergence.csv"; then
            converged=true
            convergence_round=$(grep ",1," "results/rl_convergence.csv" | head -1 | cut -d',' -f1)
            echo -e "${GREEN}✓ RL CONVERGENCE DETECTED at round $convergence_round!${NC}"
            
            # Create convergence result file
            echo "RL_Convergence_Round: $convergence_round" > "$RESULTS_DIR/rl_convergence_result.txt"
            echo "Total_Rounds_Run: $round" >> "$RESULTS_DIR/rl_convergence_result.txt"
            echo "Convergence_Achieved: Yes" >> "$RESULTS_DIR/rl_convergence_result.txt"
        fi
    fi
    
    # Check for LND
    if [ -f "results/summary.txt" ]; then
        if grep -q "LND" "results/summary.txt"; then
            echo -e "${YELLOW}LND reached before convergence${NC}"
            echo "RL_Convergence_Round: Not_Converged" > "$RESULTS_DIR/rl_convergence_result.txt"
            echo "Total_Rounds_Run: $round" >> "$RESULTS_DIR/rl_convergence_result.txt"
            echo "Convergence_Achieved: No_LND_Reached" >> "$RESULTS_DIR/rl_convergence_result.txt"
            break
        fi
    fi
done

if [ "$converged" = false ] && [ $round -ge $MAX_ROUNDS ]; then
    echo -e "${YELLOW}Maximum rounds reached without convergence${NC}"
    echo "RL_Convergence_Round: Not_Converged" > "$RESULTS_DIR/rl_convergence_result.txt"
    echo "Total_Rounds_Run: $MAX_ROUNDS" >> "$RESULTS_DIR/rl_convergence_result.txt"
    echo "Convergence_Achieved: No_Max_Rounds" >> "$RESULTS_DIR/rl_convergence_result.txt"
fi

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
        echo -e "\n${BLUE}RL-Based S0 Results Summary:${NC}"
        echo "======================================"
        cat "$RESULTS_DIR/summary.txt"
    else
        echo -e "${YELLOW}⚠ Summary not found${NC}"
    fi
    
    # Step 5: Generate plots
    echo -e "\n${BLUE}Step 5: Generating plots...${NC}"
    
    python3 generate_rl_plots.py > /dev/null 2>&1
    
    if [ -f "plots/scenarios/RL-Baseline/pdr.png" ] 2>/dev/null || [ -f "plots/RL-Baseline/pdr.png" ] 2>/dev/null; then
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
echo -e "${GREEN}✓ RL-based execution complete!${NC}"
echo -e "${YELLOW}=========================================${NC}"
echo -e "Results: $RESULTS_DIR"
echo -e "Summary: $RESULTS_DIR/metrics_summary.csv"
echo ""
