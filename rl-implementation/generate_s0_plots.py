import os
import pandas as pd
import matplotlib.pyplot as plt

BASE = 'results/scenarios/S0-Baseline'
OUT = 'plots/scenarios/S0-Baseline'
os.makedirs(OUT, exist_ok=True)

# Energy plot
try:
    e = pd.read_csv(os.path.join(BASE,'energy.csv'))
    plt.figure(figsize=(8,4))
    plt.plot(e['Round'], e['AvgResidualEnergy'], '-o')
    plt.xlabel('Round')
    plt.ylabel('Avg Residual Energy (J)')
    plt.title('S0-Baseline: Avg Residual Energy per Round')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT,'energy_avg_residual.png'))
    plt.close()
except Exception as ex:
    print('Energy plot failed:', ex)

# PDR plot
try:
    p = pd.read_csv(os.path.join(BASE,'pdr.csv'))
    plt.figure(figsize=(8,4))
    plt.plot(p['Round'], p['PDR'], '-o')
    plt.xlabel('Round')
    plt.ylabel('Packet Delivery Ratio')
    plt.title('S0-Baseline: PDR per Round')
    plt.ylim(0,1)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT,'pdr.png'))
    plt.close()
except Exception as ex:
    print('PDR plot failed:', ex)

# Throughput over time
try:
    t = pd.read_csv(os.path.join(BASE,'throughput.csv'))
    plt.figure(figsize=(8,4))
    plt.plot(t['Time'], t['Throughput_kbps'], '-o')
    plt.xlabel('Time (s)')
    plt.ylabel('Throughput (kbps)')
    plt.title('S0-Baseline: Throughput over Time')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT,'throughput.png'))
    plt.close()
except Exception as ex:
    print('Throughput plot failed:', ex)

# Stability (alive/dead nodes)
try:
    s = pd.read_csv(os.path.join(BASE,'stability.csv'))
    plt.figure(figsize=(8,4))
    plt.plot(s['Round'], s['AliveNodes'], label='Alive')
    plt.plot(s['Round'], s['DeadNodes'], label='Dead')
    plt.xlabel('Round')
    plt.ylabel('Nodes')
    plt.title('S0-Baseline: Alive and Dead Nodes')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT,'stability.png'))
    plt.close()
except Exception as ex:
    print('Stability plot failed:', ex)

# End-to-End Delay histogram
try:
    d = pd.read_csv(os.path.join(BASE,'delay.csv'))
    plt.figure(figsize=(8,4))
    plt.hist(d['Delay_s'], bins=50, edgecolor='black')
    plt.xlabel('End-to-End Delay (s)')
    plt.ylabel('Number of Packets')
    plt.title('S0-Baseline: End-to-End Delay Distribution')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT,'delay_histogram.png'))
    plt.close()
except Exception as ex:
    print('Delay plot failed:', ex)

# Control Overhead Ratio
try:
    o = pd.read_csv(os.path.join(BASE,'overhead.csv'))
    plt.figure(figsize=(8,4))
    plt.plot(o['Round'], o['OverheadRatio'], '-o')
    plt.xlabel('Round')
    plt.ylabel('Control Overhead Ratio')
    plt.title('S0-Baseline: Control Overhead Ratio per Round')
    plt.ylim(0,1)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT,'overhead_ratio.png'))
    plt.close()
    
    # Control and Data Packets in same plot
    plt.figure(figsize=(8,4))
    plt.plot(o['Round'], o['ControlPackets'], '-o', label='Control Packets')
    plt.plot(o['Round'], o['DataPackets'], '-s', label='Data Packets')
    plt.xlabel('Round')
    plt.ylabel('Number of Packets')
    plt.title('S0-Baseline: Control and Data Packets per Round')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT,'control_data_packets.png'))
    plt.close()
except Exception as ex:
    print('Overhead plot failed:', ex)

print('Plots saved to', OUT)
