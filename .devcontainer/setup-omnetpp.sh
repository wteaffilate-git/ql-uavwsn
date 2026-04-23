#!/bin/bash
set -e

echo "Setting up OMNeT++ 6.3.0 for QL-UAV-WSN project..."

# Update package sources to include Ubuntu noble
cat > /etc/apt/sources.list << 'EOF'
deb http://archive.ubuntu.com/ubuntu noble main universe multiverse restricted
deb http://archive.ubuntu.com/ubuntu noble-updates main universe multiverse restricted
deb http://archive.ubuntu.com/ubuntu noble-security main universe multiverse restricted
deb http://archive.ubuntu.com/ubuntu noble-backports main universe multiverse restricted
EOF

# Update package list
apt-get update -y

# Install OMNeT++ dependencies
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    make diffutils pkg-config ccache clang gdb lldb bison flex perl sed gawk \
    python3 python3-pip python3-venv python3-dev libxml2-dev zlib1g-dev \
    doxygen graphviz xdg-utils libdw-dev

# Clean up
apt-get clean

# Download and install OMNeT++ 6.3.0
cd /home/codespace
if [ ! -d omnetpp-6.3.0 ]; then
    echo "Downloading OMNeT++ 6.3.0..."
    curl -L -o omnetpp-6.3.0-core.tgz \
        https://github.com/omnetpp/omnetpp/releases/download/omnetpp-6.3.0/omnetpp-6.3.0-core.tgz
    tar -xzf omnetpp-6.3.0-core.tgz
    rm omnetpp-6.3.0-core.tgz
fi

# Configure and build OMNeT++
cd /home/codespace/omnetpp-6.3.0
if [ ! -f bin/opp_run ]; then
    echo "Configuring OMNeT++..."
    ./configure WITH_QTENV=no WITH_OSG=no WITH_SCAVE_PYTHON_BINDINGS=no
    echo "Building OMNeT++..."
    make -j$(nproc)
fi

# Set ownership
chown -R codespace:codespace /home/codespace/omnetpp-6.3.0

echo "OMNeT++ 6.3.0 setup complete!"

# Add to PATH in .bashrc if not already there
if ! grep -q "omnetpp-6.3.0/setenv" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# OMNeT++ 6.3.0 Environment Setup" >> ~/.bashrc
    echo "if [ -f /home/codespace/omnetpp-6.3.0/setenv ]; then" >> ~/.bashrc
    echo "    source /home/codespace/omnetpp-6.3.0/setenv" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
fi

echo "Environment setup complete. OMNeT++ will be available in new shells."