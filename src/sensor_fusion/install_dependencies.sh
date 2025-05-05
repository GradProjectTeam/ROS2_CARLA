#!/bin/bash

# Exit on error
set -e

# Print commands before executing
set -x

# Function to check if Python 3.7 is installed
check_python37() {
    if command -v python3.7 &> /dev/null; then
        echo "Python 3.7 is installed"
        return 0
    else
        echo "Python 3.7 is not installed"
        return 1
    fi
}

# Function to install Python 3.7 if not installed
install_python37() {
    echo "Installing Python 3.7..."
    
    sudo apt-get update
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y ppa:deadsnakes/ppa
    sudo apt-get update
    sudo apt-get install -y python3.7 python3.7-dev python3.7-venv python3.7-distutils
    
    # Install pip for Python 3.7
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
    python3.7 get-pip.py
    rm get-pip.py
}

# Main script
echo "Checking for Python 3.7..."
if ! check_python37; then
    install_python37
fi

echo "Installing required packages for Python 3.7..."

# Make sure pip is up to date
python3.7 -m pip install --upgrade pip

# Install required packages
python3.7 -m pip install -r requirements.txt

# Install pygame dependencies
sudo apt-get install -y \
    python3-pygame \
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libsdl2-ttf-dev \
    libfreetype6-dev \
    libportmidi-dev

echo "Installation complete!" 