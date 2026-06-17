#!/bin/bash
set -e

echo "========================================="
echo "Setting up Kinect dependencies..."
echo "========================================="

# 1. Install prerequisites
echo "[1/4] Installing build dependencies..."
sudo apt-get update
sudo apt-get install -y git build-essential cmake libusb-1.0-0-dev libudev-dev pkg-config python3-dev cython3 python3-numpy

# 2. Clone and build libfreenect
echo "[2/4] Cloning and building libfreenect..."
cd ~/Documents
if [ ! -d "libfreenect" ]; then
    git clone https://github.com/OpenKinect/libfreenect.git
fi
cd libfreenect
mkdir -p build
cd build
cmake -L ..
make -j$(nproc)
sudo make install
sudo ldconfig

# 3. Install Python wrapper
echo "[3/4] Installing freenect Python wrapper..."
cd ~/Documents/libfreenect/wrappers/python
python3 setup.py build_ext --inplace
sudo python3 setup.py install

# 4. Set up udev rules and blacklist gspca_kinect
echo "[4/4] Setting up udev rules and blacklisting default driver..."
sudo cp ~/Documents/libfreenect/platform/linux/udev/51-kinect.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo modprobe -r gspca_kinect gspca_main || true
echo 'blacklist gspca_kinect' | sudo tee /etc/modprobe.d/kinect.conf > /dev/null

echo "========================================="
echo "Setup complete! Please unplug and replug the Kinect USB cable."
echo "========================================="
