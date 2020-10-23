This is an experimental repository for exploring optimization algorithms.

The main algorithm is iterative LQR.

# Setup

Information about how to setup the dev env (on ubuntu).

## Install ProtoBuf
```
sudo apt-get install autoconf automake libtool curl make g++ unzip -y
git clone https://github.com/google/protobuf.git
cd protobuf
git submodule update --init --recursive
./autogen.sh
./configure
make
sudo make install
sudo ldconfig
```

## Install OpenCV

OpenCV was used in this project as visualization tools.
```
sudo apt update
sudo apt install libopencv-dev python3-opencv
```

# Math
Kinematics model: [Bycicle model](https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf).
