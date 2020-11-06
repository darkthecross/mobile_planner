This is an experimental repository for exploring optimization algorithms.

# Setup

Information about how to setup the dev env.

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

## OpenCV
This was used to visualize the trajectory in kinematics_trajectory_demo.cc

# Math
Kinematics model: [Bycicle model](https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf).

LQR: [pabbeel's lecture](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/LQR.pdf)

Only the trajectory following algorithm was implemented, and I lost interest on this. 

The iterative LQR algorithm should be most interesting, since it optimizes any cost given a initial trajectory. 
But Pratically applying this algorithm will require some good search output, which serves as the initialization.