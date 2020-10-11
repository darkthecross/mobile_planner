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

# Math
Kinematics model: [Bycicle model](https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf).
