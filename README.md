# Local Planner Using OMPL
This repository consists of my attempts in learning ompl libraries for robot planning (based on ompl tuturials). 


## Dependencies

* [Boost](http://www.boost.org) (version 1.54 or higher)
* [CMake](http://www.cmake.org) (version 2.8.7 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.3 or higher)
* [Assimp](http://assimp.org) (version 3.0.1270 or higher)
* [FCL](https://github.com/flexible-collision-library/fcl) (version 0.3.1 or higher)
* [OMPL](https://github.com/ompl/ompl) (version 1.3.3 or higher)


### Installing Dependencies:

To install Boost, run the following command:

```
sudo apt-get update
sudo apt-get install libboost-all-dev
```

To install Cmake, run the following command:

```
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install cmake
```

To install Eigen, run the following command:

```
sudo apt install libeigen3-dev
```

To install Assimp, run the following command:

```
sudo apt-get install assimp-utils
```

To install FCL, run the following command:

```
sudo apt-get install libccd-dev # FCL Dependency
git clone https://github.com/flexible-collision-library/fcl
cd fcl
mkdir build
cd build
cmake ..
make -jN # N is the maximum number of parallel compile jobs
sudo make install
```

To install OMPL libraries, run the following command:

```
git clone https://github.com/ompl/omplapp.git
cd omplapp
git clone https://github.com/ompl/ompl.git
mkdir -p build/Release
cd build/Release
cmake ../..
make -jN # N is the maximum number of parallel compile jobs
sudo make install
```

## How to run

After running rosnode ompl_planner_node, publish start and goal states in SE3 (as nav/msgs) and the planner will start working.

