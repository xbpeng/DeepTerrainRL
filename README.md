# Intro

Source code for the paper: Terrain-Adaptive Locomotion Skills using Deep Reinforcement Learning
https://www.cs.ubc.ca/~van/papers/2016-TOG-deepRL/index.html

# Setup

This section covers some of the steps to setup and compile the code. The software depends on many libraries that need to be carefully prepared and placed for the building and linking to work properly.

## Linux 

### Linux Build Instructions

 1. Download the most recent compressed external file from the newest release.
    wget https://github.com/xbpeng/DeepTerrainRL/releases/download/v1.0/TerrainRL-external-Linux.tar.gz
 1. Extract it and move into the DeepTerrainRL directory.     
    tar zxvf TerrainRL-external-Linux.tar.gz
 1. repair external
    cp ~/DeepTerrainRL/caffe_mods/caffe.proto ~/DeepTerrainRL/external/caffe/src/caffe/proto/caffe.proto 
    cp ~/DeepTerrainRL/caffe_mods/memory_data_layer.cpp ~/DeepTerrainRL/external/caffe/src/caffe/layers/memory_data_layer.cpp
    cp ~/DeepTerrainRL/caffe_mods/memory_data_layer.hpp ~/DeepTerrainRL/external/caffe/include/caffe/layers/memory_data_layer.hpp 
 1. install depends
	sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler  
	sudo apt-get install --no-install-recommends libboost-all-dev  
	sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev  
	sudo apt-get install libatlas-base-dev  
    sudo apt-get install f2c libglew-dev
	
	In the instruction to make and build Caffe uncomment the CPU only line  
	```
	# CPU-only switch (uncomment to build without GPU support).
	CPU_ONLY := 1
	```
	sudo apt-get install gcc-4.9-multilib g++-4.9-multilib 
 1. Rebuild caffe
  1. Cd into external/caffe
  1. Make clean
  1. Make
 1. Cd back to ../../
    cd ../..
 1. Copy the caffe libraries from external/caffe/build/lib to ./lib
    cp external/caffe/build/lib/*.* lib/
 1. Premake4 clean
    premake4 clean
 1. Premake4 gmake
    premake4 gmake
 1. Make config=debug64
    make config=debug64
 
 Note: There are some issues with the installation on Ubuntu 16.04. Some of the libraries have changed their location and name (see https://github.com/BVLC/caffe/issues/2347 for a solution).

## Runing The System

After the system has been build there are two executable files that server different purposes. The **TerrainRL** program is for visually simulating the a controller and **TerrainRL_Optimize** is for optimizing the parameters of some controller.

Examples:  
	To simulate a controller/character  
	./TerrainRL -arg_file= args/sim_dog_args.txt  
	To simulate a controller/character with a specific policy  
	./TerrainRL_Optimizer -arg_file= args/dog_slopes_mixed_args.txt
	To Train a controller  
	./TerrainRL_Optimizer -arg_file= args/opt_args_train_mace.txt  


## Key Bindings

Most of these are togglesg

 - c fixed camera mode
 - y draw COM path and contact locations
 - q draw "filmstrip" like rendering
 - f draw torques
 - h draw Actor value functions and feature visualization
 - shift + '>' step one frame
 - p toggle draw value function
 - ',' and '.' change render speed, decrease and increase.
 - "spacebar" to pause simulation
 - r restart the scenario
 - l reload the simulation (reparses the arg file)
 - g draw state features
 - x spawn projectile
 - z spawn big projectile
 
 - click on character and drag to apply force
