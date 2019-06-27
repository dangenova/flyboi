# Dan's Master's Work

Master's Work of using MPC trajectory optimization following an unknown dynamic object.  It uses ETHZ UAV MPC Solver

## Setup

Follow these Instructions in order to uses this code

### Prerequisites

Two things will need to be installed, before following the rest of Directions
* ROS (Kinetic preffered, that's the Distro I developed and tested in)
* Mavros

I currently do not have directions on how to install these.


Next you will need to create a new Catkin Workspace

```sh
  $ mkdir -p ~/<workspace_name>/src
  $ cd ~/<workspace_name>
  $ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  $ catkin init  # initialize your catkin workspace
```

Next download the following packages from the ETHZ-ASL Group in your ~/<workspace_name>/src directory

```sh
  $ sudo apt-get install liblapacke-dev
  $ git clone https://github.com/catkin/catkin_simple.git
  $ git clone https://github.com/ethz-asl/mav_comm.git
  $ git clone https://github.com/ethz-asl/eigen_catkin.git
  $ git clone https://github.com/ethz-asl/mav_control_rw.git
```

### Download and Install 

Now download this package
```sh
  $ git clone https://github.com/dangenova/flyboi.git
```

Install the packages
```sh
  $ catkin build
```

You also may have to turn all of the python scripts into excecutables.

## Running the tests

Right now, 3 Launch files will need to be excecuted in order to run tests:

* Launch File to act as a Mavros Initializer and Wrapper
* Launch File to start MPC Controller
* Launch File to Initialize Target Trajectory

```sh
   $roslaunch flyboi_ethz APM_DJI_Wrapper.launch
```

```sh
   $roslaunch flyboi_ethz mav_linear_mpc_dji_flamewheel.launch
```

'''sh
   $roslaunch flyboi_ethz Trajectory_Prediction.launch
'''




