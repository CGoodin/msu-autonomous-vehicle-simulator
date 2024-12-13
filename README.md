# The MSU Autonomous Vehicle Simulator

The MSU Autonomous Vehicle Simulator (MAVS) is

* A software library for simulating autonomous vehicles in realistic digital terrain.
* A real-time simulator for evaluating the performance of autonomous perception and navigation software.
* A physics-based sensor simulator for LIDAR, GPS, cameras, and other sensors.

While MAVS is a fully functional standalone simulator, additional wrappers allow MAVS to be integrated with robotic development tools such the Robotic Operating System (ROS). 

This documentation outlines building MAVS, running some example simulations, and interfacing with other software like ROS. 

## MAVS Architecture 
The MAVS is a software library that can be incorporated into a variety of applications through its applicaton programming interface (API). Please see the [API documentation](https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/) for information on developing applications with MAVS.

## Building Mavs
See instructions [for building MAVS](docs/MavsBuildInstructions.md).

## Running the Mavs Gui
On Linux systems with Python3 installed, MAVS simulations can be run with a [TKinter-based GUI](docs/Gui/RunningMavsGUI). The MAVS GUI can be used to set up and run sensor simulations in [randomized scenes](docs/Gui/GeneratingRandomDataWithGUI.md).

## Using the MAVS C++ API
Portions of the MAVS API can be accessed in [MATLAB](docs/Interfaces/MavsMatlab) or [Python](docs/Interfaces/MavsPython.md) through the C interfaces.

## MAVS-ROS Package
The mavs_ros package has example [ROS-nodes](docs/Interfaces/MavsROS.md) built around MAVS simulation capabilities. 

## Running simulations from the command line
Several MAVS executables can be run [from the commmand line.](docs/RunningASimulation.md)

## MAVS Input Files
MAVS primarily uses [json input files](docs/MavsSimulationInputsFiles.md). 

## MAVS Sensor Models
MAVS features several different types of sensors including [cameras](docs/Sensors/MavsCamera.md), [LIDAR](docs/Sensors/MavsLidar.md), GPS, [RADAR](docs/Sensors/MavsRadar.md) and [IMUs](docs/Sensors/MavsImu.md).

MAVS can also be used to render [photorealistic images](docs/Sensors/MavsPathTracer.md).

## MAVS Vehicle Models
MAVS has a built-in [vehicle simulator](docs/Vehicles/MavsVehicles.md) and can also be linked to the [Chrono vehicle dynamics](docs/MavsBuildInstructions.md)

## Examples and Utilities
MAVS comes with several [example codes](docs/Examples/MavsExamples.md) and [utilitys](docs/Utilities/MavsUtils.md) that demonstrate how to implement various features through the API.

## Features
MAVS can automatically generate random ecosystems complete with trails and realistic vegetation.
![forest](docs/screenshots/mrzr_forest.png)
![desert](docs/screenshots/mavs_desert.png)

MAVS can also simulate environmental features like rain and dust and their influence on sensors.
![rain](docs/screenshots/warthog_fog.png)
![dust](docs/screenshots/forester_snow.png)

