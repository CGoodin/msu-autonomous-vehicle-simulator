# The MSU Autonomous Vehicle Simulator

Software for simulating autonomous ground vehicles in off-road terrain. Simulates the sensors, vehicle, and environment. Uses physics-based models to simulate camera, lidar, and radar interacting with environmental features such as rain, dust, and fog.

MAVS is written in C++ and has a Python API.

## Installation and Documentation
See the [wiki](https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator/wikis/home) for installation and use of MAVS, including examples of python scripting with MAVS and instructions for using the GUI.

Also see the MAVS C++ API [documentation](https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/).

## Features
MAVS can automatically generate random ecosystems complete with trails and realistic vegetation.
![forest](docs/screenshots/mrzr_forest.png)
![desert](docs/screenshots/mavs_desert.png)

MAVS can also simulate environmental features like rain and dust and their influence on sensors.
![rain](docs/screenshots/warthog_fog.png)
![dust](docs/screenshots/forester_snow.png)

