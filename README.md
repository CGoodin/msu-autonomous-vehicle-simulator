# The MSU Autonomous Vehicle Simulator

Software for simulating autonomous ground vehicles in off-road terrain. Simulates the sensors, vehicle, and environment. Uses physics-based models to simulate camera, lidar, and radar interacting with environmental features such as rain, dust, and fog.

MAVS is written in C++ and has a Python API. [Detailed documentation](https://mavs-documentation.readthedocs.io/en/latest/) is available on ReadTheDocs.

The MAVS API is documented with DoxyGen [here](https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/).

## Reference MAVS
You can reference MAVS with the following citations:
* Hudson, C., Goodin, C., Miller, Z., Wheeler, W., & Carruth, D. (2020, August). Mississippi state university autonomous vehicle simulation library. In Proceedings of the Ground Vehicle Systems Engineering and Technology Symposium (pp. 11-13).
* Goodin, C., Carruth, D. W., Dabbiru, L., Hudson, C. H., Cagle, L. D., Scherrer, N., ... & Jayakumar, P. (2022, June). Simulation-based testing of autonomous ground vehicles. In Autonomous Systems: Sensors, Processing and Security for Ground, Air, Sea and Space Vehicles and Infrastructure 2022 (Vol. 12115, pp. 167-174). SPIE.

# Installation 
## Building MAVS From Source
MAVS can be built from source on both Windows and Linux/Unix systems.
 
(Jump to [building on Linux.](#building-mavs-on-linux))

(Jump to [building on Windows.](#building-mavs-on-windows))

### System Requirements
For all operating systems, you will need a [CMake](https://cmake.org/) version above 3.13.

You will also need [git](https://git-scm.com/download/). 

You will need a compiler that supports the CPP 17 standard (gcc 8 or higher or equivalent).

To run the python interface, you will also need to install [Python 3](https://www.python.org/downloads/release/python-370/). For Ubuntu users, you simply need to [install using apt](https://phoenixnap.com/kb/how-to-install-python-3-ubuntu). For Windows, you will also need to download and run the installer and add the folder containing the python.exe to your system path. 

### Cloning the MAVS repository
The steps for cloning MAVS are the same on both Windows and Linux/Unix. First clone the MAVS repo and name it "mavs". On Linux systems, it is recommended that you install MAVS in your home directory. 
```bash
cd ~
git clone --recursive https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator.git mavs
```
MAVS has multiple dependencies configured as [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules), so you **must** include the "--recursive" option to successfully build MAVS.

### Building MAVS on Linux
This section outlines how to build MAVS using terminal commands on Linux. Aptitude is the package manager used in the example commands for installing system dependencies. Users of different package managers will need to determine how to ensure the same dependencies are installed on their own. MAVS has been installed and tested on both Ubuntu and CentOS systems.

#### System Dependencies
Ensure X11, libJPEG, and zlib development packages are installed. For example, on Ubuntu or Debian systems, execute the following:
```bash
sudo apt-get install libx11-dev libjpeg-dev zlib1g-dev
```
Older versions of distros may not have a sufficiently up-to-date cmake package. If cmake is not 3.13+, it will need to be updated as well. Doing so varies depending on distro but for Ubuntu, [Kitware's APT repository](https://apt.kitware.com/) is the recommended method. Navigate to that site and follow the steps for your version of Ubuntu.

#### Installing MAVS on Linux
Presuming Embree and other system dependencies are now installed and available, MAVS can now be installed. To build MAVS in the default configuration, run the following from the repo's root directory:
```bash
cd ~/mavs
mkdir build
cd build
sudo ccmake ../
```
Use the ccmake tool to configure the MAVS build. Make sure to set the build type to "Release". Also ensure that the **DEFAULT_DATA_DIR** is set to point to your MAVS data folder. If you followed the steps above, then it should be set to *~/mavs/data*. 

It is reccomended that you set the **CMAKE_INSTALL_PREFIX** to */usr/local* if possible. If you install MAVS in a custom folder, you will need to set the CMAKE_PREFIX_PATH variable when linking other programs to MAVS using the *find_package* feature of CMake.

For a basic OpenMP-enable MAVS build, set the following options:
  * DEFAULT_DATA_DIR: ~/mavs/data
  * CMAKE_INSTALL_PREFIX: /usr/local
    * It is recommended that you install MAVS in your usr/local folder on Linux systems.
  * CMAKE_CONFIGURATION_TYPE: Release
  * Unless you wish to build Chrono, uncheck "MAVS_USE_CHRONO"
  * Unless you wish to use MAVS on a supercomputer, uncheck USE_MPI and make sure USE_OMP is checked.

You can then build MAVS with
```bash
$sudo make install
```

For running MAVS and linking MAVS to external applications, you may need to add the MAVS libraries to your system path. If MAVS was installed in */usr/local* as recommended above, this can be done by adding the following lines to your .bashrc file.
```bash
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib/"
export PATH="$PATH:/usr/local/lib"
```

### Building MAVS on Windows
This section discusses building Mavs on Windows. The [CMake GUI](https://cmake.org/download/) will be used to generate solutions for [Microsoft Visual Studio](https://www.visualstudio.com/downloads/). The zipped version of CMake's distributables can be run without installation with administrator privileges if necessary. When installing Visual Studio, make sure you select the C and C++ compilers for your installation.

If you wish to build the MPI version of the code (not recommended for desktop computers), you will also need to install the [Microsoft MPI compiler and SDK](https://www.microsoft.com/en-us/download/details.aspx?id=56727).

#### Installing MAVS on Windows
Create a directory in the MAVS folder called `build` and another called `install`. Start the CMake GUI and perform the following steps:

* Select the MAVS repo as the source directory and the select `./mavs/build` as the build directory.
* Click the Configure button and select the appropriate version of Visual Studio to populate the initial CMakeCache. New options should appear highlighted in red.
* For a basic OpenMP-enable MAVS build, set the following options:
  * DEFAULT_DATA_DIR: `./mavs/data`
  * CMAKE_INSTALL_PREFIX: `./mavs/install`
    * It is recommended that you install MAVS in your mavs/install folder. Administrator privileges are needed to install MAVS to the default location in `Program Files`.
  * CMAKE_CONFIGURATION_TYPE: Release
  * Unless you wish to build Chrono, uncheck "MAVS_USE_CHRONO"
  * Unless you wish to use MAVS on a supercomputer, uncheck USE_MPI and make sure USE_OMP is checked.    
* Configure once more
* Click Generate to generate solutions
* Click Open Project to open the solutions in Visual Studio
* Build the INSTALL solution to build and install MAVS

**If you get build errors related to ReactPhysics3D (RP3D)**: In your CMake dialog box, uncheck "BUILD_SHARED_LIBS", reconfigure, and rebuild.

You may need to [add the library install location to your system path](./InstallingMavsBinaries.md) for MAVS to run correctly. This sometimes requires a restart of your computer.

To add a shortcut to the MAVS GUI to your Desktop, open the file browser and navigate to <MAVS_INSTALL_DIR>/bin. Right click on *mavs_gui* and select *Send to->Desktop (create shortcut)*.

## Features
MAVS can automatically generate random ecosystems complete with trails and realistic vegetation.
![forest](docs/screenshots/mrzr_forest.png)
![desert](docs/screenshots/mavs_desert.png)

MAVS can also simulate environmental features like rain and dust and their influence on sensors.
![rain](docs/screenshots/warthog_fog.png)
![dust](docs/screenshots/forester_snow.png)

