# Swarmathon-Cabrillo

This repository contains Cabrillo College's 2018 & 2019 submissions to the [NASA Swarmathon](http://www.nasaswarmathon.com), a national swarm robotics competition.

The base code provided to participating schools can be found here [SwarmBaseCode-ROS](https://github.com/BCLab-UNM/SwarmBaseCode-ROS).

This repository contains:

1. Source code for ROS packages that control different aspects of the Swarmie robot, including robot behaviours such as search strategies and obstacle avoidance, diagnostics, and user interface elements. 
2. 3D .STL models for the physical Swarmie build 
3. Bash shell scripts and launch files for initializing simulated Swarmies in the Gazebo simulator, as well as physical Swarmies

Be sure you are using the latest drivers for your video card using the "additional drivers tool" in Ubuntu. Gazebo client often does not do well with the open source drivers.

![Alt text](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/readmeImages/InstallGraphics.png "Additional Drivers")

### Quick Start Installation Guide

SwarmBaseCode-ROS is designed and tested exclusively on the 64 bit version of Ubuntu 16.04 LTS (Xenial Xerus) and ROS Kinetic Kame. Other systems are untested and are therefore not supported at this time.

##### 1. Install ROS Kinetic Kame

Detailed instructions for installing ROS Kinetic Kame under Ubuntu 16.04 [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) or follow the summarized instructions below:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt update
sudo apt install ros-kinetic-desktop-full
sudo rosdep init
rosdep update      # Note this is not run with sudo
```

Note: if you accidentally ran ```sudo rosdep update``` you can repair the permissions ```sudo rosdep fix-permissions```.

Most systems will already have usb development support installed, but just in case:

```
 sudo apt install libusb-dev
```

##### 2. Environment Setup
From ROS Wiki: It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

##### 3. Install additional ROS packages

We use [catkin_tools](https://catkin-tools.readthedocs.io/) to build:

```
sudo apt install python-rosinstall python-catkin-tools
```

We use [robot_localization](http://wiki.ros.org/robot_localization) for state estimation and sensor fusion:

```
sudo apt install ros-kinetic-robot-localization
```

In simulation, we use [hector_gazebo_plugins](http://wiki.ros.org/hector_gazebo_plugins) to provide sonar and IMU sensors:

```
sudo apt install ros-kinetic-hector-gazebo-plugins
```

The Swarmies can receive commands from the thumb sticks on a Microsoft Xbox 360 controller. The ROS [joystick_drivers](http://wiki.ros.org/joystick_drivers) package, which contains a generic Linux joystick driver compatible with this controller:

```
 sudo apt install ros-kinetic-joystick-drivers
```

Joystick commands can also be simulated using the direction keys (Up=I, Down=K, Left=J, Right=L) on the keyboard. The Rover GUI window must have focus for keyboard control to work.

Install the usb camera driver:
```
sudo apt install ros-kinetic-usb-cam
```

Installing additional packages summarized 
[Preinstalled Package](https://raw.githubusercontent.com/BCLab-UNM/Swarmathon-Docs/master/PreinstalledCompetitionPackages.md)
```bash
sudo apt install ros-kinetic-desktop-full python-rosinstall ros-kinetic-video-stream-opencv git libusb-dev python-catkin-tools ros-kinetic-robot-localization ros-kinetic-hector-gazebo-plugins ros-kinetic-joystick-drivers git qtcreator libcap2-bin ros-kinetic-grid-map ros-kinetic-rosserial-python ros-kinetic-rosserial-arduino ros-kinetic-usb-cam ros-kinetic-multimaster-fkie  
```

##### 4. Clone and build this repository:

1. Install git, if necessary.
    ```
    sudo apt install git
    ```

2. Clone the repository and initialize submodules
    ```
    git clone --recursive git@github.com:BCLab-UNM/Swarmathon-Cabrillo.git
    ```

3. Build:
    ```
    catkin build
    ```
    
##### 5. Run the simulation:

1. Start the GUI
  ```
  ./run.sh
  ```

The GUI will now launch.

This is the first screen of the GUI:

![Alt text](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/readmeImages/guiFirstScreen.png "Opening Screen")

Click the "Simulation Control" tab:

![Alt text](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/readmeImages/simControlTab.png "Simulation Parameters")

There are several settings you can change:

- Target Distribution: Choose between randomly generating a uniform, clustered, or power law distribution. You can also select a custom Gazebo world file; there are several pre-made world files included in the base code!

- Number of Cubes: When you select a uniform or clustered target distribution, you can select the total number of apriltag cubes placed. This is not available for the power law distribution or custom world files at this time. The point of this option is to speed up the process of placing cubes in the simulation using these distribution types.

- Ground Texture: Choose the kind of ground texture that will appear in the Gazebo simulation.

- Round Type: Preliminary round creates a 15 meter square arena and generates a default of three rovers. Final round creates a 23.1 meter square arena and generates a default of six rovers. Unbounded round does not place any barriers around the edge of the arena, it generates a default of three rovers, and you can set the size of the arena in the dropbox below the unbounded radio button. Cubes placed using the uniform, clustered, or powerlaw target distributions will use this arena size to determine random placements.

- Set Number of Rovers: Optional. If you select this checkbox, you can customize the exact number of rovers that will be created regardless of the round type. You can create from 0 to 8 rovers.

- Simulation Length: Optional. By default there is no timer set. If you select a timer from the drop down list, when you click the "All Autonomous" button the Gazebo simulation will run for the specified time before stopping all of the rovers (the equivalent of pressing the "Stop All Rovers" button). Please note that a twenty minute timer will run for twenty minutes of simulated time. That is, the amount of time that the simulation needs to simulate twenty real world minutes of time. You can look at the real time factor in Gazebo to get a feel for this, but it is generally slower than real time.

    tldr: A simulation timer for twenty minutes *might* take anywhere from thirty minutes to sixty minutes to actually complete.

- Create Savable Gazebo World: Selecting this option will create a Gazebo simulation that does not include rovers, a collection zone, or wall barriers. The point of this is to create a uniform, clustered, or power law distribution (which takes a long time to do) and then save that Gazebo world so that you can load it MUCH more quickly for testing your simulated rovers later on.

Click the "Build Simulation" button when ready.

The Gazebo physics simulator will open.

![Alt text](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/readmeImages/buildSim.png "Gazebo Simulator")

Click back to the Swarmathon GUI and select the "Sensor Display" tab.

![Alt text](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/readmeImages/sensorDisplayTab.png "Sesnor display")

Any active rovers, simulated and/or real, will be displayed in the rover list on the left side. To the right of the rover list is their corresponding connection status. Please note that the number here reflects different things for simulated vs. physical rovers:
- physical rovers: This number represents the wireless link quality and the bitrate of their connection.
- simulated rovers: This number represents their simulation rate as a percentage of real time. For example, 1.0 = moving at 100% real time factor, 0.5 = moving at 50% real time factor, etc.
- all rovers: Green = good connection. Yellow = meh connection. Red = bad connection and/or disconnected.

To the right of the connection status is a checkbox for the map frame. If you select a checkbox for a given rover, it's map information will appear in the map to the right. You can arbitrarily select any number of rovers to be displayed in the map frame.

![Alt text](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/readmeImages/activeRovers.png "Active rovers")

Select a rover to view its sensor outputs. 

![Alt text](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/readmeImages/roverSensorOutputs.png "Rover sensor outputs")

There are four sensor display frames and one settings frame:

The camera output. This is a rover eye's view of the world.

The ultrasound output is shown as three white rays, one for each ultrasound. The length of the rays indicates the distance to any objects in front of the ultrasound. The distance in meters is displayed in text below these rays. The maximum distance reported by the ultrasounds is 3 meters.  

The IMU sensor display consists of a cube where the red face is the bottom of the rover, the blue face is the top of the rover, and the red and blue bars are the front and back of the rover. The cube is viewed from the top down. The cube is positioned according to the IMU orientation data. For example, if the rover flips over, the red side will be closest to the observer. Accelerometer data is shown as a 3D vector projected into 2D space pointing towards the sum of the accelerations in 3D space.
 
The map view shows the path taken by the currently selected rover. Green is the encoder position data. In simulation, the encoder position data comes from the odometry topic being published by Gazebo's skid steer controller plugin. In the real robots, it is the encoder output. GPS points are shown as red dots. The EKF is the output of an extended Kalman filter which fuses data from the IMU, GPS, and encoder sensors.

The map settings frame contains several settings that affect the map view:
- Frame Views: You can check whether the map displays data from EKF (MAP), odometry (ODOM), and/or GPS (NAVSAT).
    Additionally, the Global Frame checkbox moves rovers from all starting at (0,0) to their actual start points (this is for simulated rovers only at this time).
- Panning: You can select whether the map automatically pans to fit all of the map data into the map frame or you can choose to manually pan the map yourself using the mouse to click-and-drag and also zooming with the scroll wheel.
- Unique Rover Colors: Select this option to change the colors in the map frame to display unique rover colors. This will help people to tell the rover paths apart in the map frame after long simulation runs.
- Popout: Click on this box to generate a popout map window that you can maximize or resize however you want. The popout map retains all of the settings you had selected in the original map frame. Changing settings in the RQT GUI will also change things in your popout map.

Look on the left hand side of the RQT Rover GUI.

![Alt text](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/readmeImages/taskStatusTab.png "Task Status tab")

This section displays important status information:

1. (all rovers) The currently selected rover in the RQT GUI.
2. (physical rovers) The number of GPS satellites detected.
3. (all rovers) The number of obstacle avoidance calls.
4. (simulated rovers) The number of targets collected.
5. (simulated rovers) The current status of the simulation timer after selecting a timer length and clicking the "All Autonomous" button.

To close the simulation and the GUI, click the red exit button in the top left-hand corner.

### Software Documentation

Source code can be found in the repository /src directory. This directory contains several subdirectories, each of which contain a single ROS package. Here we present a high-level description of each package.

- `abridge`: A serial interface between SwarmBaseCode-ROS and the A-Star 32U4 microcontroller onboard the physical robot. In the simulation, `abridge` functionality is supplanted by `sbridge`, which interfaces with [gazebo_ros_skid_steer_drive](http://docs.ros.org/kinetic/api/gazebo_plugins/html/classgazebo_1_1GazeboRosSkidSteerDrive.html) (motor and encoders) and [hector_gazebo_plugins](http://wiki.ros.org/hector_gazebo_plugins) (sonar and IMU; see [step 3](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/README.md#3-install-additional-gazebo-plugins) of the Quick Start guide).
- `rqt_rover_gui`: A Qt-based graphical interface for the physical and simulated robots. See [How to use Qt Creator](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/README.md#how-to-use-qt-creator-to-edit-the-simulation-gui) for details on this package.
- `mobility`
- `mapping`
- `diagnostics`
- `apriltags2_ros`: An image processor that detects [AprilTag](https://april.eecs.umich.edu/wiki/index.php/AprilTags) fiducial markers in the onboard camera's video stream. This package receives images from the `usb_cam` node (for physical robots) or [gazebo_ros_camera](http://docs.ros.org/kinetic/api/gazebo_plugins/html/classgazebo_1_1GazeboRosCamera.html) (for simulated robots).
- `ublox`: An unused dependency of `rqt_rover_gui`.

### Robot Behaviors
Behavior code is launched by the task manager. The task manager is a state machine that implements the phases of the robot operation during the competition. The task states are shown below:

![Figure of task manager state machine](https://github.com/BCLab-UNM/Swarmathon-Cabrillo/blob/master/readmeImages/statemachine.jpg)

The task manager operates as a single ROS node, starting each behavior importing it as a Python module and calling its `main()` function. Separating tasks into individual modules eliminates interaction between behaviors, reducing programmer error due to unforeseen consequences. Further, each behavior can be tested in isolation. For example, you can place the rover in front of some blocks then manually launch the pick-up node. This experiment can be repeated quickly and enables programmers to write more thoroughly tested code.

* `init` Initializes the rover for a round, saving the collection area location in the odometry coordinate frame.
* `search` Implements a correlated random walk as a basic
  search method. The behavior also saves resource locations as they are discovered, so the rover can drive directly back to them.
* `pickup` Picks up a cube with the rover's claw.
* `gohome` Drives the rover back to the collection area, planning a path around any obstacles currently on the map.
* `dropoff` Drops a cube off inside the collection area.

