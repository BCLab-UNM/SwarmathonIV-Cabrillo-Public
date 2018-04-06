This guide takes you through the installation of Ubuntu 16.04 server on a swarmie. 

You'll need the following: 
1. A USB bood flash with Ubuntu 16.04 LTS
2. A monitor that accepts HDMI and a cable. 
3. A keyboard (mouse optional)
4. A wired network connection

# Install Ubuntu

Plug the flash key, monitor and keyboard. Start the NUC and press F10 to get the boot menu. The flash key will show
up twice in the menu. Pick the UEFI option. Follow the prompts to install Ubuntu. Use the default setting unless you 
see an exception in the list below.

Here are some noatble things to watch for:
1. Set the hostname to the name of the swarmie in all lower case (e.g. lovelace)
2. Set username of the first user to "robot" password "Cabri11o"
3. Unmount the partitions in use: YES 
4. Partition Disks: Select "Guided - use entire disk" (be sure to select the default 36G disk in the next screen)
5. Write changes to disk: YES
6. Software selection: Select OpenSSH server and continue

After installation is complete the NUC will reboot. Login with the robot user. Now update the software on the system:
```
sudo apt-get update
sudo apt-get dist-upgrade
```
After the process completes reboot again. 

# Connect the WiFi

In order to connect the WiFi you'll have to install some packages. Do the follwing steps with the Ethernet still plugged in. 
Afterwards you'll be able to reboot and be on the special Swarmathon WiFi.

```
sudo apt-get install wpasupplicant wireless-tools
```

Now replace the ENTIRE contents of the ```/etc/network/interfaces``` file with this:
```
# This file describes the network interfaces available on your system
# and how to activate them. For more information, see interfaces(5).

source /etc/network/interfaces.d/*

# The loopback network interface
auto lo
iface lo inet loopback

# The primary network interface
# auto eno1
iface eno1 inet dhcp

auto wlp2s0  
iface wlp2s0 inet dhcp
        wpa-ssid Swarmathon
        wpa-psk Cabri11o

iface wlp2s0 inet6 auto
```

Unplug the Ethernet and reboot the rover. You should have a WiFi connection. Test it with this command:

```
ping www.google.com
```

# Install ROS
Now you must install the Robot OS packages. Start by enabling the ROS repos:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Now setup keys:
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
This fetches the new package lists:
```
sudo apt-get update
```
Now you can install all of ROS: 
```
sudo apt-get install ros-kinetic-desktop-full
```
This will take a while! After it's done you need to initizlize ROS:
```
sudo rosdep init
rosdep update
```
Now make sure that the robot user knows where to find ROS files:
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Now install catkin and other ROS tools and libraries:
```
sudo apt-get install python-catkin-tools ros-kinetic-robot-localization ros-kinetic-hector-gazebo-plugins ros-kinetic-joystick-drivers avahi-daemon ros-kinetic-zeroconf-avahi ros-kinetic-video-stream-opencv emacs-nox
```

# Install Cryptographic Secrets
Talk to Mike about how to download the SSH identity onto the new rover. 

# Setup the Swarmathon Repo 
When you have the secrets installed you can connect to the Cabrillo private repo. 

Clone this GitHub repository to your home directory (~), renaming the repo so ROS and catkin can properly identify it:

```
cd ~
git clone git@github.com:BCLab-UNM/Swarmathon-Cabrillo.git rover_workspace
```

Change your current working directory to the root directory of the downloaded repo:

```
cd ~/rover_workspace
```

Check out the master-next branch for Xenial. 
```
git checkout master-next
```

Set up [ublox](http://wiki.ros.org/ublox) GPS submodule:

```
git submodule init
git submodule update
```
Build the project:
```
catkin build
```
Make sure that the project is in your path:
```
echo "source ~/rover_workspace/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
# Setup Hardware Permissions 
You must give the robot user access to the hardware peripherals that drive the robtot. Adding the user to the dialout 
gives them permission to modem-like things. The two Arduinos on the rover appear to be modems. 
```
sudo usermod -a -G dialout,video robot
```
You will have to logout and back in again for this change to take effect. 

# Install Arduino
It's extremely helpful to have the Arduino software installed on the Swarmie. 
```
cd /tmp
wget https://downloads.arduino.cc/arduino-1.8.3-linux64.tar.xz
cd /opt
sudo tar -xvf /tmp/arduino-1.8.3-linux64.tar.xz 
sudo ln -s arduino-1.8.3/ arduino
```
Verify that you can upload code to the Arduino:
```
~/Swarmathon-Cabrillo/misc/build_arduino.sh 
```
