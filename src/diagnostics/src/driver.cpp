// Driver for the diagnostics module. Provides an start point for the OS.
// This driver's only job is to instantiate the Diagnostics class, handle ROS initialization,
// allow ROS to handle events (spin), and determine the name to publish under (command line
// argument or, if none, the hostname).
//  

#include "Diagnostics.h"
#include <string>
#include <signal.h>
#include <gazebo/gazebo.hh>

using namespace std;

int main(int argc, char** argv) {

  ros::init(argc, argv, "diagnostics");
    
  Diagnostics diags;
  ros::spin(); // Process ROS events
  
  return EXIT_SUCCESS;
}
