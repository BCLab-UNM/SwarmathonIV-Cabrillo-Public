#define USE_USBCON

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

LSM303 lsm303;
L3G l3g;

ros::NodeHandle_<ArduinoHardware, 1, 3, 125, 125> nh;

std_msgs::Int32MultiArray lsm303_msg;
long int lsm303_data[6];
ros::Publisher lsm303_pub("/lsm303", &lsm303_msg);

std_msgs::Int32MultiArray l3g_msg;
long int l3g_data[3];
ros::Publisher l3g_pub("/l3g", &l3g_msg);

std_msgs::String status_msg;
ros::Publisher status("status", &status_msg);
char okay[] = "ok";
char e_iic[] = "E: iic";

void setup()
{
  nh.initNode();

  lsm303_msg.data = (long int *) &lsm303_data;
  lsm303_msg.data_length = 6;
  nh.advertise(lsm303_pub);
  nh.spinOnce();

  l3g_msg.data = (long int *) &l3g_data;
  l3g_msg.data_length = 3;
  nh.advertise(l3g_pub);
  nh.spinOnce();

  nh.advertise(status);
  nh.spinOnce();

  Wire.begin();

  /* Initialize the sensors */
  if(iicScan() > 0) {
	status_msg.data = okay;

    lsm303.init();
    lsm303.enableDefault();
    lsm303.setTimeout(1);

    l3g.init();
    l3g.enableDefault();
    l3g.setTimeout(1);

    // Enable temperature sensor
    byte ctrl5 = lsm303.readReg(0x24);
    ctrl5 |= 0x80;

    // Set the mag data rate to 50Hz
    ctrl5 &= ~0b00011100;
    ctrl5 |=  0b00010000;

    lsm303.writeReg(0x24, ctrl5);

    // Set the acc data rate to 50Hz
    byte ctrl1 = lsm303.readReg(0x20);
    ctrl1 &= ~0b11110000;
    ctrl1 |=  0b01010000;

    // Enable BDU - Synchronized conversion.
    ctrl1 |= 0b00001000;

    lsm303.writeReg(0x20, ctrl1);

    // Set BDU on the gyro
    //byte ctrl4 = l3g.readReg(0x23);
    //ctrl4 |= 0b10000000;
    //l3g.writeReg(ctrl4, 0x23);
  }
}

int iicScan() {
  byte numberOfDevices = 0;

  for(byte address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (!error) {
      numberOfDevices++;
    }
  }

  return numberOfDevices;
}

void loop()
{
  if (iicScan() > 0) {
	status_msg.data = okay;

	l3g.read();
    lsm303.read();

    LSM303::vector<int16_t> acc = lsm303.a;
    LSM303::vector<int16_t> mag = lsm303.m;

    lsm303_data[0] = mag.x;
    lsm303_data[1] = mag.y;
    lsm303_data[2] = mag.z;
    lsm303_data[3] = acc.x;
    lsm303_data[4] = acc.y;
    lsm303_data[5] = acc.z;

    lsm303_pub.publish(&lsm303_msg);

    nh.spinOnce();

    L3G::vector<int16_t> gyro = l3g.g;

    l3g_data[0] = gyro.x;
    l3g_data[1] = gyro.y;
    l3g_data[2] = gyro.z;

    l3g_pub.publish(&l3g_msg);
  }
  else {
	  status_msg.data = e_iic;
  }

  status.publish( &status_msg );

  nh.spinOnce();
}
