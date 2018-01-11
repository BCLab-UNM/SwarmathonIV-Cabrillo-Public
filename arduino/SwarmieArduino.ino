#include <ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>

// Arduino Libraries
#include <Wire.h>

// Bundled Libraries
#include <L3G.h>
#include <LPS.h>
#include <LSM303.h>
#include <Movement.h>
#include <NewPing.h>
#include <Odometry.h>
#include <Servo.h>
#include <EEPROM.h>
#include <CRC32.h>
#include <FastPID.h>

// Local components
#include <Calibration.h>

#define RAD2DEG(radianAngle) (radianAngle * 180.0 / PI)
#define DEG2RAD(degreeAngle) (degreeAngle * PI / 180.0)

#define DRIVE_MODE_STOP 0
#define DRIVE_MODE_PID  1

byte fingersPin = 9;
byte wristPin = 12;
int fingerMin = 800; //if you want to shift 0 to a new location raise min; this is closed
int fingerMax = 2600; //if you want to limit max travel lower max; this is open
int wristMin = 1400; //this is up
int wristMax = 2600; //this is down

// Movement (VNH5019 Motor Driver Carrier)
byte rightDirectionA = A3; //"clockwise" input
byte rightDirectionB = A2; //"counterclockwise" input
byte rightSpeedPin = 11; //PWM input
byte leftDirectionA = A5; //"clockwise" input
byte leftDirectionB = A4; //"counterclockwise" input
byte leftSpeedPin = 10; //PWM input
float feed_forward = 0;

// Odometry (8400 CPR Encoder)
byte rightEncoderA = 7;
byte rightEncoderB = 8;
byte leftEncoderA = 0;
byte leftEncoderB = 1;
float wheelBase = 27.8; //distance between left and right wheels (in cm)
float wheelDiameter = 12.2; //diameter of wheel (in cm)
int cpr = 8400; //"cycles per revolution" -- number of encoder increments per one wheel revolution

// Ultrasound (Ping))))
byte leftSignal = 4;
byte centerSignal = 5;
byte rightSignal = 6;

// Peripherals
L3G gyroscope;
LSM303 magnetometer_accelerometer;
LPS pressure;
Movement move = Movement(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);
Odometry odom = Odometry(rightEncoderA, rightEncoderB, leftEncoderA, leftEncoderB);
Servo fingers;
Servo wrist;
NewPing leftUS(leftSignal, leftSignal, 330);
NewPing centerUS(centerSignal, centerSignal, 330);
NewPing rightUS(rightSignal, rightSignal, 330);

// PIDs
FastPID left_pid;
FastPID right_pid;

void imuInit();
bool imuStatus();
void updateIMU();
void driveHandler(const geometry_msgs::Twist& message);
void fingerHandler(const std_msgs::Float32& message);
void wristHandler(const std_msgs::Float32& message);
void start_calibration(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &rsp);
void store_calibration(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &rsp);
float ticksToMeters(float ticks);
float metersToTicks(float meters);
float diffToTheta(float right, float left);
float thetaToDiff(float theta);
void do_drive();
void do_sonar();

ros::NodeHandle nh;

// Messages have to be statically allocated in Arduino
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;
sensor_msgs::Range left_sonar_msg;
sensor_msgs::Range right_sonar_msg;
sensor_msgs::Range center_sonar_msg;

geometry_msgs::Twist drive_command;

// Publishers
ros::Publisher imu_pub("/imu", &imu_msg);
ros::Publisher odom_pub("/odom", &odom_msg);
ros::Publisher left_sonar_pub("/sonarLeft", &left_sonar_msg);
ros::Publisher right_sonar_pub("/sonarRight", &right_sonar_msg);
ros::Publisher center_sonar_pub("/sonarCenter", &center_sonar_msg);

// Subscribers
ros::Subscriber<geometry_msgs::Twist> drive_sub("/driveControl", &driveHandler);
ros::Subscriber<std_msgs::Float32> finger_sub("/fingerAngle", &fingerHandler);
ros::Subscriber<std_msgs::Float32> wrist_sub("/wristAngle", &wristHandler);

// Services
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> start_cal_srv("/start_magnetometer_calibration", &start_calibration);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> store_cal_srv("/store_magnetometer_calibration", &store_calibration);

void setup() {
	nh.initNode();

	Wire.begin();

	imuInit();

	nh.advertise(imu_pub);
	nh.advertise(odom_pub);
	nh.advertise(left_sonar_pub);
	nh.advertise(right_sonar_pub);
	nh.advertise(center_sonar_pub);

	nh.subscribe(drive_sub);
	nh.subscribe(finger_sub);
	nh.subscribe(wrist_sub);

	nh.advertiseService(start_cal_srv);
	nh.advertiseService(store_cal_srv);

	// Wait for the nodehandle to connect.
	while(!nh.connected()) {
		nh.spinOnce();
	}

	// Get PID parameters.
	float scale, p, i, d, db, st, wu;
	nh.getParam("scale", &scale);
	nh.getParam("Kp", &p);
	nh.getParam("Ki", &i);
	nh.getParam("Kd", &d);
	nh.getParam("db", &db);
	nh.getParam("st", &st);
	nh.getParam("wu", &wu);
	nh.getParam("ff", &feed_forward);

	left_pid.configure(p, i, d, db, 16, true);
	right_pid.configure(p, i, d, db, 16, true);
}

void loop() {
	do_sonar();
	do_drive();
	updateIMU();
	nh.spinOnce();
}

// Drive functions

float ticksToMeters(float ticks) {
	return (M_PI * wheelDiameter * ticks) / cpr;
}

float metersToTicks(float meters) {
	return (meters * cpr) / (M_PI * wheelDiameter);
}

float diffToTheta(float right, float left) {
	return (right - left) / wheelBase;
}

float thetaToDiff(float theta) {
	return theta * wheelBase;
}

void do_drive() {
    static double odomTheta = 0;
    static double lastOdomTS = 0;

    odom.update();

	int cmd_mode = round(drive_command.angular.y);

	if (cmd_mode == DRIVE_MODE_STOP) {
		left_pid.clear();
		right_pid.clear();
	    move.stop();
	}
	else {
		// FIXME: Change this to integer math for performance.

		int linear_sp = metersToTicks(drive_command.linear.x);
		int angular_sp = metersToTicks(thetaToDiff(drive_command.angular.z));

		int left_sp = linear_sp - angular_sp;
		int right_sp = linear_sp + angular_sp;

		int l = left_pid.step(left_sp, odom.left);
		int r = right_pid.step(right_sp, odom.right);

		// Feed forward
		l += feed_forward * left_sp;
		r += feed_forward * right_sp;

	    if (l >= 0 && r >= 0) {
	      move.forward(l, r);
	    }
	    else if (l <= 0 && r <= 0) {
	      move.backward(l*-1, r*-1);
	    }
	    else if (l <= 0 && r >= 0) {
	      move.rotateLeft(l*-1, r);
	    }
	    else {
	      move.rotateRight(l, r*-1);
	    }
	}

	// Publish wheel odometry

	float rightWheelDistance = ticksToMeters(odom.right);
	float leftWheelDistance = ticksToMeters(odom.left);

    //Calculate relative angle that robot has turned
	float dtheta = diffToTheta(rightWheelDistance, leftWheelDistance);

    //Accumulate angles to calculate absolute heading
    odomTheta += dtheta;

    //Decompose linear distance into its component values
    float meanWheelDistance = (rightWheelDistance + leftWheelDistance) / 2;
    float x = meanWheelDistance * cos(dtheta);
    float y = meanWheelDistance * sin(dtheta);

    // Calculate velocities if possible.
    float vtheta = 0;
    float vx = 0;
    float vy = 0;
    if (lastOdomTS > 0) {
    	float deltaT = odom.clock - lastOdomTS;
    	vtheta = dtheta / deltaT;
    	vx = x / deltaT;
    	vy = y / deltaT;
    }
	lastOdomTS = odom.clock;

	odom_msg.header.stamp = nh.now();
	odom_msg.pose.pose.position.x += x;
	odom_msg.pose.pose.position.y += y;
	odom_msg.pose.pose.position.z = 0;
	odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(odomTheta);
	odom_msg.twist.twist.linear.x = vx;
	odom_msg.twist.twist.linear.y = vy;
	odom_msg.twist.twist.angular.z = vtheta;

	odom_pub.publish(&odom_msg);
}

void do_sonar() {

    left_sonar_msg.header.frame_id = "/left_us";
    left_sonar_msg.header.stamp = nh.now();
    left_sonar_msg.range = leftUS.ping_cm();
    left_sonar_pub.publish(&left_sonar_msg);

    right_sonar_msg.header.frame_id = "/right_us";
    right_sonar_msg.header.stamp = nh.now();
    right_sonar_msg.range = rightUS.ping_cm();
    right_sonar_pub.publish(&right_sonar_msg);

    center_sonar_msg.header.frame_id = "/center_us";
    center_sonar_msg.header.stamp = nh.now();
    center_sonar_msg.range = centerUS.ping_cm();
    center_sonar_pub.publish(&center_sonar_msg);
}

// IMU Functions
void imuInit() {
  gyroscope.init();
  gyroscope.enableDefault();
  gyroscope.setTimeout(1);

  magnetometer_accelerometer.init();
  magnetometer_accelerometer.enableDefault();

  cal_t stored_cal;
  if (readCalibration(stored_cal)) {
    stored_calibration_used = true;
    magnetometer_accelerometer.m_min = (LSM303::vector<int16_t>)
      { stored_cal.data.x_min, stored_cal.data.y_min, stored_cal.data.z_min };
    magnetometer_accelerometer.m_max = (LSM303::vector<int16_t>)
      { stored_cal.data.x_max, stored_cal.data.y_max, stored_cal.data.z_max };
  }else{
    magnetometer_accelerometer.m_min = (LSM303::vector<int16_t>){ -2247,  -2068,  -1114};
    magnetometer_accelerometer.m_max = (LSM303::vector<int16_t>){+3369,  +2877,  +3634};
  }

  magnetometer_accelerometer.setTimeout(1);

  pressure.init();
  pressure.enableDefault();
}

// IMU Functions

//Check for valid I2C connection to verify IMU
bool imuStatus() {
  byte numberOfDevices = 0;

  for(byte address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (!error) {
      numberOfDevices++;
    }
  }

  if (numberOfDevices > 0) {
    return true;
  }
  else {
    return false;
  }
}

void updateIMU() {
  //Update current sensor values
  gyroscope.read();
  magnetometer_accelerometer.read();

  if (!gyroscope.timeoutOccurred() && !magnetometer_accelerometer.timeoutOccurred()) {
    //Collect updated values
    LSM303::vector<int16_t> acc = magnetometer_accelerometer.a;
    L3G::vector<int16_t> gyro = gyroscope.g;
    LSM303::vector<int16_t> mag = magnetometer_accelerometer.m;

    // Update runtime calibration
    if (calibration.data.x_min > mag.x)
      calibration.data.x_min = mag.x;
    if (calibration.data.x_max < mag.x)
      calibration.data.x_max = mag.x;

    if (calibration.data.y_min > mag.y)
      calibration.data.y_min = mag.y;
    if (calibration.data.y_max < mag.y)
      calibration.data.y_max = mag.y;

    if (calibration.data.z_min > mag.z)
      calibration.data.z_min = mag.z;
    if (calibration.data.z_max < mag.z)
      calibration.data.z_max = mag.z;

    //Convert accelerometer digits to milligravities, then to gravities, and finally to meters per second squared
    LSM303::vector<float> linear_acceleration = {acc.y*0.061/1000*9.81, -acc.x*0.061/1000*9.81, acc.z*0.061/1000*9.81};

    //Convert gyroscope digits to millidegrees per second, then to degrees per second, and finally to radians per second
    L3G::vector<float> angular_velocity = {gyro.y*8.75/1000*(PI/180), -gyro.x*8.75/1000*(PI/180), gyro.z*8.75/1000*(PI/180)};

    //Combine normalized magnetometer and accelerometer digits to produce Euler angles, i.e. pitch, roll, and yaw
    LSM303::vector<float> orientation = {(float)mag.x, (float)mag.y, (float)mag.z};
    orientation.x -= (magnetometer_accelerometer.m_min.x + magnetometer_accelerometer.m_max.x) / 2;
    orientation.y -= (magnetometer_accelerometer.m_min.y + magnetometer_accelerometer.m_max.y) / 2;
    orientation.z -= (magnetometer_accelerometer.m_min.z + magnetometer_accelerometer.m_max.z) / 2;
    LSM303::vector_normalize(&orientation);
    float roll = atan2(linear_acceleration.y, sqrt(pow(linear_acceleration.x,2) + pow(linear_acceleration.z,2)));
    float pitch = -atan2(linear_acceleration.x, sqrt(pow(linear_acceleration.y,2) + pow(linear_acceleration.z,2)));
    float yaw = atan2(-orientation.y*cos(roll) + orientation.z*sin(roll), orientation.x*cos(pitch) + orientation.y*sin(pitch)*sin(roll) + orientation.z*sin(pitch)*cos(roll)) + PI;

    imu_msg.header.frame_id = "base_link";
    imu_msg.header.stamp = nh.now();
    imu_msg.linear_acceleration.x = linear_acceleration.x;
    imu_msg.linear_acceleration.y = linear_acceleration.y;
    imu_msg.linear_acceleration.y = linear_acceleration.z;
    imu_msg.angular_velocity.x = angular_velocity.x;
    imu_msg.angular_velocity.y = angular_velocity.y;
    imu_msg.angular_velocity.z = angular_velocity.z;

    // Convert RPY to quaternion
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);

	imu_msg.orientation.w = cy * cr * cp + sy * sr * sp;
	imu_msg.orientation.x = cy * sr * cp - sy * cr * sp;
	imu_msg.orientation.y = cy * cr * sp + sy * sr * cp;
	imu_msg.orientation.z = sy * cr * cp - cy * sr * sp;

	imu_pub.publish(&imu_msg);
  }
}

void driveHandler(const geometry_msgs::Twist& message) {
	drive_command.linear.x = message.linear.x;
	drive_command.angular.z = message.angular.z;
	drive_command.angular.y = message.angular.y;
}

void fingerHandler(const std_msgs::Float32& message) {
	int angle = RAD2DEG(message.data); // Convert float radians to int degrees
    angle = fingerMin + (fingerMax/370) * angle;
    fingers.writeMicroseconds(angle);
}

void wristHandler(const std_msgs::Float32& message) {
    int angle = RAD2DEG(message.data); // Convert float radians to int degrees
    angle = wristMin + (wristMax/370) * angle;
    wrist.writeMicroseconds(angle);
}

void start_calibration(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &rsp) {
	clearObservedCalibration();
}

void store_calibration(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &rsp) {
	commitCalibration();
}
