//////////////////////////
////Required Libraries////
//////////////////////////

//Built-in Arduino libraries
#include <Wire.h>

//Custom libraries located in arduino directory
#include <L3G.h>
#include <LPS.h>
#include <LSM303.h>
#include <Movement.h>
#include <NewPing.h>
#include <Odometry.h>
#include <Servo.h>

// Constants
// #define PI 3.14159265358979323846
#define RAD2DEG(radianAngle) (radianAngle * 180.0 / PI)
#define DEG2RAD(degreeAngle) (degreeAngle * PI / 180.0)

////////////////
////Settings////
////////////////

//Gripper (HS-485HB Servo)
byte fingersPin = 9;
byte wristPin = 12;
int fingerMin = 800; //if you want to shift 0 to a new location raise min; this is closed
int fingerMax = 2600; //if you want to limit max travel lower max; this is open
int wristMin = 1400; //this is up
int wristMax = 2600; //this is down

//Movement (VNH5019 Motor Driver Carrier)
byte rightDirectionA = A3; //"clockwise" input
byte rightDirectionB = A2; //"counterclockwise" input
byte rightSpeedPin = 11; //PWM input
byte leftDirectionA = A5; //"clockwise" input
byte leftDirectionB = A4; //"counterclockwise" input
byte leftSpeedPin = 10; //PWM input

//Odometry (8400 CPR Encoder)
byte rightEncoderA = 7;
byte rightEncoderB = 8;
byte leftEncoderA = 0;
byte leftEncoderB = 1;

//Serial (USB <--> Intel NUC)
String rxBuffer;
unsigned long watchdogTimer = 1000; //fail-safe in case of communication link failure (in ms)
unsigned long lastCommTime = 0; //time of last communication from NUC (in ms)

//Ultrasound (Ping))))
byte leftSignal = 4;
byte centerSignal = 5;
byte rightSignal = 6;


////////////////////////////
////Class Instantiations////
////////////////////////////

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


/////////////
////Setup////
/////////////

void setup()
{
  Serial.begin(115200);
  while (!Serial) {} //wait for Serial to complete initialization before moving on

  Wire.begin();

  if (imuStatus()) {
    imuInit();
  }

  fingers.attach(fingersPin,fingerMin,fingerMax);
  fingers.writeMicroseconds(fingerMin);
  wrist.attach(wristPin,wristMin,wristMax);
  wrist.writeMicroseconds(wristMin);

  rxBuffer = "";
}


/////////////////
////Main Loop////
/////////////////

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == ',' || c == '\n') {
      parse();
      rxBuffer = "";
      lastCommTime = millis();
    }
    else if (c > 0) {
      rxBuffer += c;
    }
  }
  if (millis() - lastCommTime > watchdogTimer) {
    move.stop();
  }
}


////////////////////////
//Parse receive buffer//
////////////////////////

void parse() {
  if (rxBuffer == "v") {
    int speedL = Serial.parseInt();
    int speedR = Serial.parseInt();
    
    if (speedL >= 0 && speedR >= 0) {
      move.forward(speedL, speedR);
    }
    else if (speedL <= 0 && speedR <= 0) {
      move.backward(speedL*-1, speedR*-1);
    }
    else if (speedL <= 0 && speedR >= 0) {
      move.rotateLeft(speedL*-1, speedR);
    }
    else {
      move.rotateRight(speedL, speedR*-1);
    }
  }
  else if (rxBuffer == "s") {
    move.stop();
  }
  else if (rxBuffer == "d") {
    static int ping_state = 0;
    static int leftUSValue = 300;
    static int rightUSValue = 300;
    static int centerUSValue = 300;

    Serial.print("GRF,");
    Serial.print(String(fingers.attached()) + ",");
    if (fingers.attached()) { // if fails, maybe print nothing?
      Serial.println(String(DEG2RAD(fingers.read())));
    }
    else {
      Serial.println();
    }

    Serial.print("GRW,");
    Serial.print(String(wrist.attached()) + ",");
    if (wrist.attached()) {
      Serial.println(String(DEG2RAD(wrist.read())));
    }
    else {
      Serial.println();
    }

    if (imuStatus()) {
      imuInit(); 
      Serial.println(updateIMU());
    }

    Serial.println("ODOM,1," + updateOdom());

    // Only do one sonar at a time to prevent crosstalk.
    if (ping_state == 0) {
      leftUSValue = NewPing::convert_cm(leftUS.ping_median(3));
      // leftUSValue = leftUS.ping_cm();
      // new code
      if(leftUSValue > 0.015) // hack for checking that the sonar is plugged in
	
	Serial.println("USL,1," + String(leftUSValue));
    }
    else if (ping_state == 1) {
      rightUSValue = NewPing::convert_cm(rightUS.ping_median(3));
      //rightUSValue = rightUS.ping_cm();
      if(rightUSValue > 0.015)
	Serial.println("USR,1," + String(rightUSValue));
    }
    else{
      centerUSValue = NewPing::convert_cm(centerUS.ping_median(3));
      //centerUSValue = centerUS.ping_cm();
      if(centerUSValue > 0.015)
	Serial.println("USC,1," + String(centerUSValue));
    }
    ping_state = (ping_state + 1) % 3;

  }
  else if (rxBuffer == "f") {
    float radianAngle = Serial.parseFloat();
    int angle = RAD2DEG(radianAngle); // Convert float radians to int degrees
    angle = fingerMin + (fingerMax/370) * angle;
    fingers.writeMicroseconds(angle);
  }
  else if (rxBuffer == "w") {
    float radianAngle = Serial.parseFloat();
    int angle = RAD2DEG(radianAngle); // Convert float radians to int degrees
    angle = wristMin + (wristMax/370) * angle;
    wrist.writeMicroseconds(angle);
  }
}


//////////////////////////
//Update transmit buffer//
//////////////////////////

String updateIMU() {  
  //Update current sensor values
  gyroscope.read();
  magnetometer_accelerometer.read();

  if (!gyroscope.timeoutOccurred() && !magnetometer_accelerometer.timeoutOccurred()) {
    //Collect updated values
    LSM303::vector<int16_t> acc = magnetometer_accelerometer.a;
    L3G::vector<int16_t> gyro = gyroscope.g;
    LSM303::vector<int16_t> mag = magnetometer_accelerometer.m;

    //Append data to buffer
    String txBuffer = String("IMU,1,") +
               String(acc.x) + "," +
               String(acc.y) + "," +
               String(acc.z) + "," +
               String(mag.x) + "," +
               String(mag.y) + "," +
               String(mag.z) + "," +
               String(gyro.x) + "," +
               String(gyro.y) + "," +
               String(gyro.z);

    return txBuffer;
  }

  return "";
}

String updateOdom() {
  String txBuffer;
  odom.update();

  txBuffer = String(odom.left) + "," +
             String(odom.right) + "," +
             String(odom.clock);

  return txBuffer;
}


////////////////////////////
////Initializer Functions///
////////////////////////////

//Initialize gyroscope, accelerometer, magnetometer, and pressure gauge
void imuInit() {
  gyroscope.init();
  gyroscope.enableDefault();
  gyroscope.setTimeout(1);

  magnetometer_accelerometer.init();
  magnetometer_accelerometer.enableDefault();

  magnetometer_accelerometer.setTimeout(1);

  pressure.init();
  pressure.enableDefault();
}


////////////////////////////
////Diagnostic Functions////
////////////////////////////

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
