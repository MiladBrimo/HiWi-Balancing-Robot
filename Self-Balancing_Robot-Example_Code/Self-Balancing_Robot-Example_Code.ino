//  Self-balancing Robot - Example code
//  This code controls a self-balancing robot using an MPU6050 IMU, motor encoders, and PD/PI controllers.
//  Author: Milad Brimo
//  Last Updated: 18.02.2025.


#include <Wire.h>                //Library for I2C communication (Wire)
#include <MPU6050.h>             //Library for MPU6050 by Jeff Rowberg
#include <PinChangeInterrupt.h>  //Library for interrupt pin changes by NicoHood
#include <Adafruit_NeoPixel.h>   //Library for WS2812B by Adafruit


//Motor pins
#define A_DIR_PIN 7   //Direction pin for Motor A
#define A_SPD_PIN 5   //Speed (PWM) pin for Motor A
#define B_DIR_PIN 12  //Direction pin for Motor B
#define B_SPD_PIN 6   //Speed (PWM) pin for Motor B
#define ENABLE_PIN 8  //Enable pin for motor driver


//Encoder pins
#define A_ENC_PIN 2           //Hall encoder pin for Motor A
#define B_ENC_PIN 4           //Hall encoder pin for Motor B
volatile long A_ENC_CNT = 0;  //Hall encoder count for Motor A
volatile long B_ENC_CNT = 0;  //Hall encoder count for Motor B


//Mode button
#define BUTTON 10        //Button for Mode change
volatile int State = 0;  //Each State is a Mode


//Ultrasonic distance sensor
#define trigPin 11              //Trigger pin
#define echoPin A3              //Echo pin
double Echo = 0.0;              //Time of the ultrasound to return
double distance = 0.0;          //Distance measured by the ultrasound sensor
const double max_Echo_mm = 90;  //Max. distance to object to follow


//Infrared sensor
#define IR_SND_PIN 9     //IR emission pin
#define IR_RCV_A_PIN A1  //IR A receiver pin
#define IR_RCV_B_PIN A0  //IR A receiver pin
byte IR_side = 0;        //Which IR sensor detected the signal
byte direction = 0;      //Direction detected by the infrared sensor


//Addressable LED modules
#define LED_PIN 3                                                   //Data pin for the LED modules
#define NUM_LEDS 4                                                  //Number of LED modules
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);  //Configuration for the LED modules


///MPU-6050 Accelerometer and Gyroscope
MPU6050 mpu;                         //MPU6050 object
int16_t ax, ay, az, gx, gy, gz;      //Raw MPU-6050 readings
double currentAngle = 0.0;           //Current tilt angle
double previousAngle = 0.0;          //Last tilt angle
double accelAngle;                   //Raw angle from accel rate
double gyroRate;                     //Convert gyro rate to °/s
const double gyroSens = 131.0;       //Gyroscope sensitivity: ±250°/s → 131 LSB per °/s
const double gyroOffset = 618.8;     //Offset correction after calibration
const double borderAngle_F = 25.0;   //Maximum forward tilt angle before stopping
const double borderAngle_B = -35.0;  //Maximum back tilt angle before stopping


//Timing
unsigned long currentTime = 0;
unsigned long previousTime = 0;
double deltaTime = 0.0;  //Time step: currentTime-previousTime

unsigned long prev_millis = 0;  //Store the last millis() value for the follow modes


//Wheel speeds
double A_Speed = 0.0;
double B_Speed = 0.0;
double currentSpeed = 0.0;  //Average of A_Speed and B_Speed


//PD controller parameters for balance
const double Kp = 12.0;     //Proportional gain
const double Kd = 1.00;     //Derivative gain
double targetAngle = -2.6;  //Desired tilt angle, initially the center of mass is by around -3°
double error = 0.0;         //Tilt error
double motorSpeed_A = 0.0;  //Calculated speed to be set for motor A
double motorSpeed_B = 0.0;  //Calculated speed to be set for motor B


//PI controller parameters for speed
const double Kp_S = 0.18;                   //Proportional gain
const double Ki_S = 0.25;                   //Integral gain for speed
double speedIntegral = 0.0;                 //Integral term for speed control
const double intergral_Constrain = 2000.0;  //Limit for integral term to prevent ingeral windup

const double alpha = 0.21;       //Low-pass filter coefficient for speed
double oldSpeed = 0.0;           //To store the last speed
double filteredSpeed = 0.0;      //Speed after the low-pass filter
double targetSpeed = 0.0;        //Desired speed, 0 when the robot stationary
const double walkSpeed = 400.0;  //Target speed set for moving


//Controller paramters for turning
double turnSpeed = 0.0;         //Desired turn speed
const double spinSpeed = 30.0;  //Target speed set for turn


//Kalman filter variables
const double Q_angle = 0.001;  //Process noise variance for the angle
const double Q_bias = 0.005;   //Process noise variance for the bias
const double R_measure = 1.0;  //Measurement noise variance (accelerometer)
double angle = 0.0;            //The angle calculated by the Kalman filter
double bias = 0.0;             //The gyroscope bias calculated by the Kalman filter
double rate = 0.0;             //Unbiased rate calculated from the gyroscope

double S = 0.0;                           //Estimate error
double K[2] = { 0, 0 };                   //Kalman gain
double y = 0.0;                           //Angle difference
double P[2][2] = { { 0, 0 }, { 0, 0 } };  //Error covariance matrix


//Weight measurement
const int arraySize = 50;                               //Size of the angleValues array
double angleValues[arraySize];                          //Array to store the angle values
int index = 0;                                          //Index for the array
double arraySum = 0.0;                                  //Sum of the angleArray values
double PHI = 0.0;                                       //Average angle in RAD
const double m_R = 819.6;                               //Weigth of the robot without the load
double m_L = 0.0;                                       //Weight estimate calculated from the angle
const double h_R = 36.73;                               //Vertical COM of the robot withour the load
const double d_R = tan(-targetAngle * PI / 180) * h_R;  //Horizontal COM of the robot without the load, based of the targetAngle
const double h_L = 22.0 + 10;                           //Vertical COM of the load placed on the tray
const double d_L = 95.0 - 25.00;                        //Horizontal COM of the load placed on the tray



//Function prototypes
void Stop();                                                            //Motor stop function
void Move(double speed_A, double speed_B);                              //Motor move function
double kalmanFilter(double newAngle, double newRate, double dt);        //Kalman filter implementation
static void A_ENC_INK();                                                //Encoder A interrupt handler
static void B_ENC_INK();                                                //Encoder B interrupt handler
static void MODE_CHANGE();                                              //Button interrupt handler
double Ultrasonic();                                                    //Ultrasonic distant measurement
byte Infrared();                                                        //Infrared direction measurement
void WeightMeasure();                                                   //Weight measurement
void LED_RGB(double speed_A, double speed_B, double angle, int state);  //Addressable LED modules control



void setup() {
  //Open the serial port for communication with computer
  Serial.begin(9600);

  //Motor pin setup
  pinMode(A_DIR_PIN, OUTPUT);
  pinMode(A_SPD_PIN, OUTPUT);
  pinMode(B_DIR_PIN, OUTPUT);
  pinMode(B_SPD_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  //Enable motor driver, when LOW the motors stop

  //Hall encoders pin setup
  pinMode(A_ENC_PIN, INPUT);
  pinMode(B_ENC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(A_ENC_PIN), A_ENC_INC, CHANGE);  //A hall encoder interrupt setup
  attachPCINT(digitalPinToPCINT(B_ENC_PIN), B_ENC_INC, CHANGE);          //B hall encoder interrupt setup

  //Mode button setup
  pinMode(BUTTON, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(BUTTON), MODE_CHANGE, FALLING);  //Mode button iterrupt setup

  //MPU6050 initialization
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  //Range set: ±250°/s → 131 LSB per °/s

  //Ultrasonic distance sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Infrared sensor setup
  pinMode(IR_SND_PIN, OUTPUT);
  pinMode(IR_RCV_A_PIN, INPUT_PULLUP);
  pinMode(IR_RCV_B_PIN, INPUT_PULLUP);

  //Addressable LED modules setup
  pixels.begin();
}



void loop() {
  //Timing
  currentTime = micros();                                //Get the current system time
  deltaTime = (currentTime - previousTime) / 1000000.0;  //Get the time step, convert to seconds
  previousTime = currentTime;                            //Store the current system time for the next loop

  //MPU6050 readings
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //Reading the sensor, function handled by the library

  //Calculate accelerometer angle (in degrees)
  accelAngle = atan2(ay, az) * 180 / PI;

  //Gyroscope rate (convert raw gyro data to °/s)
  gyroRate = (gx - gyroOffset) / gyroSens;


  //Use Kalman Filter to estimate the current angle
  currentAngle = kalmanFilter(accelAngle, gyroRate, deltaTime);


  //If the robot is between the defined border angles, then balance
  if (currentAngle < borderAngle_F && currentAngle > borderAngle_B)
  {
    //Encoder speed calculation with low-pass filtering
    A_Speed = (motorSpeed_A > 0 ? A_ENC_CNT : -A_ENC_CNT) / deltaTime;
    B_Speed = (motorSpeed_B > 0 ? B_ENC_CNT : -B_ENC_CNT) / deltaTime;
    A_ENC_CNT = 0;  //Reset the interrupt counter for A hall encoder
    B_ENC_CNT = 0;  //Reset the interrupt counter for B hall encoder

    currentSpeed = (A_Speed + B_Speed) * 0.5;                       //Average of hall encoder A and B
    filteredSpeed = alpha * currentSpeed + (1 - alpha) * oldSpeed;  //Low-pass filter
    oldSpeed = filteredSpeed;                                       //Update the speed value for the next loop


    //PD controller logic for balancing
    error = currentAngle - targetAngle;
    previousAngle = currentAngle;


    //PI controller logic for speed
    speedIntegral += (filteredSpeed - targetSpeed) * deltaTime;
    speedIntegral = constrain(speedIntegral, -intergral_Constrain, intergral_Constrain);  //Limit the integral term to prevent ingeral windup


    //PD + PI + Turn controller combined
    motorSpeed_A = Kp * error + Kd * gyroRate + Kp_S * filteredSpeed + Ki_S * speedIntegral + turnSpeed;
    motorSpeed_B = Kp * error + Kd * gyroRate + Kp_S * filteredSpeed + Ki_S * speedIntegral - turnSpeed;


    //Motor control logic
    Move(motorSpeed_A, motorSpeed_B);  //Set motor A and motor B speed


    //Ultrasonic distance and Infrared direction sensor follow and turn mode logic
    if (State > 4 && State < 9 && millis() - prev_millis >= 200)  //If State is 5..6..7..8... and 200ms passed since last measurement
    {
      prev_millis = millis();  //Reset timer

      if (State == 5 || State == 6) distance = Ultrasonic();  //Ultrasonic distance measurement
      if (State == 7 || State == 8) direction = Infrared();   //Infrared direction measurement

      switch (State) {
        case 5:                                                                 //State 5 = forward follow mode
          if (distance > 0 && distance < max_Echo_mm) targetSpeed = walkSpeed;  //Set the target speed, if the distance is in of bounds
          else targetSpeed = 0;
          break;

        case 6:                                                                  //State 6 = back follow mode
          if (distance > 0 && distance < max_Echo_mm) targetSpeed = -walkSpeed;  //Set the target speed, if the distance is in of bounds
          else targetSpeed = 0;
          break;

        case 7:                                                    //State 7 = forward turn mode
          if (direction) turnSpeed = (direction - 2) * spinSpeed;  //Set the turn speed, if a direction is detected
          else turnSpeed = 0;
          break;

        case 8:                                                     //State 8 = back turn mode
          if (direction) turnSpeed = (direction - 2) * -spinSpeed;  //Set the turn speed, if a direction is detected
          else turnSpeed = 0;
          break;
      }
    }

    //Weight measuring mode logic
    if (State == 9 && millis() - prev_millis >= 5000 / arraySize)  //If State is 9 and (5000/arraySize) ms time passed since last measurement
    {
      prev_millis = millis();  //Reset timer
      WeightMeasure();         //Weight measurement, results every 5000 ms
    }


    //Addressable LED modules control logic
    LED_RGB(motorSpeed_A, motorSpeed_B, currentAngle, State);

  }


  else
  {
    Stop();                             //If the robot is beyond the border angles, then stop the motors
    speedIntegral = 0;                  //Reset speed integral term for the new balancing task
    LED_RGB(0, 0, 180, State);          //Special case for the addressable LED modules
  }
}



//Motor control functions
void Stop() {
  digitalWrite(A_DIR_PIN, LOW);  //Motor A direction: LOW=forward, HIGH=back
  analogWrite(A_SPD_PIN, 0);     //Set motor A speed with pwm

  digitalWrite(B_DIR_PIN, LOW);  //Motor B direction: LOW=forward, HIGH=back
  analogWrite(B_SPD_PIN, 0);     //Set motor B speed with pwm
}

void Move(double speed_A, double speed_B) {
  if (speed_A > 0) digitalWrite(A_DIR_PIN, LOW);  //Set motor A direction based on the sign of the speed value
  else digitalWrite(A_DIR_PIN, HIGH);
  analogWrite(A_SPD_PIN, constrain(abs(speed_A), 0, 255));  //Set motor A speed with pwm, constrain it between 0...255

  if (speed_B > 0) digitalWrite(B_DIR_PIN, LOW);  //Set motor B direction based on the sign of the speed value
  else digitalWrite(B_DIR_PIN, HIGH);
  analogWrite(B_SPD_PIN, constrain(abs(speed_B), 0, 255));  //Set motor B speed with pwm, constrain it between 0...255
}



//Kalman filter implementation
double kalmanFilter(double newAngle, double newRate, double dt) {
  //Step 1: Predict the current angle based on the gyroscope rate
  //The rate is calculated by subtracting the current bias from the measured rate (newRate)
  rate = newRate - bias;  //Remove bias from the gyroscope measurement
  angle += dt * rate;     //Update the angle estimate using the rate and elapsed time

  //Update the error covariance matrix (P), which tracks the uncertainty in the angle and bias estimates
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);  //P[0][0]: Uncertainty in the angle estimate
  P[0][1] -= dt * P[1][1];                                       //P[0][1]: Cross-covariance
  P[1][0] -= dt * P[1][1];                                       //P[1][0]: Cross-covariance
  P[1][1] += Q_bias * dt;                                        //P[1][1]: Uncertainty in the bias estimate, increase over time


  //Step 2: Calculate the Kalman gain (K)
  //The Kalman gain determines how much weight should be given to the accelerometer measurement (newAngle)
  S = P[0][0] + R_measure;  //Estimate error, total uncertainty (P + measurement noise)
  K[0] = P[0][0] / S;       //Gain for angle
  K[1] = P[1][0] / S;       //Gain for bias


  //Step 3: Update the angle and bias estimates using the accelerometer measurement
  y = newAngle - angle;  //Difference between measured and predicted angle (measurement residual)
  angle += K[0] * y;     //Adjust angle estimate based on measurement
  bias += K[1] * y;      //Adjust bias estimate based on measurement


  //Step 4: Update the error covariance matrix (P)
  //This reduces the uncertainty in the estimates after incorporating the measurement
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];


  //Return the updated angle estimate
  return angle;
}



//Ultrasonic distance measurement
double Ultrasonic() {
  digitalWrite(trigPin, LOW);           //Get a clear low signal
  delayMicroseconds(2);                 //Short delay
  digitalWrite(trigPin, HIGH);          //Send trigger high signal
  delayMicroseconds(10);                //Trigger pulse width = 10µs
  digitalWrite(trigPin, LOW);           //Stop the trigger signal
  Echo = pulseIn(echoPin, HIGH, 5000);  //Measure the echo time
  return Echo * 0.343 * 0.5;            //Calculate and return the distance. Speed of sound in air is 343 m/s
}


//Infrared direction measurement
byte Infrared() {
  for (int i = 0; i < 10; i++)  //Generate ~38 kHz signal, emit 10 cycles, more cycles --> bigger range
  {
    digitalWrite(IR_SND_PIN, LOW);
    delayMicroseconds(13);
    digitalWrite(IR_SND_PIN, HIGH);
    delayMicroseconds(13);
  }

  IR_side = 0;
  if (digitalRead(IR_RCV_A_PIN) == LOW && !digitalRead(IR_RCV_B_PIN) == LOW) IR_side = 1;  //Signal detected on the right
  if (!digitalRead(IR_RCV_A_PIN) == LOW && digitalRead(IR_RCV_B_PIN) == LOW) IR_side = 3;  //Signal detected on the left

  return IR_side;  //Return the direction. Right = 1; Left = 3; Center = 0;
}



//Weight measurement
void WeightMeasure() {
  angleValues[index] = currentAngle;  //Store the next currentAngle value
  index++;                            //Increment the index every loop

  if (index >= arraySize)  //If the array is full, the measurement is done
  {
    index = 0;                                                                             //Reset the index
    arraySum = 0.0;                                                                        //Reset the sum
    for (int i = 0; i < arraySize; i++) arraySum += angleValues[i];                        //Add up the values in the array
    PHI = arraySum / arraySize * PI / 180;                                                 //Get the average angle in RAD
    m_L = (-m_R * (cos(PHI) * d_R + sin(PHI) * h_R) / (cos(PHI) * d_L + sin(PHI) * h_L));  //Calculate the weight estimate based on the equation
    Serial.print(m_L);
    Serial.print("g  ");  //Print the weight estimate to the serial monitor
    Serial.print(arraySum / arraySize);
    Serial.println("°");  //Print the average angle in DEG
  }
}



//Addressable LED modules control
void LED_RGB(double speed_A, double speed_B, double angle, int state)
{
  //PWM of motor A:
  if(speed_A > 0) pixels.setPixelColor( 1, pixels.Color(0, constrain(abs(speed_A),0,255), 0) );  //Pixel 1: Value in green when speed>0
  else pixels.setPixelColor( 1, pixels.Color(0, 0, constrain(abs(speed_A),0,255)) );             //Pixel 1: Value in blue when speed<0
  if(abs(speed_A) >= 255) pixels.setPixelColor( 1, pixels.Color(255, 0, 0) );                    //Pixel 1: Red when PWM of motor A is maximal

  //PWM of motor B:
  if(speed_B > 0) pixels.setPixelColor( 0, pixels.Color(0, constrain(abs(speed_B),0,255), 0) );  //Pixel 0: Value in green when speed>0
  else pixels.setPixelColor( 0, pixels.Color(0, 0, constrain(abs(speed_B),0,255)) );             //Pixel 0: Value in blue when speed<0
  if(abs(speed_B) >= 255) pixels.setPixelColor( 0, pixels.Color(255, 0, 0) );                    //Pixel 0: Red when PWM of motor B is maximal

  //Current angle
  if(angle > 0) pixels.setPixelColor( 2, pixels.Color(0, map(angle,0,borderAngle_F,0,255), 0) ); //Pixel 2: Value in green mapped to the defined border when angle>0
  else pixels.setPixelColor( 2, pixels.Color(0, 0, map(angle,0,borderAngle_B,0,255)) );          //Pixel 2: Value in blue mapped to the defined border when angle<0
  if(abs(angle) == 180) pixels.setPixelColor( 2, pixels.Color(255, 0, 0) );                      //Pixel 2: Red when angle is beyond the defined borders

  //Selected mode
  switch(state)
  {
    case 0:
      pixels.setPixelColor( 3, pixels.Color(85, 85, 85) );  //White for mode 0, ~1/3 brightness
    break;

    case 1:
      pixels.setPixelColor( 3, pixels.Color(0, 85, 0) );    //Green for mode 1, ~1/3 brightness
    break;

    case 2:
      pixels.setPixelColor( 3, pixels.Color(0, 0, 85) );    //Blue for mode 2, ~1/3 brightness
    break;

    case 3 ... 4:
      pixels.setPixelColor( 3, pixels.Color(85, 0, 0) );    //Red for mode 3 & 4, ~1/3 brightness
    break;

    case 5 ... 6:
      pixels.setPixelColor( 3, pixels.Color(0, 85, 85) );   //Cyan for mode 5 & 6, ~1/3 brightness
    break;

    case 7 ... 8:
      pixels.setPixelColor( 3, pixels.Color(85, 0, 85) );   //Magenta for mode 7 & 8, ~1/3 brightness
    break;

    case 9:
      pixels.setPixelColor( 3, pixels.Color(85, 85, 0) );   //Yellow for mode 9, ~1/3 brightness
    break;
  }

  pixels.show();  //When all pixel is set, send command to the addressable LED modules
}



//Interrupt functions for the hall encoders
static void A_ENC_INC() {
  A_ENC_CNT++;
}  //Increment, when A hall encoder sends signal
static void B_ENC_INC() {
  B_ENC_CNT++;
}  //Increment, when B hall encoder sends signal


//Interrupt function for the mode button
static void MODE_CHANGE() {
  State++;                     //If the button pressed, change mode
  if (State >= 10) State = 0;  //Modes are between 0...9

  switch (State) {
    case 0:  //Mode 0: Stay stationary
      targetSpeed = 0;
      turnSpeed = 0;
      break;

    case 1:  //Mode 1: Move forward with constant speed
      targetSpeed = walkSpeed;
      turnSpeed = 0;
      break;

    case 2:  //Mode 2: Move back with constant speed
      targetSpeed = -walkSpeed;
      turnSpeed = 0;
      break;

    case 3:  //Mode 3: Turn right with constant speed
      targetSpeed = 0;
      turnSpeed = spinSpeed;
      break;

    case 4:  //Mode 4: Turn left with constant speed
      targetSpeed = 0;
      turnSpeed = -spinSpeed;
      break;

    case 5:  //Mode 5: Follow forward with constant speed (actual speed controll in the main loop)
      targetSpeed = 0;
      turnSpeed = 0;
      break;

    case 6:  //Mode 6: Follow back with constant speed (actual speed controll in the main loop)
      targetSpeed = 0;
      turnSpeed = 0;
      break;

    case 7:  //Mode 7: Turn forward with constant speed (actual speed controll in the main loop)
      targetSpeed = 0;
      turnSpeed = 0;
      break;

    case 8:  //Mode 8: Turn back with constant speed (actual speed controll in the main loop)
      targetSpeed = 0;
      turnSpeed = 0;
      break;

    case 9:  //Mode 9: Weight measurement (actual measurement in the main loop)
      targetSpeed = 0;
      turnSpeed = 0;
      break;
  }
}