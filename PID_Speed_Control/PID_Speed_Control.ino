#include <Encoder.h>

// Encoder pin definitions
Encoder myEnc(2, 3);  // Change according to your setup

// Motor control pins
#define RPWM 10          //Motor RPWM pin
#define LPWM 9           //Motor LPWM pin
int PWM_PIN = 0;         // PWM output pin to control motor speed (connect to motor driver)

// Motor and Encoder specific values
const int PPR = 204;   // Pulses per revolution (adjust according to your encoder spec)

// PID control variables
double Kp = 6.0;       // Proportional gain
double Ki = 3.0;       // Integral gain for controlling overshoot
double Kd = 0.01;       // Derivative gain
double desiredSpeedRPM = 1000; // Setpoint: Desired speed in RPM
double currentSpeedRPM = 0;   // Measured speed in RPM
double previousError = 0;
double integral = 0;
double controlSignal = 0;
int maxPWM = 255;      // Max PWM value for 100% duty cycle
int minPWM = 0;        // Min PWM value
int direction = 0;     // 0 for one direction and 1 for another direction depending on pins used

// Variables to store encoder pulses and speed
unsigned long previousTime = 0;
unsigned long previousPIDTime = 0;  // Time for PID loop
unsigned long interval = 500;  // Time interval for speed calculation (in milliseconds)
long previousPosition = 0;
long currentPosition = 0;

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);

  // Set up motor control pins
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  

  // Initialize time
  previousTime = millis();
  previousPIDTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long timeElapsed = currentTime - previousTime;
  // Calculate speed every interval (0.5 second = 500 ms)
  if (timeElapsed >= interval) {
  currentPosition = myEnc.read();
  long pulses = currentPosition - previousPosition;  // Number of pulses since last measurement
  currentSpeedRPM = (pulses * 60.0) / (PPR * (timeElapsed / 1000.0)); // Calculating speed using pulses from encoder and pulse per revolution PPR of encoder disk
  if(direction == 1)
  {
  currentSpeedRPM = -currentSpeedRPM;
  PWM_PIN = RPWM;
  }
  if(direction == 0)
  {
    PWM_PIN = LPWM;
  }
  double error = desiredSpeedRPM - currentSpeedRPM;  // Error = Setpoint - Actual Speed
  integral += error * (10.0 / 1000.0);     // Integral term
  double derivative = (error - previousError) / (10.0 / 1000.0);  // Derivative term
  controlSignal = (Kp * error) + (Ki * integral) + (Kd * derivative);  // PID output
  controlSignal = constrain(controlSignal, minPWM, maxPWM);  // Constrain PWM between 0 and 255  
  
  analogWrite(PWM_PIN, controlSignal);  // Apply PWM to motor
  int currspeed = int(currentSpeedRPM); // converts double variable to int

    // Print speed in RPM to Serial Monitor
    Serial.print("PWM_Pin:");
    Serial.print(PWM_PIN);
    Serial.print(",");    
    Serial.print("Upper:");
    Serial.print("1500");
    Serial.print(",");
    Serial.print("Lower:");
    Serial.print("100");
    Serial.print(",");
    Serial.print("speed:");
    Serial.println(currspeed);
    previousError = error;
  }
}