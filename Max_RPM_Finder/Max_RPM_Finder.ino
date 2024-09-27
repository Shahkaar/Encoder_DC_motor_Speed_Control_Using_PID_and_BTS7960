#include <Encoder.h>

// Encoder pin definitions
Encoder myEnc(2, 3);  // Change according to your setup

#define RPWM 9          //Motor RPWM pin

int PWM = 255;             //The PWM signal on the motor driver pin 0-255
// Motor and Encoder specific values
const int PPR = 204;   // Pulses per revolution (adjust according to your encoder spec)

// Variables to store encoder pulses and speed
// Variables to store encoder pulses and speed
unsigned long previousTime = 0;
unsigned long interval = 500;  // Time interval for speed calculation (in milliseconds)
long previousPosition = 0;
long currentPosition = 0;
double speedRPM = 0;

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  
  // Set up encoder pin modes
  pinMode(RPWM, OUTPUT);
  analogWrite(RPWM,PWM);

 // Initialize time
 previousTime = millis();
}

void loop() {
  // Calculate the time elapsed
  unsigned long currentTime = millis();
  unsigned long timeElapsed = currentTime - previousTime;

  // If the interval has passed, calculate speed
  if (timeElapsed >= interval) {
   currentPosition = myEnc.read();
  long pulses = currentPosition - previousPosition;  // Number of pulses since last measurement
  speedRPM = (pulses * 60.0) / (PPR * (timeElapsed / 1000.0)); // Calculating speed using pulses from encoder and PPR of encoder disk

    // Print speed in RPM to Serial Monitor
    Serial.print("Speed (RPM): ");
    Serial.println(speedRPM);

    // Update previous time
    previousTime = currentTime;
    previousPosition  = currentPosition;
  }
}
