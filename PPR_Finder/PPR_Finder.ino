// Encoder pin definitions
#define ENCODER_PIN_A 2  // Pin for encoder channel A (must be an interrupt pin)
#define ENCODER_PIN_B 3  // Pin for encoder channel B (optional if only counting pulses)

volatile long pulseCount = 0;  // Variable to store pulse count

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);

  // Set up encoder pin modes
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Attach interrupt on pin A for counting pulses
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, RISING);  // Trigger on rising edge

  // Inform the user
  Serial.println("Rotate the motor shaft to complete one full revolution.");
  Serial.println("Pulses will be counted and displayed when you stop.");
}

void loop() {
  // Display the pulse count every second
  delay(1000);
  noInterrupts();  // Disable interrupts while reading shared variables
  long currentPulses = pulseCount;  // Copy pulse count to local variable
  interrupts();  // Re-enable interrupts

  // Print pulse count to Serial Monitor
  Serial.print("Pulses counted so far: ");
  Serial.println(currentPulses);
}

// Interrupt Service Routine (ISR) for counting encoder pulses
void encoderISR() {
  pulseCount++;  // Increment pulse count each time an interrupt is triggered
}