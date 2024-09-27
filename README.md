# Encoder_DC_motor_Speed_Control_Using_PID_and_BTS7960
# Introduction
  For the sake of controlling a DC motors speed using PID the first thing to do to calculate the current speed of the motor for which we need an encoder DC motor then the input to the PID control equation will be the speed and the error will be the difference between the desired speed and current speed which will be fed to the equation making a close loop signal. An encoder motor basically gives us pulses as the motor rotates and every encoder has a disk with slits carved into it. For example a disk in an encoder has 20 slots so when the MCU will read 20 pulse it will mean that the motor has completed one revolution. This value 20 used in the example is also referred as PPR(pulses per revolution).
  To find out the PPR of an encoder motor one need to mark a starting point on the motor shaft and body and then read the pulses using any MCU by manually rotating the shaft for one complete revolution. Mostly encoders have 4 pins(Vcc,Gnd,enA,enB) the enA and enB are digitally read. Use the code given for PPR to calculate the PPR as it will be used in the PID code later.
  Now finding the maximum RPM of the motor. Use the code provided to calculate that as well for a rough idea of what are the limits of ones desired speed which will be set in the PID controller. 
# Components
  1. Encoder DC motor
  2. arduino (uno, nano etc)
  3. BTS7960 motor driver
  4. 12V supply
  5. Jumpers
# PPR Finder
  Connect the components as shown in the image and power the arduino using the PC upload the code PPR_Finder to arduino mark a point on the motor shaft. After uploading that powering up the supply open the serial monitor and rotate the motor once the number of pulses will be visible.
  <p align="center">
<img src="https://github.com/Shahkaar/Encoder_DC_motor_Speed_Control_Using_PID_and_BTS7960/blob/main/graphics/Blank%20board.png" width="500" height="500" />
</p>
# Max RPM Finder
  All the 3 codes will run on the same circuit just change the code as desired.
