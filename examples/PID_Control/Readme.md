## Example: PID_Control

This basic example implements PID control of a motor using the RMT feature of the dshot library.

The class `PIDController` is the driver behind the PID section.

Other than the ESC, this example requires 2 buttons and a potentiometer. One button allows the target RPM to be set with the potentiometer, and the other tells the PID loop to drive to that speed. *(because I've found that using the potentiometer's value direct without mapping is too noisty)*








