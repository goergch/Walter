
#Architecture

In general, the information flow looks like this:

<img width="600px" src="https://github.com/jochenalt/Walter/blob/master/doc/images/wiki/image013.png"/>

The visualizer is mainly the UI-based tool to plan trajectories by defining singular points. These points are sent to the Trajectory Execution, where the trajectory is interpolated using Bézier curves. Out of each point of the interpolated curve the angles are computed and send to the controller board, where a PID controller takes care that each actuator actually follows the curve.

On the mechanical side, we have two actuators driven by a servo (mainly due to space restrictions) and four actuators driven by a stepper/rotary encoder combination.

The servos are controlled directly by the cortex controller board  via a serial interface. The steppers do not have an internal feedback loop, so we need rotary encoders detecting the absolute angle of the joint and allowing to implement feedback controllers. Depending on the actuator, the steppers provide a torque between 26Ncm (elbow) and 3,1Nm (upperarm). For space reasons, the gripper and the wrist is driven by a servo.

<img align="center" width="800px" src="https://github.com/jochenalt/Walter/blob/master/doc/images/wiki/image014.png"/>

The steppers are driven by retail stepper drivers (PiBot Stepper Driver) around the popular PWM stepper driver Toshiba 6600 (4.5A max). The stepper drivers are directly connected to the Controller Board. It receives joint angles at 20Hz, interpolates the points in between, and sends the according PWM signal to the stepper drivers and to the servos. Besides micro interpolation of the trajectory, the controller board takes care of the speed profile, i.e. it limits the acceleration and speed of each actuator. The controller board is a DIY board around an ARM Cortex M4 with 120 MHz (Teensy 3.5). I started with an ATmega 644 8-bit controller, but it turned out that the ATmega was not able to control 5 steppers with a proper sample rate, let alone reading 5 encoders on top.

The controller board is fed by the trajectory board, which is an Odroid XU4 board with a 2GHz octa core.

The trajectory controller board is encapsulated by a webserver exposing the current movement and accepting commands like new trajectories.
