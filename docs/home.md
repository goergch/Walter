<img align="right" width="100px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image002.jpg" >

Most important matter of a robot is to look good. The actuators should have human-like proportions, movements should be smooth, and – rather a matter of personal taste – I do not like humps or bulges with motors or gearboxes. All stuff should be inside the regular housing. 

After checking lots of the stuff around on youtube, I recognized that just a few DIY robots are close to what I had in mind. There's the construction by Andreas Hölldorfer ([Printable Robot Arm](https://hackaday.io/project/3800-3d-printable-robot-arm)), which got even covered recently ("Moveo"). Unfortunately without mentioning the inspiration. I got lots of ideas from his construction, and there are still a couple of parts directly derived from his design.

Another construction called Thor is coming from Ángel Larrañaga Muro ([Thor](https://hackaday.io/project/12989-thor)) which has an interesting differential gearbox for the lower actuators.

These two have the quite the most professional design, while most of the robot arms on youtube are servo based and more or less constructed the same. 

<img align="left" width="30%" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image006.png" >

This is what I had in mind. Most of the DIY robots are using servos, mostly for convenience, since the encoder is built-in already and they are easy to control. Thing is, when it comes to higher torque, the connection of the servo with the actuator becomes difficult, and hard to make of 3D printed material. If the servo or the flange moves just a little bit within the housing, the according play will magnify to a significant amount at the end of the actuator. The required precision to avoid this is way above hobby grade components. 

And servos are boringly easy to use. No fun in construction. A motor with a belt drive and a separate angle sensor solves this, it provides low backlash and allows the electronics to compensate imprecise parts with the sensor placed separately from the motor. Additionally, the motor of an actuator can be placed in the previous joint, lowering the centre of gravity of each actuator.

When a belt drive is set, choice comes naturally to stepper motors, since an additional gearbox is not necessary anymore, torque is high and the belts should compensate the vibrations coming from the stepping nature of these motors.

In general, the information flow looks like this:

<img width="600px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image013.png"/>

The visualizer is mainly the UI-based tool to plan trajectories by defining singular points. These points are sent to the Trajectory Execution, where the trajectory is interpolated using Bézier curves. Out of each point of the interpolated curve the angles are computed and send to the controller board, where a PID controller takes care that each actuator actually follows the curve.

On the mechanical side, we have two actuators driven by a servo (mainly due to space restrictions) and four actuators driven by a stepper/rotary encoder combination.

The servos are controlled directly by the cortex controller board  via a serial interface. The steppers do not have an internal feedback loop, so we need rotary encoders detecting the absolute angle of the joint and allowing to implement feedback controllers. Depending on the actuator, the steppers provide a torque between 26Ncm (elbow) and 3,1Nm (upperarm). For space reasons, the gripper and the wrist is driven by a servo.

<img align="center" width="800px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image014.png"/>

The steppers are driven by retail stepper drivers (PiBot Stepper Driver) around the popular PWM stepper driver Toshiba 6600 (4.5A max). The stepper drivers are directly connected to the Controller Board. It receives joint angles at 20Hz, interpolates the points in between, and sends the according PWM signal to the stepper drivers and to the servos. Besides micro interpolation of the trajectory, the controller board takes care of the speed profile, i.e. it limits the acceleration and speed of each actuator. The controller board is a DIY board around an ARM Cortex M4 with 120 MHz (Teensy 3.5). I started with an ATmega 644 8-bit controller, but it turned out that the ATmega was not able to control 5 steppers with a proper sample rate, let alone reading 5 encoders on top.

The controller board is fed by the trajectory board, which is an Odroid XU4 board with a 2GHz octa core.

The trajectory controller board is encapsulated by a webserver exposing the current movement and accepting commands like new trajectories.

Continue reading with [Construction](https://github.com/jochenalt/Walter/wiki/Construction).

