# Walter

<img align="right" width="100px" src="https://github.com/jochenalt/Walter/docs/images/image002.jpg" >

Most important matter of a robot is to look good. The actuators should have human-like proportions, movements should be smooth, and – rather a matter of personal taste – I do not like humps or bulges with motors or gearboxes. All stuff should be inside the regular housing. 

After checking lots of the stuff around on youtube, I recognized that just a few DIY robots are close to what I had in mind. There's the construction by Andreas Hölldorfer ([Printable Robot Arm](https://hackaday.io/project/3800-3d-printable-robot-arm)), which got even covered recently ("Moveo"). Unfortunately without mentioning the inspiration. I got lots of ideas from his construction, and there are still a couple of parts directly derived from his design.

Another construction called Thor is coming from Ángel Larrañaga Muro ([Thor](https://hackaday.io/project/12989-thor)) which has an interesting differential gearbox for the lower actuators.

These two have the quite the most professional design, while most of the robot arms on youtube are servo based and more or less constructed the same. 

<img align="left" width="30%" src="https://github.com/jochenalt/Walter/blob/master/doc/images/wiki/image006.png" >

This is what I had in mind. Most of the DIY robots are using servos, mostly for convenience, since the encoder is built-in already and they are easy to control. Thing is, when it comes to higher torque, the connection of the servo with the actuator becomes difficult, and hard to make of 3D printed material. If the servo or the flange moves just a little bit within the housing, the according play will magnify to a significant amount at the end of the actuator. The required precision to avoid this is way above hobby grade components. 

And servos are boringly easy to use. No fun in construction. A motor with a belt drive and a separate angle sensor solves this, it provides low backlash and allows the electronics to compensate imprecise parts with the sensor placed separately from the motor. Additionally, the motor of an actuator can be placed in the previous joint, lowering the centre of gravity of each actuator.

When a belt drive is set, choice comes naturally to stepper motors, since an additional gearbox is not necessary anymore, torque is high and the belts should compensate the vibrations coming from the stepping nature of these motors.

Continue reading with [Architecture](https://github.com/jochenalt/Walter/wiki/1-Architecture).

