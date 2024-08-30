# Servo-SBUS-to-Dynamixel
Control a Dynamixel servo from PPM or SBUS

The (Open Source) hardware for this project is here: https://oshwlab.com/simonrafferty/servo-to-dynamixel
you can order a bare or assembled PCB from here.  It's completely open source - I don't profit from it.
I'll try to offer assistance if you need and as time permits - but this will be limited.

I bought the Seeed Studio XAIO SAMD21 modules from RS https://uk.rs-online.com/web/p/arduino-compatible-boards-kits/2005273
Including a pre-assembled board from JLCPCB, the total cost, including the processor is about £12.

Robotis Dynamixel make some fantastic Servos.  Some of them can be controlled through a Radio Control receiver using PPM - but not all, and not the ones I wanted to use (XM540-W270-T) which can generate an incredible 10Nm torque.

 

I developed the first version as a simple PPM to Dynamixel converter, then thought, since multiple Dynamixels can be daisy-chained together - and most RC sets can use SBUS which allows a single reciever output to control multiple servos, it makes sense to use SBUS to control multiple Dynamixels.

 

I've implemented adjustable smoothing (via the variable resistor on board) which uses Kalman filters to remove judder & jerky movement from the servos.

The board has 4 settings switches with the following functions:

1 - Single Turn / Multi-Turn (10 turn)

2 - 50% / 100% Velocity

3 - Position / Velocity Mode (if you want to drive a wheel for example)

4 - PPM / SBUS

 

You need to cycle the power if you change a setting.

 

The Microcontroller (XAIO SAMD21) is powered either from USB or via the RC Receiver.  DO NOT CONNECT BOTH!  The receiver power will flow back through the USB port and (probably) damage your PC. (how do I know this!)
