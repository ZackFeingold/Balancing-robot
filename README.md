# Self-Balancing Robot

A two-wheel self-balancing robot built using an Arduino Nano, MPU6050 inertial measurement unit, stepper motors and motor drivers.

The aim of the project was to design and tune a closed-loop control system capable of keeping the robot upright by continuously measuring tilt and adjusting motor output in real time.

## Project Overview

This project combines embedded programming, sensor integration and control logic.

The robot uses:
- an Arduino Nano as the controller
- an MPU6050 IMU to measure tilt and motion
- stepper motors and motor drivers for actuation
- a PID control loop to stabilise the chassis

The main engineering challenge was tuning the controller so that the robot corrected quickly enough to prevent falling, without over-correcting and causing oscillation.

## What I Built

I:
- assembled the hardware platform
- integrated the MPU6050 sensor with the Arduino
- wrote the control code in Arduino C++
- implemented and tuned the PID control loop
- iterated on parameters to reduce overshoot and improve stability

## Hardware

- Arduino Nano
- MPU6050 IMU
- Stepper motors
- Motor drivers
- Chassis and wheels
- Battery power supply

## Software / Control Approach

The robot continuously:
1. reads orientation data from the IMU
2. estimates tilt angle
3. computes the error relative to the upright position
4. applies a PID controller
5. drives the motors to correct the error

This required repeated tuning of proportional, integral and derivative terms to balance responsiveness against stability.

## Engineering Challenges

Some of the main challenges were:
- noisy or drifting sensor readings
- preventing motor over-correction
- tuning PID parameters for stable behaviour
- dealing with real-world effects such as friction, backlash and battery variation

## Results

The final robot was able to balance upright for sustained periods after iterative tuning and debugging.

## What I Learned

This project gave me hands-on experience with:
- embedded systems
- sensor integration
- feedback control
- practical PID tuning
- debugging electromechanical systems

## Future Improvements

If I continued the project, I would look at:
- improved sensor fusion
- better chassis design
- higher quality motor control
- cleaner code modularisation
- logging angle and control data for quantitative tuning

- ![Image of the robot](<Image 1.jpg>)
