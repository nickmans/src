# Hardware Layout

## Robot Body Frame

Coordinate convention for the robot body frame:

- x axis: forward is positive
- y axis: left is positive

## LiDAR Placement

Two RPLIDAR C1 units are mounted as follows (body frame coordinates):

- RPLIDAR C1 1: x = 0.0 m, y = +0.1 m
- RPLIDAR C1 2: x = 0.0 m, y = -0.1 m

## IMU

IMU model and mode:

- BNO086
- UART-RVC mode

IMU axis note:

- IMU x axis: forward
- IMU y axis: right

## Main Compute and Control

- MCU: Nucleo H755ZI
- SBC: Raspberry Pi 5

## Drivetrain

- Wheel type: omni wheels
- Wheel diameter: 180 mm
