# 6-DOF Robot Control System

MATLAB kinematics, motion planning, and physical control software
developed for a custom 6-DOF desktop robotic arm.

## System Overview

The control workflow converts a Cartesian XYZ target into a validated
joint configuration before transmitting commands to the physical robot.

XYZ Target
↓
Inverse Kinematics
↓
Joint-Limit Validation
↓
Forward-Kinematic Verification
↓
Controller Command Generation
↓
Coordinated 6-Axis Motion

## MATLAB Files

### SIX_DOF_Manipulator.m
Defines the robot's kinematic model and generates the reachable
Tool Center Point (TCP) workspace.

### moveXYZ.m
Primary user interface for homing, returning to the ready
configuration, and commanding point-to-point XYZ positions.

### moveToPose.m
Solves the inverse-kinematics problem, evaluates candidate joint
configurations, applies physical joint limits, and converts the
selected configuration into controller commands.

### moveToJoints.m
Coordinates physical movement of the six joints, including the
NEMA 17 driven primary axes and servo-driven wrist.

### moveCartesian.m
Generates and executes straight-line Cartesian trajectories using
intermediate XYZ waypoints.

## Hardware

- MKS TinyBee controller
- 3 × NEMA 17 stepper motors
- Servo42C closed-loop stepper drivers
- 3 × wrist servos
- Mechanical limit switches
- Custom FDM-manufactured robot structure

## Control

The primary positioning joints are controlled through coordinated
stepper commands while the three wrist joints are driven by servos.
MATLAB maintains the current robot configuration and converts between
the kinematic model, physical joint coordinates, and controller
coordinates.

## Project

Full mechanical design, actuator development, electronics integration,
kinematics, and control were developed as part of a custom 6-DOF
robotic-arm project.

See the complete project portfolio for mechanical design, actuator
testing, electronics, and system validation.
