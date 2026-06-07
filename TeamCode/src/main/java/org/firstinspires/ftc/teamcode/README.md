# Team Code

## Table of Contents

- [`CustomLinearOp`](#customlinearop)
- [`AutoConfig`](#autoconfig)
- [`Auto`](#auto)
- [`DriverMode`](#drivermode)
- [`AllianceColor`](#alliancecolor)
- [`TeamSide`](#teamside)
- [`hardwareSystems/`](#hardwaresystems)
    - [`MotorType`](#motortype)
    - [`Wheels`](#wheels)
    - [`MecanumWheels`](#mecanumwheels)
    - [`Arm`](#arm)
    - [`FoldingArm`](#foldingarm)
    - [`Claw`](#claw)
    - [`SingleServoIntakeClaw`](#singleservointakeclaw)
    - [`DoubleServoIntakeClaw`](#doubleservointakeclaw)
    - [`Webcam`](#webcam)

## [`CustomLinearOp`](CustomLinearOp.java)

A custom LinearOpMode that is the parent class of [`Auto`](#auto) and [`DriverMode`](#drivermode).
Its `runOpMode()` initializes all the robot's hardware and contains methods for auto-sleep,
sleeping while motors and continuous servos are running.
For the hardware initializing methods(e.g. `initWheels()`),
replace the abstract class return type with the the desired class(e.g. `MecanumWheels`).

## [`AutoConfig`](AutoConfig.java)

A TeleOp that writes information such as the robot's [`AllianceColor`](#alliancecolor) and [
`TeamSide`](#teamside)
into a config file on the Control Hub before running autonomous.
It also stores the filepath of the config file.

Pressing the A, B, X, and Y buttons sets the robot to
blue near, red far, blue far, and red near, respectively.
Pressing D-pad up sets the robot to use its arm during autonomous.
Pressing D-pad left sets the robot to only push the piece.
Does **NOT** extend [`CustomLinearOp`](#customlinearop).

## [`Auto`](Auto.java)

The Autonomous class, which runs without driver input.
Child class of [`CustomLinearOp`](#customlinearop).
The annotation `@Autonomous(name = "Auto")` means that
the class will be considered an Autonomous program with the name of "Auto."
The `runOpMode()` method runs automatically without the need to do anything.
The first line of `runOpMode()` should be `super.runOpMode()`
to run the parent class's hardware initialization.

The TeleOp class which runs using driver input.
Child class of [`CustomLinearOp`](#customlinearop).

## [`DriverMode`](DriverMode.java)

The annotation `@TeleOp(name = "DriverMode")` means that
the class will be considered a TeleOp program with the name of "DriverMode."
The `runOpMode()` method runs automatically without the need to do anything.
The first line of `runOpMode()` should be `super.runOpMode()`
to run the parent class's hardware initialization.
The `runOpMode()` runs the code in a try-catch to detect errors.

## [`AllianceColor`](AllianceColor.java)

An enum that states whether the robot is on the red or blue side.

## [`TeamSide`](TeamSide.java)

An enum that states whether the robot is on the far or near side.

## [`hardwareSystems/`](hardwareSystems/)

This subdirectory contains helper classes.
The classes are meant to separate and organize the various systems of the robot,
e.g., arms and wheels.
Contain methods for basic tasks such as driving and lifting the arm.
Some of the classes, e.g., [`Arm`](#arm) and [`Wheels`](#wheels),
are abstract and meant to be used as superclasses.
Being abstract classes rather than interfaces prevents multiple implementing.

### [`MotorType`](hardwareSystems/MotorType.java)

An enum that stores the type of motor(e.g. Tetrix Torquenado)
and its number of ticks per revolution.

### [`Wheels`](hardwareSystems/Wheels.java)

A abstract class for the robot's wheels.
Contains a `HashSet` of all motors.
Sets each motor to float when zero power is applied.
Contains a inner class called `WheelDistances` to store the distances between the wheels,
which is needed for turning.

### [`MecanumWheels`](hardwareSystems/MecanumWheels.java)

A subclass of the [`Wheels`](#wheels) class
for controlling the driving of a four-mecanum wheel system.
Contains the inner class `MotorSet` to pass in the motors to the `MecanumWheels` constructor.

### [`Arm`](hardwareSystems/Arm.java)

An abstract class to control the robot's arm system.
Contains a `HashSet` of all motors.
Sets each motor to brake when zero power is applied.

> [!Note]
> `Arm` does not contain the servos for controlling the claw.
> For that, look at [`Claw`](#claw)

### [`FoldingArm`](hardwareSystems/FoldingArm.java)

A subclass of [`Arm`](#arm) that controls a rotating arm that can fold in two like an elbow.
Contains four inner classes(i.e. `MotorSet`, `RotationRange`,
and `ExtensionRange`) that group together parameters for the constructor.
More specific details can be found in [`Arm`](#arm).
The current of parameter classes is admittedly clunky.
If it becomes cumbersome, please do change it.

### [`Claw`](hardwareSystems/Claw.java)

An abstract class to control the robot's claw.
Has properties for servos to rotate in the x-, y-, and z-axes.
If any of the servos are not needed, set them to null.
The class methods check for null servo values.

### [`SingleServoIntakeClaw`](hardwareSystems/SingleServoIntakeClaw.java)

A subclass of [`Claw`](#claw) that controls a claw with one intake servo.

### [`DoubleServoIntakeClaw`](hardwareSystems/DoubleServoIntakeClaw.java)

A subclass of [`Claw`](#claw) that controls a claw with two intake servos.

### [`Webcam`](hardwareSystems/Webcam.java)

A class for vision and color detection.
Still a work in progress.

> [!Note]
> The webcam consumes massive amounts of memory.
