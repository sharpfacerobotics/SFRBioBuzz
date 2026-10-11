package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {

    // Drivetrain
    //public DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // Intake / Claw
    //public DcMotor intakeMotor;

    /* Launcher CHANGE FOR BIOBUZZ

    public Servo Trigger;
    public Servo push;

    // Transfer
    public DcMotor transferMotor;
  */

    // Sensors
    public DcMotorEx launchMotor1;
    public Servo hoodAngleServo1;
    public Servo hoodAngleServo2;
    public Servo hoodGapServo1;
    public Servo hoodGapServo2;
    //public GoBildaPinpointDriver pinpoint;
    //public Limelight3A limelight;
    //public VoltageSensor myControlHubVoltageSensor;
    //public IMU imu;

    public void init(HardwareMap hardwareMap) {

        // --- Hardware Map ---
        /*
        imu = hardwareMap.get(IMU.class, "imu");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");
        intakeMotor     = hardwareMap.get(DcMotor.class, "intake_motor");
        //
        //Trigger         = hardwareMap.get(Servo.class,   "Trigger");
        //push            = hardwareMap.get(Servo.class,   "push");
        //transferMotor   = hardwareMap.get(DcMotor.class, "transfer");
        pinpoint        = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight       = hardwareMap.get(Limelight3A.class, "limelight");
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // --- Directions ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        //Trigger.setDirection(Servo.Direction.FORWARD);
        //transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //__ CHANGE ORIENTATION FOR BIOBUZZ
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        */
        launchMotor1     = hardwareMap.get(DcMotorEx.class, "launch_motor");
        launchMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        /*
        hoodAngleServo1 = hardwareMap.get(Servo.class, "hood_angle_servo_1");
        hoodAngleServo2 = hardwareMap.get(Servo.class, "hood_angle_servo_2");
        hoodGapServo1 = hardwareMap.get(Servo.class, "hood_gap_servo_1");
        hoodGapServo2 = hardwareMap.get(Servo.class, "hood_gap_servo_2");
        */
    }
}
