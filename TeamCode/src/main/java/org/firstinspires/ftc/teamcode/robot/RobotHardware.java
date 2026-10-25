package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public VoltageSensor voltageSensor;
    // Hood: one servo sets how high the ball goes, the other the gap between base and shooter
    public Servo hoodAngleServo;
    public Servo hoodGapServo;
    //public GoBildaPinpointDriver pinpoint;
    public Limelight3A limelight;

    // Turret
    public CRServo turretServo;
    public DcMotorEx turretEncoder;
    public DigitalChannel turretHallEffect;
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
        launchMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        launchMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    // Kept out of init() so OpModes that only spin the flywheel don't need these in the config
    public void initVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT);
    }

    public void initHood(HardwareMap hardwareMap) {
        hoodAngleServo = hardwareMap.get(Servo.class, RobotConfig.HOOD_ANGLE_SERVO);
        hoodGapServo = hardwareMap.get(Servo.class, RobotConfig.HOOD_GAP_SERVO);
    }

    public void initTurret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, RobotConfig.TURRET_SERVO);
        turretEncoder = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET_ENCODER);
        turretHallEffect = hardwareMap.get(DigitalChannel.class, RobotConfig.TURRET_HALL);
        turretHallEffect.setMode(DigitalChannel.Mode.INPUT);
    }
}
