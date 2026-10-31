package org.firstinspires.ftc.teamcode.robot;

import android.graphics.RenderNode;

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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {

    // Drivetrain
    //public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // Sensors
    public DcMotorEx launchMotor1;
    public VoltageSensor voltageSensor;
    // Hood: one servo sets how high the ball goes, the other the gap between base and shooter
    public Servo hoodAngleServo;
    public Servo hoodGapServo1;
    public Servo hoodGapServo2;
    //public GoBildaPinpointDriver pinpoint;
    public Limelight3A limelight;

    // Turret
    public DcMotorEx turretEncoder;
    public TouchSensor turretHallEffect;
    public DcMotor triggerMotor;
    public DigitalChannel preShooterBeamBreak;
    public CRServo turretServo1;
    public CRServo turretServo2;
    public TouchSensor hallEffectSensor;
    public Servo gateServo;
    public DcMotor intakeMotor;
    public double intakePowerCommand;
    public DcMotor   frontLeftMotor   = null;
    public DcMotor   frontRightMotor  = null;
    public DcMotor   backLeftMotor    = null;
    public DcMotor   backRightMotor   = null;
    public Servo flowerRampServo;
    public GoBildaPinpointDriver pinpoint;
    //public Storage storage;
    //public IMU imu;

    public void init(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        // --- Hardware Map ---

        /*
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        */
        initDrivetrain(hardwareMap);
        initLaunchMotor(hardwareMap);
        initIntakeTransferTrigger(hardwareMap);
        initHood(hardwareMap);
        initTurret(hardwareMap);
    }

    // Kept out of init() so OpModes that only spin the flywheel don't need these in the config
    public void initVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT);
    }
    public void initDrivetrain(HardwareMap hardwareMap) {
        //imu = hardwareMap.get(IMU.class, "imu");
        frontLeftMotor  = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_LEFT_MOTOR_NAME);
        backLeftMotor   = hardwareMap.get(DcMotor.class, RobotConfig.BACK_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_RIGHT_MOTOR_NAME);
        backRightMotor  = hardwareMap.get(DcMotor.class, RobotConfig.BACK_RIGHT_MOTOR_NAME);
        intakeMotor     = hardwareMap.get(DcMotor.class, "intake_motor");
        pinpoint        = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // --- Directions ---
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initLaunchMotor(HardwareMap hardwareMap) {
        launchMotor1     = hardwareMap.get(DcMotorEx.class, "launch_motor");
        launchMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        launchMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    public void initIntakeTransferTrigger(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        triggerMotor = hardwareMap.get(DcMotor.class, "trigger_motor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        triggerMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void initHood(HardwareMap hardwareMap) {
        hoodAngleServo = hardwareMap.get(Servo.class, RobotConfig.HOOD_ANGLE_SERVO);
        hoodGapServo1 = hardwareMap.get(Servo.class, RobotConfig.HOOD_GAP_SERVO1);
        hoodGapServo2 = hardwareMap.get(Servo.class, RobotConfig.HOOD_GAP_SERVO2);
    }

    public void initTurret(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(CRServo.class, RobotConfig.TURRET_SERVO_1);
        turretServo2 = hardwareMap.get(CRServo.class, RobotConfig.TURRET_SERVO_2);
        turretEncoder = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET_ENCODER);
        //turretHallEffect = hardwareMap.get(TouchSensor.class, RobotConfig.TURRET_HALL);
        //preShooterBeamBreak = hardwareMap.get(DigitalChannel.class, RobotConfig.PRE_SHOOTER_BEAM_BREAK);
        //preShooterBeamBreak.setMode(DigitalChannel.Mode.INPUT);
    }
}
