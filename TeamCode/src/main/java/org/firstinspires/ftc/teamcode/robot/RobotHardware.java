package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {

    // Drivetrain
    public DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // Launcher / feeder
    public VoltageSensor voltageSensor;
    public DigitalChannel preShooterBeamBreak;
    public Servo hoodAngleServo;
    public DcMotorEx triggerMotor;
    public Servo gateServo;

    // Turret / vision
    public CRServo turretServo;
    public DcMotorEx turretEncoder;
    public Limelight3A limelight;

    // Intake
    public DcMotorEx intakeMotor;
    public Servo flowerRampServo;
    public double intakePowerCommand = 0; // last intake power, so Storage knows which way elements move

    // Storage
    public DigitalChannel intakeMouthBeamLow;
    public DigitalChannel intakeMouthBeamHigh;
    public DigitalChannel intakeQueueBeamLow;
    public DigitalChannel intakeQueueBeamHigh;
    public DigitalChannel storageExitBeamLow;
    public DigitalChannel storageExitBeamHigh;
    public NormalizedColorSensor intakeColorSensor;
    public double shooterFeederPowerCommand = 0; // last feeder power: Storage counts an exit only while feeding

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
        launchMotor1     = hardwareMap.get(DcMotorEx.class, RobotConfig.LAUNCH_MOTOR);
        launchMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        launchMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        // Launcher reads all of these every setLauncher() call
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        preShooterBeamBreak = hardwareMap.get(DigitalChannel.class, RobotConfig.PRE_SHOOTER_BEAM);
        preShooterBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        hoodAngleServo = hardwareMap.get(Servo.class, RobotConfig.HOOD_ANGLE_SERVO);
        hoodGapServo1 = hardwareMap.get(Servo.class, RobotConfig.HOOD_GAP_SERVO_1);
        hoodGapServo2 = hardwareMap.get(Servo.class, RobotConfig.HOOD_GAP_SERVO_2);
        /*
        hoodAngleServo1 = hardwareMap.get(Servo.class, "hood_angle_servo_1");
        hoodAngleServo2 = hardwareMap.get(Servo.class, "hood_angle_servo_2");
        hoodGapServo1 = hardwareMap.get(Servo.class, "hood_gap_servo_1");
        hoodGapServo2 = hardwareMap.get(Servo.class, "hood_gap_servo_2");
        */
    }

    // Kept out of init() so OpModes that only use the launcher don't need these in the config
    public void initIntake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flowerRampServo = hardwareMap.get(Servo.class, RobotConfig.FLOWER_RAMP_SERVO);
    }

    // Kept out of init() so OpModes that don't use storage don't need these in the config
    public void initStorage(HardwareMap hardwareMap) {
        intakeMouthBeamLow = beam(hardwareMap, RobotConfig.INTAKE_MOUTH_BEAM_LOW);
        intakeMouthBeamHigh = beam(hardwareMap, RobotConfig.INTAKE_MOUTH_BEAM_HIGH);
        intakeQueueBeamLow = beam(hardwareMap, RobotConfig.INTAKE_QUEUE_BEAM_LOW);
        intakeQueueBeamHigh = beam(hardwareMap, RobotConfig.INTAKE_QUEUE_BEAM_HIGH);
        storageExitBeamLow = beam(hardwareMap, RobotConfig.STORAGE_EXIT_BEAM_LOW);
        storageExitBeamHigh = beam(hardwareMap, RobotConfig.STORAGE_EXIT_BEAM_HIGH);
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.INTAKE_COLOR_SENSOR);
    }

    private static DigitalChannel beam(HardwareMap hardwareMap, String name) {
        DigitalChannel beam = hardwareMap.get(DigitalChannel.class, name);
        beam.setMode(DigitalChannel.Mode.INPUT);
        return beam;
    }

    // Kept out of init() so OpModes that don't drive don't need the motors in the config
    public void initDrive(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_LEFT_DRIVE);
        backLeftDrive = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_LEFT_DRIVE);
        frontRightDrive = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_RIGHT_DRIVE);
        backRightDrive = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_RIGHT_DRIVE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void initTrigger(HardwareMap hardwareMap) {
        triggerMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.TRIGGER_MOTOR);
        triggerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gateServo = hardwareMap.get(Servo.class, RobotConfig.GATE_SERVO);
    }

    public void initTurret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, RobotConfig.TURRET_SERVO);
        turretEncoder = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET_ENCODER);
    }

    public void initVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT);
    }
}
