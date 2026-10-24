package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/**
 * Every motor, servo and sensor on the robot. Owner: Arnav.
 *
 * <p>Subsystems never call hardwareMap themselves - they use the public fields here,
 * e.g. {@code robot.intakeMotor.setPower(0.8)}.</p>
 *
 * <p>The drive motors and Pinpoint are owned by Pedro's {@link #follower}, so they are not
 * mapped separately here (two owners fighting over one motor is a bug).</p>
 */
public class RobotHardware {

    // Drivetrain + localization (Pedro owns the drive motors and Pinpoint)
    public Follower follower;

    // Intake + storage
    public DcMotorEx intakeMotor;
    public NormalizedColorSensor intakeColorSensor;
    public DigitalChannel intakeBeamLow;
    public DigitalChannel intakeBeamHigh;
    public DigitalChannel storageExitBeam;

    // Launcher
    public DcMotorEx launchMotor1;
    public Servo hoodAngleServo1;
    public Servo hoodAngleServo2;
    public Servo hoodGapServo1;
    public Servo hoodGapServo2;

    // Turret
    public CRServo turretServo;
    public DcMotorEx turretEncoder;
    public DigitalChannel turretHallEffect;

    // Trigger + gate
    public DcMotorEx triggerMotor;
    public Servo gateServo;

    // Sensors
    public Limelight3A limelight;
    public VoltageSensor voltageSensor;

    /**
     * Last power commanded to the intake (Intake writes it, Storage reads it to know the roller
     * direction). Reading intakeMotor.getPower() instead would be an extra hub round trip.
     */
    public double intakePowerCommand = 0;

    private final com.qualcomm.robotcore.util.ElapsedTime voltageTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private double cachedVoltage = -1;

    private List<LynxModule> hubs;

    /** Full robot - use in comp TeleOp and autos. */
    public void init(HardwareMap hardwareMap) {
        initHubs(hardwareMap);
        follower = Constants.create(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.INTAKE_COLOR_SENSOR);
        intakeBeamLow = initBeam(hardwareMap, RobotConfig.INTAKE_BEAM_LOW);
        intakeBeamHigh = initBeam(hardwareMap, RobotConfig.INTAKE_BEAM_HIGH);
        storageExitBeam = initBeam(hardwareMap, RobotConfig.STORAGE_EXIT_BEAM);

        initLauncherMotor(hardwareMap);
        hoodAngleServo1 = hardwareMap.get(Servo.class, RobotConfig.HOOD_ANGLE_SERVO_1);
        hoodAngleServo2 = hardwareMap.get(Servo.class, RobotConfig.HOOD_ANGLE_SERVO_2);
        hoodAngleServo2.setDirection(Servo.Direction.REVERSE); // mirrored on the robot
        hoodGapServo1 = hardwareMap.get(Servo.class, RobotConfig.HOOD_GAP_SERVO_1);
        hoodGapServo2 = hardwareMap.get(Servo.class, RobotConfig.HOOD_GAP_SERVO_2);
        hoodGapServo2.setDirection(Servo.Direction.REVERSE);

        turretServo = hardwareMap.get(CRServo.class, RobotConfig.TURRET_SERVO);
        turretEncoder = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET_ENCODER);
        // No encoder reset: Turret keeps its own zero offset (found by homing).
        turretHallEffect = hardwareMap.get(DigitalChannel.class, RobotConfig.TURRET_HALL);
        turretHallEffect.setMode(DigitalChannel.Mode.INPUT);

        triggerMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.TRIGGER_MOTOR);
        triggerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gateServo = hardwareMap.get(Servo.class, RobotConfig.GATE_SERVO);

        limelight = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(RobotConfig.HIVE_PIPELINE);
        limelight.start();

        voltageSensor = hardwareMap.get(VoltageSensor.class, RobotConfig.VOLTAGE_SENSOR);
    }

    private DigitalChannel initBeam(HardwareMap hardwareMap, String name) {
        DigitalChannel beam = hardwareMap.get(DigitalChannel.class, name);
        beam.setMode(DigitalChannel.Mode.INPUT);
        return beam;
    }

    /** Launcher motor only - for the flywheel test/tuning OpModes on the shooter bench. */
    public void initLauncher(HardwareMap hardwareMap) {
        initHubs(hardwareMap);
        initLauncherMotor(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, RobotConfig.VOLTAGE_SENSOR);
    }

    private void initLauncherMotor(HardwareMap hardwareMap) {
        launchMotor1 = hardwareMap.get(DcMotorEx.class, RobotConfig.LAUNCH_MOTOR);
        launchMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Bulk reads: one hub transaction per loop instead of one per sensor read.
     * MANUAL mode means {@link #clearBulkCache()} MUST be called at the top of every loop.
     */
    private void initHubs(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /** Call once at the very top of every loop. */
    public void clearBulkCache() {
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).clearBulkCache();
        }
    }

    /** Battery voltage. Not part of the bulk read (~2 ms), so it's re-read at most every 0.25 s. */
    public double getVoltage() {
        if (cachedVoltage < 0 || voltageTimer.seconds() > 0.25) {
            voltageTimer.reset();
            cachedVoltage = voltageSensor.getVoltage();
        }
        return cachedVoltage;
    }
}
