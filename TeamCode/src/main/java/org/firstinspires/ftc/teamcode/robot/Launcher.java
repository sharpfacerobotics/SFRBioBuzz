package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.helperClasses.ShotDetector;

import java.util.Queue;

/**
 * Flywheel + hood. Owner: Alex.
 *
 * <p>Target velocity and hood angle come from ShooterConfig; the next element comes from
 * Storage. The OpMode passes both in, so Launcher never reaches into another subsystem.</p>
 */
@Configurable
public class Launcher {
    public static double kP = 0.00006;
    public static double kF = 0.0006;

    public enum ControlMode {
        /** Motor controller's built-in velocity PIDF (setVelocity). */
        VELOCITY,
        /** Our own P + feedforward, voltage compensated. */
        PF,
        /** Full power below target, off above. Fastest recovery, noisiest. */
        BANG_BANG
    }
    /** Mode used in competition. Tuning OpModes use setControlModeOverride() instead of this. */
    public static ControlMode controlMode = ControlMode.VELOCITY;

    private final RobotHardware robot;
    private final ElapsedTime hoodTimer = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();
    private final ShotDetector shotDetector = new ShotDetector();

    private ControlMode appliedMode = null;
    private ControlMode modeOverride = null;
    private double targetVel;
    private double currVel1;
    private double voltage = RobotConfig.NOMINAL_VOLTAGE;
    private double hoodAnglePos = -1;
    private String hoodElement = null;

    private boolean shotDetected = false;

    public Launcher(RobotHardware robot) {
        this.robot = robot;
    }

    // ------------------------------------------------------------------ comp method

    // Glossary: setLauncher(Queue<String> q) SET, setHoodRetraction(Queue<String> q) SET,
    // goodShooterConfig() STATUS, setShooterWallSpeed() SET. The target speed and hood angle
    // come from ShooterConfig's static outputs (team design: no speed parameter here).

    /**
     * SET. Loop call. Applies the target flywheel speed and hood config from ShooterConfig
     * for the next element ({@code q.peek()}).
     */
    public void setLauncher(Queue<String> q) {
        setBatteryVoltage(robot.getVoltage());
        setHoodRetraction(q);
        setHoodAngle(ShooterConfig.hoodAngleFor(q.peek()));
        setLauncher(ShooterConfig.targetVelocity());
        detectShot();
    }

    /**
     * SET. Loop call instead of setLauncher(q) for a wall shot: preset flywheel speed and hood
     * angle from a known spot against the wall, so it's CONSISTENT without odometry or vision.
     */
    public void setShooterWallSpeed() {
        setBatteryVoltage(robot.getVoltage());
        setHoodAngle(RobotConfig.WALL_SHOT_HOOD);
        setLauncher(RobotConfig.WALL_SHOT_VELOCITY);
        detectShot();
    }

    /**
     * Testing method (SubsystemTest, ShotTableTuner): set flywheel speed and hood by hand
     * instead of from ShooterConfig. Loop call.
     */
    public void setManual(double velocity, double hoodAngle, String element) {
        setBatteryVoltage(robot.getVoltage());
        setHoodRetraction(element);
        setHoodAngle(hoodAngle);
        setLauncher(velocity);
        detectShot();
    }

    /** STATUS. True if the flywheel and hood are ready - check before firing. */
    public boolean goodShooterConfig() {
        return isReady();
    }

    /** SET. Hood linear retraction (gap) for the next element in the queue ({@code .peek()}). */
    public void setHoodRetraction(Queue<String> q) {
        setHoodRetraction(q.peek());
    }

    /** For the tuning OpModes (which call setLauncher directly): PF voltage compensation. */
    public void setBatteryVoltage(double batteryVoltage) {
        if (batteryVoltage > 1) {
            voltage = batteryVoltage;
        }
    }

    public void stop() {
        targetVel = 0;
        robot.launchMotor1.setPower(0);
        shotDetector.reset();
    }

    // ------------------------------------------------------------------ testing methods

    /** Velocity only - used by the flywheel tuning OpModes, which have no hood mapped. */
    public void setLauncher(double targetVel) {
        this.targetVel = targetVel;
        currVel1 = robot.launchMotor1.getVelocity();
        applyMode();

        if (targetVel <= 0) {
            robot.launchMotor1.setPower(0);
            return;
        }
        switch (activeMode()) {
            case PF:
                setVelocityPF();
                break;
            case BANG_BANG:
                setVelocityBang();
                break;
            default:
                setVelocityNormal();
                break;
        }
    }

    /**
     * Use this mode for this Launcher only. Kept per-instance on purpose: setting the static
     * controlMode from a tuning OpMode would leak into the next comp OpMode.
     */
    public void setControlModeOverride(ControlMode mode) {
        modeOverride = mode;
    }

    public ControlMode activeMode() {
        return modeOverride != null ? modeOverride : controlMode;
    }

    public void setkP(double kP) {
        Launcher.kP = kP;
    }

    public void setkF(double kF) {
        Launcher.kF = kF;
    }

    // ------------------------------------------------------------------ status

    /** Flywheel at speed and hood has had time to move. Feed Trigger.fire() with this. */
    public boolean isReady() {
        return targetVel > 0
                && Math.abs(targetVel - currVel1) < RobotConfig.FLYWHEEL_VELOCITY_TOLERANCE
                && hoodTimer.seconds() > RobotConfig.HOOD_SETTLE_TIME;
    }

    /**
     * True for exactly one loop when an element leaves: the flywheel was at speed and
     * suddenly dipped. The OpMode should call Storage.removeNext() when this is true.
     */
    public boolean shotDetected() {
        return shotDetected;
    }

    public double getTargetVelocity() {
        return targetVel;
    }

    public double getCurrVelocity() {
        return currVel1;
    }

    // ------------------------------------------------------------------ internals

    private void applyMode() {
        ControlMode mode = activeMode();
        if (appliedMode == mode) {
            return;
        }
        // Built-in velocity control needs the encoder loop; our own controllers must not
        // fight it, so they run raw power.
        robot.launchMotor1.setMode(mode == ControlMode.VELOCITY
                ? DcMotor.RunMode.RUN_USING_ENCODER
                : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        appliedMode = mode;
    }

    private void setVelocityNormal() {
        robot.launchMotor1.setVelocity(targetVel);
    }

    private void setVelocityBang() {
        robot.launchMotor1.setPower(currVel1 < targetVel ? 1.0 : 0.0);
    }

    private void setVelocityPF() {
        double voltageComp = RobotConfig.NOMINAL_VOLTAGE / voltage;
        double pow1 = kF * targetVel * voltageComp + kP * (targetVel - currVel1);
        robot.launchMotor1.setPower(Range.clip(pow1, -1.0, 1.0));
    }

    private void detectShot() {
        shotDetected = shotDetector.update(isReady(), currVel1, clock.seconds(),
                RobotConfig.SHOT_DETECT_DROP, RobotConfig.SHOT_DETECT_WINDOW);
    }

    private void setHoodAngle(double angle) {
        angle = Range.clip(angle, RobotConfig.HOOD_ANGLE_MIN, RobotConfig.HOOD_ANGLE_MAX);
        // Only write when it actually changes. Small changes (the hood tracking distance as we
        // drive) settle within a loop, so only a real jump restarts the settle wait - otherwise
        // the launcher would never be "ready" while driving toward or away from the hive.
        double change = Math.abs(angle - hoodAnglePos);
        if (change > 0.005) {
            if (change > RobotConfig.HOOD_SETTLE_JUMP) {
                hoodTimer.reset();
            }
            hoodAnglePos = angle;
            robot.hoodAngleServo1.setPosition(angle);
            robot.hoodAngleServo2.setPosition(angle + RobotConfig.HOOD_ANGLE_SERVO_2_OFFSET);
        }
    }

    private void setHoodRetraction(String next) {
        if (next == null) {
            next = "POLLEN"; // empty: still put the gap somewhere known (servos power up anywhere)
        }
        if (next.equals(hoodElement)) {
            return;
        }
        hoodElement = next;
        if ("NECTAR".equals(next)) { // NECTAR is 3.6 in, POLLEN 2.8 in
            robot.hoodGapServo1.setPosition(RobotConfig.HOOD_NECTAR_GAP_1);
            robot.hoodGapServo2.setPosition(RobotConfig.HOOD_NECTAR_GAP_2);
        } else {
            // POLLEN, and UNKNOWN falls back to pollen
            robot.hoodGapServo1.setPosition(RobotConfig.HOOD_POLLEN_GAP_1);
            robot.hoodGapServo2.setPosition(RobotConfig.HOOD_POLLEN_GAP_2);
        }
        hoodTimer.reset();
    }
}
