package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.helperClasses.ShotDetector;

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

    /**
     * Call once per loop.
     *
     * @param targetVel      ticks/s from ShooterConfig (0 to coast down)
     * @param hoodAngle      hood servo position for this element, from ShooterConfig
     * @param next           next element: Storage.getElementQueue().peek() - "POLLEN" / "NECTAR" / null
     * @param batteryVoltage robot.getVoltage(), for PF voltage compensation
     */
    public void update(double targetVel, double hoodAngle, String next, double batteryVoltage) {
        if (batteryVoltage > 1) {
            voltage = batteryVoltage;
        }
        setHoodRetraction(next);
        setHoodAngle(hoodAngle);
        setLauncher(targetVel);
        detectShot();
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
        // Only write (and restart the settle timer) when it actually changes noticeably.
        if (Math.abs(angle - hoodAnglePos) > 0.005) {
            hoodAnglePos = angle;
            robot.hoodAngleServo1.setPosition(angle);
            robot.hoodAngleServo2.setPosition(angle + RobotConfig.HOOD_ANGLE_SERVO_2_OFFSET);
            hoodTimer.reset();
        }
    }

    private void setHoodRetraction(String next) {
        if (next == null || next.equals(hoodElement)) {
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
