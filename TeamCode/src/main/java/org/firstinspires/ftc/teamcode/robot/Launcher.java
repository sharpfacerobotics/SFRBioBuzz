package org.firstinspires.ftc.teamcode.robot;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Queue;

@Configurable
public class Launcher {
    public static double kP = 0.00006;
    public static double kF = 0.0006;
    // kF was tuned at this voltage; a low battery gets proportionally more power
    public static double NOMINAL_VOLTAGE = 12.0;
    // Ready to fire when within this fraction of the target velocity
    public static double AT_SPEED_TOLERANCE = 0.02;
    // Open-loop (no table yet): call it spun up this long after the power last changed
    public static double POWER_SPIN_UP_TIME = 1.5;
    // TODO(tune): hood servo positions
    public static double HOOD_ANGLE_DEFAULT = 0.5;   // until the table has hood angles
    public static double HOOD_POLLEN_GAP = 0.40;
    public static double HOOD_NECTAR_GAP = 0.55;
    // Servos give no feedback, so wait this long after a move before calling the hood set
    public static double HOOD_SETTLE_TIME = 0.15;

    private static final double VOLTAGE_READ_PERIOD = 0.1;
    private static final double MIN_VALID_VOLTAGE = 6.0;
    private static final double HOOD_MOVE_THRESHOLD = 0.005;
    private static final double POWER_CHANGE_THRESHOLD = 0.02;

    private RobotHardware robot;
    public Launcher(RobotHardware robot) {
        this.robot = robot;
    }
    private double targetVel;
    private double currVel1;
    private double voltage = NOMINAL_VOLTAGE;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double openLoopPower = 0.0;
    private final ElapsedTime openLoopTimer = new ElapsedTime();
    private double hoodAnglePos = Double.NaN;
    private double hoodGapPos = Double.NaN;
    private final ElapsedTime hoodTimer = new ElapsedTime();

    /**
     * Flywheel from ShooterConfig's latest solution: table velocity when the table has data,
     * otherwise straight-line power from the distance.
     */
    public void setFromShooterConfig() {
        if (Double.isNaN(ShooterConfig.targetVelocity)) {
            setLauncherPower(ShooterConfig.fallbackPowerForDistance(ShooterConfig.distanceToGoal));
        } else {
            setLauncher(ShooterConfig.targetVelocity);
        }
    }

    //testing
    public void setLauncher(double targetVel, Queue<String> q) {
        setLauncher(targetVel);
    }

    public void setLauncher(double targetVel) {
        currVel1 = robot.launchMotor1.getVelocity();
        this.targetVel = targetVel;
        openLoopPower = 0.0;
        //setVelocityNormal();
        //setVelocityBang();
        setVelocityPF();
    }

    /** Open-loop power, for before the velocity table exists. */
    public void setLauncherPower(double power) {
        currVel1 = robot.launchMotor1.getVelocity();
        targetVel = 0;
        power = Math.max(-1.0, Math.min(1.0, power));
        if (Math.abs(power - openLoopPower) > POWER_CHANGE_THRESHOLD) {
            openLoopTimer.reset();
        }
        openLoopPower = power;
        robot.launchMotor1.setPower(power);
    }

    /** Coast down. (setLauncher(0) would drive the wheel backwards through the P term.) */
    public void stop() {
        targetVel = 0;
        openLoopPower = 0.0;
        robot.launchMotor1.setPower(0);
        currVel1 = robot.launchMotor1.getVelocity();
    }

    private void setVelocityNormal() {
        robot.launchMotor1.setVelocity(targetVel);
    }

    private void setVelocityBang() {
        if (currVel1 < targetVel) {
            robot.launchMotor1.setPower(1.0);
        } else if (currVel1 > targetVel) {
            robot.launchMotor1.setPower(0.0);
        }
    }
    private void setVelocityPF() {
        double feedforward = kF * targetVel * NOMINAL_VOLTAGE / readVoltage();
        double pow1 = feedforward + kP * (targetVel - currVel1);
        pow1 = Math.max(-1.0, Math.min(1.0, pow1));

        robot.launchMotor1.setPower(pow1);
    }

    private double readVoltage() {
        if (robot.voltageSensor != null && voltageTimer.seconds() > VOLTAGE_READ_PERIOD) {
            double reading = robot.voltageSensor.getVoltage();
            if (reading > MIN_VALID_VOLTAGE) {
                voltage = reading;
            }
            voltageTimer.reset();
        }
        return voltage;
    }

    /** Spun up: within tolerance of the target velocity, or settled at a fixed power. */
    public boolean isAtSpeed() {
        if (targetVel > 0) {
            return Math.abs(targetVel - currVel1) <= AT_SPEED_TOLERANCE * targetVel;
        }
        return openLoopPower > 0 && openLoopTimer.seconds() > POWER_SPIN_UP_TIME;
    }

    /** Hood for the next element ("POLLEN" / "NECTAR"): angle from the table, gap per element. */
    public void setHood(String next) {
        if (robot.hoodAngleServo == null || robot.hoodGapServo == null) {
            return;
        }
        double angle = ShooterConfig.hoodAngleFor(next);
        if (Double.isNaN(angle)) {
            angle = HOOD_ANGLE_DEFAULT;
        }
        double gap = "NECTAR".equals(next) ? HOOD_NECTAR_GAP : HOOD_POLLEN_GAP;
        boolean moved = Double.isNaN(hoodAnglePos)
                || Math.abs(angle - hoodAnglePos) > HOOD_MOVE_THRESHOLD
                || Math.abs(gap - hoodGapPos) > HOOD_MOVE_THRESHOLD;
        if (moved) {
            hoodTimer.reset();
        }
        hoodAnglePos = angle;
        hoodGapPos = gap;
        robot.hoodAngleServo.setPosition(hoodAnglePos);
        robot.hoodGapServo.setPosition(hoodGapPos);
    }

    /** True when there's no hood, or it has had time to reach its last position. */
    public boolean isHoodSettled() {
        return robot.hoodAngleServo == null || hoodTimer.seconds() > HOOD_SETTLE_TIME;
    }

    public void setkP(double kP) {
        Launcher.kP = kP;
    }
    public void setkF(double kF) {
        Launcher.kF = kF;
    }

    public double getTargetVelocity() {
        return targetVel;
    }
    public double getCurrVelocity() {
        return currVel1;
    }
    public double getVoltage() {
        return voltage;
    }
    public double getHoodAnglePos() {
        return hoodAnglePos;
    }
    public double getHoodGapPos() {
        return hoodGapPos;
    }
}
