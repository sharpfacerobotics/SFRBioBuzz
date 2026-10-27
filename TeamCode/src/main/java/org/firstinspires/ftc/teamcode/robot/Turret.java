package org.firstinspires.ftc.teamcode.robot;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class Turret {
    private final RobotHardware robot;
    // Power per radian of error (kD per rad/s); kS kicks through static friction
    public static double kP = 1.2;
    public static double kD = 0.05;
    public static double kS = 0.04;
    // TODO(measure): encoder ticks per turret radian (8192 CPR through-bore at 5:1 assumed)
    public static double TICKS_PER_RAD = 8192.0 * 5.0 / (2.0 * Math.PI);
    public static boolean ENCODER_REVERSED = false;
    // Soft limits, radians from straight forward (+ = left). Keep inside what the wiring allows.
    public static double MIN_ANGLE = -Math.PI / 2;
    public static double MAX_ANGLE = Math.PI / 2;
    public static double ANGLE_TOLERANCE = Math.toRadians(1.5);
    // Homing: sweep out and back from where it starts, doubling the sweep until the hall triggers
    public static double HOMING_POWER = 0.2;
    public static double HOMING_FIRST_SWEEP = Math.toRadians(20);
    public static double HOMING_MAX_SWEEP = Math.toRadians(180);
    // TODO(measure): turret angle (rad, + = left) where the hall sensor triggers
    public static double HALL_ANGLE = 0.0;
    // REV magnetic limit switches read false when the magnet is there
    public static boolean HALL_ACTIVE_LOW = true;
    private double lastError = 0.0;
    private double power = 0.0;
    private double targetAngle = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double goalHeading = 0.0;
    private double zeroOffsetTicks = 0.0;
    private double sweep = 0.0;
    private int sweepDirection = 1;
    public enum TurretState {
        LOCALIZING,
        AIMING,
        UNWINDING,
        HOMING_FAILED
    }
    private TurretState turretState = TurretState.LOCALIZING;
    public Turret (RobotHardware robot) {
        this.robot = robot;
    }

    /** Point the turret straight forward first: this position becomes angle 0. */
    public void zeroHere() {
        zeroOffsetTicks = robot.turretEncoder.getCurrentPosition();
        lastError = 0.0;
        targetAngle = 0.0;
        turretState = TurretState.AIMING;
        timer.reset();
    }

    /** Start finding zero with the hall sensor; call updateHoming() every loop until isZeroed(). */
    public void startHoming() {
        zeroOffsetTicks = robot.turretEncoder.getCurrentPosition(); // sweeps are measured from here
        sweep = HOMING_FIRST_SWEEP;
        sweepDirection = 1;
        turretState = TurretState.LOCALIZING;
    }

    public void updateHoming() {
        if (turretState != TurretState.LOCALIZING) {
            return;
        }
        if (isHallTriggered()) {
            double raw = robot.turretEncoder.getCurrentPosition();
            double sign = ENCODER_REVERSED ? -1.0 : 1.0;
            zeroOffsetTicks = raw - sign * HALL_ANGLE * TICKS_PER_RAD;
            stop();
            lastError = 0.0;
            targetAngle = getAngle();
            turretState = TurretState.AIMING;
            timer.reset();
            return;
        }
        double angle = getAngle();
        if (sweepDirection > 0 && angle >= sweep) {
            sweepDirection = -1;
        } else if (sweepDirection < 0 && angle <= -sweep) {
            if (sweep >= HOMING_MAX_SWEEP) {
                stop();
                turretState = TurretState.HOMING_FAILED;
                return;
            }
            sweep = Math.min(sweep * 2, HOMING_MAX_SWEEP);
            sweepDirection = 1;
        }
        power = sweepDirection * HOMING_POWER;
        robot.turretServo.setPower(power);
    }

    /** Drive toward this angle (radians from forward, + = left), clipped to the soft limits. */
    public void aimAt(double angle) {
        double deltaTime = timer.seconds();
        timer.reset();
        targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        double error = targetAngle - getAngle();
        double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0.0;
        lastError = error;
        if (Math.abs(error) < ANGLE_TOLERANCE) {
            setPowerWithinLimits(0.0);
        } else {
            setPowerWithinLimits(kP * error + kD * derivative + Math.signum(error) * kS);
        }
    }

    /** Open-loop power (+ = left), still stopped at the soft limits. */
    public void setManualPower(double manualPower) {
        lastError = 0.0;
        timer.reset();
        setPowerWithinLimits(manualPower);
    }

    private void setPowerWithinLimits(double requested) {
        double angle = getAngle();
        power = Range.clip(requested, -RobotConfig.MAX_TURRET_POWER, RobotConfig.MAX_TURRET_POWER);
        if ((angle >= MAX_ANGLE && power > 0) || (angle <= MIN_ANGLE && power < 0)) {
            power = 0.0;
        }
        robot.turretServo.setPower(power);
    }

    public void stop() {
        power = 0.0;
        robot.turretServo.setPower(0.0);
    }

    public double getAngle() {
        double ticks = robot.turretEncoder.getCurrentPosition() - zeroOffsetTicks;
        return (ENCODER_REVERSED ? -ticks : ticks) / TICKS_PER_RAD;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        return targetAngle - getAngle();
    }

    public double getPower() {
        return power;
    }

    public boolean isZeroed() {
        return turretState == TurretState.AIMING;
    }

    public TurretState getState() {
        return turretState;
    }

    public boolean isOnTarget() {
        return isZeroed() && Math.abs(getError()) < ANGLE_TOLERANCE;
    }

    public boolean isHallTriggered() {
        return robot.turretHallEffect.getState() != HALL_ACTIVE_LOW;
    }

    public void localizeTurret() {
        // Cow path search for the hall effect zero point: see startHoming() / updateHoming()
        updateHoming();
    }

    public void restartTimer() {
        timer.reset();
    }
    private void relocalizeRotationIfLimitHit() {
        // if state != unwinding
                //if (limitSwitch) {
                //	reset encoder
        //}
    }
    private int targetHeadingToTargetPos(double targetHeading) {
        //gear math + robot orientation and such
        return 90; //type cast or smth
    }
    private double[] vectorSubtraction(double[] a, double[] b) {
        //vector subtraction logic
        //only get first two elements on list a
        return new double[2]; //placeholder
    }

    private void emergencyUnWind(boolean decreaseTick) {
        turretState = TurretState.UNWINDING;
        //if encoder pos is above limits, update state
        //unwind in the other direction until back to safety zone
        // if in saftey zone, update unwinding
    }

    private void updateGoalHeading() {
        // elementVelocity is [x, y, ...]; heading is measured from +x like the robot pose
        goalHeading = Math.atan2(ShooterConfig.elementVelocity[1], ShooterConfig.elementVelocity[0]);
    }

    public boolean goodTurretConfig() {
        //IMPORTANT: DO NOT USE limelight ta, since later will have shoot on the move
        // compare current heading to the target heading in ShooterConfig
        updateGoalHeading();
        return Math.abs(AngleUnit.normalizeRadians(ShooterConfig.currentRobotHeading - goalHeading)) < RobotConfig.TURRET_HEADING_TOLERANCE;
    }

}
