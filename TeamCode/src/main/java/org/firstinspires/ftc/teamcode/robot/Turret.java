package org.firstinspires.ftc.teamcode.robot;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Turret {
    private final RobotHardware robot;
    private final CRServo turretServo1;
    private final CRServo turretServo2;
    private final TouchSensor hallEffectSensor;
    private final DcMotorEx turretEncoder;

    public static double kP = 0.0001;
    public static double kD = 0.0000;
    public static double kF = 0.0;
    private double lastError = 0.0;
    private double power = 0.0;
    private int unwindTarget;


    private double localizeDx = 10;
    private double zeroOffset = 0;
    private double startTicks = 0;

    private double currPos = 0;
    private double activeTargetTicks = Double.NaN;
    private double continuousTargetHeading = Double.NaN;

    private enum LocalizeState {
        NEGATIVE,
        UNDEFINED, POSITIVE
    }
    private LocalizeState localizeState = LocalizeState.NEGATIVE;
    private final ElapsedTime timer = new ElapsedTime();
    private double goalHeading = Math.toDegrees(Math.atan2(ShooterConfig.elementVelocity[1], ShooterConfig.elementVelocity[0]));
    private enum TurretState {
        LOCALIZING,
        AIMING,
        UNWINDING
    }
    private TurretState turretState = TurretState.LOCALIZING;
    private boolean limitWasHit = false;
    //encoder resolution, ADJUST
    // One revolution of the servo gear / encoder shaft
    private static final double ENCODER_TICKS_PER_SERVO_REV = 8192.0;

    // One revolution of the turret
    private static final double TURRET_TICKS_PER_REV = ENCODER_TICKS_PER_SERVO_REV * 2.0; // 16384

    public Turret (RobotHardware robot) {
        this.robot = robot;
        turretServo1 = robot.turretServo1;
        turretServo2 = robot.turretServo2;
        hallEffectSensor = robot.hallEffectSensor;
        turretEncoder = robot.turretEncoder;
        startTicks = turretEncoder.getCurrentPosition();
    }

    private void aim(double deltaTime) {
        int transformedTargetPosition = targetHeadingToTargetPos(goalHeading);
        activeTargetTicks = transformedTargetPosition;
        rotateTo(transformedTargetPosition, deltaTime);
    }

    private void rotateTo(int targetTurretHeading, double deltaTime) {
        int transformedTargetPosition = targetTurretHeading;
        double tickError = transformedTargetPosition - currPos;
        double headingError = goalHeading - ShooterConfig.currentRobotHeading;
        double Pterm = kP * tickError;
        double Dterm = 0;
        double Fterm = kF * headingError;
        if (deltaTime > 0) {
            Dterm = kD * (tickError - lastError) / deltaTime;
        }
        if (Math.abs(tickError) < RobotConfig.TURRET_TICK_TOLERANCE) {
            power = 0;
        } else {
            power = Range.clip(Pterm + Dterm + Fterm, -RobotConfig.MAX_TURRET_POWER, RobotConfig.MAX_TURRET_POWER);
        }
        if (turretState != TurretState.UNWINDING) {

            if (power > 0 && currPos >= RobotConfig.MAX_TURRET_TICKS - RobotConfig.TURRET_LIMIT_TICK_TOLERANCE) {
                emergencyUnWind(deltaTime);
                return;
            } else if (power < 0 && currPos <= RobotConfig.MIN_TURRET_TICKS + RobotConfig.TURRET_LIMIT_TICK_TOLERANCE) {
                emergencyUnWind(deltaTime);
                return;
            }
        }
        if ((power > 0 && currPos >= RobotConfig.MAX_TURRET_TICKS - RobotConfig.TURRET_LIMIT_TICK_TOLERANCE)
                || (power < 0 && currPos <= RobotConfig.MIN_TURRET_TICKS + RobotConfig.TURRET_LIMIT_TICK_TOLERANCE)) {
            power = 0;
        }
        robot.turretServo1.setPower(power);
        robot.turretServo2.setPower(power);
        lastError = tickError;
    }


    public void localizeTurret() {
        //use the Cow Path Problem algorithm to find the hall effect zero point
        if (!hallEffectSensor.isPressed()) {

            if (startTicks + localizeDx > RobotConfig.MAX_TURRET_TICKS || startTicks - localizeDx < RobotConfig.MIN_TURRET_TICKS) {
                localizeState = LocalizeState.UNDEFINED;
            }
            switch(localizeState) {
                case NEGATIVE:
                    if (currPos <= (startTicks - localizeDx)) {
                        localizeState = LocalizeState.POSITIVE;
                        localizeDx *= 2;
                        break;
                    }
                    turretServo1.setPower(-0.5);
                    turretServo2.setPower(-0.5);
                    break;
                case POSITIVE:
                    if (currPos >= (startTicks + localizeDx)) {
                        localizeState = LocalizeState.NEGATIVE;
                        localizeDx *= 2;
                        break;
                    }
                    turretServo1.setPower(0.5);
                    turretServo2.setPower(0.5);
                    break;
                case UNDEFINED:
                    zeroOffset = startTicks;
                    turretState = TurretState.AIMING;
                    turretServo1.setPower(0.0);
                    turretServo2.setPower(0.0);
                    break;
            }
        } else {
            turretServo1.setPower(0.0);
            turretServo2.setPower(0.0);
            zeroOffset = currPos;
            turretState = TurretState.AIMING;
        }
    }

    public void updateTurret() {
        double deltaTime = timer.seconds();
        goalHeading = Math.toDegrees(Math.atan2(ShooterConfig.elementVelocity[1], ShooterConfig.elementVelocity[0]));
        currPos = robot.turretEncoder.getCurrentPosition() - zeroOffset;
        timer.reset();
        if (turretState == TurretState.LOCALIZING) {
            localizeTurret();
            return;
        }
        if (turretState != TurretState.UNWINDING) {
            aim(deltaTime);
        } else {
            continueUnWind(deltaTime);
        }
    }


    public void restartTimer() {
        timer.reset();
    }
    private void relocalizeRotationIfLimitHit() {
        if (turretState != TurretState.UNWINDING) {
            if (robot.hallEffectSensor.isPressed() && !limitWasHit) {
                zeroOffset = robot.turretEncoder.getCurrentPosition();
            }
        }
        limitWasHit = robot.hallEffectSensor.isPressed();
    }
    private int targetHeadingToTargetPos(double targetHeading) {
        double targetTurretHeading = targetHeading - ShooterConfig.currentRobotHeading;
        if (Double.isNaN(continuousTargetHeading)) {
            continuousTargetHeading = Math.IEEEremainder(targetTurretHeading, 360.0);
        } else {
            continuousTargetHeading += Math.IEEEremainder(targetTurretHeading - continuousTargetHeading, 360.0);
        }
        int targetPos = (int) Math.round(continuousTargetHeading / 360.0 * TURRET_TICKS_PER_REV);
        return targetPos;
    }
    private double[] vectorSubtraction(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1]};
    }

    private void emergencyUnWind(double deltaTime) {
        int transformedTargetPosition = targetHeadingToTargetPos(goalHeading);
        Integer target = transformTickToValidRange(transformedTargetPosition);
        if (target == null) {
            power = 0;
            activeTargetTicks = Double.NaN;
            turretServo1.setPower(0);
            turretServo2.setPower(0);
            return;
        }
        unwindTarget = target;
        continuousTargetHeading += (unwindTarget - transformedTargetPosition) / TURRET_TICKS_PER_REV * 360.0;
        activeTargetTicks = unwindTarget;
        turretState = TurretState.UNWINDING;
        rotateTo(unwindTarget, deltaTime);
    }

    private Integer transformTickToValidRange(int tickINT) {
        if (isValid(tickINT)) return tickINT;
        double tick = (double) tickINT;
        if (tick < RobotConfig.MIN_TURRET_TICKS + RobotConfig.TURRET_LIMIT_TICK_TOLERANCE) {
            for (int i = 0; i < 10; ++i) {
                tick += TURRET_TICKS_PER_REV;
                if (isValid(tick)) {
                    return (int) tick;
                }
            }
        } else {
            for (int i = 0; i < 10; ++i) {
                tick -= TURRET_TICKS_PER_REV;
                if (isValid(tick)) {
                    return (int) tick;
                }
            }
        }
        return null;
    }

    private boolean isValid(double tick) {
        return tick >= RobotConfig.MIN_TURRET_TICKS + RobotConfig.TURRET_LIMIT_TICK_TOLERANCE
                && tick <= RobotConfig.MAX_TURRET_TICKS - RobotConfig.TURRET_LIMIT_TICK_TOLERANCE;
    }

    private void continueUnWind(double deltaTime) {
        rotateTo(unwindTarget, deltaTime);
        if (Math.abs(currPos - unwindTarget) < RobotConfig.TURRET_TICK_TOLERANCE) {
            turretState = TurretState.AIMING;
            activeTargetTicks = Double.NaN;
        }
    }
    
    public boolean goodTurretConfig() {
        return turretState == TurretState.AIMING
                && Math.abs(currPos - activeTargetTicks)
                < RobotConfig.TURRET_TICK_TOLERANCE;
    }

    public void testing() {
        robot.turretServo1.setPower(1.0);
        robot.turretServo2.setPower(1.0);
    }

}
