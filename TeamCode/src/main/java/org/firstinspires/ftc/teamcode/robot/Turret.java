package org.firstinspires.ftc.teamcode.robot;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Turret {
    private final RobotHardware robot;
    public static double kP = 0.0001;
    public static double kD = 0.0000;
    public static double kF = 0.0;
    private double lastError = 0.0;
    private double power = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double goalHeading = Math.atan2(ShooterConfig.elementConfig[1], ShooterConfig.elementConfig[0]);
    private enum TurretState {
        LOCALIZING,
        AIMING,
        UNWINDING
    }
    private TurretState turretState = TurretState.LOCALIZING;
    private boolean limitWasHit = false;
    private double zerOffest1 = 0;
    private double zerOffest2 = 0;
    // Encoder resolution (adjust based on your ELC encoder specs)
    private static final double TICKS_PER_REV = 8192.0;
    public Turret (RobotHardware robot) {
        this.robot = robot;
    }
    private void aim(double deltaTime) {
        int transformedTargetPosition = targetHeadingToTargetPos(goalHeading); //smth like that
        double currPos = robot.turretEncoder1.getCurrentPosition();
        double tickError = transformedTargetPosition - currPos;
        double headingError = goalHeading - ShooterConfig.currentRobotHeading;
        double Pterm = kP * tickError;
        double Dterm = 0;
        double Fterm = kF * headingError;
        if (deltaTime > 0) {
            Dterm = kD * (tickError - lastError) / deltaTime;
        }
        if (tickError < RobotConfig.TURRET_TICK_TOLERANCE) {
            power = 0;
        } else {
            power = Range.clip(Pterm + Dterm + Fterm, -RobotConfig.MAX_TURRET_POWER, RobotConfig.MAX_TURRET_POWER);
        }

        if (power > 0 && Math.abs(currPos - RobotConfig.MAX_TURRET_TICKS) < RobotConfig.TURRET_LIMIT_TICK_TOLERANCE) {
            emergencyUnWind(true);
            return;
        } else if (power < 0 && Math.abs(currPos - RobotConfig.MIN_TURRET_TICKS) < RobotConfig.TURRET_LIMIT_TICK_TOLERANCE) {
            emergencyUnWind(false);
            return;
        }
        //configure as needed
        robot.turretServo1.setPower(power);
        robot.turretServo2.setPower(power);
        lastError = tickError;

    }


    public void localizeTurret() {
        //use the Cow Path Problem algorithm to find the hall effect zero point
    }

    public void updateTurret() {
        double deltaTime = timer.seconds();
        timer.reset();
        if (turretState == TurretState.LOCALIZING) {
            localizeTurret();
            return;
        }
        // put inside aim -> emergencyUnWindIfNeeded();
        relocalizeRotationIfLimitHit();
        if (turretState != TurretState.UNWINDING) {
            aim(deltaTime);
        }
    }


    public void restartTimer() {
        timer.reset();
    }
    private void relocalizeRotationIfLimitHit() {
        if (turretState != TurretState.UNWINDING) {
            if (robot.magLimitSwitch.isPressed() && !limitWasHit) {
                zerOffest1 = robot.turretEncoder1.getCurrentPosition();
            }
        }
        limitWasHit = robot.magLimitSwitch.isPressed();
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
    public boolean goodTurretConfig() {
        //IMPORTANT: DO NOT USE limelight ta, since later will have shoot on the move
        // compare current heading to the target heading in ShooterConfig
        return Math.abs(ShooterConfig.currentRobotHeading - goalHeading) < RobotConfig.TURRET_HEADING_TOLERANCE;
    }

}
