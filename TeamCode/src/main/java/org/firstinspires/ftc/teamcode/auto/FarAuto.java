package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Vision;

public abstract class FarAuto extends BaseAuto {

    protected Pose intakeThree, intakeN;
    protected Pose controlPointToIntakeThree = new Pose(82.08791208791206, 40.95604395604395, Math.toRadians(0));

    public static double launchPower23 = 0.85;
    public static double shootAngle    = 65.4;
    private static int shootNTimes = 3;
    private static int shotCycles = 0;

    private PathChain driveStartShootFar;
    private PathChain driveShootIntakeThree, driveIntakeThreeShoot;
    private PathChain driveShootIntakeN,     driveIntakeNShoot;

    private ElapsedTime launchTimer = new ElapsedTime();
    protected double launchTimeoutTime = 2.0; // TIMEOUT watcher
    protected double launchWaitTime = 0.4;    // WAITING phase
    protected double launchTransferTime = 0.5; // TRANSFER phase
    protected double launchShootTime = 0.2;    // SHOOT phase
    protected double launchResetTime = 0.15;   // RESET phase

    public enum LaunchState {
        RESET,
        WAITING,
        TRANSFER,
        SHOOT
    }
    protected LaunchState launchState = LaunchState.WAITING;
    protected boolean isTargetLocked = false;

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKE_3,
        DRIVE_INTAKE_3_SHOOTPOS,
        SHOOT_SAMPLES,
        DRIVE_SHOOTPOS_INTAKE_N,
        DRIVE_INTAKE_N_SHOOTPOS,
        DONE
    }

    protected PathState pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;

    /** Override and return true to skip the intake 3 sequence */
    protected boolean skipIntakeThree() { return false; }

    protected void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        isShooting = false;
    }

    
    @Override
    protected void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                if (!isShooting) {
                    follower.followPath(driveStartShootFar, true);
                    setLaunchPower(launchPower);
                    robot.Trigger.setPosition(triggerStartPos);
                    isShooting = true;
                }
                if (!follower.isBusy()) { setPathState(PathState.SHOOT_PRELOAD); shotsFired = 0; }
                break;

            case SHOOT_PRELOAD:
                shootSequence();

                if (!isShooting) {
                    if (skipIntakeThree()) {
                        follower.followPath(driveShootIntakeN, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_INTAKE_N);
                    } else {
                        follower.followPath(driveShootIntakeThree, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_INTAKE_3);
                    }
                }
                break;

            case DRIVE_SHOOTPOS_INTAKE_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    robot.transferMotor.setPower(-1);
                    follower.followPath(driveIntakeThreeShoot, true);
                    setPathState(PathState.DRIVE_INTAKE_3_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKE_3_SHOOTPOS:
                if (!follower.isBusy()) {
                    setLaunchPower(launchPower); robot.transferMotor.setPower(0);
                    setPathState(PathState.SHOOT_SAMPLES); shotsFired = 0;
                }
                break;

            case SHOOT_SAMPLES:
                shootSequence();
                if (!isShooting) {
                    follower.followPath(driveShootIntakeN, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKE_N);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKE_N:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    robot.transferMotor.setPower(-1);
                    follower.followPath(driveIntakeNShoot, true);
                    setPathState(PathState.DRIVE_INTAKE_N_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKE_N_SHOOTPOS:
                if (!follower.isBusy()) {
                    setLaunchPower(launchPower);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                    setPathState(PathState.SHOOT_SAMPLES);
                    shotsFired = 0;
                    ++shotCycles;
                    if (shotCycles >= shootNTimes || (!skipIntakeThree() && shotCycles >= shootNTimes - 1)) {
                        setPathState(PathState.DONE);
                    }
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous Complete!");
                break;

            default:
                telemetry.addLine("No State Set");
                break;
        }
    }

    private void shootOneShot() {
        if (shotsFired >= 3) return;

        if (launchTimer.seconds() > launchTimeoutTime) {
            launchState = LaunchState.WAITING;
            launchTimer.reset();
            telemetry.addLine("WARNING: Launch state timeout");
            return;
        }
        double elapsed = launchTimer.seconds();
        setLaunchPower(launchPower);
        switch (launchState) {
            case WAITING:
                telemetry.addLine("Waiting");
                if (elapsed > launchWaitTime) {
                    launchState = LaunchState.TRANSFER;
                    launchTimer.reset();
                }
                break;

            case TRANSFER:
                robot.transferMotor.setPower(1);
                telemetry.addLine("Shot " + (shotsFired + 1) + ": Firing");
                if (elapsed > launchTransferTime) {
                    launchState = LaunchState.SHOOT;
                    launchTimer.reset();
                }
                break;

            case SHOOT:
                robot.Trigger.setPosition(triggerShootPos);
                robot.transferMotor.setPower(0);
                if (elapsed > launchShootTime) {
                    launchState = LaunchState.RESET;
                    launchTimer.reset();
                }
                break;

            case RESET:
                robot.Trigger.setPosition(triggerStartPos);
                telemetry.addLine("Shot " + (shotsFired + 1) + ": Resetting");
                if (elapsed > launchResetTime) {
                    launchState = LaunchState.WAITING;
                    ++shotsFired;
                    launchTimer.reset();
                }
                break;

            default:
                // This should never happen, but recover gracefully
                telemetry.addLine("ERROR: Unknown launchState: " + launchState);
                launchState = LaunchState.WAITING;
                launchTimer.reset();
                break;
        }
    }

    private void lockOn() {
        LLResult result = robot.limelight.getLatestResult();   // get vision data
        vision.setHasTarget(result != null && result.isValid());

        // Use your existing vision method to get correction
        double correction = vision.getYaw(0.0, true, result);

        // Apply the correction – turn in place
        // (scale if needed, e.g., multiply by 0.6 to soften)
        correction *= 0.6;
        double turnPower = Range.clip(correction, -0.5, 0.5); // limit max turn
        robot.frontLeftDrive.setPower(-turnPower);
        robot.frontRightDrive.setPower(turnPower);
        robot.backLeftDrive.setPower(-turnPower);
        robot.backRightDrive.setPower(turnPower);

        // Check if locked on (vision class sets its own isLockedOn flag)
        if (vision.isLockedOn()) {
            isTargetLocked = true;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
            follower.resumePathFollowing();          // stop manual control
            launchTimer.reset();               // prepare for shooting

        }
    }

    private void shootSequence() {
        isShooting = true;
        if (!isTargetLocked) {
            if (follower.isBusy()) {
                follower.breakFollowing();
            }
            lockOn();
        }
        if (pathTimer.getElapsedTimeSeconds() < spinUpTime) {
            robot.intakeMotor.setPower(0);
            setLaunchPower(launchPower);
            launchTimer.reset();
            return;
        }
        if (shotsFired < 3) {
            shootOneShot();
            return;
        }
        robot.intakeMotor.setPower(1);
        robot.transferMotor.setPower(-1);
        isShooting = false;
        isTargetLocked = false;
    }
}
