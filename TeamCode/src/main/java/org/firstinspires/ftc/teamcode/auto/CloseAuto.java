package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Vision;

public abstract class CloseAuto extends BaseAuto {

    // --- REUSABLE: General field positions ---
    protected Pose intakeOne, intakeTwo, intakeThree;

    protected Pose controlPointToIntakeTwo   = new Pose(55.51, 54.81, Math.toRadians(185));
    protected Pose controlPointToIntakeThree = new Pose(90, 31.2,     Math.toRadians(185));

    // --- REUSABLE: Pathing sequences ---
    private PathChain driveStartShoot;
    private PathChain driveShootIntakeOne,   driveIntakeOneShoot;
    private PathChain driveShootIntakeTwo,   driveIntakeTwoShoot;
    private PathChain driveShootIntakeThree, driveIntakeThreeShoot;

    /* === DECODE SPECIFIC: Shooter timings and variables ===
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
    */

    // --- REUSABLE: State machine structures ---
    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKEONE,   DRIVE_INTAKEONE_SHOOTPOS,   SHOOT_SAMPLES_1,
        DRIVE_SHOOTPOS_INTAKETWO,   DRIVE_INTAKETWO_SHOOTPOS,   SHOOT_SAMPLES_2,
        DRIVE_SHOOTPOS_INTAKETHREE, DRIVE_INTAKETHREE_SHOOTPOS, SHOOT_SAMPLES_3,
        DONE
    }

    protected boolean isTargetLocked = false;
    protected PathState pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;

    protected void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        // isShooting = false; // DECODE SPECIFIC flag
    }

    @Override
    protected void buildPaths() {
        // --- REUSABLE: PedroPathing curve generation ---
        // (Note: 'shootPose' will just act as your general 'scorePose' for future seasons)
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootIntakeOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakeOne))
                .setConstantHeadingInterpolation(intakeOne.getHeading())
                .build();
        driveIntakeOneShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakeOne, shootPose))
                .setLinearHeadingInterpolation(intakeOne.getHeading(), shootPose.getHeading(), 0.6)
                .build();
        driveShootIntakeTwo = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPointToIntakeTwo, intakeTwo))
                .setConstantHeadingInterpolation(intakeTwo.getHeading())
                .build();
        driveIntakeTwoShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakeTwo, shootPose))
                .setLinearHeadingInterpolation(intakeTwo.getHeading(), shootPose.getHeading(), 0.6)
                .build();
        driveShootIntakeThree = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPointToIntakeThree, intakeThree))
                .setConstantHeadingInterpolation(intakeThree.getHeading())
                .build();
        driveIntakeThreeShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakeThree, shootPose))
                .setLinearHeadingInterpolation(intakeThree.getHeading(), shootPose.getHeading(), 0.6)
                .build();
    }

    @Override
    protected void statePathUpdate() {
        // --- REUSABLE: Autonomous state machine flow ---
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                // if (!isShooting) {
                    follower.followPath(driveStartShoot, true);
                    
                    /* === DECODE SPECIFIC ===
                    setLaunchPower(launchPower);
                    robot.Trigger.setPosition(triggerStartPos);
                    isShooting = true;
                    */
                // }
                if (!follower.isBusy()) { 
                    setPathState(PathState.SHOOT_PRELOAD); 
                    // shotsFired = 0; // DECODE SPECIFIC
                }
                break;

            case SHOOT_PRELOAD:
                // shootSequence(); // DECODE SPECIFIC
                
                // --- REUSABLE: Generic condition to move to next path ---
                // if (!isShooting) {
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEONE);
                    follower.followPath(driveShootIntakeOne, true);
                // }
                break;

            case DRIVE_SHOOTPOS_INTAKEONE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    // robot.transferMotor.setPower(-1); // DECODE SPECIFIC
                    follower.followPath(driveIntakeOneShoot, true);
                    setPathState(PathState.DRIVE_INTAKEONE_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKEONE_SHOOTPOS:
                if (!follower.isBusy()) {
                    /* === DECODE SPECIFIC ===
                    setLaunchPower(launchPower); 
                    robot.transferMotor.setPower(0);
                    shotsFired = 0;
                    */
                    setPathState(PathState.SHOOT_SAMPLES_1); 
                }
                break;

            case SHOOT_SAMPLES_1:
                // shootSequence(); // DECODE SPECIFIC
                // if (!isShooting) {
                    follower.followPath(driveShootIntakeTwo, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKETWO);
                // }
                break;

            case DRIVE_SHOOTPOS_INTAKETWO:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    // robot.transferMotor.setPower(-1); // DECODE SPECIFIC
                    follower.followPath(driveIntakeTwoShoot, true);
                    setPathState(PathState.DRIVE_INTAKETWO_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKETWO_SHOOTPOS:
                if (!follower.isBusy()) {
                    /* === DECODE SPECIFIC ===
                    setLaunchPower(launchPower); 
                    robot.intakeMotor.setPower(0); 
                    robot.transferMotor.setPower(0);
                    shotsFired = 0;
                    */
                    setPathState(PathState.SHOOT_SAMPLES_2); 
                }
                break;

            case SHOOT_SAMPLES_2:
                // shootSequence(); // DECODE SPECIFIC
                // if (!isShooting) {
                    follower.followPath(driveShootIntakeThree, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKETHREE);
                // }
                break;

            case DRIVE_SHOOTPOS_INTAKETHREE:
                if (!follower.isBusy()) {
                    // robot.transferMotor.setPower(-1); // DECODE SPECIFIC
                    follower.followPath(driveIntakeThreeShoot, true);
                    setPathState(PathState.DRIVE_INTAKETHREE_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKETHREE_SHOOTPOS:
                if (!follower.isBusy()) {
                    /* === DECODE SPECIFIC ===
                    setLaunchPower(launchPower); 
                    robot.transferMotor.setPower(0);
                    shotsFired = 0;
                    */
                    setPathState(PathState.SHOOT_SAMPLES_3); 
                }
                break;

            case SHOOT_SAMPLES_3:
                // shootSequence(); // DECODE SPECIFIC
                // if (!isShooting) {
                    setPathState(PathState.DONE);
                // }
                break;

            case DONE:
                telemetry.addLine("Autonomous Complete!");
                break;

            default:
                telemetry.addLine("No State Set");
                break;
        }
    }

    /* === DECODE SPECIFIC: The entire shooting logic is tied to DECODE artifacts ===
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
                telemetry.addLine("ERROR: Unknown launchState: " + launchState);
                launchState = LaunchState.WAITING;
                launchTimer.reset();
                break;
        }
    }
    */

    // --- REUSABLE: Limelight Vision Alignment ---
    // (This calculates yaw error and adjusts the drivetrain directly. Highly reusable!)
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
            
            // launchTimer.reset(); // DECODE SPECIFIC
        }
    }

    /* === DECODE SPECIFIC: Combination of vision and shooting sequence ===
    private void shootSequence() {
        isShooting = true;
        if (!isTargetLocked) {
            if (follower.isBusy()) {
                follower.breakFollowing();
            }
            lockOn(); // REUSABLE, but called inside this decode specific method
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
    */
}
