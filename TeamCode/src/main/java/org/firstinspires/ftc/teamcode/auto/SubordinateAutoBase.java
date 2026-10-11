package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class SubordinateAutoBase extends LinearOpMode {
    /*
    protected Follower follower;
    protected RobotHardware robot = new RobotHardware();

    protected Command updateShooter;

    protected Pose startPose;

    protected Pose shootingPose = new Pose(58, 71, Math.toRadians(180));
    protected Pose middlePickupPose = new Pose(13.990, 55, Math.toRadians(180));
    protected Pose middlePickupControlPoint2 = new Pose(42, 57);
    protected Pose closePickupPose = new Pose(20.590, 81.860, Math.toRadians(180));
    protected Pose gateClearControlPoint = new Pose(56.090, 59.210);
    protected Pose gateClearPose = new Pose(19.847, 60.5, Math.toRadians(180));
    protected Pose gatePickupControlPoint = new Pose(20.590, 55.260);

    private double xOffset = -0.4;
    private double yOffset = 1;
    protected Pose[] gatePickupPoses = {
            new Pose(13.3 + xOffset, 56 + yOffset, Math.toRadians(150)),
            new Pose(13.3 + xOffset, 56.25 + yOffset, Math.toRadians(150)),
            new Pose(13.3 + xOffset, 56.5 + yOffset, Math.toRadians(150)),
            new Pose(13.3 + xOffset, 56.75 + yOffset, Math.toRadians(150)),
            new Pose(13.3 + xOffset, 57 + yOffset, Math.toRadians(150)),
    };
    protected Pose farPickupPose = new Pose(11.590, 33.210, Math.toRadians(180));
    protected Pose farPickupControlPoint = new Pose(45, 34);
    protected Pose cornerPose = new Pose(13.990, 17.860, Math.toRadians(210));
    protected Pose cornerBackupPose = new Pose(11.690, 8.360, Math.toRadians(180));
    protected Pose farShootingPose = new Pose(51.247, 10.099);
    protected Pose parkPose = new Pose(45.747, 15.099);
    protected Pose closeParkPose = new Pose(56.990, 102.860, Math.toRadians(180));
    protected Pose goalPose = Constants.BLUE_GOAL_POSE;

    protected PathChain shootPreloads;
    protected PathChain pickupMiddle;
    protected PathChain shootMiddle;
    protected PathChain clearGate;
    protected PathChain[] pickupGates;
    protected PathChain[] shootGates;
    protected PathChain shootGateAndPark;
    // PathChain shootGate2;
    protected PathChain pickupClose;
    protected PathChain shootClose;
    protected PathChain shootCloseAndPark;
    protected PathChain pickupFar;
    protected PathChain shootFar;
    protected PathChain shootFarAndPark;
    protected PathChain pickupCorner;
    protected PathChain backupCorner;
    protected PathChain shootCorner;
    protected PathChain park;
    protected PathChain shootCornerClose;

    abstract void setPoses();

    abstract void setColor();

    protected void createAutoCommands() {
        updateShooter = robot.updateShootingSubsystems();

        double shootTime = 150;

        schedule(updateShooter,
                sequential(
                        shootPreloads(),
                        race(
                                waitUntil(() -> robot.isShooterReady()),
                                waitMs(500)
                        ),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime, 1000),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupClose, shootClose, shootTime, 900, 500),
                        gateCycle(shootTime, 1000),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupFar, shootFarAndPark, shootTime + 125, 700, 750),
                        shootAndSetIntaking(),
                        waitMs(500),
                        robot.setIntakePower(0),
                        robot.deactivateShooter()
                )
        );
    }


    protected Command shootPreloads() {
        return sequential(startFlywheel(), follow(robot.getFollower(), shootPreloads));
    }

    protected Command runCycle(PathChain pickupPath, PathChain shootPath, double shootDelayMs,
                               double intakeDelayMs, double shootingDelayMs) {
        return sequential(
                parallel(
                        shootAndSetIntaking(),
                        sequential(
                                waitMs(shootDelayMs),
                                parallel(
                                        follow(robot.getFollower(), pickupPath),
                                        sequential(
                                                waitMs(intakeDelayMs),
                                                robot.setIntakePower(1)
                                        )
                                ).raceWith(waitUntil(() -> robot.beamBroken()))
                        )
                ),
                parallel(
                        sequential(
                                waitMs(200),
                                conditional(
                                        () -> robot.beamBroken(),
                                        instant(() -> {}), // do nothing
                                        sequential(
                                                robot.setIntakePower(-1),
                                                waitMs(50),
                                                robot.setIntakePower(0)
                                        )
                                )
                        ),
                        follow(robot.getFollower(), shootPath),
                        sequential(
                                waitMs(shootingDelayMs),
                                robot.setIntakePower(0)
                        )
                )
        );
    }

    protected Command gateCycle(double shootDelayMs, double gateWaitMs) {
        gateCycleNum++;
        return sequential(
                parallel(
                        shootAndSetIntaking(),
                        sequential(waitMs(shootDelayMs),
                                robot.setIntakePower(1),
                                follow(robot.getFollower(), pickupGates[gateCycleNum])
                        )
                ),
                race(
                        waitMs(gateWaitMs),
                        waitUntil(() -> robot.beamBroken()) //leave gate early if we have all balls
                ),
                parallel(
                        sequential(
                                waitMs(200),
                                conditional(
                                        () -> robot.beamBroken(),
                                        instant(() -> {}), // do nothing
                                        sequential(
                                                robot.setIntakePower(-1),
                                                waitMs(50),
                                                robot.setIntakePower(0)
                                        )
                                )
                        ),
                        follow(robot.getFollower(), shootGates[gateCycleNum]),
                        sequential(
                                waitMs(200),
                                robot.setIntakePower(0)
                        )
                )
        );
    }

    protected Command gateCycleAndPark(double shootDelayMs, double gateWaitMs) {
        gateCycleNum++;
        return sequential(
                parallel(shootAndSetIntaking(),
                        sequential(waitMs(shootDelayMs), robot.setIntakePower(1),
                                follow(robot.getFollower(), pickupGates[gateCycleNum]))),
                waitMs(gateWaitMs), parallel(follow(robot.getFollower(), shootGateAndPark),
                        sequential(waitMs(1000), robot.setIntakePower(0))));
    }

    protected Command cornerCycle(double shootDelayMs, double intakeDelayMs,
                                  double shootingDelayMs) {
        return sequential(parallel(shootAndSetIntaking(),
                        // shoot and pickup corner
                        sequential(waitMs(shootDelayMs),
                                parallel(follow(robot.getFollower(), pickupCorner),
                                        sequential(waitMs(intakeDelayMs), robot.deactivateFlywheel(),
                                                robot.setIntakePower(1))))),
                follow(robot.getFollower(), backupCorner),
                instant(() -> VelocityCompensationCalculator.useAutoLimit = false),
                robot.activateShooter(),
                parallel(sequential(waitMs(shootingDelayMs), robot.setIntakePower(0)),
                        follow(robot.getFollower(), shootCorner)),
                shootAndSetIntaking(), waitMs(shootDelayMs), follow(robot.getFollower(), park));
    }

    protected Command cornerCycleClosePark(double shootDelayMs, double intakeDelayMs,
                                           double shootingDelayMs) {
        return sequential(
                parallel(shootAndSetIntaking(), sequential(waitMs(shootDelayMs),
                        parallel(follow(robot.getFollower(), pickupCorner),
                                sequential(waitMs(intakeDelayMs), robot.deactivateFlywheel(),
                                        robot.setIntakePower(1))))),
                follow(robot.getFollower(), backupCorner),
                parallel(
                        sequential(waitMs(shootingDelayMs), robot.setIntakePower(0),
                                robot.activateShooter()),
                        follow(robot.getFollower(), shootCornerClose)));
    }

    public static Command turnTo(Follower follower, double radians) {
        return new CommandBuilder().setStart(() -> {
            Pose pose = follower.getPose();
            Path path = new Path(new BezierPoint(pose));
            path.setHeadingInterpolation(HeadingInterpolator.constant(radians));
            follower.followPath(path);
        }).setDone(() -> !follower.isBusy());
    }

    protected Command shootAndSetIntaking() {
        return instant(() -> robot.setState(States.SHOOTING));
    }

    protected Command startFlywheel() {
        return instant(() -> robot.activateShooter());
    }

    private void generatePaths() {
        shootPreloads = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pickupMiddle = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose, middlePickupControlPoint2, middlePickupPose))
//                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();
                .setTangentHeadingInterpolation().build();

        shootMiddle = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(middlePickupPose, shootingPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        for (int i = 0; i < gatePickupPoses.length; i++) {
            if (i == 0) {
                pickupGates = new PathChain[gatePickupPoses.length];
                shootGates = new PathChain[gatePickupPoses.length];
            }
            pickupGates[i] = robot.getFollower().pathBuilder()
                    .addPath(new BezierLine(shootingPose, gatePickupPoses[i]))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(0, 0.6, HeadingInterpolator.tangent),
                            new HeadingInterpolator.PiecewiseNode(0.6, 1, HeadingInterpolator.constant(gatePickupPoses[i].getHeading()))
                    ))
                    .build();

            shootGates[i] = robot.getFollower().pathBuilder()
                    .addPath(new BezierLine(gatePickupPoses[i], shootingPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }



        shootGateAndPark = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(gatePickupPoses[gatePickupPoses.length-1], closeParkPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pickupClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, closePickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        shootClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(closePickupPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        shootCloseAndPark = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(closePickupPose, closeParkPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        pickupFar = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose, farPickupControlPoint, farPickupPose))
//                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();
                .setTangentHeadingInterpolation().build();

        shootFar = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(farPickupPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        shootFarAndPark = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(farPickupPose, closeParkPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pickupCorner =
                robot.getFollower().pathBuilder().addPath(new BezierLine(shootingPose, cornerPose))
                        .setConstantHeadingInterpolation(cornerPose.getHeading()).build();

        backupCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(cornerPose, cornerBackupPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(),
                        cornerBackupPose.getHeading())
                .setConstraints(new PathConstraints(0.8, 2, 2, 0.03, 50, 1, 10, 1)).build();

        shootCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(cornerBackupPose, farShootingPose))
                .setConstantHeadingInterpolation(cornerBackupPose.getHeading()).build();

        park = robot.getFollower().pathBuilder().addPath(new BezierLine(farShootingPose, parkPose))
                .setTangentHeadingInterpolation().build();

        shootCornerClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(cornerBackupPose, closeParkPose))
                .setConstantHeadingInterpolation(cornerBackupPose.getHeading()).build();
    }

    public void initialize() {
        follower = Constants.create(hardwareMap);
        setColor();
        setPoses();
        Scheduler.reset();


        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose, Robot.LocalizationMode.FOLLOWER);
        robot.setPose(startPose);
        robot.init();

        generatePaths();
        adjustGoalPose();
    }

    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        robot.setState(States.INTAKING);
        createAutoCommands();
        while (opModeIsActive()) {
            robot.clearCaches();
            Scheduler.execute();
        }
        robot.stop();
    }

     */
}