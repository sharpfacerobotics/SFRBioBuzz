package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Vision;

public abstract class BaseAuto extends OpMode {

    protected RobotHardware robot = new RobotHardware();
    protected Follower follower;
    protected Timer pathTimer, opModeTimer;

    // DECODE tunable constants
    //public static double launchPower     = 0.75;
    //public static double triggerStartPos = 0.11;
    //public static double triggerShootPos = 0.4;
    //public static double spinUpTime      = 0.2;

    // DECODE state
    //protected int     shotsFired = 1;
    //protected boolean isShooting = false;

    // Poses - subclasses set these in definePoses()
    protected Pose startPose;
    //protected Pose shootPose;

    /** Set startPose, shootPose, and any subclass-specific poses here */
    protected abstract void definePoses();

    /** Build PathChains here using poses from definePoses() */
    protected abstract void buildPaths();

    /** Run the state machine each loop */
    protected abstract void statePathUpdate();

    protected Vision vision;

    /** Override in velocity autos to use setVelocity instead of setPower */
    //protected void setLaunchPower(double power) {
        //robot.launchMotor.setPower(power);
    //}

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.imu.resetYaw();
        follower    = Constants.createFollower(hardwareMap);
        pathTimer   = new Timer();
        opModeTimer = new Timer();
        //shootTimer  = new Timer();

        vision = new Vision(robot);

        definePoses();
        robot.Trigger.setPosition(triggerStartPos);
        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("x",           follower.getPose().getX());
        telemetry.addData("y",           follower.getPose().getY());
        telemetry.addData("heading",     follower.getPose().getHeading());
        telemetry.addData("path time",   pathTimer.getElapsedTimeSeconds());
        //telemetry.addData("shots fired", shotsFired);
        telemetry.update();
        double currentFieldHeading = Math.toDegrees(follower.getPose().getHeading());

        robot.limelight.updateRobotOrientation(currentFieldHeading);
    }
    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}
