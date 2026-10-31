package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.curves.bezier.BezierCurve;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain {

    private RobotHardware robot;
    public Follower follower;
    private final PoseFactory p = PoseFactory.degrees(); // add this
    public boolean autoDriving = false;

    public Drivetrain(RobotHardware robot) {
        this.robot = robot;
    }

    /** Call every loop with joystick inputs (and yaw already overridden if lock-on is active) */
    public void drive(double axial, double lateral, double yaw) {
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize so no wheel exceeds 100%
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.frontRightMotor.setPower(frontRightPower);
        robot.backLeftMotor.setPower(backLeftPower);
        robot.backRightMotor.setPower(backRightPower);
    }
    private void cancelAutoDrive() {
        follower.stop();
        autoDriving = false;
    }

    private void stop() {
        drive(0, 0, 0);
    }

    private void driveForward(double power, long ms) throws InterruptedException {
        drive(power, 0, 0);
        Thread.sleep(ms);
        stop();
    }

    public void init(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    private void startAutoDrive(Pose targetPose) {
        // Get current robot pose from Pedro's odometry
        Pose currentPose = follower.pose();
        Path path = Paths.line(currentPose, targetPose).linear(currentPose, targetPose);
        // Build a straight line path from here to the target
        follower.follow(path);
        autoDriving = true;
    }

    private void startAutoDrive(Pose targetPose, Pose controlPoint) {
        Pose currentPose = follower.pose();
        Path path = Paths.curve(currentPose, controlPoint, targetPose).linear(currentPose, targetPose);
        follower.follow(path);
        autoDriving = true;
    }


    public Command startAutoDriveTo(Pose targetPose) {
        return Command.build()
                .setStart(() -> {

                    startAutoDrive(targetPose);
                })
                .setExecute(() -> {

                })
                .setDone(() -> {
                    return !follower.isBusy() || ! autoDriving;
                });
    }
    public Command driveWith(Command command1, Command command2){
        return parallel(
                command1,
                command2
        );
    }

    public Command move(double y, double x, double yaw){
        return Command.build()
                .setExecute(() -> {

                    follower.manual(y, x, yaw);

                });

    }
    public double[] getPedroDistanceToHive(Pose currentPose, String allianceColor){
        double[] Hive1 = {0.0, 0.0};
        double[] Hive2 = {0.0, 0.0};

        if (allianceColor.equals("red")){
            Hive1 = new double[] {58.746, 24.076}; // close red shoot
            Hive2 = new double[] {58.746, 115.815};
        }
        else{
            Hive1 = new double[] {82.754, 24.076}; // replace with blue hive coordinates
            Hive2 = new double[] {82.754, 115.815};
        }
        double[] vector1 = {Hive1[0] - currentPose.x(), Hive1[1] - currentPose.y()};
        double[] vector2 = {Hive2[0] - currentPose.x(), Hive2[1] - currentPose.y()};
        if (Math.hypot(vector1[0], vector1[1]) < Math.hypot(vector2[0], vector2[1])) {
            return vector1;
        }
        else{
            return vector2;
        }
    }
    public double getRobotHeading(){return Math.toDegrees(follower.pose().heading());}
    public double[] getRobotTranslationalVelocity(){
        Velocity velocity = follower.velocity();
        Vector2D vector = velocity.toVector2D();
        double vx = vector.x();
        double vy = vector.y();
        return new double[] {vx, vy};
    }



}
