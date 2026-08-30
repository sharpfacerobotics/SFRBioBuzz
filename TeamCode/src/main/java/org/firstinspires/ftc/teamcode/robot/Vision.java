package org.firstinspires.ftc.teamcode.robot;


import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;



public class Vision {

    private RobotHardware robot;

    // Lock-on constants CHANGE FOR BIOBUZZ
    public static double TAG_LOCK_KP = 0.05;
    public static double TAG_LOCK_TOLERANCE = 0.7;
    private static final double TAG_LOCK_MIN_POWER = 0.00;
    private static final double TAG_LOCK_MAX_POWER = 0.4;

    // State CHANGE FOR BIOBUZZ
    private boolean targetVisible   = false;
    private boolean lockedOn  = false;
    public Pose startPose = null;
    private double  distance    = 0.0;
    public double fieldOffset = 0;

    public Vision(RobotHardware robot) {
        this.robot = robot;
    }

    public void init() {
        robot.limelight.start();
      // CHANGE FOR BIOBUZZ
        robot.limelight.pipelineSwitch(0);
    }

    /**
     * Call every loop. Reads limelight and updates internal state.
     * @return the LLResult so TeleOp can use it for telemetry if needed
     */
    public LLResult update() {
        LLResult result = robot.limelight.getLatestResult();
        targetVisible = result != null && result.isValid();

        if (targetVisible) {
            distance = distanceCalc(result.getTa());
        }

        return result;
    }

    /**
     * Call with the driver's manual yaw input.
     * If lock-on is active and a target is visible, returns a corrected yaw.
     * Otherwise returns the manual yaw unchanged.
     */
    public double getYaw(double manualYaw, boolean lockOnButton, LLResult result) {
        if (lockOnButton && targetVisible && result != null) {
            double tx = result.getTx();
            lockedOn = Math.abs(tx) < TAG_LOCK_TOLERANCE;

            if (lockedOn) {
                return 0.0;
            } else {
                double correction = Range.clip(tx * TAG_LOCK_KP, -TAG_LOCK_MAX_POWER, TAG_LOCK_MAX_POWER);
                if (Math.abs(correction) < TAG_LOCK_MIN_POWER) {
                    correction = TAG_LOCK_MIN_POWER * Math.signum(correction);
                }
                return correction;
            }
        } else {
            lockedOn = false;
            return manualYaw;

        }
    }

    public boolean setPose() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES) + fieldOffset);
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            startPose = limelightToPedroPose(result.getBotpose_MT2());
            return true;
        } else {
            return false;
        }
    }


    // |============================================|
    // |--------------- IMPORTANT! -----------------|
    // |                                            |
    // |_____________ Check Accuracy _______________|
    // |                                            |
    // |--------------- IMPORTANT! -----------------|
    // |============================================|
  // CHANGE FOR BIOBUZZ
    public Pose limelightToPedroPose(Pose3D botpose) {
        // Convert meters to inches and shift origin from center to bottom-left
        double pedroX = (botpose.getPosition().x * 39.3701) + 72.0;
        double pedroY = (botpose.getPosition().y * 39.3701) + 72.0;

        // Heading - verify sign at team meeting by rotating robot and checking
        double pedroHeading = Math.toRadians(botpose.getOrientation().getYaw(AngleUnit.DEGREES));

        return new Pose(pedroX, pedroY, pedroHeading);
    }
    // CHANGE FOR BIOBUZZ
    public double[] limelightToPedroPose(Pose3D botpose, boolean test) {
        // Convert meters to inches and shift origin from center to bottom-left
        double pedroX = (botpose.getPosition().x * 39.3701) + 72.0;
        double pedroY = (botpose.getPosition().y * 39.3701) + 72.0;

        // Heading - verify sign at team meeting by rotating robot and checking
        double pedroHeading = Math.toRadians(botpose.getOrientation().getYaw(AngleUnit.DEGREES));


        double[] pedroDetails = new double[3];
        pedroDetails[0] = botpose.getPosition().x;
        pedroDetails[1] = botpose.getPosition().y;
        pedroDetails[2] = pedroHeading;
        return pedroDetails;
    }


    //*** CHANGE FOR BIOBUZZ
    public double distanceCalc(double ta) {
        return 72.06169 * Math.pow(ta, -0.509117);
    }

    public boolean hasTarget()  { return targetVisible; }
    public void setHasTarget(boolean val) {
        targetVisible = val;
    }
    public boolean isLockedOn() { return lockedOn; }
    public double  getDistance(){ return distance; }
}
