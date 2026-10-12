package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry;

import java.util.List;

/**
 * Limelight AprilTag tracking of our hive. Owner: Gautham.
 *
 * <p>Gives ShooterConfig a smoothed target area (ta) and gives the OpMode a MegaTag2 field
 * pose it can snap odometry to (converted into Pedro coordinates).</p>
 */
@Configurable
public class HiveVision {
    private final RobotHardware robot;

    /** Drop the average if the tag hasn't been seen for this long. */
    public static double STALE_TIME = 0.3;
    /** Only trust MegaTag2 while the robot is turning slower than this (rad/s). */
    public static double MAX_RELOCALIZE_OMEGA = 1.0;
    /**
     * Rotation from the official FTC field frame to ours. FTC: origin at centre, +Y away from
     * the red wall, +X to the right seen from the red wall. Ours: +x away from the red wall.
     * So ours = FTC rotated by -90 deg, for positions AND heading.
     */
    public static double FTC_TO_FIELD_ROTATION = -Math.PI / 2;

    private final double[] taWindow = new double[RobotConfig.TA_AVERAGE_WINDOW];
    private int taIndex = 0;
    private int taCount = 0;
    private double taSum = 0;
    private final ElapsedTime lastSeenTimer = new ElapsedTime();

    private boolean hasTarget = false;
    private boolean seesOtherCell = false;
    private double tx = 0;
    private double ta = 0;

    private boolean hasBotpose = false;
    private double botposeX = 0;
    private double botposeY = 0;
    private double botposeHeading = 0;

    public HiveVision(RobotHardware robot) {
        this.robot = robot;
    }

    /** Switch the Limelight to the AprilTag pipeline (ElementVision uses a different one). */
    public void activate() {
        robot.limelight.pipelineSwitch(RobotConfig.HIVE_PIPELINE);
    }

    /**
     * Call once per loop.
     *
     * @param targetTagBase first tag ID under our upward (target) cell, from FieldGeometry.tagBase
     * @param otherTagBase  first tag ID under our other cell - seeing it means the hive tipped
     * @param robotHeading  field heading (rad), MegaTag2 needs it
     * @param robotOmega    yaw rate (rad/s); MegaTag2 is unreliable while spinning fast
     */
    public void update(int targetTagBase, int otherTagBase, double robotHeading, double robotOmega) {
        // MegaTag2 wants the FTC-frame yaw in degrees.
        robot.limelight.updateRobotOrientation(Math.toDegrees(robotHeading - FTC_TO_FIELD_ROTATION));

        hasTarget = false;
        seesOtherCell = false;
        hasBotpose = false;
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid() && result.getPipelineIndex() == RobotConfig.HIVE_PIPELINE) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (int i = 0; i < tags.size(); i++) {
                LLResultTypes.FiducialResult tag = tags.get(i);
                int id = tag.getFiducialId();
                if (!hasTarget && FieldGeometry.inCluster(id, targetTagBase)) {
                    hasTarget = true;
                    tx = tag.getTargetXDegrees();
                    ta = tag.getTargetArea();
                } else if (FieldGeometry.inCluster(id, otherTagBase)) {
                    seesOtherCell = true;
                }
            }
            if (Math.abs(robotOmega) < MAX_RELOCALIZE_OMEGA) {
                readBotpose(result.getBotpose_MT2());
            }
        }

        if (hasTarget) {
            lastSeenTimer.reset();
            addTa(ta);
        } else if (lastSeenTimer.seconds() > STALE_TIME) {
            clearTa();
        }
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    /** We can see the tags under our OTHER cell (feed HiveTracker to detect tips). */
    public boolean seesOtherCell() {
        return seesOtherCell;
    }

    /** Horizontal angle to the hive tag (deg, + = right). */
    public double getTx() {
        return tx;
    }

    /** Rolling average target area, or 0 if the tag isn't (recently) in view. */
    public double getAverageTa() {
        return taCount == 0 ? 0 : taSum / taCount;
    }

    public boolean hasBotpose() {
        return hasBotpose;
    }

    /** MegaTag2 robot x in Pedro coordinates (in). Only valid when hasBotpose(). */
    public double getBotposeX() {
        return botposeX;
    }

    public double getBotposeY() {
        return botposeY;
    }

    public double getBotposeHeading() {
        return botposeHeading;
    }

    private void readBotpose(Pose3D pose) {
        if (pose == null) {
            return;
        }
        Position position = pose.getPosition().toUnit(DistanceUnit.INCH);
        double ftcX = position.x;
        double ftcY = position.y;
        if (ftcX == 0 && ftcY == 0) {
            return; // Limelight reports all zeros when it has no solution
        }
        // FTC frame (origin at centre) -> ours (origin at corner): rotate, then shift.
        // TODO(verify) on the field: drive to a known spot and compare.
        double cos = Math.cos(FTC_TO_FIELD_ROTATION);
        double sin = Math.sin(FTC_TO_FIELD_ROTATION);
        botposeX = cos * ftcX - sin * ftcY + FieldGeometry.CENTER;
        botposeY = sin * ftcX + cos * ftcY + FieldGeometry.CENTER;
        botposeHeading = pose.getOrientation().getYaw(AngleUnit.RADIANS) + FTC_TO_FIELD_ROTATION;
        hasBotpose = true;
    }

    private void addTa(double value) {
        if (taCount == taWindow.length) {
            taSum -= taWindow[taIndex];
        } else {
            taCount++;
        }
        taWindow[taIndex] = value;
        taSum += value;
        taIndex = (taIndex + 1) % taWindow.length;
    }

    private void clearTa() {
        taCount = 0;
        taIndex = 0;
        taSum = 0;
    }
}
