package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/**
 * Limelight AprilTags under our HIVE cells. Distance and bearing come from each tag's 3D pose in
 * camera space, which only needs the tag size set in the pipeline (no field map).
 */
@Configurable
public class Vision {
    // TODO(measure): how far the camera is tilted up from horizontal, degrees
    public static double CAMERA_PITCH_DEG = 0.0;
    public static long MAX_RESULT_AGE_MS = 100;

    private final RobotHardware robot;
    private boolean red = true;
    private boolean hasTarget = false;
    private double distanceInches = 0.0;
    private double bearingDeg = 0.0;
    private double averageTa = 0.0;
    private final List<Integer> visibleIds = new ArrayList<>();

    public Vision(RobotHardware robot) {
        this.robot = robot;
    }

    public void start() {
        robot.limelight.pipelineSwitch(RobotConfig.HIVE_PIPELINE);
        robot.limelight.start();
    }

    public void stop() {
        robot.limelight.stop();
    }

    public void setAlliance(boolean red) {
        this.red = red;
    }

    /** Call once per loop. */
    public void update() {
        hasTarget = false;
        visibleIds.clear();
        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_RESULT_AGE_MS) {
            return;
        }
        int firstId = red ? RobotConfig.RED_FIRST_TAG : RobotConfig.BLUE_FIRST_TAG;
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        double forwardSum = 0;
        double rightSum = 0;
        double taSum = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id < firstId || id >= firstId + RobotConfig.TAGS_PER_ALLIANCE) {
                continue;
            }
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null) {
                continue;
            }
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            if (p.x == 0 && p.y == 0 && p.z == 0) {
                continue; // no 3D solve for this tag
            }
            // Camera space: x right, y down, z out of the lens. Level it with the camera pitch.
            forwardSum += p.z * Math.cos(pitch) + p.y * Math.sin(pitch);
            rightSum += p.x;
            taSum += tag.getTargetArea();
            visibleIds.add(id);
            count++;
        }
        if (count == 0) {
            return;
        }
        // Averaging the visible tags puts us near the cluster centre
        double forward = forwardSum / count;
        double right = rightSum / count;
        distanceInches = Math.hypot(forward, right);
        bearingDeg = Math.toDegrees(Math.atan2(right, forward));
        averageTa = taSum / count;
        hasTarget = true;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    /** Level distance from the camera to the cluster, inches. Last value if the tags are lost. */
    public double getDistanceInches() {
        return distanceInches;
    }

    /** Angle from the camera's forward to the cluster, degrees, + = right (like tx). */
    public double getBearingDeg() {
        return bearingDeg;
    }

    public double getAverageTa() {
        return averageTa;
    }

    public String getVisibleIds() {
        return visibleIds.toString();
    }
}
