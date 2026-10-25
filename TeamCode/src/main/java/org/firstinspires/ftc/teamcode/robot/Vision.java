package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry;

import java.util.ArrayList;
import java.util.List;

/**
 * Limelight AprilTags under our HIVE cells. Distance and bearing come from each tag's 3D pose in
 * camera space, which only needs the tag size set in the pipeline (no field map). Tags of the
 * target cell are averaged over the last few frames; tags of our other cell are only reported
 * (the HiveTracker uses them to notice a tip).
 */
@Configurable
public class Vision {
    // TODO(measure): how far the camera is tilted up from horizontal, degrees
    public static double CAMERA_PITCH_DEG = 0.0;
    // TODO(measure): true if the Limelight turns with the turret, false if it's on the chassis
    public static boolean CAMERA_ON_TURRET = false;
    // Where the camera points, degrees from the robot's front (or the turret's), + = left
    public static double CAMERA_YAW_DEG = 0.0;
    public static long MAX_RESULT_AGE_MS = 100;
    // Average this many frames (a new frame only arrives every ~20 ms)
    public static int SMOOTHING_FRAMES = 5;

    private static final int MAX_SMOOTHING_FRAMES = 20;

    private final RobotHardware robot;
    private int targetFirstId = RobotConfig.RED_FIRST_TAG;
    private int otherFirstId = RobotConfig.RED_FIRST_TAG + FieldGeometry.TAGS_PER_CELL;
    private boolean hasTarget = false;
    private boolean seesOtherCell = false;
    private double distanceInches = 0.0;
    private double bearingDeg = 0.0;
    private double targetFieldHeading = Double.NaN;
    private double averageTa = 0.0;
    private final List<Integer> visibleIds = new ArrayList<>();
    private long lastFrameStamp = -1;

    // Rolling window of per-frame cluster centres (camera-level frame, inches) and areas
    private final double[] windowForward = new double[MAX_SMOOTHING_FRAMES];
    private final double[] windowRight = new double[MAX_SMOOTHING_FRAMES];
    private final double[] windowTa = new double[MAX_SMOOTHING_FRAMES];
    private int windowCount = 0;
    private int windowSize = 0;
    private int windowNext = 0;

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

    /** Aim at this cell's tag cluster (first ID of its four); also watch the alliance's other cell. */
    public void setTargetCluster(int targetFirstId, int otherFirstId) {
        if (targetFirstId != this.targetFirstId) {
            clearWindow();
        }
        this.targetFirstId = targetFirstId;
        this.otherFirstId = otherFirstId;
    }

    /**
     * Call once per loop.
     *
     * @param robotHeading robot heading (rad) in the frame you aim in; 0 for a robot-frame aim
     * @param turretAngle  turret angle (rad, + = left), only used when CAMERA_ON_TURRET
     */
    public void update(double robotHeading, double turretAngle) {
        hasTarget = false;
        seesOtherCell = false;
        visibleIds.clear();
        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_RESULT_AGE_MS) {
            clearWindow();
            return;
        }
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        double forwardSum = 0;
        double rightSum = 0;
        double taSum = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (FieldGeometry.inCluster(id, otherFirstId)) {
                seesOtherCell = true;
                continue;
            }
            if (!FieldGeometry.inCluster(id, targetFirstId)) {
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
            clearWindow();
            return;
        }
        // The hub polls faster than the camera: add each camera frame to the window only once
        long stamp = result.getControlHubTimeStamp();
        if (stamp != lastFrameStamp) {
            lastFrameStamp = stamp;
            addToWindow(forwardSum / count, rightSum / count, taSum / count);
        }

        double forward = 0;
        double right = 0;
        double ta = 0;
        for (int i = 0; i < windowCount; i++) {
            forward += windowForward[i];
            right += windowRight[i];
            ta += windowTa[i];
        }
        forward /= windowCount;
        right /= windowCount;
        distanceInches = Math.hypot(forward, right);
        bearingDeg = Math.toDegrees(Math.atan2(right, forward));
        averageTa = ta / windowCount;
        double cameraHeading = robotHeading + Math.toRadians(CAMERA_YAW_DEG)
                + (CAMERA_ON_TURRET ? turretAngle : 0.0);
        targetFieldHeading = FieldGeometry.wrap(cameraHeading - Math.toRadians(bearingDeg));
        hasTarget = true;
    }

    private void addToWindow(double forward, double right, double ta) {
        int size = Math.max(1, Math.min(MAX_SMOOTHING_FRAMES, SMOOTHING_FRAMES));
        if (size != windowSize) {
            clearWindow();
            windowSize = size;
        }
        windowForward[windowNext] = forward;
        windowRight[windowNext] = right;
        windowTa[windowNext] = ta;
        windowNext = (windowNext + 1) % size;
        windowCount = Math.min(windowCount + 1, size);
    }

    private void clearWindow() {
        windowCount = 0;
        windowNext = 0;
    }

    /** The target cell's tags are in view right now. */
    public boolean hasTarget() {
        return hasTarget;
    }

    /** Tags of our other cell are in view (it has tipped up). */
    public boolean seesOtherCell() {
        return seesOtherCell;
    }

    /** Level distance from the camera to the cluster, inches. Last value if the tags are lost. */
    public double getDistanceInches() {
        return distanceInches;
    }

    /** Angle from the camera's forward to the cluster, degrees, + = right (like tx). */
    public double getBearingDeg() {
        return bearingDeg;
    }

    /** Heading (rad) from the robot to the cluster in the frame passed to update(); NaN if not seen. */
    public double getTargetFieldHeading() {
        return hasTarget ? targetFieldHeading : Double.NaN;
    }

    public double getAverageTa() {
        return averageTa;
    }

    public String getVisibleIds() {
        return visibleIds.toString();
    }
}
