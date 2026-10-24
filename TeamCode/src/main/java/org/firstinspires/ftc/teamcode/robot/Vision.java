package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;
import org.firstinspires.ftc.teamcode.robot.helperClasses.HiveTracker;

import java.util.List;

/**
 * Limelight ("LL" in the glossary): AprilTags on our HIVE. Owner: Gautham.
 *
 * <p>Glossary methods:</p>
 * <ul>
 *   <li>{@link #updateRead(String)}: UPDATE. Loop call. Refreshes the Limelight reads and
 *       the hive tip state.</li>
 *   <li>{@link #getAverageTa()}: STATUS, double. Average AprilTag area of the target cell.</li>
 *   <li>{@link #getTargetHeading()}: STATUS, double. Field-centric heading to the target cell
 *       (NaN when it isn't in view).</li>
 *   <li>{@link #getHiveTipState()}: STATUS, String. "GARDEN_SIDE" or "OPPOSITE_GARDEN": which
 *       of our two cells is facing up, relative to our own GARDEN.</li>
 *   <li>{@link #getAverageBotPose()}: STATUS, Pose3D. Averaged MegaTag2 pose for
 *       {@code Drivetrain.relocalize()} (FTC field frame, null if none recent).</li>
 * </ul>
 *
 * <p>Hive tip state uses HiveTracker: our own launches predict tips (the hive tips at 8
 * POLLEN or 3 POLLEN + 3 NECTAR) and the tags catch tips our partner causes. The tags are on
 * each cell's underside, so the cell facing up is the one whose tags we can see.</p>
 *
 * <p>Robot pose and turn rate come from {@code robot.follower} (a RobotHardware call).
 * Assumes the Limelight is fixed to the chassis, facing forward.</p>
 */
@Configurable
public class Vision {
    private final RobotHardware robot;

    /** Drop the ta average if the tag hasn't been seen for this long (s). */
    public static double STALE_TIME = 0.3;
    /** Ignore Limelight results older than this (ms): a frozen or unplugged camera keeps
     *  returning its last "valid" frame. */
    public static long MAX_RESULT_AGE_MS = 100;
    /** Only take MegaTag2 while turning slower than this (rad/s). */
    public static double MAX_RELOCALIZE_OMEGA = 1.0;
    /** Only take MegaTag2 when the tags it used were closer than this on average (m). */
    public static double MAX_BOTPOSE_TAG_DIST_M = 3.0;
    /** getAverageBotPose averages MegaTag2 poses from the last this many seconds. */
    public static double BOTPOSE_MAX_AGE = 0.5;

    private static final int BOTPOSE_WINDOW = 6;

    private final ElapsedTime clock = new ElapsedTime();

    // ta rolling average (allocated once)
    private final double[] taWindow = new double[RobotConfig.TA_AVERAGE_WINDOW];
    private int taIndex = 0;
    private int taCount = 0;
    private double taSum = 0;
    private double lastSeenTime = -1e9;

    // MegaTag2 rolling window, FTC frame (metres, radians)
    private final double[] bpX = new double[BOTPOSE_WINDOW];
    private final double[] bpY = new double[BOTPOSE_WINDOW];
    private final double[] bpYaw = new double[BOTPOSE_WINDOW];
    private final double[] bpTime = new double[BOTPOSE_WINDOW];
    private int bpIndex = 0;

    private boolean hasTarget = false;
    private boolean seesOtherCell = false;
    private double tx = 0;
    private double targetHeading = Double.NaN;
    private long lastResultStamp = -1;

    private String alliance = null;
    private boolean red = true;
    private HiveTracker hive = new HiveTracker(FieldGeometry.startingUpCell(true));

    public Vision(RobotHardware robot) {
        this.robot = robot;
        for (int i = 0; i < BOTPOSE_WINDOW; i++) {
            bpTime[i] = -1e9;
        }
    }

    /** Switch the Limelight to the AprilTag pipeline (ElementVision uses another one). */
    public void activate() {
        robot.limelight.pipelineSwitch(RobotConfig.HIVE_PIPELINE);
    }

    /**
     * UPDATE. Loop call: refresh the Limelight reads and the hive tip state.
     *
     * @param allianceColor "RED" or "BLUE"
     */
    public void updateRead(String allianceColor) {
        if (!allianceColor.equals(alliance)) { // first call (or alliance changed): start fresh
            alliance = allianceColor;
            red = "RED".equals(allianceColor);
            hive = new HiveTracker(FieldGeometry.startingUpCell(red));
        }
        double now = clock.seconds();
        Pose pose = robot.follower.pose();
        double heading = pose.heading();
        double omega = robot.follower.velocity().omega;

        // MegaTag2 wants the FTC-frame yaw in degrees.
        robot.limelight.updateRobotOrientation(Math.toDegrees(heading - FieldGeometry.FTC_TO_FIELD_ROTATION));

        Cell up = hive.getUpCell();
        int targetBase = FieldGeometry.tagBase(red, up);
        int otherBase = FieldGeometry.tagBase(red, FieldGeometry.other(up));

        hasTarget = false;
        seesOtherCell = false;
        LLResult result = robot.limelight.getLatestResult();
        boolean fresh = result != null && result.getStaleness() < MAX_RESULT_AGE_MS;
        // The hub polls faster than the camera: only use each frame once.
        boolean newFrame = fresh && result.getControlHubTimeStamp() != lastResultStamp;
        if (newFrame) {
            lastResultStamp = result.getControlHubTimeStamp();
        }

        double ta = 0;
        if (fresh && result.isValid() && result.getPipelineIndex() == RobotConfig.HIVE_PIPELINE) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (int i = 0; i < tags.size(); i++) {
                LLResultTypes.FiducialResult tag = tags.get(i);
                int id = tag.getFiducialId();
                if (!hasTarget && FieldGeometry.inCluster(id, targetBase)) {
                    hasTarget = true;
                    tx = tag.getTargetXDegrees();
                    ta = tag.getTargetArea();
                } else if (FieldGeometry.inCluster(id, otherBase)) {
                    seesOtherCell = true;
                }
            }
            if (newFrame && Math.abs(omega) < MAX_RELOCALIZE_OMEGA
                    && result.getBotposeTagCount() > 0 && result.getBotposeAvgDist() < MAX_BOTPOSE_TAG_DIST_M) {
                addBotpose(result.getBotpose_MT2(), now);
            }
        }

        if (hasTarget) {
            lastSeenTime = now;
            // tx + = target right of the camera = clockwise from our heading
            targetHeading = FieldGeometry.wrap(heading - Math.toRadians(tx));
            if (newFrame) {
                addTa(ta);
            }
        } else {
            targetHeading = Double.NaN;
            tx = 0;
            if (now - lastSeenTime > STALE_TIME) {
                clearTa();
            }
        }

        hive.update(hasTarget, seesOtherCell, now);
    }

    /** STATUS. Rolling average target area of the target cell's tags, 0 if not recently seen. */
    public double getAverageTa() {
        return taCount == 0 ? 0 : taSum / taCount;
    }

    /** STATUS. Field-centric heading (rad) from the robot to the target cell; NaN if not in view. */
    public double getTargetHeading() {
        return targetHeading;
    }

    /** STATUS. "GARDEN_SIDE" or "OPPOSITE_GARDEN" - which of our cells is facing up. */
    public String getHiveTipState() {
        return FieldGeometry.tipState(red, hive.getUpCell());
    }

    /**
     * STATUS. Average of the recent MegaTag2 poses (FTC field frame) for Drivetrain.relocalize,
     * or null if there are none. Allocates - call when relocalizing, not every loop.
     */
    public Pose3D getAverageBotPose() {
        double now = clock.seconds();
        double sx = 0, sy = 0, sSin = 0, sCos = 0;
        int n = 0;
        for (int i = 0; i < BOTPOSE_WINDOW; i++) {
            if (now - bpTime[i] <= BOTPOSE_MAX_AGE) {
                sx += bpX[i];
                sy += bpY[i];
                sSin += Math.sin(bpYaw[i]);
                sCos += Math.cos(bpYaw[i]);
                n++;
            }
        }
        if (n == 0) {
            return null;
        }
        return new Pose3D(new Position(DistanceUnit.METER, sx / n, sy / n, 0, 0),
                new YawPitchRollAngles(AngleUnit.RADIANS, Math.atan2(sSin, sCos), 0, 0, 0));
    }

    // ------------------------------------------------------------------ hive model hooks

    /** Call when an element leaves into the launcher ("POLLEN" / "NECTAR"). */
    public void recordLaunch(String element) {
        hive.recordShot("NECTAR".equals(element), clock.seconds());
    }

    /** True right after a tip, while the hive swings: don't launch. */
    public boolean isHiveSettling() {
        return hive.isSettling(clock.seconds());
    }

    /** Driver override: "the hive just tipped". */
    public void flipHiveTipState() {
        hive.flip();
    }

    /** Continue from a saved state (TeleOp after auto). */
    public void restoreHive(String allianceColor, Cell upCell, double weight) {
        alliance = allianceColor;
        red = "RED".equals(allianceColor);
        hive = new HiveTracker(upCell, weight);
    }

    public Cell getHiveUpCell() {
        return hive.getUpCell();
    }

    public double getHiveLoad() {
        return hive.getWeight();
    }

    public int getHiveTips() {
        return hive.getTips();
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public boolean seesOtherCell() {
        return seesOtherCell;
    }

    // ------------------------------------------------------------------ internals

    private void addBotpose(Pose3D p, double now) {
        if (p == null) {
            return;
        }
        Position pos = p.getPosition();
        if (pos.x == 0 && pos.y == 0) {
            return; // Limelight reports all zeros when it has no solution
        }
        bpX[bpIndex] = pos.unit.toMeters(pos.x); // (toUnit() would allocate a new Position)
        bpY[bpIndex] = pos.unit.toMeters(pos.y);
        bpYaw[bpIndex] = p.getOrientation().getYaw(AngleUnit.RADIANS);
        bpTime[bpIndex] = now;
        bpIndex = (bpIndex + 1) % BOTPOSE_WINDOW;
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
