package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Element;

import java.util.List;

/**
 * Finds pollen/nectar on the field with the Limelight's neural detector pipeline. Owner: Gautham.
 *
 * <p>Picks the best detection (biggest = usually closest), works out how far away it is from
 * the camera angle, and can turn that into a field position for autos to drive to.</p>
 *
 * <p>Detector class names must be "pollen", "red_nectar" and "blue_nectar". Pass the
 * opponent's NECTAR colour as "not wanted" - we may never pick it up (G408).</p>
 */
@Configurable
public class ElementVision {
    private final RobotHardware robot;

    public static double MIN_CONFIDENCE = 0.6;
    /** Ignore Limelight results older than this (ms) - a frozen camera repeats its last frame. */
    public static long MAX_RESULT_AGE_MS = 100;

    private boolean hasElement = false;
    private boolean searching = false;
    private Element opponentNectar = null;
    private Element type = Element.UNKNOWN;
    private double tx = 0;
    private double ty = 0;
    private double area = 0;

    // ------------------------------------------------------------------ glossary methods
    // beginAutoFind() START, elementDetected() STATUS, getElementDeltaX() STATUS, stopAutoFind() STOP

    /** SET (once). Our alliance, so the opponent's NECTAR is never reported (G408). */
    public void setAllianceColor(String allianceColor) {
        opponentNectar = "RED".equals(allianceColor) ? Element.BLUE_NECTAR : Element.RED_NECTAR;
    }

    /** START. Switch the Limelight to the element detector and start searching. Once per search. */
    public void beginAutoFind() {
        if (!searching) {
            searching = true;
            hasElement = false;
            activate();
        }
    }

    /** STATUS. True if a (legal) element is visible. Loop call while searching. */
    public boolean elementDetected() {
        if (searching) {
            update(null, opponentNectar);
        }
        return searching && hasElement;
    }

    /** STATUS. tx (deg, + = right) of the largest detected element. */
    public double getElementDeltaX() {
        return tx;
    }

    /** STOP. End the search and give the Limelight back to the hive AprilTag pipeline. */
    public void stopAutoFind() {
        if (searching) {
            searching = false;
            hasElement = false;
            robot.limelight.pipelineSwitch(RobotConfig.HIVE_PIPELINE);
        }
    }

    public boolean isSearching() {
        return searching;
    }

    public ElementVision(RobotHardware robot) {
        this.robot = robot;
    }

    /** Switch the Limelight to the element pipeline. Takes a few frames to take effect. */
    public void activate() {
        robot.limelight.pipelineSwitch(RobotConfig.ELEMENT_PIPELINE);
    }

    /**
     * Call once per loop while looking for elements.
     *
     * @param wanted only report this type, or null for any
     * @param never  never report this type - pass RobotConfig.opponentNectar(alliance) (G408)
     */
    public void update(Element wanted, Element never) {
        hasElement = false;
        LLResult result = robot.limelight.getLatestResult();
        if (result == null || result.getStaleness() > MAX_RESULT_AGE_MS
                || !result.isValid() || result.getPipelineIndex() != RobotConfig.ELEMENT_PIPELINE) {
            return;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        double bestArea = 0;
        for (int i = 0; i < detections.size(); i++) {
            LLResultTypes.DetectorResult d = detections.get(i);
            if (d.getConfidence() < MIN_CONFIDENCE) {
                continue;
            }
            Element detectedType = parse(d.getClassName());
            // Never chase something we can't identify - it could be opponent NECTAR (G408).
            if (detectedType == Element.UNKNOWN
                    || (wanted != null && detectedType != wanted) || detectedType == never) {
                continue;
            }
            // At or above the horizon it can't be on the floor (and distance would be infinite).
            if (RobotConfig.CAMERA_PITCH_DEG + d.getTargetYDegrees() >= -0.5) {
                continue;
            }
            if (d.getTargetArea() > bestArea) {
                bestArea = d.getTargetArea();
                hasElement = true;
                type = detectedType;
                tx = d.getTargetXDegrees();
                ty = d.getTargetYDegrees();
                area = bestArea;
            }
        }
    }

    public boolean hasElement() {
        return hasElement;
    }

    public Element getType() {
        return type;
    }

    /** Horizontal angle (deg, + = right of camera). */
    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getArea() {
        return area;
    }

    /** Ground distance (in) from the camera lens to the element, from camera pitch + ty. */
    public double getDistance() {
        double angle = Math.toRadians(RobotConfig.CAMERA_PITCH_DEG + ty);
        if (angle >= -0.01) {
            return Double.POSITIVE_INFINITY; // at/above horizon - can't be on the floor
        }
        return (RobotConfig.ELEMENT_HEIGHT_IN - RobotConfig.CAMERA_HEIGHT_IN) / Math.tan(angle);
    }

    /** Element field x (in), given where the robot is. */
    public double getFieldX(double robotX, double robotHeading) {
        double bearing = robotHeading - Math.toRadians(tx); // tx + = right = clockwise
        return robotX + RobotConfig.CAMERA_FORWARD_OFFSET_IN * Math.cos(robotHeading)
                + getDistance() * Math.cos(bearing);
    }

    public double getFieldY(double robotY, double robotHeading) {
        double bearing = robotHeading - Math.toRadians(tx);
        return robotY + RobotConfig.CAMERA_FORWARD_OFFSET_IN * Math.sin(robotHeading)
                + getDistance() * Math.sin(bearing);
    }

    /** Detector class names: "pollen", "red_nectar", "blue_nectar" (case-insensitive). */
    static Element parse(String className) {
        if (className == null) {
            return Element.UNKNOWN;
        }
        if (contains(className, "pollen")) {
            return Element.POLLEN;
        }
        if (contains(className, "nectar")) {
            if (contains(className, "red")) {
                return Element.RED_NECTAR;
            }
            if (contains(className, "blue")) {
                return Element.BLUE_NECTAR;
            }
        }
        return Element.UNKNOWN;
    }

    /** Case-insensitive contains without allocating (toLowerCase makes a new String). */
    private static boolean contains(String text, String part) {
        for (int i = 0; i + part.length() <= text.length(); i++) {
            if (text.regionMatches(true, i, part, 0, part.length())) {
                return true;
            }
        }
        return false;
    }
}
