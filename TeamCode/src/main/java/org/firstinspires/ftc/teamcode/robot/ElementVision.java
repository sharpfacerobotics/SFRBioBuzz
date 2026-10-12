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
 * <p>Detector class names must be "pollen" and "nectar" (case doesn't matter).</p>
 */
@Configurable
public class ElementVision {
    private final RobotHardware robot;

    public static double MIN_CONFIDENCE = 0.6;

    private boolean hasElement = false;
    private Element type = Element.UNKNOWN;
    private double tx = 0;
    private double ty = 0;
    private double area = 0;

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
     * @param wanted only report this type, or null for either
     */
    public void update(Element wanted) {
        hasElement = false;
        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getPipelineIndex() != RobotConfig.ELEMENT_PIPELINE) {
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
            if (wanted != null && detectedType != wanted) {
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

    private static Element parse(String className) {
        if (className == null) {
            return Element.UNKNOWN;
        }
        if (className.equalsIgnoreCase("pollen")) {
            return Element.POLLEN;
        }
        if (className.equalsIgnoreCase("nectar")) {
            return Element.NECTAR;
        }
        return Element.UNKNOWN;
    }
}
