package org.firstinspires.ftc.teamcode.hardwaresystems;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

public class Webcam {
    /**
     * Enum that holds HSV ranges for different colors used by the robot.
     */
    public enum Color {
        /**
         * Red to reddish-orange hues.
         */
        RED(new Scalar(0, 128, 64), new Scalar(10, 255, 255)),
        /**
         * Yellow-orange to yellow-green hues.
         */
        YELLOW(new Scalar(20, 128, 64), new Scalar(33, 255, 255)),
        /**
         * Green hues (present for completeness / future use).
         */
        GREEN(new Scalar(50, 128, 64), new Scalar(70, 255, 255)),
        /**
         * Teal to blue hues (used for blue alliance pixels).
         */
        BLUE(new Scalar(90, 128, 64), new Scalar(125, 255, 255)),
        /**
         * Magenta to red hues (used for purple-style elements). The hue range conceptually wraps around zero in HSV.
         */
        MAGENTA(new Scalar(-170, 128, 64), new Scalar(180, 255, 255));
        /**
         * Lower HSV bound for this color.
         */
        private final Scalar lowerBound;
        /**
         * Upper HSV bound for this color.
         */
        private final Scalar upperBound;


        /**
         * Constructor for each color with its lower and upper HSV bounds.
         *
         * @param lowerBound The lower HSV bound of the color.
         * @param upperBound The upper HSV bound of the color.
         */
        Color(Scalar lowerBound, Scalar upperBound) {
            this.lowerBound = lowerBound;
            this.upperBound = upperBound;
        }

        /**
         * Return the lower HSV bound for this color.
         *
         * @return The lower HSV bound for this color.
         */
        public Scalar getLowerBound() {
            return lowerBound;
        }

        /**
         * Return the upper HSV bound for this color.
         *
         * @return The upper HSV bound for this color.
         */
        public Scalar getUpperBound() {
            return upperBound;
        }

        /**
         * Return both bounds as a two-element array [lower, upper].
         *
         * @return Both bounds as a two-element array [lower, upper].
         */
        public Scalar[] getRange() {
            return new Scalar[]{lowerBound, upperBound};
        }
    }

    /**
     * VisionPortal used to communicate with the webcam and run vision processors.
     */
    private final VisionPortal VISION_PORTAL;
    /**
     * Processor that detects AprilTags in the camera image.
     */
    private final AprilTagProcessor APRIL_TAG;
    /**
     * Processor that finds the predominant color in a region of interest.
     */
    private final PredominantColorProcessor COLOR_PROCESSOR;
    /**
     * Stores camera resolution so other code can compute pixel error.
     */
    private final int[] RESOLUTION;
    private final int WIDTH_PIXELS;
    private final int HEIGHT_PIXELS;
    /**
     * Alliance or team color you want to remember for this webcam (used by your code).
     */
    private Color targetColor;
    /**
     * Offset of the camera relative to the robot center [x, y, z] in inches. Used by localization or pose-estimation
     * code outside this class.
     */
    private double[] poseAdjust;

    /**
     * Construct a webcam wrapper with default pose offset (0,0,0). This version does not use a custom preview
     * container. VisionPortal will handle the normal RC/DS preview.
     *
     * @param webcamName The name used by the webcam.
     * @param resolution The resolution that the camera uses.
     */
    public Webcam(WebcamName webcamName, int[] resolution) {
        this(webcamName, resolution, new double[]{0.0, 0.0, 0.0});
    }

    /**
     * Construct a webcam wrapper with a specified pose offset and default preview.
     *
     * @param webcamName The name used by the webcam.
     * @param resolution The resolution that the camera uses.
     * @param poseAdjust The adjustment for positioning of the camera relative to the robot.
     */
    public Webcam(WebcamName webcamName, int[] resolution, double[] poseAdjust) {
        this(webcamName, resolution, poseAdjust, -1);
    }

    /**
     * Construct a webcam wrapper with an optional preview container ID. If `cameraMonitorViewId` is -1, VisionPortal
     * uses the default preview. If `cameraMonitorViewId` is not -1, a custom preview container is used.
     *
     * @param webcamName          The name used by the webcam.
     * @param resolution          The resolution that the camera uses.
     * @param poseAdjust          The adjustment for positioning of the camera relative to the robot.
     * @param cameraMonitorViewId The ID for the preview container.
     */
    public Webcam(WebcamName webcamName, int[] resolution, double[] poseAdjust, int cameraMonitorViewId) {
        // Save pose adjustment values (reference is stored directly).
        this.poseAdjust = poseAdjust;

        // Saves resolution for pixel-based aiming
        this.RESOLUTION = resolution;
        this.WIDTH_PIXELS = resolution[0];
        this.HEIGHT_PIXELS = resolution[1];

        // No target color selected by default.
        this.targetColor = null;

        // Create an AprilTag processor with default settings.
        APRIL_TAG = new AprilTagProcessor.Builder().build();

        // Create a predominant color processor with a center ROI and a set of swatches.
        COLOR_PROCESSOR = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
            .setSwatches(
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE
            )
            .build();

        // Build the VisionPortal using the Builder pattern.
        // It owns the USB camera and runs the color and AprilTag processors.
        // autoStopLiveView means the RC preview pauses when no processors are enabled.
        VisionPortal.Builder builder = new VisionPortal.Builder()
            .addProcessor(COLOR_PROCESSOR)
            .addProcessor(APRIL_TAG)
            .setCamera(webcamName)
            .setCameraResolution(new Size(resolution[0], resolution[1]))
            .setAutoStopLiveView(true);

        // Create the VisionPortal instance.
        VISION_PORTAL = builder.build();
    }

    /**
     * Return the VisionPortal managing this webcam. You can use this to enable/disable processors or pause/resume the
     * preview.
     *
     * @return The VisionPortal managing this webcam.
     */
    public VisionPortal getVisionPortal() {
        return VISION_PORTAL;
    }

    /**
     * Return the AprilTag processor for this webcam.
     *
     * @return The AprilTag processor for this webcam.
     */
    public AprilTagProcessor getAprilTag() {
        return APRIL_TAG;
    }

    /**
     * Return a copy of the current AprilTag detections. The list may be empty if no tags are seen.
     *
     * @return A copy of the current AprilTag detections. The list may be empty if no tags are seen.
     */
    public List<AprilTagDetection> getAprilTagDetections() {
        // Copy into a new list so callers cannot modify the internal list.
        return new ArrayList<>(APRIL_TAG.getDetections());
    }

    /**
     * Return the predominant color processor. You can use this directly if you want to read more detailed color info.
     *
     * @return The predominant color processor.
     */
    public PredominantColorProcessor getColorProcessor() {
        return COLOR_PROCESSOR;
    }

    /**
     * Return the camera resolution.
     *
     * @return Return the camera resolution.
     */
    public int[] getResolution() {
        return RESOLUTION;
    }

    /**
     * Tuning helper for AprilTags.
     */
    public void setAprilTagDecimation() {
        setAprilTagDecimation(2.0f);
    }

    /**
     * Tuning helper for AprilTags.
     *
     * @param decimation The frame rate to use.
     */
    public void setAprilTagDecimation(float decimation) {
        APRIL_TAG.setDecimation(decimation);
    }

    /**
     * Return the latest color analysis from the predominant color processor. May be null if no frame has been processed
     * yet.
     *
     * @return The latest color analysis from the predominant color processor. May be {@code null} if no frame has been
     * processed yet.
     */
    public PredominantColorProcessor.Result getColorResult() {
        return COLOR_PROCESSOR.getAnalysis();
    }

    /**
     * Return the current pose adjustment [x, y, z] for the camera.
     *
     * @return The current pose adjustment [x, y, z] for the camera.
     */
    public double[] getPoseAdjust() {
        return poseAdjust;
    }

    /**
     * Set a new pose adjustment [x, y, z] for the camera. This stores the array reference directly.
     *
     * @param poseAdjust The new pose adjustment [x, y, z].
     */
    public void setPoseAdjust(double[] poseAdjust) {
        this.poseAdjust = poseAdjust;
    }

    /**
     * Return the currently selected target color (may be null).
     *
     * @return The currently selected target color (may be null).
     */
    public Color getTargetColor() {
        return targetColor;
    }

    /**
     * Set the alliance or team color used by your own code.
     *
     * @param targetColor The alliance or team color used by your own code.
     */
    public void setTargetColor(Color targetColor) {
        this.targetColor = targetColor;
    }
}