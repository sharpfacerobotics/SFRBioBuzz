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
    // Enum that holds HSV ranges for different colors used by the robot.
    public enum Color {
        // Red to reddish-orange hues.
        RED(new Scalar(0, 128, 64), new Scalar(10, 255, 255)),
        // Yellow-orange to yellow-green hues.
        YELLOW(new Scalar(20, 128, 64), new Scalar(33, 255, 255)),
        // Green hues (present for completeness / future use).
        GREEN(new Scalar(50, 128, 64), new Scalar(70, 255, 255)),
        // Teal to blue hues (used for blue alliance pixels).
        BLUE(new Scalar(90, 128, 64), new Scalar(125, 255, 255)),
        // Magenta to red hues (used for purple-style elements).
        // The hue range conceptually wraps around zero in HSV.
        MAGENTA(new Scalar(-170, 128, 64), new Scalar(180, 255, 255));
        // Lower HSV bound for this color.
        private final Scalar lowerBound;
        // Upper HSV bound for this color.
        private final Scalar upperBound;


        // Constructor for each color with its lower and upper HSV bounds.
        Color(Scalar lowerBound, Scalar upperBound) {
            this.lowerBound = lowerBound;
            this.upperBound = upperBound;
        }


        // Return the lower HSV bound for this color.
        public Scalar getLowerBound() {
            return lowerBound;
        }


        // Return the upper HSV bound for this color.
        public Scalar getUpperBound() {
            return upperBound;
        }


        // Return both bounds as a two-element array [lower, upper].
        public Scalar[] getRange() {
            return new Scalar[]{lowerBound, upperBound};
        }
    }

    // VisionPortal used to communicate with the webcam and run vision processors.
    private final VisionPortal visionPortal;
    // Processor that detects AprilTags in the camera image.
    private final AprilTagProcessor aprilTag;
    // Processor that finds the predominant color in a region of interest.
    private final PredominantColorProcessor colorProcessor;
    // Stores camera resolution so other code can compute pixel error
    private final int widthPx;
    private final int heightPx;
    // Alliance or team color you want to remember for this webcam (used by your code).
    private Color targetColor;
    // Offset of the camera relative to the robot center [x, y, z] in inches.
    // Used by localization or pose-estimation code outside this class.
    private double[] poseAdjust;


    // Construct a webcam wrapper with default pose offset (0,0,0).
    // This version does not use a custom preview container.
    // VisionPortal will handle the normal RC/DS preview.
    public Webcam(WebcamName webcamName, int[] resolution) {
        this(webcamName, resolution, new double[]{0.0, 0.0, 0.0});
    }


    // Construct a webcam wrapper with a specified pose offset and default preview.
    public Webcam(WebcamName webcamName, int[] resolution, double[] poseAdjust) {
        this(webcamName, resolution, poseAdjust, -1);
    }


    // Construct a webcam wrapper with an optional preview container ID.
    // If cameraMonitorViewId is -1, VisionPortal uses the default preview.
    // If cameraMonitorViewId is not -1, a custom preview container is used.
    public Webcam(
        WebcamName webcamName, int[] resolution,
        double[] poseAdjust, int cameraMonitorViewId
    ) {


        // Save pose adjustment values (reference is stored directly).
        this.poseAdjust = poseAdjust;

        // Saves resolution for pixel-based aiming
        this.widthPx = resolution[0];
        this.heightPx = resolution[1];

        // No target color selected by default.
        this.targetColor = null;


        // Create an AprilTag processor with default settings.
        aprilTag = new AprilTagProcessor.Builder().build();


        // Create a predominant color processor with a center ROI and a set of swatches.
        colorProcessor = new PredominantColorProcessor.Builder()
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
            .addProcessor(colorProcessor)
            .addProcessor(aprilTag)
            .setCamera(webcamName)
            .setCameraResolution(new Size(resolution[0], resolution[1]))
            .setAutoStopLiveView(true);


        // Create the VisionPortal instance.
        visionPortal = builder.build();
    }

    // Return the VisionPortal managing this webcam.
    // You can use this to enable/disable processors or pause/resume the preview.
    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    // Return the AprilTag processor for this webcam.
    public AprilTagProcessor getAprilTag() {
        return aprilTag;
    }

    // Return a copy of the current AprilTag detections.
    // The list may be empty if no tags are seen.
    public List<AprilTagDetection> getAprilTagDetections() {
        // Copy into a new list so callers cannot modify the internal list.
        return new ArrayList<>(aprilTag.getDetections());
    }

    // Return the predominant color processor.
    // You can use this directly if you want to read more detailed color info.
    public PredominantColorProcessor getColorProcessor() {
        return colorProcessor;
    }

    // Allows other code to read camera resolution
    public int getWidthPx() {
        return widthPx;
    }

    public int getHeightPx() {
        return heightPx;
    }

    // Tuning helper
    public void setAprilTagDecimation(float decimation) {
        aprilTag.setDecimation(2.0f);
    }

    // Return the latest color analysis from the predominant color processor.
    // May be null if no frame has been processed yet.
    public PredominantColorProcessor.Result getColorResult() {
        return colorProcessor.getAnalysis();
    }

    // Return the current pose adjustment [x, y, z] for the camera.
    public double[] getPoseAdjust() {
        return poseAdjust;
    }

    // Set a new pose adjustment [x, y, z] for the camera.
    // This stores the array reference directly.
    public void setPoseAdjust(double[] poseAdjust) {
        this.poseAdjust = poseAdjust;
    }

    // Return the currently selected target color (may be null).
    public Color getTargetColor() {
        return targetColor;
    }

    // Set the alliance or team color used by your own code.
    public void setTargetColor(Color targetColor) {
        this.targetColor = targetColor;
    }

    // Stub kept for backward compatibility.
    // Older code expected an OpenCvCamera here; now we only use VisionPortal.
    // Always returns null.
    public Object getOpenCvCamera() {
        return null;
    }

    // Stub kept for backward compatibility.
    // Older code used a custom pipeline object; this is no longer used here.
    // Always returns null.
    public Object getPipeLine() {
        return null;
    }

    // Stub kept for backward compatibility.
    // Previously returned the center of a detected color blob.
    // Now this class does not do blob detection, so we return [-1.0, -1.0].
    public double[] getContourPosition() {
        return new double[]{-1.0, -1.0};
    }
}