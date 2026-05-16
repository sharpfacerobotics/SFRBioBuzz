package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoConfig.AllianceColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardwaresystems.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "DriverMode")
public class DriverMode extends CustomLinearOp {
    // TODO: Replace the driving sensitivity with an appropriate level of sensitivity.
    /**
     * The sensitivity of the robot's driving joystick.
     */
    private static final double DRIVING_SENSITIVITY = 1.0;

    /**
     * Minimum joystick magnitude required to register movement. Inputs below this threshold will be treated as zero.
     * This helps prevent unintended robot motion when the driver releases the sticks and eliminates small negative
     * values (e.g. -0.29) shown in telemetry caused by joystick drift.
     */
    private static final double JOY_STICK_DEADBAND = 0.07;

    // Replace with your real IDs for this season
    private static final int[] RED_TAG_IDS = {24};
    private static final int[] BLUE_TAG_IDS = {20};

    /**
     * Measured resting offsets for the driver controls. These values are sampled during the init phase (before the
     * match begins) while the driver holds all sticks and triggers at their neutral positions. By subtracting these
     * offsets from the raw inputs each loop, we ensure that small bias or drift does not cause the robot to creep when
     * released.
     */
    private double verticalOffset = 0.0;
    /**
     * Measured resting offsets for the driver controls. These values are sampled during the init phase (before the
     * match begins) while the driver holds all sticks and triggers at their neutral positions. By subtracting these
     * offsets from the raw inputs each loop, we ensure that small bias or drift does not cause the robot to creep when
     * released.
     */
    private double horizontalOffset = 0.0;
    /**
     * Measured resting offsets for the driver controls. These values are sampled during the init phase (before the
     * match begins) while the driver holds all sticks and triggers at their neutral positions. By subtracting these
     * offsets from the raw inputs each loop, we ensure that small bias or drift does not cause the robot to creep when
     * released.
     */
    private double pivotOffset = 0.0;

    /**
     * Default setting of the camera stream. Set `false` if you want it off by default.
     */
    private boolean cameraStreamEnabled = true;
    private boolean lastToggleBtn = false;

    /**
     * Apply a deadband to the given value. If the absolute value is less than {@link #JOY_STICK_DEADBAND}, return zero;
     * otherwise return the original value.
     *
     * @param value The raw joystick value.
     * @return 0 if the value is within the deadband; otherwise, the unchanged input value.
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < JOY_STICK_DEADBAND ? 0.0 : value;
    }

    // Pick “best” detection: prefer alliance IDs
    private AprilTagDetection pickBestDetection(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) {
            return null;
        }

        int[] ids = (ALLIANCE_COLOR == AllianceColor.RED) ? RED_TAG_IDS : BLUE_TAG_IDS;

        // Need frame width for pixel scoring; fall back safely if webcam is null.
        double frameW = (WEBCAM != null) ? WEBCAM.getWidthPx() : 800.0;
        double cx = frameW / 2.0;

        AprilTagDetection best = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (AprilTagDetection detection : detections) {
            if (detection == null || detection.ftcPose == null || detection.center == null) {
                continue;
            }

            boolean idMatch = false;
            for (int id : ids) {
                if (detection.id == id) {
                    idMatch = true;
                    break;
                }
            }
            if (!idMatch) {
                continue;
            }

            // Prefer detections closest to camera center, then closer range.
            double errPx = Math.abs(detection.center.x - cx);
            double rangeM = detection.ftcPose.range * 0.0254; // inches -> meters

            // Higher score is better: small pixel error dominates.
            double score = -errPx - (25.0 * rangeM);

            if (score > bestScore) {
                bestScore = score;
                best = detection;
            }
        }

        if (best != null) {
            return best;
        }

        // Fallback: choose the smallest pixel error among all detections
        for (AprilTagDetection detection : detections) {
            if (detection == null || detection.center == null) {
                continue;
            }
            double score = -Math.abs(detection.center.x - cx);
            if (score > bestScore) {
                bestScore = score;
                best = detection;
            }
        }
        return best;
    }

    public void applyAllianceToWebcam() {
        if (WEBCAM == null) {
            return;
        }
        Webcam.Color color = ALLIANCE_COLOR == AllianceColor.RED ? Webcam.Color.RED : Webcam.Color.BLUE;
        WEBCAM.setTargetColor(color);
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
        if (cameraStreamEnabled) {
            FtcDashboard.getInstance().startCameraStream(WEBCAM.getVisionPortal(), 0);
        }

        boolean toggleBtn = gamepad2.dpad_up;
        if (toggleBtn && !lastToggleBtn) {
            cameraStreamEnabled = !cameraStreamEnabled;
            if (cameraStreamEnabled) {
                FtcDashboard.getInstance().startCameraStream(WEBCAM.getVisionPortal(), 0);
            } else {
                FtcDashboard.getInstance().stopCameraStream();
            }
        }
        lastToggleBtn = toggleBtn;

        applyAllianceToWebcam();

        // -----------------------------------------------------------------
        // Calibrate driver control offsets.
        // Ask the driver to release all sticks and triggers during the init period.
        // Sample the raw values over a brief interval to compute average offsets.
        // These offsets are subtracted from the raw inputs each loop
        // to cancel out any bias caused by imperfect centring of the controls.
        // -----------------------------------------------------------------
        telemetry.addLine("Calibrating controls... release sticks/triggers");
        telemetry.update();
        long sampleEnd = System.currentTimeMillis() + 500; // sample for 0.5 s
        double verticalSum = 0.0;
        double horizontalSum = 0.0;
        double pivotSum = 0.0;
        int samples = 0;
        while (!isStopRequested() && System.currentTimeMillis() < sampleEnd) {
            // Sample the raw inputs using the same axes used in runLoop.
            // Use the same conventions as runLoop: negate right stick Y for forward,
            // and use the difference of triggers for strafe. Do not apply
            // deadband here; we want the true rest position.
            double verticalRaw = -gamepad1.right_stick_y;
            double pivotRaw = gamepad1.left_stick_x;
            double horizontalRaw = gamepad1.right_trigger - gamepad1.left_trigger;

            verticalSum += verticalRaw;
            horizontalSum += horizontalRaw;
            pivotSum += pivotRaw;
            samples++;
            sleep(10);
        }
        if (samples > 0) {
            verticalOffset = verticalSum / samples;
            horizontalOffset = horizontalSum / samples;
            pivotOffset = pivotSum / samples;
        }
        telemetry.addData(
            "Control offsets", "V=%.2f H=%.2f P=%.2f",
            verticalOffset,
            horizontalOffset,
            pivotOffset
        );
        telemetry.update();


        while (opModeIsActive()) {
            try {
                runLoop();
            } catch (Exception e) {
                telemetry.addLine("\nWARNING AN ERROR OCCURRED!!!");
                telemetry.addLine(e.getMessage());
            }
        }
    }

    /**
     * Run the loop once.
     */
    private void runLoop() {
        /* Gamepad 1 (Wheel and Webcam Controls) */

        /* Wheel Controls */
        /*
         * Drive robot based on joystick input from gamepad1.
         * Right stick moves the robot forwards and backwards, left stick turns it.
         * The triggers control strafing.
         *
         * Adjust inputs from gamepad controls.
         * Subtract the offsets measured during init so that small biases
         * from imperfectly centred sticks/triggers does not cause the robot to move by itself.
         * Then apply a deadband to each value to clamp tiny drift to zero.
         *
         * Forward/backward comes from the right stick Y-axis (up = forward).
         * Negate the value so pushing forward yields positive.
         * Subtract `verticalOffset` measured during init.
         */
        double verticalRaw = -gamepad1.right_stick_y - verticalOffset;

        // Turning (pivot) comes from the left stick X-axis.
        // Subtract pivotOffset measured during init.
        // Positive values produce clockwise rotation.
        double pivotRaw = gamepad1.left_stick_x - pivotOffset;

        // Strafing (horizontal) comes from the triggers: right trigger (ZR)
        // minus left trigger (ZL). Subtract horizontalOffset measured during
        // init. Positive values strafe right, negative values strafe left.
        double horizontalRaw = (gamepad1.right_trigger - gamepad1.left_trigger) - horizontalOffset;

        // Apply deadband to each input to eliminate small stick drift and
        // unintended motion. Scale the inputs by the driving sensitivity.
        double vertical = applyDeadband(verticalRaw) * DRIVING_SENSITIVITY;
        double horizontal = applyDeadband(horizontalRaw) * DRIVING_SENSITIVITY;
        double pivot = applyDeadband(pivotRaw) * DRIVING_SENSITIVITY;

        /*
         * If all inputs are within the deadband, stop all four drive motors.
         * This prevents the robot from creeping when the sticks return to
         * center. By writing zero to each motor directly, we avoid any
         * lingering motion from previous commands.
         */
        if (vertical == 0.0 && horizontal == 0.0 && pivot == 0.0) {
            if (WHEELS != null) {
                WHEELS.drive(0);

            } else if (MECANUM_DRIVE != null) {
                // For RoadRunner fallback, send zero drive powers.
                MECANUM_DRIVE.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }

        } else {
            if (WHEELS != null) {
                WHEELS.drive(vertical, horizontal, pivot);

            } else if (MECANUM_DRIVE != null) {
                /*
                 * For Road Runner fallback, convert our directional commands to the +y forward/+x right convention.
                 * Note that vertical controls forward/backward; horizontal controls strafe; pivot controls rotation.
                 *  The forward value must be negated because PoseVelocity2d expects +y forward.
                 */
                PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(horizontal, -vertical), pivot);
                MECANUM_DRIVE.setDrivePowers(velocity);
            }
        }

        /*
         * Telemetry: report the raw and processed inputs as well as the computed motor powers. This aids in
         * diagnosing drift or inversion issues when testing on the field.
         */
        telemetry.addData("Vertical/raw", "%5.2f / %5.2f", vertical, verticalRaw);
        telemetry.addData("Horizontal/raw", "%5.2f / %5.2f", horizontal, horizontalRaw);
        telemetry.addData("Pivot/raw", "%5.2f / %5.2f", pivot, pivotRaw);

        telemetry.update();
    }
}