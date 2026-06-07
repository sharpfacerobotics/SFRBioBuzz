package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp(name = "DriverMode")
public class DriverMode extends CustomLinearOp {
    /**
     * Store the return values of {@link #processInput()}.
     */
    private static class ControlInput {
        private final double horizontal;
        private final double vertical;
        private final double pivot;

        private ControlInput(double horizontal, double vertical, double pivot) {
            this.horizontal = horizontal;
            this.vertical = vertical;
            this.pivot = pivot;
        }
    }

    /**
     * The sensitivity of the robot's driving joystick.
     * TODO: Replace the driving sensitivity with an appropriate level of
     * sensitivity.
     */
    private static final double DRIVING_SENSITIVITY = 1.0;
    /**
     * Minimum joystick magnitude required to register movement. Inputs below
     * this threshold will be treated as zero. This helps prevent unintended
     * robot motion when the driver releases the sticks and eliminates small
     * negative values (e.g., -0.29) shown in telemetry caused by joystick
     * drift.
     */
    private static final double JOY_STICK_DEADBAND = 0.07;
    private static final Logger log = LoggerFactory.getLogger(DriverMode.class);

    /**
     * Measured resting offsets for the driver controls. These values are
     * sampled during the init phase (before the match begins) while the driver
     * holds all sticks and triggers at their neutral positions. By subtracting
     * these offsets from the raw inputs each loop, we ensure that small bias or
     * drift does not cause the robot to creep when released.
     */
    private double verticalOffset = 0.0;
    /**
     * Measured resting offsets for the driver controls. These values are
     * sampled during the init phase (before the match begins) while the driver
     * holds all sticks and triggers at their neutral positions. By subtracting
     * these offsets from the raw inputs each loop, we ensure that small bias or
     * drift does not cause the robot to creep when released.
     */
    private double horizontalOffset = 0.0;
    /**
     * Measured resting offsets for the driver controls. These values are
     * sampled during the init phase (before the match begins) while the driver
     * holds all sticks and triggers at their neutral positions. By subtracting
     * these offsets from the raw inputs each loop, we ensure that small bias or
     * drift does not cause the robot to creep when released.
     */
    private double pivotOffset = 0.0;

    /**
     * Default setting of the camera stream. Set to {@code false} if you want it
     * off by default.
     */
    private boolean cameraStreamEnabled = true;
    /**
     * Check whether the camera is already on or off so that it is not
     * mistakenly opened or closed multiple times.
     */
    private boolean lastToggleBtn = false;

    /**
     * Clamp {@code value} between {@code min} and {@code max}. Do the exact
     * same thing as {@code  Math.clamp(float, float, float)} because
     * {@code  Math.code(float, float, float)} is restricted to SDK 35, while
     * FTC uses a minimum of SDK 24.
     *
     * @return {@code min} if {@code value} is less than {@code min}.
     * <p>
     * {@code max} if {@code value} is greater than {@code max}.
     * <p>
     * Otherwise, {@code value}.
     */
    private double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    /**
     * Apply a deadband to the given value. If the absolute value is less than
     * {@link #JOY_STICK_DEADBAND}, return 0; otherwise return the original
     * value.
     *
     * @param value The raw joystick value.
     * @return 0 if the value is within the deadband; otherwise, the unchanged
     * input value.
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < JOY_STICK_DEADBAND ? 0.0 : value;
    }

    /**
     * Calibrate driver control offsets. Ask the driver to release all sticks
     * and triggers during the init period. Sample the raw values over a brief
     * interval to compute average offsets. These offsets are subtracted from
     * the raw inputs each loop to cancel out any joystick drift.
     */
    private void calibrateOffsets() {
        telemetry.addLine("Calibrating controls... release sticks/triggers.");
        telemetry.update();
        long sampleEnd = System.currentTimeMillis() + 500; // sample for 0.5 s
        double verticalSum = 0.0;

        double horizontalSum = 0.0;
        double pivotSum = 0.0;
        int samples;
        for (
            samples = 0;
            !isStopRequested() && System.currentTimeMillis() < sampleEnd;
            samples++
        ) {
            // Sample the raw inputs using the same axes used in runLoop.
            // Use the same conventions as runLoop: negate right stick Y for
            // forward,
            // and use the difference of triggers for strafe. Do not apply
            // deadband here; we want the true rest position.
            verticalSum += -gamepad1.right_stick_y;
            horizontalSum += gamepad1.right_trigger - gamepad1.left_trigger;
            pivotSum += gamepad1.left_stick_x;
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
    }

    /**
     * Adjust inputs from gamepad controls. Subtract the offsets measured during
     * init so that small biases from imperfectly centred sticks/triggers does
     * not cause joystick drift.
     * <p>
     * Then apply a deadband to each value to clamp tiny drift to zero.
     * Forward/backward comes from the right stick Y-axis (up = forward). Negate
     * the value so pushing forward yields positive. Subtract
     * {@link #verticalOffset} measured during init.
     * <p>
     * Finally, multiplies each value by {@link #DRIVING_SENSITIVITY}
     *
     * @return An instance of {@link ControlInput} storing the processed input
     * values.
     */
    private ControlInput processInput() {
        // Strafing (horizontal) comes from the triggers: right trigger (ZR)
        // minus left trigger (ZL). Subtract horizontalOffset measured during
        // init. Positive values strafe right, negative values strafe left.
        double rawHorizontal = gamepad1.right_trigger - gamepad1.left_trigger;
        double correctedHorizontal = (
                                         gamepad1.right_trigger
                                         - gamepad1.left_trigger
                                     ) - horizontalOffset;

        double rawVertical = -gamepad1.right_stick_y;
        double correctedVertical = -gamepad1.right_stick_y - verticalOffset;

        // Turning (pivot) comes from the left stick X-axis.
        // Subtract pivotOffset measured during init.
        // Positive values produce clockwise rotation.
        double rawPivot = gamepad1.left_stick_x;
        double correctedPivot = gamepad1.left_stick_x - pivotOffset;

        // Apply deadband to each input to eliminate small stick drift and
        // unintended motion.
        // Scale the inputs by the driving sensitivity.
        double horizontal = applyDeadband(correctedHorizontal)
                            * DRIVING_SENSITIVITY;
        double vertical = applyDeadband(correctedVertical)
                          * DRIVING_SENSITIVITY;
        double pivot = applyDeadband(correctedPivot) * DRIVING_SENSITIVITY;

        // It is unlikely to cause issues, but for safety, clamp each value.
        horizontal = clamp(horizontal, -1.0, 1.0);
        vertical = clamp(vertical, -1.0, 1.0);
        pivot = clamp(vertical, -1.0, 1.0);

        /*
         * Telemetry: report the raw and processed inputs as well as the
         * computed motor powers. This aids in
         * diagnosing drift or inversion issues when testing on the field.
         */
        telemetry.addData(
            "Horizontal/corrected/rcorrected/aw",
            "%5.2f/%5.2f/%5.2f",
            horizontal,
            correctedHorizontal,
            rawHorizontal
        );
        telemetry.addData(
            "Vertical/corrected/raw",
            "%5.2f/%5.2f/%5.2f",
            vertical,
            correctedVertical,
            rawVertical
        );
        telemetry.addData(
            "Pivot/corrected/raw",
            "%5.2f/%5.2f/%5.2f",
            pivot,
            correctedPivot,
            rawPivot
        );

        return new ControlInput(horizontal, vertical, pivot);
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
        if (cameraStreamEnabled) {
            FtcDashboard.getInstance()
                        .startCameraStream(WEBCAM.getVisionPortal(), 0);
        }

        boolean toggleBtn = gamepad2.dpad_up;
        if (toggleBtn && !lastToggleBtn) {
            cameraStreamEnabled = !cameraStreamEnabled;
            if (cameraStreamEnabled) {
                FtcDashboard.getInstance()
                            .startCameraStream(WEBCAM.getVisionPortal(), 0);
            } else {
                FtcDashboard.getInstance().stopCameraStream();
            }
        }
        lastToggleBtn = toggleBtn;

        calibrateOffsets();

        while (opModeIsActive()) {
            try {
                runLoop();
            } catch (Exception exception) {
                telemetry.addLine("\nWARNING AN ERROR OCCURRED!!!");
                telemetry.addLine(exception.getMessage());
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
         * Right stick moves the robot forwards and backwards, left stick
         * turns it.
         * The triggers control strafing.
         */
        ControlInput result = processInput();

        /*
         * If all inputs are within the deadband, stop all four drive motors.
         * This prevents the robot from creeping when the sticks return to
         * center. By writing zero to each motor directly, we avoid any
         * lingering motion from previous commands.
         */
        if (result.vertical == 0.0
            && result.horizontal == 0.0
            && result.pivot == 0.0) {
            if (WHEELS != null) {
                WHEELS.drive(0);

            } else if (MECANUM_DRIVE != null) {
                // For RoadRunner fallback, send zero drive powers.
                MECANUM_DRIVE.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                        0,
                        0
                    ), 0
                ));
            }

        } else {
            if (WHEELS != null) {
                WHEELS.drive(result.vertical, result.horizontal, result.pivot);

            } else if (MECANUM_DRIVE != null) {
                /*
                 * For Road Runner fallback, convert our directional commands
                 *  to the +y forward/+x right convention.
                 * Note that vertical controls forward/backward; horizontal
                 * controls strafe; pivot controls rotation.
                 */
                PoseVelocity2d velocity = new PoseVelocity2d(
                    new Vector2d(result.horizontal, result.vertical),
                    result.pivot
                );
                MECANUM_DRIVE.setDrivePowers(velocity);
            }
        }

        telemetry.update();
    }
}