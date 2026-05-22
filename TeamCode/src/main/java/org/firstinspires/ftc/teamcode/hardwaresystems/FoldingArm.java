package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashSet;
import java.util.Set;

/**
 * An {@link Arm} that is capable of folding in the middle (much like a human
 * elbow) and rotating the entire arm (like a shoulder, but with only one degree
 * of freedom).
 */
public class FoldingArm extends Arm {
    /**
     * Passed into the
     * {@link FoldingArm#FoldingArm(FoldingArmMotors, RotationParameters,
     * FoldingParameters)} constructor. Contains the motors and motor types.
     */
    public static class FoldingArmMotors {
        /**
         * All the motors to be used by {@link FoldingArm}.
         */
        protected final Set<DcMotor> motorSet;

        /**
         * The motor that rotates the entire arm.
         */
        protected DcMotor rotationMotor;
        /**
         * The motor that folds the arm.
         */
        protected DcMotor foldingMotor;

        public FoldingArmMotors(DcMotor rotationMotor, DcMotor foldingMotor) {
            motorSet = new HashSet<>();
            motorSet.add(rotationMotor);
            motorSet.add(foldingMotor);

            this.rotationMotor = rotationMotor;
            this.foldingMotor = foldingMotor;
        }

        public FoldingArmMotors() {
            motorSet = new HashSet<>();

            rotationMotor = null;
            foldingMotor = null;
        }

        /**
         * Check if either motor ({@link #rotationMotor} or
         * {@link #foldingMotor}) is {@code null}.
         *
         * @return {@code true} if either motor ({@link #rotationMotor} or
         * {@link #foldingMotor}) is {@code null}.
         * <p>
         * {@code false} otherwise.
         */
        protected boolean containsNull() {
            return rotationMotor == null || foldingMotor == null;
        }

        /**
         * Set the motor that controls folding, i.e., moving the entire arm in a
         * circular manner.
         *
         * @param rotationMotor The new motor to control rotation, i.e., moving
         *                      the entire arm in a circular manner.
         */
        protected void setRotationMotor(DcMotor rotationMotor) {
            motorSet.remove(this.rotationMotor);

            this.rotationMotor = rotationMotor;
            motorSet.add(rotationMotor);
        }

        /**
         * Set the motor that controls folding, i.e., bending the arm in the
         * middle.
         *
         * @param foldingMotor The new motor to control folding, i.e., bending
         *                     the arm in the middle.
         */
        protected void setFoldingMotor(DcMotor foldingMotor) {
            motorSet.remove(this.foldingMotor);

            this.foldingMotor = foldingMotor;
            motorSet.add(foldingMotor);
        }
    }

    /**
     * Passed into the
     * {@link FoldingArm#FoldingArm(FoldingArmMotors, RotationParameters,
     * FoldingParameters)} constructor. Contains the minimum rotation, maximum
     * rotation, and ticks per degree.
     */
    public static class RotationParameters {
        /**
         * The minimum rotation of the arm in ticks.
         */
        private int minTicks;
        /**
         * The maximum rotation of the arm in ticks.
         */
        private int maxTicks;
        /**
         * Whether the minimum and maximum was set using ticks or degrees. If
         * they were set using degrees, then they must be adjusted if the
         * initial angle or ticks per degree are changed.
         */
        private boolean degreesMode;

        /**
         * The angle that the arm rotation starts from. 0 ticks will be
         * considered to be equal to this angle.
         */
        private double initialAngle;

        /**
         * How many ticks it takes to rotate the arm by one degree.
         */
        private double ticksPerDegree;

        public RotationParameters(
            int minTicks,
            int maxTicks,
            double ticksPerDegree
        ) {
            this(minTicks, maxTicks, 0, ticksPerDegree);
        }

        public RotationParameters(
            int minTicks,
            int maxTicks,
            double initialAngle,
            double ticksPerDegree
        ) {
            this.minTicks = minTicks;
            this.maxTicks = maxTicks;
            degreesMode = false;

            this.initialAngle = initialAngle;
            this.ticksPerDegree = ticksPerDegree;
        }

        protected boolean isInvalid() {
            return minTicks >= maxTicks || ticksPerDegree <= 0;
        }
    }

    /**
     * Passed into the
     * {@link FoldingArm#FoldingArm(FoldingArmMotors, RotationParameters,
     * FoldingParameters)} constructor. Contains the minimum folding, maximum
     * folding, and ticks per degree.
     */
    public static class FoldingParameters {
        /**
         * The minimum folding of the arm in ticks.
         */
        private int minTicks;
        /**
         * The maximum folding of the arm in ticks.
         */
        private int maxTicks;
        /**
         * Whether the minimum and maximum was set using ticks or degrees. If
         * they were set using degrees, then they must be adjusted if the
         * initial angle or ticks per degree are changed.
         */
        private boolean degreesMode;

        /**
         * How many ticks are in a degree.
         */
        private double ticksPerDegree;
        /**
         * The angle that the arm folding starts from. 0 ticks will be
         * considered to be equal to this angle.
         */
        private double initialAngle;

        public FoldingParameters(
            int minTicks,
            int maxTicks,
            double ticksPerDegree
        ) {
            this.minTicks = minTicks;
            this.maxTicks = maxTicks;

            this.ticksPerDegree = ticksPerDegree;
        }

        public FoldingParameters(
            int minTicks,
            int maxTicks,
            double initialAngle,
            double ticksPerDegree
        ) {
            this.minTicks = minTicks;
            this.maxTicks = maxTicks;
            degreesMode = false;

            this.initialAngle = initialAngle;
            this.ticksPerDegree = ticksPerDegree;
        }

        protected boolean isInvalid() {
            return minTicks >= maxTicks || ticksPerDegree <= 0;
        }
    }

    @SuppressWarnings("UnusedReturnValue")
    public static class Builder extends Arm.Builder {
        protected final FoldingArmMotors foldingArmMotors;
        protected final RotationParameters rotationParameters;
        protected final FoldingParameters foldingParameters;

        /**
         * Whether the minimum and maximum was set using ticks or degrees. If
         * they were set using degrees, then they must be adjusted if the
         * initial angle or ticks per degree are changed.
         */
        protected boolean degreesMode;

        public Builder() {
            foldingArmMotors = new FoldingArmMotors();
            rotationParameters = new RotationParameters(0, 0, 0);
            foldingParameters = new FoldingParameters(0, 0, 0);

            degreesMode = false;
        }

        public Builder setRotationMotor(DcMotor rotationMotor) {
            foldingArmMotors.setRotationMotor(rotationMotor);
            return this;
        }

        public Builder setFoldingMotor(DcMotor foldingMotor) {
            foldingArmMotors.setFoldingMotor(foldingMotor);
            return this;
        }

        public Builder setRotationRangeTicks(int minTicks, int maxTicks) {
            rotationParameters.minTicks = minTicks;
            rotationParameters.maxTicks = maxTicks;

            rotationParameters.degreesMode = false;
            return this;
        }

        public Builder setRotationRangeDegrees(
            double minDegrees,
            double maxDegrees
        ) {
            rotationParameters.degreesMode = true;
            return setRotationRangeTicks(
                (int) Math.round(
                    (minDegrees - rotationParameters.initialAngle)
                    * rotationParameters.ticksPerDegree
                ),
                (int) Math.round(
                    (maxDegrees - rotationParameters.initialAngle)
                    * rotationParameters.ticksPerDegree
                )
            );
        }

        public Builder setRotationTicksPerDegree(double ticksPerDegree) {
            // setRotationRangeDegrees() relies on ticksPerDegree, so we also
            // need to correct the range.
            if (rotationParameters.degreesMode) {
                setRotationRangeTicks(
                    (int) Math.round(
                        rotationParameters.minTicks * ticksPerDegree
                        / rotationParameters.ticksPerDegree
                    ),
                    (int) Math.round(
                        rotationParameters.maxTicks * ticksPerDegree
                        / rotationParameters.ticksPerDegree
                    )
                );
            }

            rotationParameters.ticksPerDegree = ticksPerDegree;
            return this;
        }

        public Builder setRotationInitialAngle(double initialAngle) {
            // If the range was set using degrees, correct for the new
            // initial angle.
            if (rotationParameters.degreesMode) {
                setRotationRangeTicks(
                    (int) Math.round(
                        rotationParameters.minTicks
                        + (rotationParameters.initialAngle - initialAngle)
                          * rotationParameters.ticksPerDegree
                    ),
                    (int) Math.round(
                        rotationParameters.maxTicks
                        + (rotationParameters.initialAngle - initialAngle)
                          * rotationParameters.ticksPerDegree
                    )
                );
            }

            rotationParameters.initialAngle = initialAngle;
            return this;
        }

        public Builder setFoldingRangeTicks(int minTicks, int maxTicks) {
            foldingParameters.minTicks = minTicks;
            foldingParameters.maxTicks = maxTicks;

            foldingParameters.degreesMode = false;
            return this;
        }

        public Builder setFoldingRangeDegrees(
            double minDegrees,
            double maxDegrees
        ) {
            foldingParameters.degreesMode = true;
            return setFoldingRangeTicks(
                (int) Math.round(
                    (minDegrees - foldingParameters.initialAngle)
                    * foldingParameters.ticksPerDegree
                ),
                (int) Math.round(
                    (maxDegrees - foldingParameters.initialAngle)
                    * foldingParameters.ticksPerDegree
                )
            );
        }

        public Builder setFoldingTicksPerDegree(int ticksPerDegree) {
            // setFoldingRangeDegrees() relies on ticksPerDegree, so we also
            // need to correct the range.
            if (foldingParameters.degreesMode) {
                setFoldingRangeTicks(
                    (int) Math.round(
                        foldingParameters.minTicks * ticksPerDegree
                        / foldingParameters.ticksPerDegree
                    ),
                    (int) Math.round(
                        foldingParameters.maxTicks * ticksPerDegree
                        / foldingParameters.ticksPerDegree
                    )
                );
            }

            foldingParameters.ticksPerDegree = ticksPerDegree;
            return this;
        }

        public Builder setFoldingInitialAngle(double initialAngle) {
            // If the range was set using degrees, correct for the new
            // initial angle.
            if (foldingParameters.degreesMode) {
                setRotationRangeTicks(
                    (int) Math.round(
                        foldingParameters.minTicks
                        + (foldingParameters.initialAngle - initialAngle)
                          * foldingParameters.ticksPerDegree
                    ),
                    (int) Math.round(
                        foldingParameters.maxTicks
                        + (foldingParameters.initialAngle - initialAngle)
                          * foldingParameters.ticksPerDegree
                    )
                );
            }

            foldingParameters.initialAngle = initialAngle;
            return this;
        }

        @Override
        public FoldingArm build() {
            if (
                foldingArmMotors.containsNull()
                || rotationParameters.isInvalid()
                || foldingParameters.isInvalid()
            ) {
                return null;
            }

            return new FoldingArm(
                foldingArmMotors,
                rotationParameters,
                foldingParameters
            );
        }
    }

    /**
     * The motor power that the arm uses when rotating.
     */
    private static final double ROTATION_POWER = 1;
    /**
     * The motor power that the arm uses when rotating.
     */
    private static final double FOLDING_POWER = 0.75;

    /**
     * The motor that rotates the arm up and down.
     */
    private final DcMotor ROTATION_MOTOR;
    /**
     * The minimum rotation of the arm in ticks.
     */
    private final int MIN_ROTATION;
    /**
     * The maximum rotation of the arm in ticks.
     */
    private final int MAX_ROTATION;
    /**
     * The angle that the arm rotation starts from. 0 ticks will be considered
     * equivalent to this angle.
     */
    private final double INITIAL_ROTATION_ANGLE;
    /**
     * How many ticks it takes to rotate the arm by one degree.
     */
    private final double TICKS_PER_ROTATION_DEGREE;

    /**
     * The motor that folds and retracts the arm.
     */
    private final DcMotor FOLDING_MOTOR;
    /**
     * The minimum extension of the arm in ticks.
     */
    private final int MIN_FOLDING;
    /**
     * The maximum extension of the arm in ticks.
     */
    private final int MAX_FOLDING;
    /**
     * The angle that the arm folding starts from. 0 ticks will be considered
     * equivalent to this angle.
     */
    private final double INITIAL_FOLDING_ANGLE;
    /**
     * How many ticks it takes to rotate the arm by one degree.
     */
    private final double TICKS_PER_FOLDING_DEGREE;

    /**
     * Instantiate a foldable arm.
     *
     * @param foldingArmMotors   The motors and motor types.
     * @param rotationParameters The minimum rotation, maximum rotation, and
     *                           ticks per degree.
     * @param foldingParameters  The minimum folding, maximum folding, and ticks
     *                           per degree.
     */
    public FoldingArm(
        FoldingArmMotors foldingArmMotors,
        RotationParameters rotationParameters,
        FoldingParameters foldingParameters
    ) {
        super(foldingArmMotors.motorSet);

        this.ROTATION_MOTOR = foldingArmMotors.rotationMotor;
        this.MIN_ROTATION = rotationParameters.minTicks;
        this.MAX_ROTATION = rotationParameters.maxTicks;
        this.INITIAL_ROTATION_ANGLE = rotationParameters.initialAngle;
        this.TICKS_PER_ROTATION_DEGREE = rotationParameters.ticksPerDegree;

        this.FOLDING_MOTOR = foldingArmMotors.foldingMotor;
        this.FOLDING_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.MIN_FOLDING = foldingParameters.minTicks;
        this.MAX_FOLDING = foldingParameters.maxTicks;
        this.INITIAL_FOLDING_ANGLE = foldingParameters.initialAngle;
        this.TICKS_PER_FOLDING_DEGREE = foldingParameters.ticksPerDegree;

        // Reset position to 0
        for (DcMotor motor : motorSet) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getRotationPower() {
        return ROTATION_POWER;
    }

    public double getFoldingPower() {
        return FOLDING_POWER;
    }

    public DcMotor getRotationMotor() {
        return ROTATION_MOTOR;
    }

    public DcMotor getFoldingMotor() {
        return FOLDING_MOTOR;
    }

    public int getRotationTicks() {
        return ROTATION_MOTOR.getCurrentPosition();
    }

    /**
     * Return the rotation of the arm in degrees.
     *
     * @return A {@code double} representing the rotation angle of the arm in
     * degrees.
     */
    public double getRotationDegrees() {
        return ROTATION_MOTOR.getCurrentPosition() / TICKS_PER_ROTATION_DEGREE
               + INITIAL_ROTATION_ANGLE;
    }

    /**
     * Rotate the arm with a set velocity. Stop the motor if it is out of
     * bounds.
     *
     * @param power The power that the arm should rotate with. Positive values
     *              rotate it up, negative values rotate it down, and zero stops
     *              it.
     */
    public void rotate(double power) throws IllegalStateException {
        // Check for out of bounds position.
        if (
            ROTATION_MOTOR.getCurrentPosition() > MAX_ROTATION
            || ROTATION_MOTOR.getCurrentPosition() < MIN_ROTATION
        ) {
            ROTATION_MOTOR.setPower(0);
            throw new IllegalStateException("Arm rotation reached limits");
        }

        ROTATION_MOTOR.setPower(power * ROTATION_POWER);
    }

    /**
     * Rotates the arm to a position specified in degrees.
     *
     * @param degrees The position the arm moves to. The arm's starting position
     *                is 0 degrees.
     */
    public void rotateToAngle(double degrees) {
        double targetDegrees = degrees - INITIAL_ROTATION_ANGLE;
        int targetPosition = (int) -Math.round(targetDegrees
                                               * TICKS_PER_ROTATION_DEGREE);
        // Keep the target position within acceptable bounds
        targetPosition = Math.min(
            Math.max(targetPosition, MIN_ROTATION),
            MAX_ROTATION
        );
        ROTATION_MOTOR.setTargetPosition(targetPosition);

        /*
         * Calculate the direction that the arm will have to rotate.
         * Negative is down, positive is up
         */
        int direction = (int) Math.signum(
            targetPosition
            - ROTATION_MOTOR.getCurrentPosition()
        );
        ROTATION_MOTOR.setPower(direction * ROTATION_POWER);

        ROTATION_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getFoldingTicks() {
        return FOLDING_MOTOR.getCurrentPosition();
    }

    /**
     * Return the folding of the arm in degrees.
     *
     * @return A double representing the folding angle of the arm in degrees.
     */
    public double getFoldingDegrees() {
        return FOLDING_MOTOR.getCurrentPosition() / TICKS_PER_FOLDING_DEGREE
               + INITIAL_FOLDING_ANGLE;
    }

    /**
     * Fold the arm with a given power.
     *
     * @param direction The direction that the extension motor moves. Positive
     *                  values fold the arm, negative values retract it.
     */
    public void fold(double direction) throws IllegalStateException {
        if (
            FOLDING_MOTOR.getCurrentPosition() > MAX_FOLDING
            || FOLDING_MOTOR.getCurrentPosition() < MIN_FOLDING
        ) {
            FOLDING_MOTOR.setPower(0);
            throw new IllegalStateException("Arm folding reached limits.");
        }

        FOLDING_MOTOR.setPower(direction * FOLDING_POWER);
    }

    /**
     * Fold the arm to a certain number of degrees.
     *
     * @param degrees The position to move the joint of the arm to in degrees.
     */
    public void foldToAngle(double degrees) {
        double targetDegrees = degrees - INITIAL_FOLDING_ANGLE;
        int targetPosition = (int) Math.round(
            targetDegrees * TICKS_PER_FOLDING_DEGREE
        );
        // Keep the target position within acceptable bounds
        targetPosition = -Math.min(
            Math.max(targetPosition, MIN_FOLDING),
            MAX_FOLDING
        );
        FOLDING_MOTOR.setTargetPosition(targetPosition);

        int direction = (int) Math.signum(
            targetPosition
            - FOLDING_MOTOR.getCurrentPosition()
        );
        FOLDING_MOTOR.setPower(direction * FOLDING_POWER);

        FOLDING_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Fold the arm to a certain number of ticks.
     *
     * @param targetPosition The position to move the joint of the arm to in
     *                       ticks.
     */
    public void foldToPosition(int targetPosition) {
        // Keep the target position within acceptable bounds
        targetPosition = Math.min(
            Math.max(targetPosition, MIN_FOLDING),
            MAX_FOLDING
        );
        FOLDING_MOTOR.setTargetPosition(targetPosition);

        // Get the direction of turning.
        int direction = (int) Math.signum(
            targetPosition
            - FOLDING_MOTOR.getCurrentPosition()
        );
        FOLDING_MOTOR.setPower(direction * FOLDING_POWER);

        FOLDING_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}