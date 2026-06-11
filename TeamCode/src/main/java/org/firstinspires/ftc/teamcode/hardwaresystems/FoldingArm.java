package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Set;

/**
 * An {@link Arm} that is capable of folding in the middle (much like a human
 * elbow) and rotating the entire arm (like a shoulder, but with only one degree
 * of freedom).
 */
@SuppressWarnings("unused")
public class FoldingArm extends Arm {
    /**
     * A builder for creating instances of {@link FoldingArm}.
     */
    @SuppressWarnings("UnusedReturnValue")
    public static class Builder extends Arm.Builder {
        /**
         * The motor that rotates the entire arm. Used to set
         * {@link #ROTATION_MOTOR}.
         */
        protected DcMotor rotationMotor;
        /**
         * The motor that folds the arm in the middle. Used to set
         * {@link #FOLDING_MOTOR}.
         */
        protected DcMotor foldingMotor;

        /**
         * The minimum rotation of the arm in ticks.
         */
        private int minRotationTicks;
        /**
         * The maximum rotation of the arm in ticks.
         */
        private int maxRotationTicks;
        /**
         * Whether the minimum and maximum was set using ticks or degrees. If
         * they were set using degrees, then they must be adjusted if the
         * initial angle or ticks per degree are changed.
         */
        private boolean rotationDegreesMode;
        /**
         * The angle that the arm rotation starts from. 0 ticks will be
         * considered to be equal to this angle.
         */
        private double initialRotationAngle;
        /**
         * How many ticks it takes to rotate the arm by one degree.
         */
        private double ticksPerRotationDegree;
        /**
         * The maximum power that the {@link #ROTATION_MOTOR} moves with. Used
         * to set {@link #MAX_ROTATION_POWER}.
         */
        private double maxRotationPower;

        /**
         * The minimum rotation of the arm in ticks.
         */
        private int minFoldingTicks;
        /**
         * The maximum rotation of the arm in ticks.
         */
        private int maxFoldingTicks;
        /**
         * Whether the minimum and maximum was set using ticks or degrees. If
         * they were set using degrees, then they must be adjusted if the
         * initial angle or ticks per degree are changed.
         */
        private boolean foldingDegreesMode;
        /**
         * The angle that the arm rotation starts from. 0 ticks will be
         * considered to be equal to this angle.
         */
        private double initialFoldingAngle;
        /**
         * How many ticks it takes to rotate the arm by one degree.
         */
        private double ticksPerFoldingDegree;
        /**
         * The maximum power that the {@link #ROTATION_MOTOR} moves with. Used
         * to set {@link #MAX_ROTATION_POWER}.
         */
        private double maxFoldingPower;

        /**
         * Instantiate a new {@link Builder} with all parameters set to their
         * default value.
         */
        public Builder() {
            super();

            rotationMotor = null;
            foldingMotor = null;

            minRotationTicks = -1;
            maxRotationTicks = -1;
            initialRotationAngle = 0.0;
            ticksPerRotationDegree = -1.0;
            rotationDegreesMode = false;
            maxRotationPower = 1.0;

            minRotationTicks = 0;
            maxRotationTicks = 0;
            initialRotationAngle = 0.0;
            ticksPerRotationDegree = 0.0;
            rotationDegreesMode = false;
            maxFoldingPower = 0.75;

        }

        /**
         * Set the motor that controls the {@link Arm}'s rotation.
         *
         * @param rotationMotor The new {@link DcMotor} to control the
         *                      {@link Arm}'s rotation.
         * @return This {@link Builder} so that methods can be chained.
         */
        public Builder setRotationMotor(DcMotor rotationMotor) {
            this.rotationMotor = rotationMotor;
            return this;
        }

        /**
         * Set the motor that controls the {@link Arm}'s folding.
         *
         * @param foldingMotor The new motor to set the {@link Arm}'s folding.
         * @return This {@link Builder} so that methods can be chained.
         */
        public Builder setFoldingMotor(DcMotor foldingMotor) {
            this.foldingMotor = foldingMotor;
            return this;
        }

        /**
         * Set the minimum and maximum rotation in ticks.
         *
         * @param minTicks The minimum number of ticks the
         *                 {@link #ROTATION_MOTOR} can reach.
         * @param maxTicks The maximum number of ticks the
         *                 {@link #ROTATION_MOTOR} can reach.
         * @return This {@link Builder} so that the methods can be chained.
         */
        public Builder setRotationRangeTicks(int minTicks, int maxTicks) {
            minRotationTicks = minTicks;
            maxRotationTicks = maxTicks;

            rotationDegreesMode = false;
            return this;
        }

        /**
         * Set the minimum and maximum rotation in degrees. Internally, it
         * converts the degrees to ticks using the {@link #initialRotationAngle}
         * and {@link #ticksPerRotationDegree} properties, so everytime those
         * two properties are updated, the ticks will also be automatically
         * updated.
         *
         * @param minDegrees The minimum range of the {@link #ROTATION_MOTOR} in
         *                   degrees.
         * @param maxDegrees The maximum range of the {@link #ROTATION_MOTOR} in
         *                   degrees.
         * @return This {@link Builder} so that setters can be chained.
         */
        public Builder setRotationRangeDegrees(
            double minDegrees,
            double maxDegrees
        ) {
            rotationDegreesMode = true;
            return setRotationRangeTicks(
                (int) Math.round(
                    (minDegrees - initialRotationAngle)
                    * ticksPerRotationDegree
                ),
                (int) Math.round(
                    (maxDegrees - initialRotationAngle)
                    * ticksPerRotationDegree
                )
            );
        }

        /**
         * Set the number of ticks it takes to rotate the
         * {@link #ROTATION_MOTOR} by one degree.
         * <p>
         * If {@link #setRotationRangeDegrees(double, double)} was used, update
         * the range.
         *
         * @param ticksPerDegree The number of ticks it takes to rotate the
         *                       {@link #ROTATION_MOTOR} by one degree
         * @return This {@link Builder} so that setters can be chained.
         */
        public Builder setRotationTicksPerDegree(double ticksPerDegree) {
            // setRotationRangeDegrees() relies on ticksPerDegree, so we also
            // need to correct the range.
            if (rotationDegreesMode) {
                setRotationRangeTicks(
                    (int) Math.round(
                        minRotationTicks * ticksPerDegree
                        / ticksPerRotationDegree
                    ),
                    (int) Math.round(
                        maxRotationTicks * ticksPerDegree
                        / ticksPerRotationDegree
                    )
                );
            }

            ticksPerRotationDegree = ticksPerDegree;
            return this;
        }

        /**
         * Set the initial rotation angle of the {@link Arm}. It does
         * <em>not</em> move the {@link Arm}. Rather, it just changes the
         * baseline for 0 degrees.
         * <p>
         * If {@link #setRotationRangeDegrees(double, double)} was used, update
         * the range.
         *
         * @param initialAngle The initial rotation angle of the {@link Arm},
         *                     which determines the offset that the angle is
         *                     measured from.
         * @return This {@link Builder} so that setters can be chained.
         */
        public Builder setRotationInitialAngle(double initialAngle) {
            // If the range was set using degrees, correct for the new
            // initial angle.
            if (rotationDegreesMode) {
                setRotationRangeTicks(
                    (int) Math.round(
                        minRotationTicks
                        + (initialRotationAngle - initialAngle)
                          * ticksPerRotationDegree
                    ),
                    (int) Math.round(
                        maxRotationTicks
                        + (initialRotationAngle - initialAngle)
                          * ticksPerRotationDegree
                    )
                );
            }

            initialRotationAngle = initialAngle;
            return this;
        }

        /**
         * Set the power that the {@link #ROTATION_MOTOR} moves with, which is
         * clamped to be between 0.0 and 1.0 (inclusive).
         *
         * @param power The power to set the {@link #ROTATION_MOTOR} to. Any
         *              values less than 0.0 or greater than 1.0 will be
         *              clamped.
         * @return The {@link Builder} instance to allow for method chaining.
         */
        public Builder setRotationPower(double power) {
            maxRotationPower = Math.min(Math.max(power, 0.0), 1.0);
            return this;
        }

        /**
         * Set the minimum and maximum folding in ticks.
         *
         * @param minTicks The minimum number of ticks the
         *                 {@link #FOLDING_MOTOR} can reach.
         * @param maxTicks The maximum number of ticks the
         *                 {@link #FOLDING_MOTOR} can reach.
         * @return This {@link Builder} so that the methods can be chained.
         */
        public Builder setFoldingRangeTicks(int minTicks, int maxTicks) {
            minFoldingTicks = minTicks;
            maxFoldingTicks = maxTicks;

            foldingDegreesMode = false;
            return this;
        }

        /**
         * Set the minimum and maximum folding in degrees. Internally, it
         * converts the degrees to ticks using the {@link #initialFoldingAngle}
         * and {@link #ticksPerFoldingDegree} properties, so everytime those two
         * properties are updated, the ticks will also be automatically
         * updated.
         *
         * @param minDegrees The minimum range of the {@link #FOLDING_MOTOR} in
         *                   degrees.
         * @param maxDegrees The maximum range of the {@link #FOLDING_MOTOR} in
         *                   degrees.
         * @return This {@link Builder} so that setters can be chained.
         */
        public Builder setFoldingRangeDegrees(
            double minDegrees,
            double maxDegrees
        ) {
            foldingDegreesMode = true;
            return setFoldingRangeTicks(
                (int) Math.round(
                    (minDegrees - initialFoldingAngle)
                    * ticksPerFoldingDegree
                ),
                (int) Math.round(
                    (maxDegrees - initialFoldingAngle)
                    * ticksPerFoldingDegree
                )
            );
        }

        /**
         * Set the number of ticks it takes to rotate the {@link #FOLDING_MOTOR}
         * by one degree.
         * <p>
         * If {@link #setRotationRangeDegrees(double, double)} was used, update
         * the range.
         *
         * @param ticksPerDegree The number of ticks it takes to rotate the
         *                       {@link #FOLDING_MOTOR} by one degree
         * @return This {@link Builder} so that setters can be chained.
         */
        public Builder setFoldingTicksPerDegree(int ticksPerDegree) {
            // setFoldingRangeDegrees() relies on ticksPerDegree, so we also
            // need to correct the range.
            if (foldingDegreesMode) {
                setFoldingRangeTicks(
                    (int) Math.round(minFoldingTicks * ticksPerDegree
                                     / ticksPerFoldingDegree),
                    (int) Math.round(maxFoldingPower * ticksPerDegree
                                     / ticksPerFoldingDegree)
                );
            }

            ticksPerFoldingDegree = ticksPerDegree;
            return this;
        }

        /**
         * Set the initial folding angle of the {@link Arm}. It does
         * <em>not</em> move the {@link Arm}. Rather, it just changes the
         * baseline for 0 degrees.
         * <p>
         * If {@link #setRotationRangeDegrees(double, double)} was used, update
         * the range.
         *
         * @param initialAngle The initial folding angle of the {@link Arm},
         *                     which determines the offset that the angle is
         *                     measured from.
         * @return This {@link Builder} so that setters can be chained.
         */
        public Builder setFoldingInitialAngle(double initialAngle) {
            // If the range was set using degrees, correct for the new
            // initial angle.
            if (foldingDegreesMode) {
                setRotationRangeTicks(
                    (int) Math.round(
                        minFoldingTicks
                        + (initialFoldingAngle - initialAngle)
                          * ticksPerFoldingDegree
                    ),
                    (int) Math.round(
                        maxFoldingPower
                        + (initialFoldingAngle - initialAngle)
                          * ticksPerFoldingDegree
                    )
                );
            }

            initialFoldingAngle = initialAngle;
            return this;
        }

        /**
         * Set the power that the {@link #FOLDING_MOTOR} moves with, which is
         * clamped to be between 0.0 and 1.0 (inclusive).
         *
         * @param power The power to set the {@link #FOLDING_MOTOR} to. Any
         *              values less than 0.0 or greater than 1.0 will be
         *              clamped.
         * @return The {@link Builder} instance to allow for method chaining.
         */
        public Builder setFoldingPower(double power) {
            maxFoldingPower = Math.min(Math.max(power, 0.0), 1.0);
            return this;
        }

        @Override
        public boolean isValid() {
            return super.isValid()
                   && rotationMotor != null
                   && foldingMotor != null
                   && minRotationTicks < maxRotationPower
                   && ticksPerRotationDegree > 0.0
                   && 0.0 < maxRotationTicks
                   && maxRotationPower < 1.0
                   && minFoldingTicks < maxFoldingPower
                   && ticksPerFoldingDegree > 0.0
                   && 0.0 < maxFoldingPower
                   && maxFoldingPower < 1.0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public FoldingArm build() {
            return isValid() ? new FoldingArm(this) : null;
        }
    }

    /**
     * The motor that rotates the arm up and down.
     */
    private final DcMotor ROTATION_MOTOR;
    /**
     * The minimum rotation of the arm in ticks.
     */
    private final int MIN_ROTATION_TICKS;
    /**
     * The maximum rotation of the arm in ticks.
     */
    private final int MAX_ROTATION_TICKS;
    /**
     * The angle that the arm rotation starts from. 0 ticks will be considered
     * equivalent to this angle.
     */
    private final double INITIAL_ROTATION_ANGLE;
    /**
     * The number of motor ticks per degree of arm rotation. <strong>Includes
     * gearing</strong>, so it might <em>not</em> be the same as 360° /
     * {@link MotorConfigurationType#getTicksPerRev()}.
     */
    private final double TICKS_PER_ROTATION_DEGREE;
    /**
     * The maximum power that the {@link #ROTATION_MOTOR} moves with.
     */
    private final double MAX_ROTATION_POWER;

    /**
     * The motor that folds and retracts the arm.
     */
    private final DcMotor FOLDING_MOTOR;
    /**
     * The minimum extension of the arm in ticks.
     */
    private final int MIN_FOLDING_TICKS;
    /**
     * The maximum extension of the arm in ticks.
     */
    private final int MAX_FOLDING_TICKS;
    /**
     * The angle that the arm folding starts from. 0 ticks will be considered
     * equivalent to this angle.
     */
    private final double INITIAL_FOLDING_ANGLE;
    /**
     * The number of motor ticks per degree of arm folding. <strong>Includes
     * gearing</strong>, so it might <em>not</em> be the same as 360° /
     * {@link MotorConfigurationType#getTicksPerRev()}.
     */
    private final double TICKS_PER_FOLDING_DEGREE;
    /**
     * The maximum motor power that the {@link #FOLDING_MOTOR} moves with.
     */
    private final double MAX_FOLDING_POWER;

    /**
     * Instantiate a foldable arm with a given {@link Builder} containing the
     * necessary parameters.
     *
     * @throws IllegalArgumentException If the {@link Builder} object is invalid
     *                                  as defined by
     *                                  {@link Builder#isValid()}.
     */
    protected FoldingArm(Builder builder) throws IllegalArgumentException {
        super(Set.of(builder.rotationMotor, builder.foldingMotor));

        if (!builder.isValid()) {
            throw new IllegalArgumentException("FoldingArm builder is invalid"
                                               + ".");
        }

        ROTATION_MOTOR = builder.rotationMotor;
        MIN_ROTATION_TICKS = builder.minRotationTicks;
        MAX_ROTATION_TICKS = builder.maxRotationTicks;
        INITIAL_ROTATION_ANGLE = builder.initialRotationAngle;
        TICKS_PER_ROTATION_DEGREE = builder.ticksPerRotationDegree;
        MAX_ROTATION_POWER = builder.maxRotationPower;

        FOLDING_MOTOR = builder.foldingMotor;
        // TODO: You may need to change the direction.
        FOLDING_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        MIN_FOLDING_TICKS = builder.minFoldingTicks;
        MAX_FOLDING_TICKS = builder.maxFoldingTicks;
        INITIAL_FOLDING_ANGLE = builder.initialFoldingAngle;
        TICKS_PER_FOLDING_DEGREE = builder.ticksPerFoldingDegree;
        MAX_FOLDING_POWER = builder.maxFoldingPower;

        // Reset position to 0.
        for (DcMotor motor : MOTORS) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * {@return the {@link DcMotor} that rotates the entire arm, like a shoulder
     * with only one degree of freedom}
     */
    public DcMotor getRotationMotor() {
        return ROTATION_MOTOR;
    }

    /**
     * {@return the maximum power that the rotation motor moves with, which is
     * also a multiplier for the argument passed into {@link #rotate(double)}}
     */
    public double getMaxRotationPower() {
        return MAX_ROTATION_POWER;
    }

    /**
     * {@return the rotation position of the arm in motor ticks}
     */
    public int getRotationTicks() {
        return ROTATION_MOTOR.getCurrentPosition();
    }

    /**
     * {@return the rotation angle of the arm in degrees}
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
            ROTATION_MOTOR.getCurrentPosition() > MAX_ROTATION_TICKS
            || ROTATION_MOTOR.getCurrentPosition() < MIN_ROTATION_TICKS
        ) {
            ROTATION_MOTOR.setPower(0);
            throw new IllegalStateException("Arm rotation reached limits");
        }

        double clampedPower = Math.max(Math.min(power, 1.0), -1.0);

        ROTATION_MOTOR.setPower(clampedPower * MAX_ROTATION_POWER);
    }

    /**
     * Rotates the arm to a position specified in degrees.
     *
     * @param degrees The position the arm moves to. The arm's starting position
     *                is 0 degrees.
     */
    public void rotateToAngle(double degrees) {
        int targetPosition = (int) -Math.round(
            (
                degrees
                - INITIAL_ROTATION_ANGLE
            )
            * TICKS_PER_ROTATION_DEGREE
        );
        // Keep the target position within acceptable bounds
        targetPosition = Math.min(
            Math.max(targetPosition, MIN_ROTATION_TICKS),
            MAX_ROTATION_TICKS
        );
        ROTATION_MOTOR.setTargetPosition(targetPosition);

        /*
         * Calculate the direction that the arm will have to rotate.
         * Negative is down, positive is up.
         */
        int direction = (int) Math.signum(
            targetPosition
            - ROTATION_MOTOR.getCurrentPosition()
        );
        ROTATION_MOTOR.setPower(direction * MAX_ROTATION_POWER);

        ROTATION_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * {@return the {@link DcMotor} that is used to fold the arm}
     */
    public DcMotor getFoldingMotor() {
        return FOLDING_MOTOR;
    }

    /**
     * {@return the maximum power that the folding arm moves with, which is also
     * a multiplier for the argument passed into {@link #fold(double)}}
     */
    public double getMaxFoldingPower() {
        return MAX_FOLDING_POWER;
    }

    /**
     * {@return the current position of the folding motor in ticks}
     */
    public int getFoldingTicks() {
        return FOLDING_MOTOR.getCurrentPosition();
    }

    /**
     * {@return the folding angle of the arm in degrees}
     */
    public double getFoldingDegrees() {
        return FOLDING_MOTOR.getCurrentPosition() / TICKS_PER_FOLDING_DEGREE
               + INITIAL_FOLDING_ANGLE;
    }

    /**
     * Fold the arm with a given power.
     *
     * @param power The power that the extension motor moves wit. Positive
     *              values fold the arm, negative values retract it, and zero
     *              stops the motor.
     */
    public void fold(double power) throws IllegalStateException {
        if (
            FOLDING_MOTOR.getCurrentPosition() > MAX_FOLDING_TICKS
            || FOLDING_MOTOR.getCurrentPosition() < MIN_FOLDING_TICKS
        ) {
            FOLDING_MOTOR.setPower(0);
            throw new IllegalStateException("Arm folding reached limits.");
        }

        double clampedPower = Math.max(Math.min(power, 1.0), -1.0);

        FOLDING_MOTOR.setPower(clampedPower * MAX_FOLDING_POWER);
    }

    /**
     * Fold the arm to a certain number of degrees.
     *
     * @param degrees The position to move the joint of the arm to in degrees.
     */
    public void foldToAngle(double degrees) {
        int targetPosition = (int) Math.round((degrees - INITIAL_FOLDING_ANGLE)
                                              * TICKS_PER_FOLDING_DEGREE);
        // Keep the target position within acceptable bounds.
        targetPosition = -Math.min(
            Math.max(targetPosition, MIN_FOLDING_TICKS),
            MAX_FOLDING_TICKS
        );
        FOLDING_MOTOR.setTargetPosition(targetPosition);

        // Move in the appropriate direction.
        int direction = (int) Math.signum(
            targetPosition
            - FOLDING_MOTOR.getCurrentPosition()
        );
        FOLDING_MOTOR.setPower(direction * MAX_FOLDING_POWER);

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
            Math.max(targetPosition, MIN_FOLDING_TICKS),
            MAX_FOLDING_TICKS
        );
        FOLDING_MOTOR.setTargetPosition(targetPosition);

        // Get the direction of turning.
        int direction = (int) Math.signum(
            targetPosition
            - FOLDING_MOTOR.getCurrentPosition()
        );
        FOLDING_MOTOR.setPower(direction * MAX_FOLDING_POWER);

        FOLDING_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}