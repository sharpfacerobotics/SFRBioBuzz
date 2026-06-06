package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

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
    protected static class FoldingArmMotors implements BuilderParameters {
        /**
         * All the motors to be used by this {@link FoldingArm}. Used to set
         * {@link Arm#motors}.
         */
        protected final Set<DcMotor> motors;

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
         * Create a new instance of {@link FoldingArmMotors}. If the parameters
         * are {@code null}, they will not be added to {@link #motors}.
         *
         * @param rotationMotor The motor that rotates the entire arm around.
         * @param foldingMotor  The motor that folds the arm in the middle.
         */
        public FoldingArmMotors(DcMotor rotationMotor, DcMotor foldingMotor) {
            motors = new HashSet<>();

            if (rotationMotor != null) {
                motors.add(rotationMotor);
            }
            if (foldingMotor != null) {
                motors.add(foldingMotor);
            }

            this.rotationMotor = rotationMotor;
            this.foldingMotor = foldingMotor;
        }

        /**
         * Override {@link FoldingArmMotors#FoldingArmMotors(DcMotor, DcMotor)}
         * with two {@code null} {@link DcMotor}s, essentially serving as a
         * blank constructor.
         * <p>
         * Note that in this state, it will fail {@link Builder#build()}.
         */
        public FoldingArmMotors() {
            this(null, null);
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
        public boolean isValid() {
            return rotationMotor != null && foldingMotor != null;
        }

        /**
         * Set the motor that controls folding, i.e., moving the entire arm in a
         * circular manner.
         *
         * @param rotationMotor The new motor to control rotation, i.e., moving
         *                      the entire arm in a circular manner.
         */
        protected void setRotationMotor(DcMotor rotationMotor) {
            motors.remove(this.rotationMotor);

            this.rotationMotor = rotationMotor;
            if (rotationMotor != null) {
                motors.add(rotationMotor);
            }
        }

        /**
         * Set the motor that controls folding, i.e., bending the arm in the
         * middle.
         *
         * @param foldingMotor The new motor to control folding, i.e., bending
         *                     the arm in the middle.
         */
        protected void setFoldingMotor(DcMotor foldingMotor) {
            motors.remove(this.foldingMotor);

            this.foldingMotor = foldingMotor;
            if (foldingMotor != null) {
                motors.add(foldingMotor);
            }
        }
    }

    /**
     * Passed into the
     * {@link FoldingArm#FoldingArm(FoldingArmMotors, RotationParameters,
     * FoldingParameters)} constructor. Contains the minimum rotation, maximum
     * rotation, and ticks per degree.
     */
    protected static class RotationParameters implements BuilderParameters {
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
        /**
         * The maximum power that the {@link #ROTATION_MOTOR} moves with. Used
         * to set {@link #MAX_ROTATION_POWER}.
         */
        private double maxPower;

        /**
         * Instantiate an instance of {@link RotationParameters} using only the
         * minimum number of ticks, maximum number of ticks, and ticks per
         * degree. The initial angle defaults to 0.0 and the power defaults to
         * 1.0.
         * <p>
         * See {@link #RotationParameters(int, int, double, double, double)} for
         * the full parameter version.
         *
         * @param minTicks       The minimum number of ticks that the rotation
         *                       motor can reach.
         * @param maxTicks       The maximum number of ticks that the rotation
         *                       motor can reach.
         * @param ticksPerDegree The number of motor ticks per degree of
         *                       rotation. <strong>Includes gearing</strong>, so
         *                       it might <em>not</em> be the same as 360° /
         *                       {@link
         *                       MotorConfigurationType#getTicksPerRev()}.
         */
        public RotationParameters(
            int minTicks,
            int maxTicks,
            double ticksPerDegree
        ) {
            this(minTicks, maxTicks, 0.0, ticksPerDegree, 1.0);
        }

        /**
         * Instantiate an instance of {@link RotationParameters} with the
         * minimum number of ticks, maximum number of ticks, initial angle,
         * ticks per degree, and maximum power.
         * <p> See
         * {@link #RotationParameters(int, int, double)}  RotationParameters}
         * for the abbreviated version.
         *
         * @param minTicks       The minimum number of ticks that the rotation
         *                       motor can reach.
         * @param maxTicks       The maximum number of ticks that the rotation
         *                       motor can reach.
         * @param initialAngle   The angle that the arm rotation starts from. 0
         *                       ticks will be considered equivalent to this
         *                       angle.
         * @param ticksPerDegree The number of motor ticks per degree of
         *                       rotation. <strong>Includes gearing</strong>, so
         *                       it might <em>not</em> be the same as 360° /
         *                       {@link
         *                       MotorConfigurationType#getTicksPerRev()}.
         * @param maxPower       The maximum power that the
         *                       {@link #ROTATION_MOTOR} moves with.
         */
        public RotationParameters(
            int minTicks,
            int maxTicks,
            double initialAngle,
            double ticksPerDegree,
            double maxPower
        ) {
            this.minTicks = minTicks;
            this.maxTicks = maxTicks;
            degreesMode = false;

            this.initialAngle = initialAngle;
            this.ticksPerDegree = ticksPerDegree;

            this.maxPower = maxPower;
        }


        /**
         * Return whether the current {@link RotationParameters} are valid,
         * i.e., {@link #minTicks} is less than {@link #maxTicks},
         * {@link #ticksPerDegree} is positive, and {@link #maxPower} is between
         * 0.0 and 1.0 (inclusive).
         *
         * @return {@code true} current {@link RotationParameters} are valid,
         * i.e., {@link #minTicks} is less than {@link #maxTicks},
         * {@link #ticksPerDegree} is positive, and {@link #maxPower} is between
         * 0.0 and 1.0 (inclusive).
         * <p>
         * {@code false} otherise.
         */
        @Override
        public boolean isValid() {
            return minTicks < maxTicks
                   && ticksPerDegree > 0.0
                   && 0.0 < maxPower
                   && maxPower < 1.0;
        }
    }

    /**
     * Passed into the
     * {@link FoldingArm#FoldingArm(FoldingArmMotors, RotationParameters,
     * FoldingParameters)} constructor. Contains the minimum folding, maximum
     * folding, and ticks per degree.
     */
    protected static class FoldingParameters implements BuilderParameters {
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
         * considered to be equal to this angle. Used to set
         * {@link #INITIAL_FOLDING_ANGLE}
         */
        private double initialAngle;

        /**
         * The maximum power that the {@link #FOLDING_MOTOR} moves with. Used to
         * set {@link #MAX_FOLDING_POWER}.
         */
        private double maxPower;

        /**
         * Instantiate an instance of {@link FoldingParameters} using only the
         * minimum number of ticks, maximum number of ticks, and ticks per
         * degree. The initial angle defaults to 0.0 and the power defaults to
         * 1.0.
         * <p>
         * See {@link #FoldingParameters(int, int, double, double, double)} for
         * the full parameter version.
         *
         * @param minTicks       The minimum number of ticks that the folding
         *                       motor can reach.
         * @param maxTicks       The maximum number of ticks that the folding
         *                       motor can reach.
         * @param ticksPerDegree The number of motor ticks per degree of
         *                       folding. <strong>Includes gearing</strong>, so
         *                       it might <em>not</em> be the same as 360° /
         *                       {@link
         *                       MotorConfigurationType#getTicksPerRev()}.
         */
        public FoldingParameters(
            int minTicks,
            int maxTicks,
            double ticksPerDegree
        ) {
            this(minTicks, maxTicks, 0, ticksPerDegree, 1);
        }

        /**
         * Instantiate an instance of {@link RotationParameters} with the
         * minimum number of ticks, maximum number of ticks, initial angle,
         * ticks per degree, and maximum power.
         * <p>
         * See {@link #FoldingParameters(int, int, double)} for the abbreviated
         * version.
         *
         * @param minTicks       The minimum number of ticks that the folding
         *                       motor can reach.
         * @param maxTicks       The maximum number of ticks that the folding
         *                       motor can reach.
         * @param initialAngle   The angle that the arm folding starts from. 0
         *                       ticks will be considered equivalent to this
         *                       angle.
         * @param ticksPerDegree The number of motor ticks per degree of
         *                       folding. <strong>Includes gearing</strong>, so
         *                       it might <em>not</em> be the same as 360° /
         *                       {@link
         *                       MotorConfigurationType#getTicksPerRev()}.
         * @param maxPower       The maximum power that the
         *                       {@link #FOLDING_MOTOR} moves with.
         */
        public FoldingParameters(
            int minTicks,
            int maxTicks,
            double initialAngle,
            double ticksPerDegree,
            double maxPower
        ) {
            this.minTicks = minTicks;
            this.maxTicks = maxTicks;
            degreesMode = false;

            this.initialAngle = initialAngle;
            this.ticksPerDegree = ticksPerDegree;

            this.maxPower = maxPower;
        }

        /**
         * Return whether the current {@link FoldingParameters} are valid, i.e.,
         * {@link #minTicks} is less than {@link #maxTicks},
         * {@link #ticksPerDegree} is positive, and {@link #maxPower} is between
         * 0.0 and 1.0 (inclusive).
         *
         * @return {@code true} current {@link FoldingParameters} are valid,
         * i.e., {@link #minTicks} is less than {@link #maxTicks},
         * {@link #ticksPerDegree} is positive, and {@link #maxPower} is between
         * 0.0 and 1.0 (inclusive).
         * <p>
         * {@code false} otherise.
         */
        @Override
        public boolean isValid() {
            return minTicks < maxTicks
                   && ticksPerDegree > 0.0
                   && 0.0 < maxPower
                   && maxPower < 1.0;
        }
    }

    /**
     * A builder for creating instances of {@link FoldingArm}.
     */
    @SuppressWarnings("UnusedReturnValue")
    public static class Builder extends Arm.Builder {
        /**
         * The current configuration of the {@link Builder}'s motors.
         */
        protected final FoldingArmMotors foldingArmMotors;
        /**
         * The current configuration of the {@link Builder}'s
         * {@link RotationParameters}.
         */
        protected final RotationParameters rotationParameters;
        /**
         * The current configuration of the {@link Builder}'s
         * {@link FoldingParameters}.
         *
         */
        protected final FoldingParameters foldingParameters;

        /**
         * Whether the minimum and maximum was set using ticks or degrees. If
         * they were set using degrees, then they must be adjusted if the
         * initial angle or ticks per degree are changed.
         */
        protected boolean degreesMode;

        /**
         * Instantiate a new {@link Builder} with all {@link BuilderParameters}
         * set to their default value.
         */
        public Builder() {
            foldingArmMotors = new FoldingArmMotors();
            rotationParameters = new RotationParameters(0, 0, 0);
            foldingParameters = new FoldingParameters(0, 0, 0);

            degreesMode = false;
        }

        /**
         * Set the motor that controls the {@link Arm}'s rotation.
         *
         * @param rotationMotor The new {@link DcMotor} to control the
         *                      {@link Arm}'s rotation.
         * @return This {@link Builder} so that methods can be chained.
         */
        public Builder setRotationMotor(DcMotor rotationMotor) {
            foldingArmMotors.setRotationMotor(rotationMotor);
            return this;
        }

        /**
         * Set the motor that controls the {@link Arm}'s folding.
         *
         * @param foldingMotor The new motor to set the {@link Arm}'s folding.
         * @return This {@link Builder} so that methods can be chained.
         */
        public Builder setFoldingMotor(DcMotor foldingMotor) {
            foldingArmMotors.setFoldingMotor(foldingMotor);
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
            rotationParameters.minTicks = minTicks;
            rotationParameters.maxTicks = maxTicks;

            rotationParameters.degreesMode = false;
            return this;
        }

        /**
         * Set the minimum and maximum rotation in degrees. Internally, it
         * converts the degrees to ticks using the
         * {@link RotationParameters#initialAngle} and
         * {@link RotationParameters#ticksPerDegree} properties, so everytime
         * those two properties are updated, the ticks will also be
         * automatically updated.
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
            rotationParameters.maxPower = Math.min(Math.max(power, 0.0), 1.0);
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
            foldingParameters.minTicks = minTicks;
            foldingParameters.maxTicks = maxTicks;

            foldingParameters.degreesMode = false;
            return this;
        }

        /**
         * Set the minimum and maximum folding in degrees. Internally, it
         * converts the degrees to ticks using the
         * {@link FoldingParameters#initialAngle} and
         * {@link FoldingParameters#ticksPerDegree} properties, so everytime
         * those two properties are updated, the ticks will also be
         * automatically updated.
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
            foldingParameters.maxPower = Math.min(Math.max(power, 0.0), 1.0);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public FoldingArm build() {
            if (
                !foldingArmMotors.isValid()
                || !rotationParameters.isValid()
                || !foldingParameters.isValid()
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
     * Instantiate a foldable arm.
     *
     * @param foldingArmMotors   The motors and motor types.
     * @param rotationParameters The minimum rotation, maximum rotation, and
     *                           ticks per degree.
     * @param foldingParameters  The minimum folding, maximum folding, and ticks
     *                           per degree.
     */
    protected FoldingArm(
        FoldingArmMotors foldingArmMotors,
        RotationParameters rotationParameters,
        FoldingParameters foldingParameters
    ) {
        super(foldingArmMotors.motors);

        ROTATION_MOTOR = foldingArmMotors.rotationMotor;
        MIN_ROTATION_TICKS = rotationParameters.minTicks;
        MAX_ROTATION_TICKS = rotationParameters.maxTicks;
        INITIAL_ROTATION_ANGLE = rotationParameters.initialAngle;
        TICKS_PER_ROTATION_DEGREE = rotationParameters.ticksPerDegree;
        MAX_ROTATION_POWER = rotationParameters.maxPower;

        FOLDING_MOTOR = foldingArmMotors.foldingMotor;
        // TODO: You may need to change the direction.
        FOLDING_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        MIN_FOLDING_TICKS = foldingParameters.minTicks;
        MAX_FOLDING_TICKS = foldingParameters.maxTicks;
        INITIAL_FOLDING_ANGLE = foldingParameters.initialAngle;
        TICKS_PER_FOLDING_DEGREE = foldingParameters.ticksPerDegree;
        MAX_FOLDING_POWER = foldingParameters.maxPower;

        // Reset position to 0.
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Get the {@link DcMotor} that is used to rotate the arm.
     *
     * @return The {@link DcMotor} that is used to rotate the arm.
     */
    public DcMotor getRotationMotor() {
        return ROTATION_MOTOR;
    }

    /**
     * Get the power multiplier that the arm motor rotates with.
     *
     * @return The power multiplier that the arm motor rotates with.
     */
    public double getRotationPower() {
        return MAX_ROTATION_POWER;
    }

    /**
     * Return the rotation position of the arm in motor ticks.
     *
     * @return The rotation position of the arm in motor ticks.
     */
    public int getRotationTicks() {
        return ROTATION_MOTOR.getCurrentPosition();
    }

    /**
     * Return the rotation position of the arm in degrees.
     *
     * @return The rotation angle of the arm in degrees.
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
        double targetDegrees = degrees - INITIAL_ROTATION_ANGLE;
        int targetPosition = (int) -Math.round(targetDegrees
                                               * TICKS_PER_ROTATION_DEGREE);
        // Keep the target position within acceptable bounds
        targetPosition = Math.min(
            Math.max(targetPosition, MIN_ROTATION_TICKS),
            MAX_ROTATION_TICKS
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
        ROTATION_MOTOR.setPower(direction * MAX_ROTATION_POWER);

        ROTATION_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Get the {@link DcMotor} that is used to fold the arm.
     *
     * @return The {@link DcMotor} that is used to fold the arm.
     */
    public DcMotor getFoldingMotor() {
        return FOLDING_MOTOR;
    }

    /**
     * Get the power multiplier that the arm motor rotates with.
     *
     * @return The power multiplier that the arm motor rotates with.
     */
    public double getFoldingPower() {
        return MAX_FOLDING_POWER;
    }

    /**
     * Get the current position of the folding motor in ticks.
     *
     * @return The current position of the folding motor in ticks.
     */
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
        double targetDegrees = degrees - INITIAL_FOLDING_ANGLE;
        int targetPosition = (int) Math.round(
            targetDegrees * TICKS_PER_FOLDING_DEGREE
        );
        // Keep the target position within acceptable bounds
        targetPosition = -Math.min(
            Math.max(targetPosition, MIN_FOLDING_TICKS),
            MAX_FOLDING_TICKS
        );
        FOLDING_MOTOR.setTargetPosition(targetPosition);

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