package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

/**
 * Abstract class to represent all possible arms that a robot could have and
 * their common characteristics.
 */
@SuppressWarnings("unused")
public abstract class Arm {
    /**
     * A builder to simplify instantiation of {@link Arm} objects since arms are
     * complex, with possibly multiple motors and settings for each motor.
     */
    public static abstract class Builder extends HardwareSystemBuilder {
        /**
         * See parent constructor
         * {@link HardwareSystemBuilder#HardwareSystemBuilder()}.
         */
        public Builder() {
            super();
        }

        /**
         * Trivially return {@code true} because {@link Arm} has no fields to
         * check.
         * <p>
         * For subclasses, they should check that their field values make
         * logical sense (e.g., power should be positive, motors should be
         * non-null, etc.).
         *
         * @return {@code true} because {@link Arm} has no fields to check.
         */
        @Override
        public boolean isValid() {
            return true;
        }

        /**
         * Instantiate an {@link Arm} object using the given fields. If the
         * fields are not valid (as defined by {@link #isValid()}), fail and
         * return {@code null}.
         *
         * @return If the fields are valid (as defined by {@link #isValid()}), a
         * new {@link Arm} instance based on the values set in instances of
         * fields.
         * <p>
         * If the fields are invalid (as defined by {@link #isValid()}), fail
         * and return {@code null}.
         */
        @Override
        public abstract Arm build();
    }

    /**
     * A {@link Set} containing all the {@link DcMotor}s that are used by this
     * wheel system.
     */
    protected final Set<DcMotor> motors;

    /**
     * Instantiate a new {@link Arm} with a {@link Set} of {@link DcMotor}s.
     *
     * @param motors The {@link DcMotor}s contained within this {@link Arm}.
     */
    public Arm(Set<DcMotor> motors) {
        this.motors = motors;
        // The arm motors will attempt to resist external forces　(e.g.,
        // gravity).
        for (DcMotor motor : this.motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Get all the {@link DcMotor}s that are included in this arm system.
     *
     * @return A {@link Set} that contains every {@link DcMotor} included in
     * this arm system.
     */
    public Set<DcMotor> getMotors() {
        return motors;
    }
}