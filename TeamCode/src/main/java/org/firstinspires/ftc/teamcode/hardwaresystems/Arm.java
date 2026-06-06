package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

/**
 * Abstract class to represent all possible arms that a robot could have and
 * their common characteristics.
 */
public abstract class Arm {
    /**
     * A builder to simplify instantiation of {@link Arm} objects since arms are
     * complex, with possibly multiple motors and settings for each motor.
     */
    public static abstract class Builder implements HardwareSystemBuilder {
        /**
         * Instantiate an {@link Arm} object using the given
         * {@link BuilderParameters}. If the {@link BuilderParameters} are not
         * valid (as defined by {@link BuilderParameters#isValid()}), fail and
         * return {@code null}.
         *
         * @return If the {@link BuilderParameters} are valid (as defined by
         * {@link BuilderParameters#isValid()}), a new {@link Arm} instance
         * based on the values set in instances of {@link BuilderParameters}.
         * <p>
         * If the {@link BuilderParameters} are invalid (as defined by
         * {@link BuilderParameters#isValid()}), fail and return {@code null}.
         */
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