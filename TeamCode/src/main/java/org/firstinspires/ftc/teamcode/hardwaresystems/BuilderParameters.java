package org.firstinspires.ftc.teamcode.hardwaresystems;

/**
 * Parameters to store the state of {@link HardwareSystemBuilder}.
 */
public abstract class BuilderParameters {
    /**
     * Return whether the current values stored for the parameters are valid,
     * such that the builder can create a meaningful object (e.g., the motors
     * are not {@code null} or the power is negative).
     *
     * @return Whether the current values stored for the parameters are valid,
     * such that the builder can create a meaningful object (e.g., the motors
     * are not {@code null} or the power is negative).
     */
    public abstract boolean isValid();
}
