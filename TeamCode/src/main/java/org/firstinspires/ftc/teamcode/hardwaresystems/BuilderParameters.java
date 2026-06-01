package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashSet;
import java.util.Set;

/**
 * Parameters to store the state of {@link HardwareSystemBuilder}.
 */
public interface BuilderParameters {
    Set<DcMotor> motors = new HashSet<>();
    Set<DcMotor> servos = new HashSet<>();

    /**
     * Return whether the current values stored for the parameters are valid,
     * such that the builder can create a meaningful object (e.g., the motors
     * are not {@code null} or the power is negative).
     *
     * @return Whether the current values stored for the parameters are valid,
     * such that the builder can create a meaningful object (e.g., the motors
     * are not {@code null} or the power is negative).
     */
    boolean isValid();
}
