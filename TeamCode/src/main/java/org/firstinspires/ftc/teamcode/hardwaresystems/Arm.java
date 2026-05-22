package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

/**
 * Abstract class to represent all possible arms that a robot could have and
 * their common characteristics.
 */
public abstract class Arm {
    protected final Set<DcMotor> motorsSet;

    /**
     * Instantiate a new {@link Arm} with a {@link Set} of {@link DcMotor}s.
     *
     * @param motorsSet The {@link DcMotor}s contained within this {@link Arm}.
     */
    public Arm(Set<DcMotor> motorsSet) {
        this.motorsSet = motorsSet;
        // The arm motors will attempt to resist external forces　(e.g.,
        // gravity).
        for (DcMotor motor : this.motorsSet) {
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
        return motorsSet;
    }
}