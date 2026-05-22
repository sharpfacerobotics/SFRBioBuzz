package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

/**
 * Abstract class to represent all possible arms that a robot could have and
 * their common characteristics.
 */
public abstract class Arm {
    protected final Set<DcMotor> motorSet;

    /**
     * Instantiate a new {@link Arm} with a {@link Set} of {@link DcMotor}s.
     *
     * @param motorSet The {@link DcMotor}s contained within this {@link Arm}.
     */
    public Arm(Set<DcMotor> motorSet) {
        this.motorSet = motorSet;
        // The arm motors will attempt to resist external forces　(e.g.,
        // gravity).
        for (DcMotor motor : this.motorSet) {
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
        return motorSet;
    }
}