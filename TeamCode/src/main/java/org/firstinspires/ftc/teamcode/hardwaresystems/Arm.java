package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

/**
 * Abstract class to define the methods that robot arms are capable of.
 */
public abstract class Arm {
    protected final Set<DcMotor> MOTORS;

    /**
     * Instantiate a new {@link Arm} with a {@link Set} of {@link DcMotor}s.
     *
     * @param motors The {@link DcMotor}s contained within this {@link Arm}.
     */
    public Arm(Set<DcMotor> motors) {
        MOTORS = motors;
        // The arm motors will attempt to resist external forces　(e.g., gravity).
        for (DcMotor motor : MOTORS) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Get all the {@link DcMotor}s that are included in this arm system.
     *
     * @return A {@link Set} that contains every DcMotor included in this arm system.
     */
    public Set<DcMotor> getMotors() {
        return MOTORS;
    }
}