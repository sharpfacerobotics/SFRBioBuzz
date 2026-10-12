package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Intake roller. Owner: Vibhav.
 *
 * <p>Call {@link #update} once per loop. It stops on its own when storage is full and
 * runs backwards for a moment if it detects a jam (motor current stays high).</p>
 */
public class Intake {
    private final RobotHardware robot;

    private enum State { STOPPED, INTAKING, OUTTAKING, UNJAMMING }
    private State state = State.STOPPED;

    // Made once here, never inside the loop.
    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime unjamTimer = new ElapsedTime();
    private final ElapsedTime currentPollTimer = new ElapsedTime();
    private boolean overCurrent = false;

    public Intake(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * @param wantIntake  driver (or auto) wants to pick elements up
     * @param wantOuttake driver wants to spit elements out (wins over intake)
     * @param storageFull from Storage.isFull() - stops intaking so we don't over-possess
     */
    public void update(boolean wantIntake, boolean wantOuttake, boolean storageFull) {
        if (state == State.UNJAMMING) {
            if (unjamTimer.seconds() < RobotConfig.INTAKE_UNJAM_TIME) {
                robot.intakeMotor.setPower(RobotConfig.OUTTAKE_POWER);
                return;
            }
            jamTimer.reset();
        }

        if (wantOuttake) {
            state = State.OUTTAKING;
            robot.intakeMotor.setPower(RobotConfig.OUTTAKE_POWER);
        } else if (wantIntake && !storageFull) {
            state = State.INTAKING;
            robot.intakeMotor.setPower(RobotConfig.INTAKE_POWER);
            checkForJam();
        } else {
            stop();
        }
    }

    public void runIntake() {
        update(true, false, false);
    }

    public void reverse() {
        update(false, true, false);
    }

    public void stop() {
        state = State.STOPPED;
        robot.intakeMotor.setPower(0);
        jamTimer.reset();
    }

    public boolean isUnjamming() {
        return state == State.UNJAMMING;
    }

    public String getState() {
        return state.name();
    }

    private void checkForJam() {
        if (!RobotConfig.INTAKE_JAM_DETECTION) {
            return;
        }
        // getCurrent() isn't part of the bulk read, so don't call it every loop.
        if (currentPollTimer.seconds() >= RobotConfig.INTAKE_CURRENT_POLL) {
            currentPollTimer.reset();
            overCurrent = robot.intakeMotor.getCurrent(CurrentUnit.AMPS) > RobotConfig.INTAKE_JAM_CURRENT;
        }
        if (!overCurrent) {
            jamTimer.reset();
        } else if (jamTimer.seconds() > RobotConfig.INTAKE_JAM_TIME) {
            state = State.UNJAMMING;
            overCurrent = false;
            unjamTimer.reset();
            robot.intakeMotor.setPower(RobotConfig.OUTTAKE_POWER);
        }
    }
}
