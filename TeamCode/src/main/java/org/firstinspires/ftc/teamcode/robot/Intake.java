package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private RobotHardware robot;
    /* Change for BIOBUZZ
    private ElapsedTime jitterTimer = new ElapsedTime();

    // Jitter timing
    private static final double JITTER_REVERSE_TIME = 0.05;
    private static final double JITTER_FORWARD_TIME = 0.10;

    private enum JitterState { IDLE, REVERSE, FORWARD }
    private JitterState jitterState = JitterState.IDLE;
    */

    public Intake(RobotHardware robot) {
        this.robot = robot;
    }

    public void update(boolean runFull, boolean runSlow, boolean runReverse,
                       boolean jitter, boolean tripleShotActive) { // change arguments for biobuzz
        
        // Don't interfere with transfer during triple shot
        // CHANGE FOR BIOBUZZ
        //if (tripleShotActive) return;
        /*
        // --- Jitter state machine ---
        switch (jitterState) {
            case IDLE:
                if (jitter) {
                    jitterState = JitterState.REVERSE;
                    jitterTimer.reset();
                }
                break;
            case REVERSE:
                if (jitterTimer.seconds() >= JITTER_REVERSE_TIME) {
                    jitterState = JitterState.FORWARD;
                    jitterTimer.reset();
                }
                break;
            case FORWARD:
                if (jitterTimer.seconds() >= JITTER_FORWARD_TIME) {
                    jitterState = JitterState.IDLE;
                }
                break;
        }
        
        // --- Motor power based on priority ---
        
        if (jitterState == JitterState.REVERSE) {
            robot.intakeMotor.setPower(-1.0);
        } else if (jitterState == JitterState.FORWARD) {
            robot.intakeMotor.setPower(1.0);
        } else 
          */
        if (runFull) {
            robot.intakeMotor.setPower(1.0);
        } else if (runReverse) {
            robot.intakeMotor.setPower(-1.0);
        } else if (runSlow) {
            robot.intakeMotor.setPower(0.5);
        } else {
            robot.intakeMotor.setPower(0.0);
        }
      
    }
    /* CHANGE FOR BIOBUZZ
    public void setTransfer(boolean forward, boolean reverse) {
        if (forward) {
            robot.transferMotor.setPower(1.0);
        } else if (reverse) {
            robot.transferMotor.setPower(-1.0);
        } else {
            robot.transferMotor.setPower(0.0);
        }
    }
  */
}
