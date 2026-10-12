package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Trigger {
    private RobotHardware robot;

    // tune these on the robot (or move them into RobotConfig if the team wants)
    private double triggerPower = 1.0;    // feed power once the shot is good
    private double gateOpenPos = 0.6;     // servo position when gate is open
    private double gateClosedPos = 0.2;   // servo position when gate is closed
    private double gateOpenTime = 0.15;   // seconds for the gate to physically swing open before feeding

    private boolean gateOpen = false;
    private ElapsedTime gateTimer;        // made once here, NOT inside the loop

    public Trigger(RobotHardware robot) {
        this.robot = robot;
        this.gateTimer = new ElapsedTime();
    }

    // call every loop while the driver is holding the fire button
    // goodShooterConfig -> launcher velocity (or voltage) is in range
    // goodTurretConfig  -> turret is aimed
    public void fire(boolean goodShooterConfig, boolean goodTurretConfig) {
        boolean inGoodRange = goodShooterConfig && goodTurretConfig;

        // gate has to open first
        openGate(inGoodRange);

        // only push elements once the gate has had time to fully open
        if (inGoodRange && gateOpen && gateTimer.seconds() >= gateOpenTime) {
            robot.triggerMotor.setPower(triggerPower);
        } else {
            robot.triggerMotor.setPower(0);
        }
    }

    // call when the driver lets go of the fire button (and once in init)
    public void stopFiring() {
        robot.triggerMotor.setPower(0);
        closeGate();
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    // servo powered gate
    private void openGate(boolean inGoodRange) {
        if (inGoodRange) {
            if (!gateOpen) {
                robot.gateServo.setPosition(gateOpenPos);
                gateOpen = true;
                gateTimer.reset(); // start counting how long it's been open
            }
        } else {
            closeGate();
        }
    }

    private void closeGate() {
        robot.gateServo.setPosition(gateClosedPos);
        gateOpen = false;
    }
}
