package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Feeds elements from storage into the flywheel, behind a servo gate. Owner: Ryan.
 *
 * <p>The gate opens the first time both the launcher and turret say they're ready, and then
 * STAYS open until the driver lets go of fire. The feeder only pushes while everything is
 * ready, so after each shot it pauses for the flywheel to recover. (Closing the gate on every
 * post-shot flywheel dip would slam it onto the next element mid-feed.)</p>
 */
public class Trigger {
    private final RobotHardware robot;

    private boolean gateOpen = false;
    private final ElapsedTime gateTimer = new ElapsedTime(); // made once here, NOT inside the loop

    public Trigger(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Call every loop while the driver is holding the fire button.
     *
     * @param goodShooterConfig launcher velocity + hood are in range (Launcher.isReady())
     * @param goodTurretConfig  turret is aimed (Turret.isOnTarget())
     */
    public void fire(boolean goodShooterConfig, boolean goodTurretConfig) {
        boolean inGoodRange = goodShooterConfig && goodTurretConfig;

        // gate has to open first; once open it stays open for this burst
        if (inGoodRange) {
            openGate();
        }

        // only push elements once the gate has had time to fully open
        if (inGoodRange && gateOpen && gateTimer.seconds() >= RobotConfig.GATE_OPEN_TIME) {
            robot.triggerMotor.setPower(RobotConfig.TRIGGER_POWER);
        } else {
            robot.triggerMotor.setPower(0);
        }
    }

    /** Call when the driver lets go of the fire button (and once in init). */
    public void stopFiring() {
        robot.triggerMotor.setPower(0);
        closeGate();
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    // servo powered gate
    private void openGate() {
        if (!gateOpen) {
            robot.gateServo.setPosition(RobotConfig.GATE_OPEN_POS);
            gateOpen = true;
            gateTimer.reset(); // start counting how long it's been open
        }
    }

    private void closeGate() {
        robot.gateServo.setPosition(RobotConfig.GATE_CLOSED_POS);
        gateOpen = false;
    }
}
