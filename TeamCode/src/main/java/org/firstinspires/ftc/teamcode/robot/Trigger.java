package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Trigger {
    private RobotHardware robot;

    // public static + @Configurable -> tune live from Panels (same as Launcher/Turret)
    public static double triggerPower = 1.0;    // feed power once the shot is good
    public static double gateOpenPos = 0.6;     // servo position when gate is open
    public static double gateClosedPos = 0.2;   // servo position when gate is closed
    public static double gateOpenTime = 0.15;   // seconds for the gate to physically swing open before feeding

    private boolean gateOpen = false;
    private final ElapsedTime gateTimer = new ElapsedTime(); // made once, NOT inside the loop

    public Trigger(RobotHardware robot) {
        this.robot = robot;
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
