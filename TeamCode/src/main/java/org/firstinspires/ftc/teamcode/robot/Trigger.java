package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Trigger {
    private RobotHardware robot;

    // public static + @Configurable -> tune live from Panels (same as Launcher/Turret)

    public Trigger(RobotHardware robot) {
        this.robot = robot;
    }

    // call every loop while the driver is holding the fire button
    // goodShooterConfig -> launcher velocity (or voltage) is in range
    // goodTurretConfig  -> turret is aimed
    public void runTrigger(double power) {
        robot.triggerMotor.setPower(power);
    }
    public void fire(boolean goodShooterConfig, boolean goodTurretConfig) {
        boolean inGoodRange = goodShooterConfig && goodTurretConfig;

        if (inGoodRange) {
            robot.triggerMotor.setPower(RobotConfig.TRIGGER_FEED_POWER);
        } else {
            robot.triggerMotor.setPower(0);
        }
    }

    // call when the driver lets go of the fire button (and once in init)
    public void stopFiring() {
        robot.triggerMotor.setPower(0);
    }
}
