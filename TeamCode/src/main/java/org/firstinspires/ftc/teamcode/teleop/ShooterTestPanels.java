package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@TeleOp(name="ShooterTestPanels", group="OpMode")
@Configurable
public class ShooterTestPanels extends OpMode {
    public static double velocity = 0;
    private RobotHardware robot = new RobotHardware();
    private Launcher launcher = new Launcher(robot);


    @Override
    public void init() {
        robot.initLauncher(hardwareMap);
        launcher.setControlModeOverride(Launcher.ControlMode.VELOCITY);
    }

    @Override
    public void loop() {
        launcher.setLauncher(velocity);
        telemetry.addData("Target Velocity", velocity);
        telemetry.addData("Current Velocity", launcher.getCurrVelocity());
        telemetry.update();
    }
}