package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@TeleOp(name="FlywheelPFTuningPanels", group="OpMode")
@Configurable
public class FlywheelPFTuningPanels extends OpMode {

    private TelemetryManager panelsTelemetry;

    public static double velocity = 0;

    private RobotHardware robot = new RobotHardware();
    private Launcher launcher = new Launcher(robot);

    @Override
    public void init() {
        robot.init(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        PanelsConfigurables.INSTANCE.refreshClass(
                FlywheelPFTuningPanels.class
        );
    }

    @Override
    public void loop() {
        launcher.setLauncher(velocity);

        double currentVelocity = launcher.getCurrVelocity();
        double error = velocity - currentVelocity;

        panelsTelemetry.addData("Target Velocity", velocity);
        panelsTelemetry.addData("Current Velocity", currentVelocity);
        panelsTelemetry.addData("Error", error);

        panelsTelemetry.update(telemetry);
    }
}