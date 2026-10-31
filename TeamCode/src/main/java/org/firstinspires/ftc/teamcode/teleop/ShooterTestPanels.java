package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@TeleOp(name="ShooterTestPanels", group="OpMode")
@Configurable
public class ShooterTestPanels extends OpMode {

    public static double velocity = 0;

    private TelemetryManager panelsTelemetry;
    private ElapsedTime sanityCheck = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        PanelsConfigurables.INSTANCE.refreshClass(ShooterTestPanels.class);
    }

    @Override
    public void start() {
        sanityCheck.reset();
    }

    @Override
    public void loop() {
        panelsTelemetry.addData("Target Velocity", velocity);
        panelsTelemetry.addData("Time", sanityCheck.seconds());

        panelsTelemetry.update();
    }
}