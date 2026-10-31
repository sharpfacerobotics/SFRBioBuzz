package org.firstinspires.ftc.teamcode.teleop;

import android.service.autofill.LuhnChecksumValidator;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

import java.util.ArrayDeque;
import java.util.Queue;

@TeleOp(name="ShooterTestPanels", group="OpMode")
@Configurable
public class ShooterTestPanels extends OpMode {

    public static double velocity = 0;
    private RobotHardware robot = new RobotHardware();
    private final Queue<String> testQueue = new ArrayDeque<>();
    public static String selectedElement = "POLLEN";
    private Launcher launcher;
    private Drivetrain drivetrain;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        robot.init(hardwareMap);
        launcher = new Launcher(robot);
        drivetrain = new Drivetrain(robot);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(ShooterTestPanels.class);
    }

    @Override
    public void loop() {
        launcher.setLauncher(velocity);
        panelsTelemetry.addData("Target Velocity", velocity);

        panelsTelemetry.update();
    }
}