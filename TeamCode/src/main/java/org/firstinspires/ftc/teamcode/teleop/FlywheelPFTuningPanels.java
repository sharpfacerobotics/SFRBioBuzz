package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.PureLauncherTesting;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Trigger;

@TeleOp(name="FlywheelPFTuningPanels", group="OpMode")
@Configurable
public class FlywheelPFTuningPanels extends OpMode {
    public static double intakePower = 0.0;
    private RobotHardware robot = new RobotHardware();
    private Intake intake = new Intake(robot);
    private Trigger trigger =  new Trigger(robot);
    private TelemetryManager panelsTelemetry;

    public static double velocity = 0;

    private PureLauncherTesting launcher = new PureLauncherTesting(robot);

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
        if (gamepad1.a) {
            intakePower = 0.5;
            trigger.runTrigger(0.5);
        } else if (gamepad1.b) {
            intakePower = 1;
            trigger.runTrigger(1.00);
        }
        else {
            intakePower = 0.0;
            trigger.stopFiring();
        }
        intake.setIntakePower(intakePower);
        launcher.setLauncher(velocity);

        double currentVelocity = launcher.getCurrVelocity();
        double error = velocity - currentVelocity;

        panelsTelemetry.addData("Target Velocity", velocity);
        panelsTelemetry.addData("Current Velocity", currentVelocity);
        panelsTelemetry.addData("Error", error);

        panelsTelemetry.update(telemetry);

        telemetry.addData("Status", "Running");

    }

}