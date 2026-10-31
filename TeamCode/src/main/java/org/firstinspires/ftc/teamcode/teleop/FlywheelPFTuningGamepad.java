package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Launcher;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.helperClasses.ValueTuner;


@TeleOp(name="FlywheelPFTuningGamepad", group="OpMode")
public class FlywheelPFTuningGamepad extends OpMode {
    public static double velocity = 0;
    private RobotHardware robot = new RobotHardware();
    private Launcher launcher = new Launcher(robot);
    private final double[] stepSizesVel = {1000, 100, 10, 1};
    private ValueTuner tunerVelocity = new ValueTuner(0.0, "Tuning Velocity", "X/Y/LeftBumper", stepSizesVel);
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        tunerVelocity.update(
                gamepad1.xWasPressed(),
                gamepad1.yWasPressed(),
                gamepad1.leftBumperWasPressed()
        );
        launcher.setLauncher(tunerVelocity.getValue());
        velocity = tunerVelocity.getValue();
        tunerVelocity.printTelemetry(telemetry);
        telemetry.addData("Target Velocity", velocity);
        telemetry.addData(
                "Current Velocity",
                launcher.getCurrVelocity()
        );
        telemetry.addData("Error:", velocity - launcher.getCurrVelocity());
        telemetry.update();

    }
}
