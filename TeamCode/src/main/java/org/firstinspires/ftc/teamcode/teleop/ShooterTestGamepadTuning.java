package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.helperClasses.ValueTuner;

@TeleOp(name="ShooterTestGamepadTuning", group="OpMode")
public class ShooterTestGamepadTuning extends OpMode {
    public static double velocity = 0;
    private RobotHardware robot = new RobotHardware();
    private Launcher launcher = new Launcher(robot);
    private final double[] stepSizesVel = {1000, 100, 10, 1};
    private ValueTuner tunerVelocity = new ValueTuner(0.0, "Tuning Velocity", "A/B", stepSizesVel);
    @Override
    public void init() {
        robot.init(hardwareMap);
    }


    @Override
    public void loop() {
        tunerVelocity.update(
                gamepad1.aWasPressed(),
                gamepad1.bWasPressed(),
                gamepad1.xWasPressed()
        );
        launcher.setLauncher(tunerVelocity.getValue());

        tunerVelocity.printTelemetry(telemetry);
        telemetry.update();
    }
}
