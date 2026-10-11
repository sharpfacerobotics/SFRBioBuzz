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
    private final double[] stepSizes = {0.0001, 0.001, 0.01, 0.1};

    private ValueTuner tunerP = new ValueTuner(0.0, "Tuning P", "D-Pad R/L/A", stepSizes);
    private ValueTuner tunerF = new ValueTuner(0.0, "Tuning F", "D-Pad U/D/B", stepSizes);
    private final double[] stepSizesVel = {1000, 100, 10, 1};
    private ValueTuner tunerVelocity = new ValueTuner(0.0, "Tuning Velocity", "X/Y/LeftBumper", stepSizesVel);
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        // 1. Update P Tuner (Right increases, Left decreases, B cycles step size)
        tunerP.update(
                gamepad1.dpadRightWasPressed(),
                gamepad1.dpadLeftWasPressed(),
                gamepad1.aWasPressed()
        );

        // 2. Update F Tuner (Up increases, Down decreases, B cycles step size)
        tunerF.update(
                gamepad1.dpadUpWasPressed(),
                gamepad1.dpadDownWasPressed(),
                gamepad1.bWasPressed()
        );
        tunerVelocity.update(
                gamepad1.xWasPressed(),
                gamepad1.yWasPressed(),
                gamepad1.leftBumperWasPressed()
        );


        // 3. Push tuned values to your subsystem hardware
        launcher.setkP(tunerP.getValue());
        launcher.setkF(tunerF.getValue());

        // remember to switch to PF mode before tuning
        launcher.setLauncher(tunerVelocity.getValue());
        // 4. Print formatted telemetry for each tuner instance
        tunerP.printTelemetry(telemetry);
        tunerF.printTelemetry(telemetry);
        tunerVelocity.printTelemetry(telemetry);
        telemetry.update();

    }
}