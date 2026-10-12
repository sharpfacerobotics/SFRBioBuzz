package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Alliance;

/** Competition TeleOp, Blue alliance. All logic lives in {@link TeleopBase}. Owner: Arnav. */
@TeleOp(name = "CompTeleopBlue", group = "Comp")
public class CompTeleopBlue extends TeleopBase {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
