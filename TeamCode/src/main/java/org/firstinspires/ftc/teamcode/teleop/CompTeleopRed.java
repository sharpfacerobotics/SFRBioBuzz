package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Alliance;

/** Competition TeleOp, Red alliance. All logic lives in {@link TeleopBase}. Owner: Arnav. */
@TeleOp(name = "CompTeleopRed", group = "Comp")
public class CompTeleopRed extends TeleopBase {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
