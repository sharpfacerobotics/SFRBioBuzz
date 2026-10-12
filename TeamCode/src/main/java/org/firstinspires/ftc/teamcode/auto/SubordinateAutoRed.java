package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Alliance;

/** {@link SubordinateAutoBase}, Red alliance. Owner: Alex. */
@Autonomous(name = "SubordinateAuto Red", group = "Comp", preselectTeleOp = "CompTeleopRed")
public class SubordinateAutoRed extends SubordinateAutoBase {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
