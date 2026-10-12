package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Alliance;

/** {@link SubordinateAutoBase}, Blue alliance. Owner: Alex. */
@Autonomous(name = "SubordinateAuto Blue", group = "Comp", preselectTeleOp = "CompTeleopBlue")
public class SubordinateAutoBlue extends SubordinateAutoBase {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
