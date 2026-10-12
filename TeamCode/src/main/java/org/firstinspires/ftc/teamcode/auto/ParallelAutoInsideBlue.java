package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Alliance;

/** {@link ParallelAutoInsideBase}, Blue alliance. Owner: Vivek. */
@Autonomous(name = "ParallelAutoInside Blue", group = "Comp", preselectTeleOp = "CompTeleopBlue")
public class ParallelAutoInsideBlue extends ParallelAutoInsideBase {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
