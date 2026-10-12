package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Alliance;

/** {@link ParallelAutoInsideBase}, Red alliance. Owner: Vivek. */
@Autonomous(name = "ParallelAutoInside Red", group = "Comp", preselectTeleOp = "CompTeleopRed")
public class ParallelAutoInsideRed extends ParallelAutoInsideBase {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
