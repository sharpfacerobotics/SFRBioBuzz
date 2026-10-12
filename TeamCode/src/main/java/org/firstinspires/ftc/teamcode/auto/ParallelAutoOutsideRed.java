package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Alliance;

/** {@link ParallelAutoOutsideBase}, Red alliance. Owner: Vivek. */
@Autonomous(name = "ParallelAutoOutside Red", group = "Comp", preselectTeleOp = "CompTeleopRed")
public class ParallelAutoOutsideRed extends ParallelAutoOutsideBase {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
