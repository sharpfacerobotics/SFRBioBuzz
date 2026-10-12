package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Alliance;

/** {@link ParallelAutoOutsideBase}, Blue alliance. Owner: Vivek. */
@Autonomous(name = "ParallelAutoOutside Blue", group = "Comp", preselectTeleOp = "CompTeleopBlue")
public class ParallelAutoOutsideBlue extends ParallelAutoOutsideBase {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
