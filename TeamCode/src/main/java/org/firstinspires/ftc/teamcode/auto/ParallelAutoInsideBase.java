/*
package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ElementVision;
import org.firstinspires.ftc.teamcode.robot.HiveVision;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Storage;
import org.firstinspires.ftc.teamcode.robot.Trigger;
import org.firstinspires.ftc.teamcode.robot.Turret;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class ParallelAutoInsideBase extends LinearOpMode {
    private Follower follower;
    private RobotHardware robot = new RobotHardware();
    private Drivetrain drivetrain = new Drivetrain(robot);
    private Intake intake = new Intake(robot);
    private Launcher launcher = new Launcher(robot);
    private Turret turret = new Turret(robot);
    private ElementVision elementVision = new ElementVision(robot);
    private HiveVision hiveVision = new HiveVision(robot);
    private Turret turret = new Turret(robot);
    private Storage storage = new Storage(robot);
    private Trigger trigger = new Trigger(robot);

    private final PoseFactory poseFactory = PoseFactory.degrees();
    //estos son random doodoo points
    protected final Pose start = poseFactory.of(31.5677, 8.5936, 180);
    protected final Pose flowerIntake = poseFactory.of(7.9534, 8.5448, 180);
    protected final Pose park = poseFactory.of(7.9534, 8.5448, 180);
    protected final Pose pre_park = poseFactory.of(31.3937, 81.6325, 180);

    // Autonomous routine incorporating paths, parallel actions, and waits
    public Command autoRoutine() {
        return sequential(
                startShootCommand(),
                // Path 1:  intaking during path, then wait 500 ms

                parallel(
                        follow(follower, path_StartFlower()),
                        intakeCommand(),
                        deployFlowerRampCommand()
                ),
                waitMs(500),

                // Path 2: Intake running during path, 0 ms wait
                parallel(
                        follow(follower, path2()),
                        intakeCommand()
                ),

                // Path 3: Wait 1000 ms while deploying flower ramp
                follow(follower, path3()),
                race(
                        deployFlowerRampCommand(),
                        waitMs(1000)
                ),

                // Path 4: Keep flower ramp down, run intake, 1000 ms wait while shooting everything intook
                follow(follower, path4()),
                race(
                        parallel(
                                keepFlowerRampDownCommand(),
                                intakeCommand(),
                                startShootCommand()
                        ),
                        waitMs(1000)
                ),

                // Path 5: Run intake during path with no wait
                parallel(
                        follow(follower, path5()),
                        intakeCommand()
                ),

                // Path 6: Bring flower ramp back up during a 400 ms wait
                follow(follower, path6()),
                race(
                        restoreFlowerRampCommand(),
                        waitMs(400)
                ),

                // Path 7: Run intake and keep shooter on during path, plus 500 ms wait
                parallel(
                        follow(follower, path7()),
                        parallel(intakeCommand(), shooterOnCommand())
                ),
                waitMs(500),

                // Path 8: Run intake during path and nothing else
                parallel(
                        follow(follower, path8()),
                        intakeCommand()
                )
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path_StartFlower() {
        return line(start, flowerIntake).linear(start, flowerIntake);
    }
    public Path path_FlowerStart() {
        return line(flowerIntake, start).tangent();
    }

    public Path path_PrePark(Pose currPose) {
        return line(currPose, pre_park).linear(currPose, pre_park);
    }

    public Path path_Park() {
        return line(pre_park, park).tangent();
    }


    /* ========================================================================
       SUBSYSTEM COMMAND PLACEHOLDERS
       Replace the telemetry lines with your actual motor/servo controls.
       ========================================================================

    private Command startShootCommand() {
        return Command.build()
                .setStart(() -> pid.setTarget(target))
                .setExecute(() -> armMotor.setPower(pid.calculate(armMotor.getCurrentPosition())))
                .setDone(() -> Math.abs(target - armMotor.getCurrentPosition()) < 10)
                .setEnd(endCondition -> armMotor.setPower(0))
                .requiring(armMotor);
    }
    private Command endShootCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Shooting...");
        });
    }

    private Command intakeCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Running intake...");
        });
    }

    private Command deployFlowerRampCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Deploying flower ramp...");
        });
    }


    private Command retractFlowerRampCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Bringing flower ramp back up...");
        });
    }

} */