package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*; // sequential, parallel, race, deadline, ...
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class ParallelAutoInsideBase extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 180);
    private final Pose path1 = poseFactory.of(7.8853, 7.3208, 180);
    private final Pose point2Start = poseFactory.of(7.8853, 7.3208, 180);
    private final Pose point2 = poseFactory.of(7.9162, 16.1214, 180);
    private final Pose point3 = poseFactory.of(14.3302, 16.0788, 179.6191);
    private final Pose point4 = poseFactory.of(58.9936, 128.1268, 95.6276);
    private final Pose point4Control1 = poseFactory.of(71.6189, 0, 0);
    private final Pose point5 = poseFactory.of(32.3863, 8.1959, 179.0806);
    private final Pose point5Control1 = poseFactory.of(70.4761, 7.5846, 0);
    private final Pose point6 = poseFactory.of(58.1638, 7.6864, 178.8678);
    private final Pose point7Start = poseFactory.of(58.1638, 7.6864, 180);
    private final Pose point7 = poseFactory.of(58.863, 119.014, 180);
    private final Pose point8 = poseFactory.of(19.4744, 118.566, -179.3483);

    // ------------------------------------------------------------------------
    // Requirements: any object can be a requirement. Swap each of these for
    // the real motor / servo / subsystem object once your hardware is wired in.
    // Two commands that share a requirement can't run at the same time.
    // ------------------------------------------------------------------------
    private static final Object SHOOTER = new Object();
    private static final Object INTAKE = new Object();

    // Timing (ms)
    private static final long PRELOAD_SHOT_MS = 1000;   // before path 1: wait + shoot
    private static final long PATH_4_WAIT_MS = 1000;    // after path 4: wait + shoot

    // Autonomous routine
    //
    // The whole sequence is the "deadline" of an outer deadline(), with
    // intakeCommand() as the other member. That means the intake starts
    // immediately and runs through every path, wait, and shot.
    //
    // The last step, shootCommand(), has no timeout and never finishes, so
    // the shooter (and intake) simply run until the auto period ends.
    public Command autoRoutine() {
        return deadline(
                sequential(
                        // Before path 1: wait 1000 ms, shooting
                        race(shootCommand(), waitMs(PRELOAD_SHOT_MS)),

                        follow(follower, path1()),
                        follow(follower, path2()),
                        follow(follower, path3()),
                        follow(follower, path4()),

                        // After path 4: wait 1000 ms, shooting
                        race(shootCommand(), waitMs(PATH_4_WAIT_MS)),

                        follow(follower, path5()),
                        follow(follower, path6()),
                        follow(follower, path7()),
                        follow(follower, path8()),

                        // After path 8: shoot continuously until the end of auto
                        shootCommand()
                ),
                intakeCommand()
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

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).tangent();
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).tangent();
    }

    public Path path6() {
        return Paths.line(point5, point6).reverseTangent();
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point7, point8).tangent();
    }

    /* ========================================================================
       SUBSYSTEM COMMANDS (Ivy Command Builder API)

       CONTINUOUS  - runs until something else ends it (race / deadline / until).
                     setStart   -> turn the mechanism on
                     setExecute -> optional per-loop logic (PID, etc.)
                     setDone    -> () -> false  (never finishes on its own)
                     setEnd     -> turn the mechanism off (runs when cancelled)

       Replace the TODO comments with your motor/servo code.
       ======================================================================== */

    /** CONTINUOUS: intake runs until the surrounding deadline() cancels it. */
    private Command intakeCommand() {
        return Command.build()
                .setStart(() -> {
                    // TODO: intakeMotor.setPower(1.0);
                })
                .setDone(() -> false)
                .setEnd(endCondition -> {
                    // TODO: intakeMotor.setPower(0);
                })
                .requiring(INTAKE);
    }

    /** CONTINUOUS: shooter runs until a surrounding race() cancels it (or the auto ends). */
    private Command shootCommand() {
        return Command.build()
                .setStart(() -> {
                    // TODO: spin up shooter flywheel(s)
                })
                .setExecute(() -> {
                    // TODO: per-loop shooter logic (velocity PID, feeding, etc.)
                })
                .setDone(() -> false)
                .setEnd(endCondition -> {
                    // TODO: stop shooter flywheel(s)
                })
                .requiring(SHOOTER);
    }
}