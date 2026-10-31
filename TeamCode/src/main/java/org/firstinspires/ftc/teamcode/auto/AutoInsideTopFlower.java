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
public class AutoInsideTopFlower extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.2239, 9.056, 90);
    private final Pose path1 = poseFactory.of(60.1903, 110.2, 180);
    private final Pose point2 = poseFactory.of(47.3843, 110.2, 180);
    private final Pose point3 = poseFactory.of(47.1847, 127.5019, 90.661);
    private final Pose point4 = poseFactory.of(47.3284, 117.5187, 90.8244);
    private final Pose point5 = poseFactory.of(60.2164, 117.6623, -179.3614);
    private final Pose point6 = poseFactory.of(60.056, 8.1772, -90.084);
    private final Pose point7 = poseFactory.of(60.416, 109.6959, -90.2032);
    private final Pose point8 = poseFactory.of(11.4739, 109.8862, -0.2228);

    // ------------------------------------------------------------------------
    // Requirements: any object can be a requirement. Swap each of these for
    // the real motor / servo / subsystem object once your hardware is wired in.
    // Two commands that share a requirement can't run at the same time.
    // ------------------------------------------------------------------------
    private static final Object SHOOTER = new Object();
    private static final Object INTAKE = new Object();
    private static final Object RAMP = new Object();

    // Timing (ms)
    private static final long PRELOAD_SHOT_MS = 500;    // before path 1: wait + shoot
    private static final long PATH_4_WAIT_MS = 1000;    // after path 4: wait + shoot
    private static final long FINAL_SHOT_MS = 3000;     // after path 8: shoot, then stop

    // Autonomous routine
    //
    // The whole sequence is the "deadline" of an outer deadline(), with
    // intakeCommand() as the other member. That means the intake starts
    // immediately, runs through every path, wait, and shot, and is cancelled
    // (and turned off by its setEnd) when the final 3000 ms shot finishes.
    public Command autoRoutine() {
        return deadline(
                sequential(
                        // Before path 1: wait 500 ms, shooting
                        race(shootCommand(), waitMs(PRELOAD_SHOT_MS)),

                        follow(follower, path1()),

                        // Path 2: deploy the flower ramp while driving
                        parallel(
                                follow(follower, path2()),
                                deployFlowerRampCommand()
                        ),

                        follow(follower, path3()),
                        follow(follower, path4()),

                        // After path 4: wait 1000 ms, shooting
                        race(shootCommand(), waitMs(PATH_4_WAIT_MS)),

                        follow(follower, path5()),
                        follow(follower, path6()),
                        follow(follower, path7()),
                        follow(follower, path8()),

                        // After path 8: shoot for 3000 ms, then stop (shootCommand's setEnd turns the shooter off)
                        race(shootCommand(), waitMs(FINAL_SHOT_MS))
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
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }

    public Path path5() {
        return Paths.line(point4, point5).reverseTangent();
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).reverseTangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }

    /* ========================================================================
       SUBSYSTEM COMMANDS (Ivy Command Builder API)

       CONTINUOUS  - runs until something else ends it (race / deadline / until).
                     setStart   -> turn the mechanism on
                     setExecute -> optional per-loop logic (PID, etc.)
                     setDone    -> () -> false  (never finishes on its own)
                     setEnd     -> turn the mechanism off (runs when cancelled)

       ONE-SHOT    - sets a state once and finishes right away (e.g. a servo move).
                     setStart   -> set the position / state
                     setDone    -> () -> true

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

    /** CONTINUOUS: shooter runs until the surrounding race() cancels it. */
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

    /** ONE-SHOT: move the ramp servo to its deployed position. */
    private Command deployFlowerRampCommand() {
        return Command.build()
                .setStart(() -> {
                    // TODO: rampServo.setPosition(RAMP_DEPLOYED);
                })
                .setDone(() -> true)
                .requiring(RAMP);
    }
}