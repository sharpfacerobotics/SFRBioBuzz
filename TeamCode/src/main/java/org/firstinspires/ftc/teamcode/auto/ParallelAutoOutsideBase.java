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
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class ParallelAutoOutsideBase extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(44.3234, 8.5504, 180);
    private final Pose path1 = poseFactory.of(8.5963, 8.2688, 180);
    private final Pose point2Start = poseFactory.of(8.5963, 8.2688, 180);
    private final Pose point2 = poseFactory.of(31.8443, 125.1593, 180);
    private final Pose point3Start = poseFactory.of(31.8443, 125.1593, 0);
    private final Pose point3 = poseFactory.of(58.8911, 8.5931, 0);
    private final Pose point3Control1 = poseFactory.of(6.6842, 91.1886, 0);
    private final Pose point3Control2 = poseFactory.of(22.4572, 23.6406, 0);
    private final Pose point4 = poseFactory.of(14.6746, 100.8167, 90);
    private final Pose point4Control1 = poseFactory.of(11.6875, 65.1105, 0);
    private final Pose point4Segment1Start = poseFactory.of(14.6746, 100.8167, 0);
    private final Pose point4Segment1End = poseFactory.of(14.6746, 100.8167, 0);
    private final Pose point4Segment2Start = poseFactory.of(14.6746, 100.8167, 0);
    private final Pose point4Segment2End = poseFactory.of(14.6746, 100.8167, 90);

    // ------------------------------------------------------------------------
    // Requirements: any object can be a requirement. Swap each of these for
    // the real motor / servo / subsystem object once your hardware is wired in.
    // Two commands that share a requirement can't run at the same time.
    // ------------------------------------------------------------------------
    private static final Object SHOOTER = new Object();
    private static final Object INTAKE = new Object();

    // Timing (ms)
    private static final long PRELOAD_SHOT_MS = 500;     // before path 1: wait + shoot
    private static final long PATH_2_WAIT_MS = 1500;     // after path 2: plain wait
    private static final long PRE_PATH_3_SHOT_MS = 500;  // before path 3: wait + shoot
    private static final long FINAL_SHOT_MS = 3000;      // after path 4: shoot, then stop

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
                        follow(follower, path2()),

                        // After path 2: wait 1500 ms
                        waitMs(PATH_2_WAIT_MS),

                        // Before path 3: wait 500 ms, shooting
                        race(shootCommand(), waitMs(PRE_PATH_3_SHOT_MS)),

                        follow(follower, path3()),
                        follow(follower, path4()),

                        // After path 4: shoot for 3000 ms, then stop (shootCommand's setEnd turns the shooter off)
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
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3Control2, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).heading(Interpolator.piecewise().until(0.2988, Interpolator.linear(point4Segment1Start, point4Segment1End)).until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
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
}