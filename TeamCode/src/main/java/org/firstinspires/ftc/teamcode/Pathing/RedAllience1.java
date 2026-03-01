package org.firstinspires.ftc.teamcode.Pathing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.states.Intake;
import org.firstinspires.ftc.teamcode.states.Shooter;

@Autonomous(name = "Red 1", group = "Autonomous")
public class RedAllience1 extends LinearOpMode {

    static final double TURN_SPEED = 1;

    static final double STRAFE_SPEED = 0.6;

    Intake intake = null;
    Shooter shooter = null;

    Auto auto = null;

    @Override
    public void runOpMode() {

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        auto = new Auto(hardwareMap,this);

        waitForStart();

        // SECOND CYCLE FOR THE FIRST 3 BALLS

        shooter.startChargingAuto(900);

        auto.turnToHeading(
                TURN_SPEED,
                140,
                () -> {
                    shooter.update();
                }
        );

        while (opModeIsActive() && shooter.isBusy()) {
            shooter.update();
        }

        auto.strafe(
                STRAFE_SPEED,
                85,
                0.7,
                () -> {
                    intake.starCollectBall(); // strafe for take balls in the first cycle
                }
        );

        auto.driveStraight(
                STRAFE_SPEED,
                80,
                140,
                0.6,
                () -> {
                }
        );

        auto.driveStraight(
                STRAFE_SPEED,
                -79,
                140,
                0.6,
                () -> {
                }
        );

        auto.strafe(
                STRAFE_SPEED,
                -120,
                0.8,//STRAFE FOR SHOOT THE SECOND CYCLE
                () -> {

                }
        );

        auto.turnToHeading(
                TURN_SPEED,
                160,
                () -> {
                    shooter.stop();
                    intake.stopCollectBall();
                }
        );

        shooter.moveToCharging();
        sleep(300);
        shooter.stop();


        // THIRD CYCLE FOR MORE 3 BALL

        auto.turnToHeading(
                TURN_SPEED,
                140,
                () -> {
                }
        );

        shooter.startChargingAuto(900);

        auto.strafe(
                STRAFE_SPEED,
                200,
                0.65,
                () -> {
                    intake.starCollectBall();
                    shooter.update();
                }
        );

        auto.driveStraight(
                STRAFE_SPEED,
                85,
                140,
                0.6,
                () -> {
                }
        );

        auto.driveStraight(
                STRAFE_SPEED,
                -80,
                140,
                0.6,
                () -> {
                }
        );

        auto.strafe(
                STRAFE_SPEED,
                -185,
                0.6,
                () -> {

                }
        );

        auto.turnToHeading(
                TURN_SPEED,
                160,
                () -> {
                    shooter.stop();
                    intake.stopCollectBall();
                }
        );

        shooter.moveToCharging();
        sleep(300);
        shooter.stop();


    }
}
