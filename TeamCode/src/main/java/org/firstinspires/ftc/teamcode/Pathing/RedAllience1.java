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

        /* Primeiro Ciclo de shooting
        shooter.moveToShoot();
        sleep(1000);

        //Segundo ciclo de shooting
        shooter.stop();
        auto.driveStraight(0.6, 70, 0, 0.6);
       shooter.moveToCharging();
        sleep(1000);
        shooter.moveToShoot();
        sleep(1000);
        shooter.stop();
        auto.turnToHeading(TURN_SPEED, -42, 1);
        auto.strafe(STRAFE_SPEED, -70, 0.6);
        intake.starCollectBall();
        shooter.moveToCharging();
        auto.driveStraight(0.25, -70, 0, 0.25);
        shooter.stop();
        sleep(800);
        auto.driveStraight(0.2, 30, 0, 0.2);
        auto.strafe(0.6, 110, 1);
        auto.turnToHeading(TURN_SPEED, -1, 0.8);
        auto.driveStraight(1, -30, 0, 0.8);
        intake.stopCollectBall();
        shooter.moveToCharging();
        sleep(1000);
        shooter.moveToShoot();
        sleep(1000);
        shooter.stop();

        //Terceiro ciclo de shooting

        auto.driveStraight(0.6, 60, 0, 0.6);
        auto.turnToHeading(TURN_SPEED, -43, 1);
        auto.strafe(STRAFE_SPEED, -162, 1);
        intake.starCollectBall();
       shooter.moveToCharging();
        auto.driveStraight(0.2, -75, 0, 0.2);
        shooter.stop();
        sleep(800);
        intake.stopCollectBall();
        auto.driveStraight(0.2, 10, 0, 0.2);
        auto.strafe(1, 210, 1);
        auto.turnToHeading(TURN_SPEED, -3, 1);
        auto.driveStraight(1, -51, 0, 0.8);
        shooter.moveToCharging();
        sleep(1000);
        shooter.moveToShoot();
        sleep(1000);
        shooter.stop();



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.*/


    }
}
