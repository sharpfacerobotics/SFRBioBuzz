package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDriveFieldRelative;
import org.firstinspires.ftc.teamcode.states.Intake;
import org.firstinspires.ftc.teamcode.states.Shooter;


@TeleOp
public class FinalOpMode extends OpMode {

    private enum RobotState{

        DEFAULT,

        PREPARAR,

        EXPELIR,

        DESLIGA,

    }

    private enum Marchas {
        ALTA,

        MEDIA,

        BAIXA,
    }

    RobotState robotState = RobotState.DEFAULT;
    RobotState previousRobotState = robotState;

    Marchas marchaAtual = Marchas.ALTA;
    MecanumDriveFieldRelative drive = new MecanumDriveFieldRelative();
    Intake intake = null;
    Shooter shooter = null;
    double forward, strafe, rotate;




    @Override
    public void init () {


    drive.init(hardwareMap);
    intake = new Intake(hardwareMap);
   shooter = new Shooter(hardwareMap);

    }
    boolean previousXButtonValue = false;
    boolean previousAButtonValue = false;
    boolean previousYButtonValue = false;

    boolean isPreviousBButtonValue = false;

    boolean previousXButtonValue2 = false;

    boolean previousBButtonValue2 = false;

    boolean isPreviousYButtonValue = false;

    boolean previousRightBumper = false;
    boolean previousLeftBumper = false;



    @Override
    public void loop() {

        boolean buttonX2 = gamepad2.x;
        boolean buttonY2 = gamepad2.y;
        boolean buttonB2 = gamepad2.b;

        switch (marchaAtual){
            case ALTA:
                drive.setMaxSpeed(0.9);
                if (buttonY2 && !isPreviousYButtonValue) {marchaAtual = Marchas.MEDIA;}
                else if (buttonB2 && !previousBButtonValue2){marchaAtual = Marchas.BAIXA;;}
                break;
            case MEDIA:
                drive.setMaxSpeed(0.6);
                if (buttonB2 && !previousBButtonValue2) {marchaAtual = Marchas.BAIXA;}
                else if (buttonX2 && !previousXButtonValue2){marchaAtual = Marchas.ALTA;}
                break;
            case BAIXA:
                drive.setMaxSpeed(0.3);
                if (buttonX2 && !previousXButtonValue2) {marchaAtual = Marchas.ALTA;}
                else if (buttonB2 && !previousBButtonValue2){marchaAtual = Marchas.MEDIA;}
                break;
        }

        boolean buttonX = gamepad1.x;
        boolean buttonY = gamepad1.y;
        boolean buttonA = gamepad1.a;
        boolean buttonB = gamepad1.b;
        boolean triggerRight = gamepad1.right_bumper;
        boolean triggerLeft = gamepad1.left_bumper;



     switch (robotState){
         case DEFAULT:
                intake.starCollectBall();
             if(buttonX && !previousXButtonValue) {previousRobotState = robotState;
                 robotState =robotState.PREPARAR;}
             else if (buttonY && !previousYButtonValue){ previousRobotState = robotState;
                 robotState = robotState.DESLIGA;}
             break;
         case PREPARAR:
             intake.defaultCollect();
             if(buttonY && !previousYButtonValue){ previousRobotState = robotState;
             robotState = robotState.DESLIGA;}
             else if (buttonA && !previousAButtonValue){previousRobotState = robotState;
                 robotState = robotState.DEFAULT;
             }
             break;
         case DESLIGA:
             intake.stopCollectBall();
             if (buttonB && !previousXButtonValue) {previousRobotState = robotState;
                 robotState = robotState.EXPELIR;}
             else if (buttonA && !previousAButtonValue){previousRobotState = robotState;
                 robotState = robotState.DEFAULT;
             }
             break;
         case EXPELIR:
             intake.expelBall();
             if(buttonA && !previousAButtonValue) {previousRobotState = robotState;
                 robotState = robotState.DEFAULT;
         }
            else if(buttonY && !previousYButtonValue){ previousRobotState = robotState;
                 robotState = robotState.DESLIGA;}
             break;
     }

        /*if(buttonB && !isPreviousBButtonValue) {
           robotState = previousRobotState;
        }*/

        if (triggerRight) {
            shooter.moveToCharging();
        }
        else {
            shooter.stop();
        }



    forward = -gamepad2.left_stick_y;
    strafe = gamepad2.left_stick_x*1.1;
    rotate = gamepad2.right_stick_x;

    drive.drive(forward,strafe,rotate);

        telemetry.addData("Marcha atual", marchaAtual);
        telemetry.addData("Estado atual do robo", robotState);
        telemetry.addData("Estado anterior do robo", previousRobotState);
        telemetry.addData("Shooter Left", shooter.getLeftPosition());
        telemetry.addData("Shooter Right", shooter.getRightPosition());
        telemetry.addData("Shooter Avg", shooter.getAveragePosition());
        telemetry.update();

        previousXButtonValue2 = buttonX2;
        previousBButtonValue2 = buttonB2;
        isPreviousYButtonValue = buttonY2;

        previousXButtonValue = buttonX;
        previousAButtonValue = buttonA;
        previousYButtonValue = buttonY;
        isPreviousBButtonValue = buttonB;



    }
}
