package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    MecanumDriveFieldRelative driveFieldRelative = new MecanumDriveFieldRelative();
    Intake intake = null;
    Shooter shooter = null;
    double forward, strafe, rotate;





    @Override
    public void init () {


    driveFieldRelative.init(hardwareMap);
    intake = new Intake(hardwareMap);
   shooter = new Shooter(hardwareMap);


    }
    boolean previousXButtonValue = false;
    boolean previousAButtonValue = false;
    boolean previousYButtonValue = false;

    boolean isPreviousBButtonValue = false;

    boolean isPreviousRightStick = false;

    boolean isPreviousLeftStick = false;

    boolean previousBButtonValue2 = false;

    boolean isPreviousYButtonValue = false;

    boolean previousRightBumper = false;
    boolean previousLeftBumper = false;



    @Override
    public void loop() {

        boolean rightStick = gamepad1.right_stick_button;
        boolean leftStick = gamepad1.left_stick_button;

        switch (marchaAtual){
            case ALTA:
                driveFieldRelative.setMaxSpeed(0.9);
                if (leftStick && !isPreviousLeftStick) {marchaAtual = Marchas.MEDIA;}
                break;
            case MEDIA:
                driveFieldRelative.setMaxSpeed(0.6);
                if (leftStick && !isPreviousLeftStick) {marchaAtual = Marchas.BAIXA;}
                else if (rightStick && !isPreviousRightStick){marchaAtual = Marchas.ALTA;}
                break;
            case BAIXA:
                driveFieldRelative.setMaxSpeed(0.3);
                if (rightStick && !isPreviousRightStick) {marchaAtual = Marchas.MEDIA;}
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
                 robotState = RobotState.PREPARAR;}
             else if (buttonY && !previousYButtonValue){ previousRobotState = robotState;
                 robotState = RobotState.DESLIGA;}
             break;
         case PREPARAR:
             intake.defaultCollect();
             if(buttonY && !previousYButtonValue){ previousRobotState = robotState;
             robotState = RobotState.DESLIGA;}
             else if (buttonA && !previousAButtonValue){previousRobotState = robotState;
                 robotState = RobotState.DEFAULT;
             }
             break;
         case DESLIGA:
             intake.stopCollectBall();
             if (buttonB && !previousXButtonValue) {previousRobotState = robotState;
                 robotState = RobotState.EXPELIR;}
             else if (buttonA && !previousAButtonValue){previousRobotState = robotState;
                 robotState = RobotState.DEFAULT;
             }
             break;
         case EXPELIR:
             intake.expelBall();
             if(buttonA && !previousAButtonValue) {previousRobotState = robotState;
                 robotState = RobotState.DEFAULT;
         }
            else if(buttonY && !previousYButtonValue){ previousRobotState = robotState;
                 robotState = RobotState.DESLIGA;}
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



    forward = -gamepad1.left_stick_y;
    strafe = gamepad1.left_stick_x*1.1;
    rotate = gamepad1.right_stick_x;

    driveFieldRelative.driveFieldRelative(forward,strafe,rotate);

        telemetry.addData("Marcha atual", marchaAtual);
        telemetry.addData("Estado atual do robo", robotState);
        telemetry.addData("Estado anterior do robo", previousRobotState);
        telemetry.addData("Yaw (Z)", driveFieldRelative.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", driveFieldRelative.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", driveFieldRelative.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        telemetry.update();

        isPreviousRightStick = rightStick;
        isPreviousLeftStick = leftStick;

        previousXButtonValue = buttonX;
        previousAButtonValue = buttonA;
        previousYButtonValue = buttonY;
        isPreviousBButtonValue = buttonB;



    }
}
