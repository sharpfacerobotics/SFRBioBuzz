package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.annotations.Configurable;


@Configurable
public class Shooter {

    private static final String SHOOTER_LEFT = "shooterLeft";
    private static final String SHOOTER_RIGHT = "shooterRight";

    private static final int POS_CHARGING = -500;
    private static final int POS_SHOOT = 500;
    private static final int POS_DEFAULT = 0;

    private static double POWER_UP = -1;



    public DcMotorEx leftMotorShoot, rightMotorShoot;
    public Shooter(HardwareMap hardwareMap) {

        leftMotorShoot = hardwareMap.get(DcMotorEx.class, SHOOTER_LEFT);
        rightMotorShoot = hardwareMap.get(DcMotorEx.class, SHOOTER_RIGHT);

        leftMotorShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotorShoot.setDirection(DcMotor.Direction.REVERSE);
        rightMotorShoot.setDirection(DcMotor.Direction.FORWARD);

        leftMotorShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }


    private void moveUsingEncoder(int position){

        leftMotorShoot.setTargetPosition(position);
        rightMotorShoot.setTargetPosition(position);

        leftMotorShoot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorShoot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotorShoot.setPower(POWER_UP);
        rightMotorShoot.setPower(POWER_UP);
    }

    public void moveToCharging() {

    moveUsingEncoder(550);
    }


    public void moveToShoot() {

        moveUsingEncoder(500);

    }

    public void moveToDefault() {

        moveUsingEncoder(500);
    }

    public void stop() {
        leftMotorShoot.setPower(0);
        rightMotorShoot.setPower(0);
    }

    public int getLeftPosition() {
        return leftMotorShoot.getCurrentPosition();
    }


    public int getRightPosition() {
        return rightMotorShoot.getCurrentPosition();
    }

    public int getAveragePosition() {
        return (leftMotorShoot.getCurrentPosition()
                + rightMotorShoot.getCurrentPosition()) / 2;
    }
}
