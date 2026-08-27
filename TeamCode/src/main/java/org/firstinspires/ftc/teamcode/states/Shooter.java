package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
public class Shooter {

    private static final String SHOOTER_LEFT = "shooterLeft";
    private static final String SHOOTER_RIGHT = "shooterRight";

    private final ElapsedTime timer = new ElapsedTime();
    private boolean chargingAuto = false;
    private long chargingDuration = 0;



    public DcMotorEx leftMotorShoot, rightMotorShoot;
    public Shooter(HardwareMap hardwareMap) {

        leftMotorShoot = hardwareMap.get(DcMotorEx.class, SHOOTER_LEFT);
        rightMotorShoot = hardwareMap.get(DcMotorEx.class, SHOOTER_RIGHT);

        leftMotorShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotorShoot.setDirection(DcMotor.Direction.REVERSE);
        rightMotorShoot.setDirection(DcMotor.Direction.FORWARD);


    }


    public void moveToCharging() {
        leftMotorShoot.setPower(-1);
        rightMotorShoot.setPower(-1);

    }

    public void startChargingAuto(long durationMs) {

        leftMotorShoot.setPower(-1);
        rightMotorShoot.setPower(-1);

        chargingDuration = durationMs;
        timer.reset();
        chargingAuto = true;
    }
    public void update() {

        if (chargingAuto && timer.milliseconds() >= chargingDuration) {
            leftMotorShoot.setPower(0);
            rightMotorShoot.setPower(0);
            chargingAuto = false;
        }
    }
    public boolean isBusy() {
        return chargingAuto;
    }


    public void moveToShoot() {

        leftMotorShoot.setPower(0);
        rightMotorShoot.setPower(0);


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
