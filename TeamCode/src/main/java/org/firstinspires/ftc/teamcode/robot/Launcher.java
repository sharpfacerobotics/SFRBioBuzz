package org.firstinspires.ftc.teamcode.robot;
import com.bylazar.configurables.annotations.Configurable;
import java.util.Queue;

@Configurable
public class Launcher {
    public static double kP = 0.00006;
    public static double kF = 0.0006;
    private RobotHardware robot;
    public Launcher(RobotHardware robot) {
        this.robot = robot;
    }
    private double targetVel;
    private double currVel1;
    private double currVel2;
    private double hoodRetractionPos1;
    private double hoodRetractionPos2;
    private double hoodAnglePos1;
    private double hoodAnglePos2;

    /*
    public void setLauncher(Queue<String> q) {
        String next = q.peek();
        double vx = ShooterConfig.elementVelocity[0];
        double vy = ShooterConfig.elementVelocity[1];

        targetVel = Math.hypot(vx, vy);

        currVel1 = robot.launchMotor1.getVelocity();
        currVel2 = robot.launchMotor2.getVelocity();
        setHoodRetraction(next);
        setHoodAngle(next);
        setVelocityNormal(targetVel);
        //setVelocityBang(targetVel);
        //setVelocityPFPow(targetVel);
        //setVelocityPFVol(targetVel);
    }
    */

    public void setLauncher(double targetVel, Queue<String> q) {
        currVel1 = robot.launchMotor1.getVelocity();
        currVel2 = robot.launchMotor2.getVelocity();
        //setHoodRetraction(Queue<String> q);
        //setHoodAngle(Queue<String> q);
        setVelocityNormal(targetVel);
        //setVelocityBang(targetVel);
        //setVelocityPFPow(targetVel);
        //setVelocityPFVol(targetVel);
    }

    private void setVelocityNormal(double targetVel) {
        robot.launchMotor1.setVelocity(targetVel);
        robot.launchMotor2.setVelocity(targetVel);
    }

    private void setVelocityBang() {
        if (currVel1 < targetVel) {
            robot.launchMotor1.setPower(1.0);
        } else if (currVel1 > targetVel) {
            robot.launchMotor1.setPower(0.0);
        }

        if (currVel2 < targetVel) {
            robot.launchMotor2.setPower(1.0);
        } else if (currVel2 > targetVel) {
            robot.launchMotor2.setPower(0.0);
        }
    }
    private void setVelocityPF() {
        double pow1 = kF * targetVel + kP * (targetVel - currVel1);
        double pow2 = kF * targetVel + kP * (targetVel - currVel2);
        pow1 = Math.max(-1.0, Math.min(1.0, pow1));
        pow2 = Math.max(-1.0, Math.min(1.0, pow2));

        robot.launchMotor1.setPower(pow1);
        robot.launchMotor2.setPower(pow2);
    }

    public double getTargetVelocity() {
        return targetVel;
    }
    /*
    public void setHoodAngle(String next) {
        if (next.equals("POLLEN")) {
            hoodAnglePos1 = ShooterConfig.elementVelocity[2] + RobotConfig.hoodAngleServo1Transform;
            hoodAnglePos2 = ShooterConfig.elementVelocity[2] + RobotConfig.hoodAngleServo2Transform;
        } else if (next.equals("NECTAR")) {
            hoodAnglePos1 = ShooterConfig.elementVelocity[3] + RobotConfig.hoodAngleServo1Transform;
            hoodAnglePos2 = ShooterConfig.elementVelocity[3] + RobotConfig.hoodAngleServo2Transform;
        }
        robot.hoodAngleServo1.setPosition(hoodAnglePos1);
        robot.hoodAngleServo2.setPosition(hoodAnglePos2);
    }

    public boolean goodShooterConfig() {
        boolean hoodRetracted = Math.abs((robot.hoodGapServo1.getPosition() + robot.hoodGapServo2.getPosition())/2 - hoodRetractionPos) < RobotConfig.hoodRetractionTolerance;
        boolean flywheelSpunUp = Math.abs(targetVel - (currVel1 + currVel2)/2) < RobotConfig.flywheelVelocityTolerance;
        boolean hoodAngleSet = Math.abs((robot.hoodAngleServo1.getPosition() + robot.hoodAngleServo2.getPosition())/2 - hoodRetractionPos) < RobotConfig.hoodAngleTolerance;
        return hoodRetracted && flywheelSpunUp && hoodAngleSet;
    }

    public void setHoodRetraction(String next) {
        if (next.equals("POLLEN")) {
            hoodRetractionPos1 = RobotConfig.HOOD_POLLEN_GAP_1;
            hoodRetractionPos2 = RobotConfig.HOOD_POLLEN_GAP_2;
        } else if (next.equals("NECTAR")) {
            hoodRetractionPos1 = RobotConfig.HOOD_NECTAR_GAP_1;
            hoodRetractionPos2 = RobotConfig.HOOD_NECTAR_GAP_2;
        }
        robot.hoodGapServo1.setPosition(hoodRetractionPos1);
        robot.hoodGapServo2.setPosition(hoodRetractionPos2);
    }
    */
}