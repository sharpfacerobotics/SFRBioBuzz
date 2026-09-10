package org.firstinspires.ftc.teamcode.pedro;

public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    //BEFORE RUNNING THE BELOW CODE, CONTINUE FROM Open Autotune at https://pedropathing.com/docs/pathing/tuning/drivetrain/mecanum. 
    //*Ensure that you first build the code and upload it to the robot
    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);
    }
}
