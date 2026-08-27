package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {


    private static final String INTAKE_NAME = "Intake";

    private static double INTAKE_COLLECT = 0.9;

    private static double INTAKE_STOP = 0;
    
    private static double INTAKE_EXPEL = -0.8;

    private static double INTAKE_DEFAULT = 0.4;

    public  DcMotor IntakeMotor;

    public Intake (HardwareMap hardwareMap){IntakeMotor = hardwareMap.get(DcMotor.class, INTAKE_NAME);}

    public  void starCollectBall () {
        IntakeMotor.setPower(INTAKE_COLLECT);
    }

    public void stopCollectBall (){
        IntakeMotor.setPower(INTAKE_STOP);
    }

    public void expelBall (){
        IntakeMotor.setPower(INTAKE_EXPEL);
    }

    public void defaultCollect(){IntakeMotor.setPower(INTAKE_DEFAULT);}


}
