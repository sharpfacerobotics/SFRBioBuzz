package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DrivetrainAimTest {
    @Test
    public void alreadyAimedMeansNoTurn() {
        assertEquals(0, Drivetrain.aimTurn(1.0, 1.0), 1e-12);
    }

    @Test
    public void turnsTheShortWayAcrossPi() {
        // target just past +180 (i.e. -179), robot at +179: should turn CCW (+) by 2 deg, not CW 358
        double turn = Drivetrain.aimTurn(Math.toRadians(-179), Math.toRadians(179));
        assertTrue(turn > 0);
        assertEquals(RobotConfig.AIM_ASSIST_KP * Math.toRadians(2), turn, 1e-9);
    }

    @Test
    public void capsTurnPower() {
        assertEquals(RobotConfig.AIM_ASSIST_MAX_TURN, Drivetrain.aimTurn(Math.PI / 2, 0), 1e-12);
        assertEquals(-RobotConfig.AIM_ASSIST_MAX_TURN, Drivetrain.aimTurn(-Math.PI / 2, 0), 1e-12);
    }

    @Test
    public void elementOnTheRightMeansTurnRight() {
        assertTrue(Drivetrain.bearingTurn(10) < 0);   // CW
        assertTrue(Drivetrain.bearingTurn(-10) > 0);  // CCW
        assertEquals(0, Drivetrain.bearingTurn(0), 1e-12);
    }
}
