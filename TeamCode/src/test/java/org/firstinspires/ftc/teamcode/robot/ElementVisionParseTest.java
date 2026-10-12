package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Element;
import org.junit.Test;

public class ElementVisionParseTest {
    @Test
    public void detectorClassNames() {
        assertEquals(Element.POLLEN, ElementVision.parse("Pollen"));
        assertEquals(Element.RED_NECTAR, ElementVision.parse("red_nectar"));
        assertEquals(Element.BLUE_NECTAR, ElementVision.parse("NECTAR-Blue"));
        assertEquals(Element.UNKNOWN, ElementVision.parse("nectar"));
        assertEquals(Element.UNKNOWN, ElementVision.parse(null));
    }

    @Test
    public void opponentNectarIsTheOtherColour() {
        assertEquals(Element.BLUE_NECTAR, RobotConfig.opponentNectar(RobotConfig.Alliance.RED));
        assertEquals(Element.RED_NECTAR, RobotConfig.ownNectar(RobotConfig.Alliance.RED));
    }
}
