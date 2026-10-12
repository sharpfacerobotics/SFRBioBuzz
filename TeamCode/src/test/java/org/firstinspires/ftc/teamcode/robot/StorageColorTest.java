package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Element;
import org.junit.Test;

public class StorageColorTest {
    @Test
    public void primaryHues() {
        assertEquals(0, Storage.hue(1, 0, 0), 1e-9);
        assertEquals(120, Storage.hue(0, 1, 0), 1e-9);
        assertEquals(240, Storage.hue(0, 0, 1), 1e-9);
        assertEquals(60, Storage.hue(1, 1, 0), 1e-9);
    }

    @Test
    public void greyIsUnknown() {
        assertEquals(-1, Storage.hue(0.5, 0.5, 0.5), 1e-9);
        assertEquals(Element.UNKNOWN, Storage.classifyHue(-1));
    }

    @Test
    public void classifiesConfiguredWindows() {
        double pollen = (RobotConfig.POLLEN_HUE_MIN + RobotConfig.POLLEN_HUE_MAX) / 2;
        double nectar = (RobotConfig.NECTAR_HUE_MIN + RobotConfig.NECTAR_HUE_MAX) / 2;
        assertEquals(Element.POLLEN, Storage.classifyHue(pollen));
        assertEquals(Element.NECTAR, Storage.classifyHue(nectar));
        assertEquals(Element.UNKNOWN, Storage.classifyHue(180));
    }
}
