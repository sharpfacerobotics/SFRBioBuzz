package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;

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
        assertEquals(null, Storage.nectarColorFromHue(-1));
    }

    @Test
    public void nectarColours() {
        assertEquals("BLUE", Storage.nectarColorFromHue(Storage.hue(0.1, 0.3, 1))); // blue
        assertEquals("RED", Storage.nectarColorFromHue(Storage.hue(1, 0.1, 0.15)));  // red, hue ~355
        assertEquals("RED", Storage.nectarColorFromHue(5));                          // red wraps past 0
        assertEquals(null, Storage.nectarColorFromHue(120));                         // green
        assertEquals(null, Storage.nectarColorFromHue(Storage.hue(1, 0.85, 0.1)));  // yellow pollen isn't nectar
    }
}
