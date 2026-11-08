package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subSystems.kalman;
import org.junit.Test;
import static org.junit.Assert.*;

public class KalmanTest {

    @Test
    public void testPredict() {
        kalman k = new kalman(0.0, 1, false); // start at x=0, P=1, not an angle
        k.predict(5.0, 0.5);
        assertEquals(5.0, k.x, 1e-9);
        assertEquals(1.5, k.P, 1e-9);

        // predict a delta, check x moved and P grew
    }

    @Test
    public void testUpdate_pullsTowardMeasurement() {
        kalman k = new kalman (0.0, 1, false);
        k.update(10, 1);
        assertEquals(5.0, k.x, 1e-9);

        // start at x=0, give a measurement of 10
        // after update, x should be between 0 and 10
    }

    @Test
    public void testAccepts_withinBounds() {
        kalman k = new kalman (0.0, 1.0, false);
        assertTrue(k.accepts(0.1,9.0));
    }

    @Test
    public void testAccepts_tooFar() {
        kalman k = new kalman (0.0, 1.0, false);
        assertFalse(k.accepts(10,9.0));
        // a measurement way outside 3*sqrt(P+R) should return false
    }

    @Test
    public void testAngleWrap_predict() {
        kalman k = new kalman (3.1, 1.0, true);
        k.predict(0.1, 0.5);
        assertTrue(k.x<0);
        // start near PI, predict a small positive delta that pushes past PI
        // result should wrap to negative (near -PI)
    }

}