package org.firstinspires.ftc.teamcode.subSystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class kalman {
    public double x, P; //x=guess of pos; P = uncertainty of pos
    public boolean isAngle;
    public kalman (double startX, double startP, boolean isAngle) {
        x=startX;
        P=startP;
        this.isAngle = isAngle;
    }

    public void predict(double delta, double Q) {
        x += delta;
        if (isAngle) x = AngleUnit.normalizeRadians(x);
        P += Q;
    }

    public void update(double z, double R) {
        double K = P/(P + R);
        double error = z - x;
        if (isAngle) error = AngleUnit.normalizeRadians(error);
        x = x + K * error;
        if (isAngle) x = AngleUnit.normalizeRadians(x);
        P = (1 - K) * P;
    }

    public boolean accepts(double z, double R) {
        double expected = Math.sqrt( P + R);
        return Math.abs(error(z)) < (3*expected);
        // 1. error = z - x   (wrap it if isAngle — same as in update)
        // 2. expected = Math.sqrt(P + R)
        // 3. return true if |error| is less than 3 * expected
    }

    public double error(double z) { //calcualtes error in case of 10 bad readings
        double e = z-x;
        if (isAngle) e = AngleUnit.normalizeRadians(e);
        return e;
    }
}

