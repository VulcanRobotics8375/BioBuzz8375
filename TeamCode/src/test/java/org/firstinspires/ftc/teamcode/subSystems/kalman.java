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
        P += Q;
    }

    public void update(double z, double R) {
        double K = P/(P + R);
        double error = z - x;
        if (isAngle) error = AngleUnit.normalizeRadians(error);
        if (isAngle) x = AngleUnit.normalizeRadians(x);
        x = x + K * error;
        P = (1 - K) * P;
    }
}
