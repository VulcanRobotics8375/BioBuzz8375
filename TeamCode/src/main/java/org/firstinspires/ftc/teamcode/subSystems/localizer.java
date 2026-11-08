package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.ArrayList;
import robotcore.Subsystem;
public class localizer extends Subsystem {


    ArrayList<Double> errX = new ArrayList<>();
    ArrayList<Double> errY = new ArrayList<>();
    ArrayList<Double> errH = new ArrayList<>();
    public static final int BATCH = 10;
    kalman xAxis, yAxis, hAxis;
    int rejected = 0;
    public static final double Q_PER_INCH = 0.01;
    public static final double Q_PER_RAD = 0.001;
    public static final double R_XY = 9;
    public static final double R_H = 0.003;
    public static final double startP_XY= 1;
    public static final double startP_H = 0.0012;

    public localizer(double startx, double starty, double starth) {
        xAxis = new kalman(startx, startP_XY, false);
        yAxis = new kalman(starty, startP_XY, false);
        hAxis = new kalman(starth, startP_H, true);
    }

    public void predict(double dx, double dy, double dh) {
        double d = Math.hypot(dx, dy);
        xAxis.predict(dx, Q_PER_INCH * d);
        yAxis.predict(dy, Q_PER_INCH * d);
        hAxis.predict(dh, Math.abs(dh) * Q_PER_RAD);
    }

    public void update(double camX, double camY, double camH) {
        if (!xAxis.accepts(camX, R_XY) || !yAxis.accepts(camY, R_XY) || !hAxis.accepts(camH, R_H)) {
            telemetry.addData("CAMERA", "BEYOND EXPECTED");

            errX.add(xAxis.error(camX));
            errY.add(yAxis.error(camY));
            errH.add(hAxis.error(camH));

            if (errX.size() >= BATCH) { //triggers condition for a potential reset
                telemetry.addData("BATCH", "TRIGGERED");
                if (consistent(errX, R_XY) && consistent(errY, R_XY) && consistent(errH, R_H)){
                    xAxis.x += mean(errX);
                    xAxis.P = startP_XY;
                    yAxis.x += mean (errY);
                    yAxis.P = startP_XY;
                    hAxis.x += mean(errH);
                    hAxis.x = AngleUnit.normalizeRadians(hAxis.x);
                    hAxis.P = startP_H;
                    errX.clear();
                    errY.clear();
                    errH.clear();
                } else {
                    telemetry.addData("INCONSISTENT", "BATCH");
                    errX.clear();
                    errY.clear();
                    errH.clear();
                }
            }
        } else {
            errX.clear();
            errY.clear();
            errH.clear();
            xAxis.update(camX, R_XY);
            yAxis.update(camY, R_XY);
            hAxis.update(camH, R_H);
        }
    }

    private boolean consistent(ArrayList<Double> errors, double R) {
        for (double e: errors){
            if (Math.abs(e-mean(errors))> (3*Math.sqrt(R))) {
                return false;
            }
        }
        return true;
    }
    private double mean (ArrayList<Double> errors) {
        double esum = 0;
        for (double e : errors) {
            esum += e;
        }

        return esum / errors.size();
    }

    public double getX() {
        return xAxis.x;
    }
    public double getY() {
        return yAxis.x;
    }
    public double getH() {
        return hAxis.x;
    }

    @Override
    public void init(OpMode opMode) {

    }
}
