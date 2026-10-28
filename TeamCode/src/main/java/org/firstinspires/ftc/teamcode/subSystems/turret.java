package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import robotcore.Subsystem;

public class turret extends Subsystem {
    public static final double MIN_ANGLE = Math.toRadians(-200);
    public static final double MAX_ANGLE = Math.toRadians(200);
    public boolean reachable = false;
    public double wturret, theta, wbot, vtan, d, targety, targetx, x, y, kS, kV, vx, vy, heading, pError, vError, kP, kD;
    //inputs: x, y, heading, vx, vy, wbot,
    //set: targetx, targety, kP,kD, sync servos
    //outputs: d, vtan, wturret, theta, pError, vError
    public static final double TICKS_PER_TURRET_RAD = 8192 * (96.0 / 34.0) / (2 * Math.PI), DEADBAND = 0.05;
    DcMotorEx encoder;
    CRServo turretServo1, turretServo2;


    @Override
    public void init(OpMode opMode) {
        instantiateSubsystem(opMode);
        encoder = hardwareMap.get(DcMotorEx.class, "tEnc");
        turretServo1 = hardwareMap.get(CRServo.class, "tServo1");
        turretServo2 = hardwareMap.get(CRServo.class, "tServo2");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double turretAngle() {
        return encoder.getCurrentPosition() / TICKS_PER_TURRET_RAD;  // rad, relative to where it was zeroed
    } // turrent angle from encoder

    public void track() {
        double rx = targetx - x;
        double ry = targety - y;
        d = Math.sqrt((Math.pow(rx, 2)) + Math.pow(ry, 2));
        double dSafe = Math.max(d, 10);  // only for the division
        vtan = (rx * vy - ry * vx) / dSafe;
        wturret = -(wbot + (vtan / dSafe)); //output = input
        theta = AngleUnit.normalizeRadians(atan2(ry, rx) - heading); //normalize radinas angles into [-pi, pi]


    }

    public double chooseTarget(double theta, double current) {
        double[] candidates = {theta, theta + 2 * PI, theta - 2 * PI};
        double best = Double.NaN;                          // "none yet"
        for (double c : candidates) {
            if (c < MIN_ANGLE || c > MAX_ANGLE) {
                continue;
            }// outside limits: skip
            if (Double.isNaN(best) || Math.abs(c - current) < Math.abs(best - current)) {
                best = c;                                  // keep the CANDIDATE, not the distance
            }
        }
        reachable = !Double.isNaN(best);   // found at least one valid candidate?
        if (Double.isNaN(best)) {                          // target is behind the stops
            best = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, theta));
        }
        return best;
    }

    public void pidCalc(double current) { //handling the part that matches the actual pos and speed to the intended pos and speed
        pError = chooseTarget(theta, current) - current;
    }

    public double tVelocity() {
        return encoder.getVelocity() / TICKS_PER_TURRET_RAD;
    }

    public double velToPower(double velocity) {
        if (Math.abs(velocity) < DEADBAND) {
            return 0;
        }
        double w = velocity, power;
        power = (kS * Math.signum(w)) + (kV * w); //kS = power to start moving; kV = slope; w = turret speed wanted
        power = Math.max(Math.min(power, 1), -1);
        return power;
    }

    public void tUpdate() {
        double current = turretAngle();
        track();
        pidCalc(current);
        double ff = wturret;
        if (!reachable) ff = 0;                          // parked at a limit
        if (current >= MAX_ANGLE && ff > 0) ff = 0;      // don't push past the top stop
        if (current <= MIN_ANGLE && ff < 0) ff = 0;
        vError = ff - tVelocity(); // ...or the bottom one
        double finalPower;
        finalPower = velToPower(ff) + kP * pError + kD * vError;
        finalPower = Math.max(Math.min(finalPower, 1), -1);
        turretServo1.setPower(finalPower);
        turretServo2.setPower(finalPower);

    }

}
