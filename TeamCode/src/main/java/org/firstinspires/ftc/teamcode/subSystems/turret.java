package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import robotcore.Subsystem;

@Configurable
public class turret extends Subsystem {
    public static final double MIN_ANGLE = Math.toRadians(-200);
    public static final double MAX_ANGLE = Math.toRadians(200);
    public boolean reachable = false;
    public double wturret, theta, wbot, vtan, d, targety=-18.5805750382, targetx=12.75, x, y, kS=0.1083424, kV=0.09881655, vx, vy, heading, pError, vError, virX, virY;
    public static double kP, kD;
    //inputs: x, y, heading, vx, vy, wbot,
    //set: targetx, targety, kP,kD, sync servos
    //outputs: d, vtan, wturret, theta, pError, vError,virtualX, virtualY
    // Negative: encoder counts opposite to turret CCW-positive (verified in TurretCharacterize)
    public static final double TICKS_PER_TURRET_RAD = -8192 * (96.0 / 34.0) / (2 * Math.PI), DEADBAND = 0.05;

    // Servo pulse range (microseconds). SDK default is 600-2400.
    // After the fine sweep, set to 1500 -/+ 900 * p_sat so power +/-1 = servo full speed.
    public static final double PWM_LOW = 1095, PWM_HIGH = 1905;

    public DcMotorEx encoder;
    public CRServoImplEx turretServo1, turretServo2;

    // Velocity from position differences: hub getVelocity() is 16-bit and
    // overflows at 32767 ticks/s (~8.9 rad/s turret), which is the turret's top speed.
    private int lastTicks = 0;
    private long lastNanos = 0;
    private double velocity = 0;


    @Override
    public void init(OpMode opMode) {
        instantiateSubsystem(opMode);
        encoder = hardwareMap.get(DcMotorEx.class, "tEnc");
        turretServo1 = hardwareMap.get(CRServoImplEx.class, "tServo1");
        turretServo2 = hardwareMap.get(CRServoImplEx.class, "tServo2");
        turretServo1.setDirection(DcMotorSimple.Direction.REVERSE);
        turretServo1.setPwmRange(new PwmControl.PwmRange(PWM_LOW, PWM_HIGH));
        turretServo2.setPwmRange(new PwmControl.PwmRange(PWM_LOW, PWM_HIGH));
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastTicks = 0;
        lastNanos = System.nanoTime();
    }

    public void setPower(double power) {
        turretServo1.setPower(power);
        turretServo2.setPower(power);
    }

    /** Call exactly once per loop, before using tVelocity(). */
    public void updateVelocity() {
        int ticks = encoder.getCurrentPosition();
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        if (dt > 0) {
            velocity = (ticks - lastTicks) / TICKS_PER_TURRET_RAD / dt;
        }
        lastTicks = ticks;
        lastNanos = now;
    }

    public double turretAngle() {
        return 0;
//        return encoder.getCurrentPosition() / TICKS_PER_TURRET_RAD;  // rad, relative to where it was zeroed
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
        return velocity;  // rad/s, updated by updateVelocity()
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
        updateVelocity();
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
        setPower(finalPower);
    }

    double tof(double d) {
        return 0.5;
    }

    public void virGoal() {
        virX = targetx;
        virY = targety;

    }

}
