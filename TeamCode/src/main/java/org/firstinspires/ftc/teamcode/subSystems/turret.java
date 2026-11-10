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
    public double wturret, theta, wbot, vtan, d, targety=-18.5805750382, targetx=12.75, x, y, vx, vy, heading, pError, vError, virX, virY;
//    public static double kP = 3.5, kD=0.08;   // kP is now (rad/s asked for) per (rad of error)
    public static double kP = 6.2, kD=0.065;   // kP is now (rad/s asked for) per (rad of error)

    public double wCmd;                // total speed asked for = feedforward + kP * error
    public double lastPower;           // power actually sent to the servos (for telemetry)
    // ---- wire safety ----
    public static double SAFETY_DEG = 170;    // never push further out past this
    public static double STALL_POWER = 0.3;   // "trying hard" threshold
    public static double STALL_S = 0.5;       // trying hard but not moving this long -> FAULT
    public boolean faulted = false;           // latched: turret stays off until the OpMode restarts
    public String faultReason = "";
    private long stallStartNanos = -1, wrongWayStartNanos = -1;
    // Feedforward per direction (fit: power = kV * |vel| + kS). POS = turret turning CCW (angle up).
    // Swapped after the servo flip so each value stays with the same PHYSICAL direction.
    // Re-run Turret Characterize to confirm.
//    public static double kS_POS = 0.18, kV_POS = 0.105;
//    public static double kS_NEG = 0.27, kV_NEG = 0.09;
    public static double kS_POS = 0.215, kV_POS = 0.06;
    public static double kS_NEG = 0.153, kV_NEG = 0.1;
    //inputs: x, y, heading, vx, vy, wbot,
    //set: targetx, targety, kP,kD, sync servos
    //outputs: d, vtan, wturret, theta, pError, vError,virtualX, virtualY
    // Negative: encoder counts opposite to turret CCW-positive (verified in TurretCharacterize)
    public static final double TICKS_PER_TURRET_RAD = -1 * 8192 * (96.0 / 34.0) / (2 * Math.PI), DEADBAND = 0.05;

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
        encoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        turretServo1 = hardwareMap.get(CRServoImplEx.class, "tServo1");
        turretServo2 = hardwareMap.get(CRServoImplEx.class, "tServo2");
        // Positive power must turn the turret CCW (angle goes UP). Both flipped together
        // so the two servos still agree with each other.
        turretServo1.setDirection(DcMotorSimple.Direction.FORWARD);
        turretServo2.setDirection(DcMotorSimple.Direction.REVERSE);
        turretServo1.setPwmRange(new PwmControl.PwmRange(PWM_LOW, PWM_HIGH));
        turretServo2.setPwmRange(new PwmControl.PwmRange(PWM_LOW, PWM_HIGH));
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastTicks = 0;
        lastNanos = System.nanoTime();
    }

    /** Raw encoder count, no matter what direction another subsystem set on this motor port.
     *  (The encoder shares the "leftFront" drive port, and the drive REVERSEs that motor,
     *   which would otherwise flip this reading in MainOpMode but not in the test OpModes.) */
    private int rawTicks() {
        int ticks = encoder.getCurrentPosition();
        return encoder.getDirection() == DcMotorSimple.Direction.REVERSE ? -ticks : ticks;
    }

    public void setPower(double power) {
        turretServo1.setPower(power);
        turretServo2.setPower(power);
    }

    /** Call exactly once per loop, before using tVelocity(). */
    public void updateVelocity() {
        int ticks = rawTicks();
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        if (dt > 0) {
            velocity = (ticks - lastTicks) / TICKS_PER_TURRET_RAD / dt;
        }
        lastTicks = ticks;
        lastNanos = now;
    }

    public double turretAngle() {
//        return 0;
        return rawTicks() / TICKS_PER_TURRET_RAD;  // rad, relative to where it was zeroed
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
        if (w > 0) {
            power = kS_POS + kV_POS * w;    // kS = power to start moving; kV = power per rad/s
        } else {
            power = -kS_NEG + kV_NEG * w;   // w is negative here, so both terms are negative
        }
        power = Math.max(Math.min(power, 1), -1);
        return power;
    }

    public void tUpdate() {
        updateVelocity();
        double current = turretAngle();
        track();
        pidCalc(current);
        double ff = wturret;
        if (!reachable) ff = 0;                          // parked at a limit: no feedforward

        // P becomes part of the speed we ask for, so velToPower's kS helps it beat friction
        wCmd = ff + kP * pError;                         // rad/s
        if (current >= MAX_ANGLE && wCmd > 0) wCmd = 0;  // don't push past the top stop
        if (current <= MIN_ANGLE && wCmd < 0) wCmd = 0;  // ...or the bottom one

        vError = wCmd - tVelocity();                     // D: damp the difference from the asked-for speed
        double finalPower;
        finalPower = velToPower(wCmd) + kD * vError;
        finalPower = Math.max(Math.min(finalPower, 1), -1);

        // ---- wire safety ----
        // 1. over-travel: never push further out past SAFETY_DEG (pushing back in is allowed)
        if (Math.abs(current) > Math.toRadians(SAFETY_DEG) && Math.signum(finalPower) == Math.signum(current)) {
            finalPower = 0;
        }
        // 2. encoder unplugged / wrong sign / jammed: pushing hard but not moving -> latch off
        if (Math.abs(finalPower) > STALL_POWER && Math.abs(tVelocity()) < 0.05) {
            if (stallStartNanos < 0) stallStartNanos = System.nanoTime();
            if ((System.nanoTime() - stallStartNanos) / 1e9 > STALL_S) {
                faulted = true;
                faultReason = String.format("stalled at %.0f deg, power %.2f", Math.toDegrees(current), finalPower);
            }
        } else {
            stallStartNanos = -1;
        }
        // 3. wrong sign: pushing one way but moving fast the OTHER way -> latch off
        if (Math.abs(finalPower) > STALL_POWER && Math.abs(tVelocity()) > 1.0
                && Math.signum(tVelocity()) != Math.signum(finalPower)) {
            if (wrongWayStartNanos < 0) wrongWayStartNanos = System.nanoTime();
            if ((System.nanoTime() - wrongWayStartNanos) / 1e9 > 0.25) {
                faulted = true;
                faultReason = String.format("WRONG WAY: power %.2f but moving %.1f rad/s at %.0f deg",
                        finalPower, tVelocity(), Math.toDegrees(current));
            }
        } else {
            wrongWayStartNanos = -1;
        }
        if (faulted) finalPower = 0;

        lastPower = finalPower;
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
