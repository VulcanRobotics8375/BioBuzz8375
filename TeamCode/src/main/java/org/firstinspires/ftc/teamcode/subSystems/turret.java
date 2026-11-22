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
    // Turret travel limits, DEGREES from straight forward (CCW = +). Live-editable in Panels.
    // Set ~5-10 deg INSIDE the physical stops / where the wires get tight.
    public static double MIN_ANGLE_DEG = -120;
    public static double MAX_ANGLE_DEG = 180;
    // ---- smart aim point: aim DEEPER into the goal when shooting from off to the side ----
    // Each goal's FACING = the direction its opening points OUT into the field (deg, field frame, CCW from +x).
    // Calibrate: park straight in front of a goal -> "off-axis" telemetry should read ~0. Adjust until it does.
    public static double FACING_BA_DEG = 0, FACING_BS_DEG = 0, FACING_RA_DEG = 0, FACING_RS_DEG = 0;
    public static boolean SMART_AIM = true;
    public static double AIM_DEPTH_MAX = 0;     // in: how far INTO the goal to aim at full off-axis angle (0 = off)
    public static double AIM_FULL_DEG = 60;     // off-axis angle where the full depth is used (linear below it)
    public double goalX = -18.5805750382, goalY = -12.75, goalFacing = 0;  // front-center of the chosen goal
    // LOCK: hold the turret at lockAngleDeg instead of aiming (Shot Table Tuner). track() still runs, so
    // d (distance) and aimTheta (where the goal is, relative to the robot) keep updating.
    // Driver fix for a turret that wasn't straight at init: + = turret aims more to the RIGHT (clockwise).
    // Shifts what the code thinks "forward" is, so aiming AND the angle limits stay correct.
    public double zeroOffsetDeg = 0;
    public boolean lockMode = false;
    public double lockAngleDeg = 0;
    public double aimTheta = 0;                 // rad: turret angle that WOULD point at the goal
    public boolean goalSet = false;             // only true once targetUpdate() picked a goal (tuners set targetx directly)
    public double offAxisDeg = 0, aimDepth = 0; // telemetry
    private static double minA() { return Math.toRadians(MIN_ANGLE_DEG); }
    private static double maxA() { return Math.toRadians(MAX_ANGLE_DEG); }
    public boolean reachable = false;
    public double wturret, theta, wbot, vtan, d, targety=-12.75, targetx=-18.5805750382, x, y, vx, vy, heading, pError, vError, virX, virY;
//    public static double kP = 3.5, kD=0.08;   // kP is now (rad/s asked for) per (rad of error)
    public static double kP = 4, kD=0.0575;   // kP is now (rad/s asked for) per (rad of error)
    //P:4, I:14, D:0.05

    // ---- I term: removes the small leftover lag/offset that P + feedforward can't ----
    // kI = (rad/s asked for) per (rad of error * second). Start at 0, raise slowly.
    public static double kI = 0.0;
    public static double I_ZONE_DEG = 8;      // only integrate when this close (big moves = P's job)
    public static double I_MAX = 1.0;         // max rad/s the I term may add (anti-windup clamp)
    public static boolean I_RESET_ON_CROSS = true; // dump the I term when error changes sign (less overshoot)
    public double iSum = 0;                   // integral of error (rad*s)
    public double iTerm = 0;                  // kI * iSum, rad/s (for telemetry)
    private double lastDt = 0, lastPError = 0;

    // ---- shoot on the move ----
    // Turret pivot relative to the robot center Pinpoint tracks (robot frame: +x forward, +y left).
    // Same numbers as AprilTag.TURRET_DX/DY.
    public static double TURRET_DX = -0.835, TURRET_DY = 0;
    public static boolean SOTM = true;        // false = aim straight at the real goal (no lead)
    public static double LATENCY_S = 0.10;    // aim where the robot WILL be when the ball leaves
    public static double TOF_SCALE = 1.0;     // >1 = lead more, <1 = lead less (tune Thursday)
    public static double VG_MAX_SHIFT = 60;   // in: virtual goal never moves further than this
    public static int VG_ITERS = 3;
    public boolean sotmEnabled = true;        // per-OpMode switch (PD tuner turns it off; SOTM is the master)
    public double sx, sy, svx, svy;          // turret pivot position + velocity (field, predicted)
    public double dReal, tofNow;              // distance to REAL goal, time of flight used

    public double wCmd;                // total speed asked for = feedforward + kP * error
    public double lastPower;           // power actually sent to the servos (for telemetry)
    // ---- wire safety ----
    public static double SAFETY_DEG = 200;    // never push further out past this
    // (wrong-way fault check REMOVED: it was shutting the turret off during normal hard braking.
    //  `faulted` stays only so other files still compile - nothing sets it any more.)
    public boolean faulted = false;
    public String faultReason = "";
    // Feedforward per direction (fit: power = kV * |vel| + kS). POS = turret turning CCW (angle up).
    // Swapped after the servo flip so each value stays with the same PHYSICAL direction.
    // Re-run Turret Characterize to confirm.
//    public static double kS_POS = 0.18, kV_POS = 0.105;
//    public static double kS_NEG = 0.27, kV_NEG = 0.09;
    public static double kS_POS = 0.14, kV_POS = 0.08;//0.02, 0.125
    public static double kS_NEG = 0.16, kV_NEG = 0.09;//0.025, 0.145
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
        encoder = hardwareMap.get(DcMotorEx.class, "tEnc");
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
        return encoder.getDirection() == DcMotorSimple.Direction.FORWARD ? -ticks : ticks;
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
        lastDt = Math.min(dt, 0.1);   // cap: a long pause (init, breakpoint) shouldn't dump a big I step
        lastTicks = ticks;
        lastNanos = now;
    }

    public double turretAngle() {
//        return 0;
        return rawTicks() / TICKS_PER_TURRET_RAD + Math.toRadians(zeroOffsetDeg);  // rad, + driver's zero fix
    } // turrent angle from encoder

    public void  targetUpdate(String target) {
        // Field frame = Panels frame (rotated -90 deg from the old one: old (x, y) -> new (y, -x)).
        // old: ba (12.75, -18.58)  bs (12.75, 18.58)  ra (-12.75, -18.58)  rs (-12.75, 18.58)
        // Colors SWAPPED by hand (blue = +y, red = -y). Each goal uses its own FACING_ value.
        if (target == "ra") setGoal(-18.5805750382, -12.75, FACING_RA_DEG);
        if (target == "rs") setGoal(18.5805750382, -12.75, FACING_RS_DEG);
        if (target == "ba") setGoal(-18.5805750382, 12.75, FACING_BA_DEG);
        if (target == "bs") setGoal(18.5805750382, 12.75, FACING_BS_DEG);   // blue = +y side
    }

    private void setGoal(double gx, double gy, double facingDeg) {
        goalX = gx;
        goalY = gy;
        goalFacing = Math.toRadians(facingDeg);
        goalSet = true;
    }

    /**
     * Aim point for where the robot is. Straight in front of the goal -> the front-center.
     * Off to the side -> slide the aim point back INTO the goal (against the facing direction),
     * so the ball clears the near side wall instead of clipping it.
     */
    private void updateAimPoint() {
        if (!goalSet) return;                                   // tuners that set targetx/targety directly
        double fromGoal = atan2(y - goalY, x - goalX);          // direction goal -> robot
        double off = AngleUnit.normalizeRadians(fromGoal - goalFacing);
        offAxisDeg = Math.toDegrees(off);
        double frac = SMART_AIM ? Math.min(1, Math.abs(offAxisDeg) / Math.max(AIM_FULL_DEG, 1)) : 0;
        aimDepth = AIM_DEPTH_MAX * frac;
        targetx = goalX - aimDepth * Math.cos(goalFacing);
        targety = goalY - aimDepth * Math.sin(goalFacing);
    }

    public void track() {
        updateAimPoint();                // targetx/targety = best aim point for where we are
        virGoal();                       // sets sx, sy, svx, svy, virX, virY, tofNow
        double h = (SOTM && sotmEnabled) ? heading + wbot * LATENCY_S : heading;
        double rx = virX - sx;
        double ry = virY - sy;
        d = Math.sqrt((Math.pow(rx, 2)) + Math.pow(ry, 2));   // distance to the (virtual) goal -> shot table
        double dSafe = Math.max(d, 10);  // only for the division
        vtan = (rx * svy - ry * svx) / dSafe;
        wturret = -(wbot + (vtan / dSafe)); //output = input
        theta = AngleUnit.normalizeRadians(atan2(ry, rx) - h); //normalize radinas angles into [-pi, pi]
    }

    public double chooseTarget(double theta, double current) {
        double[] candidates = {theta, theta + 2 * PI, theta - 2 * PI};
        double best = Double.NaN;                          // "none yet"
        for (double c : candidates) {
            if (c < minA() || c > maxA()) {
                continue;
            }// outside limits: skip
            if (Double.isNaN(best) || Math.abs(c - current) < Math.abs(best - current)) {
                best = c;                                  // keep the CANDIDATE, not the distance
            }
        }
        reachable = !Double.isNaN(best);   // found at least one valid candidate?
        if (Double.isNaN(best)) {                          // target is behind the stops
            best = Math.max(minA(), Math.min(maxA(), theta));
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
        aimTheta = theta;
        if (lockMode) {                                  // hold still: aim at the lock angle, no feedforward
            theta = Math.toRadians(lockAngleDeg);
            wturret = 0;
        }
        pidCalc(current);
        double ff = wturret;
        if (!reachable) ff = 0;                          // parked at a limit: no feedforward

        // ---- I term (with anti-windup) ----
        boolean atLimit = (current >= maxA() && pError > 0) || (current <= minA() && pError < 0);
        boolean crossed = Math.signum(pError) != Math.signum(lastPError);
        if (kI == 0 || faulted || !reachable || atLimit) {
            iSum = 0;                                            // nothing useful to integrate
        } else if (I_RESET_ON_CROSS && crossed) {
            iSum = 0;                                            // passed the target: stop pushing
        } else if (Math.abs(pError) < Math.toRadians(I_ZONE_DEG)) {
            iSum += pError * lastDt;                             // only near the target
            double sumMax = I_MAX / kI;                          // so kI*iSum stays within +/-I_MAX
            iSum = Math.max(-sumMax, Math.min(sumMax, iSum));
        }
        // outside the zone: hold iSum (don't grow it, don't throw it away)
        iTerm = kI * iSum;
        lastPError = pError;

        // P (and I) become part of the speed we ask for, so velToPower's kS helps beat friction
        wCmd = ff + kP * pError + iTerm;                 // rad/s
        if (current >= maxA() && wCmd > 0) wCmd = 0;  // don't push past the top stop
        if (current <= minA() && wCmd < 0) wCmd = 0;  // ...or the bottom one

        vError = wCmd - tVelocity();                     // D: damp the difference from the asked-for speed
        double finalPower;
        finalPower = velToPower(wCmd) + kD * vError;
        finalPower = Math.max(Math.min(finalPower, 1), -1);

        // ---- wire safety ----
        // 1. over-travel: never push further out past SAFETY_DEG (pushing back in is allowed)
        if (Math.abs(current) > Math.toRadians(SAFETY_DEG) && Math.signum(finalPower) == Math.signum(current)) {
            finalPower = 0;
        }

        lastPower = finalPower;
        setPower(finalPower);
    }

    /**
     * Virtual goal: the ball keeps the turret's velocity, so aim at goal - v * TOF.
     * TOF depends on distance and distance depends on the aim point, so iterate a few times.
     */
    public void virGoal() {
        double cosH = Math.cos(heading), sinH = Math.sin(heading);
        double ox = TURRET_DX * cosH - TURRET_DY * sinH;   // pivot offset, field frame
        double oy = TURRET_DX * sinH + TURRET_DY * cosH;
        // pivot velocity = robot velocity + spin * offset (omega x r)
        svx = vx - wbot * oy;
        svy = vy + wbot * ox;
        sx = x + ox;
        sy = y + oy;
        dReal = Math.hypot(targetx - sx, targety - sy);

        virX = targetx;
        virY = targety;
        tofNow = 0;
        if (!SOTM || !sotmEnabled) return;

        sx += svx * LATENCY_S;             // where the pivot will be when the ball actually leaves
        sy += svy * LATENCY_S;
        for (int i = 0; i < VG_ITERS; i++) {
            double dist = Math.hypot(virX - sx, virY - sy);
            tofNow = ShotTable.tof(dist) * TOF_SCALE;
            double shiftX = -svx * tofNow, shiftY = -svy * tofNow;
            double shift = Math.hypot(shiftX, shiftY);
            if (shift > VG_MAX_SHIFT) {    // bad velocity reading: don't send the turret wild
                shiftX *= VG_MAX_SHIFT / shift;
                shiftY *= VG_MAX_SHIFT / shift;
            }
            virX = targetx + shiftX;
            virY = targety + shiftY;
        }
    }

}
