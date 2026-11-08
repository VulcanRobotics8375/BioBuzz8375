package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.turret;

/**
 * Measures steady-state turret speed at a list of powers.
 * All hardware setup (servo directions, encoder sign, PWM range) comes from turret.init(),
 * so this test always matches the real robot code.
 *
 * Wiring: powers alternate +/-, so each run unwinds the last one. Power is cut the moment
 * the turret passes +/-LIMIT_DEG; the speed measured up to that point is still used.
 * Point the turret straight forward BEFORE init (encoder zeroes at init).
 */
@Configurable
@TeleOp(name = "Turret Characterize", group = "Test")
public class TurretCharacterize extends LinearOpMode {

    public static double LIMIT_DEG = 150;   // cut power past this (from where it was zeroed)
    public static double SETTLE_S = 0.2;    // let speed stabilize before measuring
    public static double MEASURE_S = 1.0;   // measure up to this long (or until the limit)

    // true  = fine sweep 0.05..0.5 to find p_sat (where speed stops increasing)
    // false = coarse sweep 0.2..1.0 for fitting kS/kV
    public static boolean FINE_SWEEP = false;

    public static double[] FINE = {0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2, 0.25, -0.25,
                                   0.3, -0.3, 0.35, -0.35, 0.4, -0.4, 0.45, -0.45, 0.5, -0.5};
    public static double[] COARSE = {0.2, -0.2, 0.35, -0.35, 0.5, -0.5, 0.65, -0.65, 0.8, -0.8, 1, -1};

    turret t = new turret();

    @Override
    public void runOpMode() {
        // ---------- 1. INIT ----------
        t.init(this);   // directions, encoder sign, PWM range, encoder reset
        double[] powers = FINE_SWEEP ? FINE : COARSE;

        telemetry.addLine("Point turret straight forward, then press START");
        telemetry.addData("Sweep", FINE_SWEEP ? "FINE (find p_sat)" : "COARSE (fit kS/kV)");
        telemetry.addData("Limit", "+/- %.0f deg", LIMIT_DEG);
        telemetry.update();

        // ---------- 2. WAIT ----------
        waitForStart();

        // ---------- 3. RUN ----------
        double[] results = new double[powers.length];
        String[] notes = new String[powers.length];

        for (int i = 0; i < powers.length && opModeIsActive(); i++) {
            double power = powers[i];
            notes[i] = "";
            t.setPower(power);

            ElapsedTime timer = new ElapsedTime();
            boolean measuring = false;
            double startAngle = 0, startTime = 0;
            double endAngle = 0, endTime = 0;

            while (opModeIsActive() && timer.seconds() < SETTLE_S + MEASURE_S) {
                t.updateVelocity();
                double angle = t.turretAngle();

                // wire safety: only stop if it's heading further OUT past the limit
                if (Math.abs(Math.toDegrees(angle)) > LIMIT_DEG && Math.signum(angle) == Math.signum(power)) {
                    notes[i] = " (hit limit)";
                    break;
                }
                // Average speed = total angle moved / time.
                if (timer.seconds() > SETTLE_S) {
                    if (!measuring) {
                        measuring = true;
                        startAngle = angle;
                        startTime = timer.seconds();
                    }
                    endAngle = angle;
                    endTime = timer.seconds();
                }
                telemetry.addData("Run", "%d / %d  power %+.2f", i + 1, powers.length, power);
                telemetry.addData("Angle (deg)", "%.1f", Math.toDegrees(angle));
                telemetry.addData("Vel (rad/s)", "%.2f", t.tVelocity());
                telemetry.update();
            }

            t.setPower(0);
            double dt = endTime - startTime;
            if (measuring && dt > 0.05) {
                results[i] = (endAngle - startAngle) / dt;
            } else {
                results[i] = Double.NaN;
                notes[i] += " no data: hit limit during settle (lower SETTLE_S)";
            }
            sleep(500); // let it stop before reversing
        }

        t.setPower(0);

        // ---------- 4. SHOW RESULTS ----------
        while (opModeIsActive()) {
            telemetry.addLine("Fit each direction: x = |vel|, y = |power|  ->  kV = slope, kS = intercept");
            for (int i = 0; i < powers.length; i++) {
                telemetry.addData(String.format("power %+.2f", powers[i]),
                        String.format("%.3f rad/s%s", results[i], notes[i]));
            }
            telemetry.update();
        }
    }
}
