package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.turret;

/**
 * Tests the turret MATH only — no turret hardware needed, just a Control Hub.
 * We never call turret.init(), so nothing touches hardwareMap, the encoder, or the servos.
 * Each check prints PASS/FAIL with the expected and actual value.
 */
@TeleOp(name = "Turret Math Test", group = "Test")
public class TurretMathTest extends LinearOpMode {

    static final double TOL = 1e-3;
    int passed = 0, failed = 0;

    @Override
    public void runOpMode() {
        telemetry.addLine("Press START to run tests");
        telemetry.update();
        waitForStart();

        turret t = new turret();   // NOT calling init() on purpose

        // ---------------- track() ----------------
        // Robot at origin facing +x, target at (30, 40), sliding +x at 10 in/s
        setInputs(t, 0, 0, 0, 10, 0, 0, 30, 40);
        t.track();
        check("A1 d",              t.d, 50);
        check("A2 theta (deg)",    Math.toDegrees(t.theta), 53.130);
        check("A3 vtan",           t.vtan, -8);
        check("A4 wturret",        t.wturret, 0.16);

        // Same, but robot facing 90° (+y)
        setInputs(t, 0, 0, Math.toRadians(90), 10, 0, 0, 30, 40);
        t.track();
        check("B1 theta (deg)",    Math.toDegrees(t.theta), -36.870);

        // Spinning in place at 1 rad/s, not moving: turret must spin -1 rad/s
        setInputs(t, 0, 0, 0, 0, 0, 1.0, 30, 40);
        t.track();
        check("C1 wturret (spin)", t.wturret, -1.0);

        // Close range: d = 5 < 10, so dSafe = 10 is used for both divisions
        setInputs(t, 0, 0, 0, 10, 0, 0, 3, 4);
        t.track();
        check("D1 d (real, unclamped)", t.d, 5);
        check("D2 wturret (clamped)",   t.wturret, 0.4);

        // Robot not at origin: same geometry shifted by (100, 100)
        setInputs(t, 100, 100, 0, 10, 0, 0, 130, 140);
        t.track();
        check("E1 theta (deg)",    Math.toDegrees(t.theta), 53.130);
        check("E2 wturret",        t.wturret, 0.16);

        // ---------------- chooseTarget() ----------------
        check("F1 simple",          t.chooseTarget(0.5, 0), 0.5);
        checkBool("F1 reachable",   t.reachable, true);
        check("F2 short way past 180", t.chooseTarget(-3.054, 2.967), -3.054 + 2 * Math.PI);
        check("F3 forced unwind",   t.chooseTarget(0.524, -3.316), 0.524);

        // ---------------- results ----------------
        while (opModeIsActive()) {
            telemetry.addLine(String.format("PASSED %d   FAILED %d", passed, failed));
            telemetry.update();
        }
    }

    void setInputs(turret t, double x, double y, double heading,
                   double vx, double vy, double wbot, double tx, double ty) {
        t.x = x; t.y = y; t.heading = heading;
        t.vx = vx; t.vy = vy; t.wbot = wbot;
        t.targetx = tx; t.targety = ty;
    }

    void check(String name, double actual, double expected) {
        boolean ok = Math.abs(actual - expected) < TOL;
        if (ok) passed++; else failed++;
        telemetry.addLine(String.format("%s %s  exp %.4f  got %.4f",
                ok ? "PASS" : "FAIL", name, expected, actual));
    }

    void checkBool(String name, boolean actual, boolean expected) {
        boolean ok = actual == expected;
        if (ok) passed++; else failed++;
        telemetry.addLine(String.format("%s %s  exp %b  got %b",
                ok ? "PASS" : "FAIL", name, expected, actual));
    }
}