package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.turret;

/**
 * Tunes turret kP / kD with the robot sitting still (drive motors never run).
 * Uses the REAL turret.tUpdate(), so whatever you tune here is what MainOpMode gets.
 * Edit turret.kP / turret.kD live in Panels -> Configurables. Watch the graph in Panels.
 *
 * Controls (gamepad1):
 *   X = HOLD  : aim straight forward (0 deg)
 *   A = STEP  : jump between +STEP_DEG and -STEP_DEG each press  -> tune kP, then kD
 *   B = SPIN  : pretends the robot is rocking back and forth      -> checks feedforward + kP together
 *               (turret should stay pointed at the same field direction)
 */
@Configurable
@TeleOp(name = "Turret PD Tuner", group = "Test")
public class TurretPDTuner extends OpMode {
    public static double STEP_DEG = 60;       // size of the step test
    public static double SPIN_AMP_DEG = 90;   // fake robot rocks +/- this much
    public static double SPIN_PERIOD_S = 3;   // seconds per rock back-and-forth

    enum Mode { HOLD, STEP, SPIN }

    turret t = new turret();
    TelemetryManager panels;
    ElapsedTime clock = new ElapsedTime();
    Mode mode = Mode.HOLD;
    double stepSign = -1, spinStart = 0;
    boolean lastA = false, lastUp, lastDown, lastLeft, lastRight;
    public static double KP_STEP = 0.5;    // D-pad up/down changes kP by this
    public static double KD_STEP = 0.005;  // D-pad right/left changes kD by this
    public static double KI_STEP = 0.5;    // right/left bumper changes kI by this

    @Override
    public void init() {
        t.init(this);   // turret must point straight forward NOW: encoder zeroes here
        t.sotmEnabled = false;   // fake robot: no lead / latency, just pure tracking
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.addLine("Point turret straight forward before init. X=hold, A=step, B=spin");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---- buttons ----
        if (gamepad1.a && !lastA) { mode = Mode.STEP; stepSign = -stepSign; }
        lastA = gamepad1.a;
        if (gamepad1.b && mode != Mode.SPIN) { mode = Mode.SPIN; spinStart = clock.seconds(); }
        if (gamepad1.x) mode = Mode.HOLD;

        // ---- live gains on the D-pad (Panels configurables not applying) ----
        if (gamepad1.dpad_up && !lastUp)       turret.kP += KP_STEP;
        if (gamepad1.dpad_down && !lastDown)   turret.kP = Math.max(0, turret.kP - KP_STEP);
        if (gamepad1.dpad_right && !lastRight) turret.kD += KD_STEP;
        if (gamepad1.dpad_left && !lastLeft)   turret.kD = Math.max(0, turret.kD - KD_STEP);
        lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
        lastRight = gamepad1.dpad_right; lastLeft = gamepad1.dpad_left;
        // bumpers: kI
        if (gamepad1.rightBumperWasPressed()) turret.kI += KI_STEP;
        if (gamepad1.leftBumperWasPressed())  turret.kI = Math.max(0, turret.kI - KI_STEP);

        // ---- fake robot: sits at (0,0); target is placed 100 in away at targetAngle ----
        double targetAngle = 0, heading = 0, wbot = 0;
        switch (mode) {
            case STEP:
                targetAngle = stepSign * Math.toRadians(STEP_DEG);
                break;
            case SPIN:
                double w = 2 * Math.PI / SPIN_PERIOD_S;
                double amp = Math.toRadians(SPIN_AMP_DEG);
                double tt = clock.seconds() - spinStart;
                heading = amp * Math.sin(w * tt);          // robot "rotates"
                wbot = amp * w * Math.cos(w * tt);         // and its rotation speed
                break;
            default:
                break;
        }

        t.x = 0; t.y = 0; t.vx = 0; t.vy = 0;
        t.heading = heading;
        t.wbot = wbot;
        t.targetx = 100 * Math.cos(targetAngle);
        t.targety = 100 * Math.sin(targetAngle);
        t.tUpdate();

        // ---- graph these in Panels ----
        panels.addData("FAULT", t.faulted ? "YES: " + t.faultReason + " (restart OpMode)" : "no");
        panels.addData("mode", mode.toString());
        panels.addData("power", t.lastPower);
        panels.addData("target (deg)", Math.toDegrees(t.theta));
        panels.addData("turret (deg)", Math.toDegrees(t.turretAngle()));
        panels.addData("error (deg)", Math.toDegrees(t.pError));
        panels.addData("ff vel (rad/s)", t.wturret);
        panels.addData("cmd vel (rad/s)", t.wCmd);
        panels.addData("actual vel (rad/s)", t.tVelocity());
        panels.addData("kP  (dpad up/down)", turret.kP);
        panels.addData("kD  (dpad right/left)", turret.kD);
        panels.addData("kI  (bumpers R+/L-)", turret.kI);
        panels.addData("I term (rad/s)", t.iTerm);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        t.setPower(0);
    }
}
