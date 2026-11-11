package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.fieldCentricMecanum;
import org.firstinspires.ftc.teamcode.subSystems.localizer;
import org.firstinspires.ftc.teamcode.subSystems.turret;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Collect the shot table: distance -> (hood, rpm). Robot drives, turret auto-aims at the goal,
 * shooter works like MainOpMode. Shoot from STANDING STILL at each distance.
 *
 * gamepad1: sticks = drive
 *           A = HIT   B = MISS   X = reset hit/miss count   Y = delete last saved row

 * gamepad2: Shooter controls as usual (Y flywheel, X intake, A transfer, hold LB = shoot)
 *           dpad up/down    = RPM +/- RPM_STEP
 *           dpad right/left = hood +/- HOOD_STEP
 *           B               = SAVE row (distance, rpm, hood, hits/shots) and reset count
 *
 * Saved rows show on the Driver Hub and go to Logcat (filter: SHOT_TABLE) for copying.
 * Distance = turret.d = robot center to goal, from odometry. Start on the taped start pose!
 */
@Configurable
@TeleOp(name = "Shot Table Tuner", group = "Test")
public class ShotTableTuner extends OpMode {
    public static double START_X = -63, START_Y = -63, START_H_DEG = 90;   // same as MainOpMode
    public static double RPM_STEP = 50, HOOD_STEP = 0.01;
    public static double MAX_RPM = 3600;   // keep headroom below the ~4200 max for recovery

    fieldCentricMecanum drive = new fieldCentricMecanum();
    Shooter shooter = new Shooter();
    turret turret = new turret();
    localizer loc;
    GoBildaPinpointDriver odo;
    TelemetryManager panels;
    List<LynxModule> hubs;

    double prevX, prevY, prevH;
    int hits = 0, shots = 0;
    final List<String> rows = new ArrayList<>();

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drive.instantiateSubsystem(this);
        drive.init(this);
        shooter.init(this);
        turret.init(this);   // turret must point straight forward at init
        loc = new localizer(START_X, START_Y, Math.toRadians(START_H_DEG));
        loc.instantiateSubsystem(this);
        odo = drive.odo;
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("Robot on the taped start pose, turret straight forward.");
        telemetry.update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        // ---- drive ----
        drive.fieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // ---- shot settings (gamepad2 dpad) ----
        if (gamepad2.dpadUpWasPressed())    Shooter.TARGET_RPM = Math.min(MAX_RPM, Shooter.TARGET_RPM + RPM_STEP);
        if (gamepad2.dpadDownWasPressed())  Shooter.TARGET_RPM = Math.max(0, Shooter.TARGET_RPM - RPM_STEP);
        if (gamepad2.dpadRightWasPressed()) Shooter.HOOD_POS = Math.min(1, Shooter.HOOD_POS + HOOD_STEP);
        if (gamepad2.dpadLeftWasPressed())  Shooter.HOOD_POS = Math.max(0, Shooter.HOOD_POS - HOOD_STEP);
        shooter.update();

        // ---- localization (same as MainOpMode, odometry only) ----
        double px = odo.getPosX(DistanceUnit.INCH), py = odo.getPosY(DistanceUnit.INCH);
        double ph = normalizeRadians(odo.getHeading());
        double dx = px - prevX, dy = py - prevY, dh = normalizeRadians(ph - prevH);
        double offset = normalizeRadians(loc.getH() - prevH);
        loc.predict(dx * Math.cos(offset) - dy * Math.sin(offset),
                    dx * Math.sin(offset) + dy * Math.cos(offset), dh);
        prevX = px; prevY = py; prevH = ph;

        // ---- turret auto-aim ----
        turret.x = loc.getX();
        turret.y = loc.getY();
        turret.heading = loc.getH();
        double vx = odo.getVelX(DistanceUnit.INCH), vy = odo.getVelY(DistanceUnit.INCH);
        turret.vx = vx * Math.cos(offset) - vy * Math.sin(offset);
        turret.vy = vx * Math.sin(offset) + vy * Math.cos(offset);
        turret.wbot = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        turret.tUpdate();

        // ---- scoring log ----
        if (gamepad1.aWasPressed()) { hits++; shots++; }
        if (gamepad1.bWasPressed()) { shots++; }
        if (gamepad1.xWasPressed()) { hits = 0; shots = 0; }
        if (gamepad1.yWasPressed() && !rows.isEmpty()) rows.remove(rows.size() - 1);
        if (gamepad2.bWasPressed()) {
            String row = String.format(Locale.US, "d %.1f in | rpm %.0f | hood %.3f | %d/%d hits",
                    turret.d, Shooter.TARGET_RPM, Shooter.HOOD_POS, hits, shots);
            rows.add(row);
            RobotLog.ii("SHOT_TABLE", row);
            hits = 0;
            shots = 0;
        }

        // ---- display ----
        telemetry.addData("DISTANCE (in)", "%.1f", turret.d);
        telemetry.addData("robot x, y, h", "%.1f, %.1f, %.0f", loc.getX(), loc.getY(), Math.toDegrees(loc.getH()));
        telemetry.addData("turret error (deg)", "%.1f%s", Math.toDegrees(turret.pError),
                turret.faulted ? "  FAULT: " + turret.faultReason : "");
        telemetry.addData("TARGET RPM (g2 dpad up/down)", "%.0f", Shooter.TARGET_RPM);
        telemetry.addData("HOOD (g2 dpad left/right)", "%.3f", Shooter.HOOD_POS);
        telemetry.addData("hits / shots (g1 A/B)", "%d / %d", hits, shots);
        telemetry.addLine("---- saved rows (g2 B save, g1 Y delete last) ----");
        for (String r : rows) telemetry.addLine(r);

        panels.addData("rpm", shooter.rpm);
        panels.addData("target rpm", Shooter.TARGET_RPM);
        panels.addData("distance", turret.d);
        panels.update();
    }

    @Override
    public void stop() {
        for (String r : rows) RobotLog.ii("SHOT_TABLE", "FINAL " + r);
    }
}
