package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subSystems.AprilTag;
import org.firstinspires.ftc.teamcode.subSystems.MotorOuttake;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSensor;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.fieldCentricMecanum;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.localizer;
import org.firstinspires.ftc.teamcode.subSystems.turret;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.FieldManager;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Configurable
@TeleOp(name="MainOpMode")
    public class MainOpMode extends OpMode {
    TelemetryManager panels;
    FieldManager field;
    private double axial, lateral, yaw;
    fieldCentricMecanum drive = new fieldCentricMecanum();
    Intake Intake;
    MotorOuttake Outtake = new MotorOuttake();
    Shooter shooter = new Shooter();
    IntakeSensor intakeSensor = new IntakeSensor();
    PanelsPads pads = new PanelsPads();   // real + Panels controllers merged

    //    localizer loc = new localizer(-63, -63, 0);
    // Panels frame. Old frame was (12.75, 63, -90 deg) -> rotated -90 deg: (63, -12.75, 180 deg)
    public static double startx=63, starty=-12.75, starth=Math.PI;
    localizer loc = new localizer(startx, starty, starth);

    GoBildaPinpointDriver odo;
    turret turret = new turret();
    AprilTag apriltag = new AprilTag();
    public double prevX, prevY, prevH, dxField, dyField;
    public boolean sawTag;
    public boolean camOn = true;
    boolean usedAutonPose = false;
    public boolean alliance = true; //true = blue, red = false

    long lastLoopNanos = System.nanoTime();
    // Panels DRAWING only (localization/aiming untouched): rotate what's drawn by this many degrees.
    // -90 = undo a field that shows up rotated 90 deg CCW. Wrong way round? try +90 (live in Panels).
    public static double FIELD_DRAW_ROT_DEG = 0;   // localization now uses the Panels frame -> no rotation
    public static long PANELS_PERIOD_MS = 50;   // Panels graphs + field drawing at 20 Hz, not every loop
    java.util.List<LynxModule> hubs;
    long lastPanelsMs = 0;
    // loop profiler: worst time of each part over the last second
    double[] secMax = new double[5], secShow = new double[5];
    double loopMax = 0, loopMaxShow = 0, loopSum = 0; int loopCount = 0; double loopAvgShow = 0;
    long profWindowStart = System.nanoTime();

    public static boolean USE_AUTON_POSE = true;   // start teleop where auton ended (if auton ran < 60 s ago)
    public void init() {
        if (USE_AUTON_POSE && org.firstinspires.ftc.teamcode.auton.PoseStorage.fresh(60)) {
            loc = new localizer(org.firstinspires.ftc.teamcode.auton.PoseStorage.x,
                    org.firstinspires.ftc.teamcode.auton.PoseStorage.y,
                    org.firstinspires.ftc.teamcode.auton.PoseStorage.heading);
            usedAutonPose = true;
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());
        turret.instantiateSubsystem(this);
        drive.instantiateSubsystem(this);
        shooter.instantiateSubsystem(this);
//        Intake.instantiateSubsystem(this);
//        Outtake.instantiateSubsystem(this);
//        apriltag.instantiateSubsystem(this);
        loc.instantiateSubsystem(this);
        drive.init(this);
        shooter.init(this);
        intakeSensor.init(this);
//        Intake.init(this);
//        Outtake.init(this);
        turret.init(this);
//        apriltag.init(this);
        odo = drive.odo;
        shooter.useGamepads(pads.g1, pads.g2);
    }

    @Override
    public void loop() {
        long t0 = System.nanoTime();
        for (LynxModule hub : hubs) hub.clearBulkCache();
        pads.update(this);

        axial = pads.g1.left_stick_y;
        lateral = -pads.g1.left_stick_x;
        yaw = -pads.g1.right_stick_x;

        drive.fieldCentric(axial, lateral, yaw);   // also reads the Pinpoint
        long t1 = System.nanoTime();
//        Intake.runIntake();
//        Outtake.runOuttake();

        if (pads.g2.rightBumperWasPressed()) {
            alliance = !alliance;
        }

        if (alliance) { //BLUE
            if (loc.getX() <= 0){   // side split (was y <= 0 in the old frame)
                turret.targetUpdate("ba");
            } else {
                turret.targetUpdate("bs");

            }
        } else {
            if (loc.getX() <= 0){ //red
                turret.targetUpdate("ra");
            } else {
                turret.targetUpdate("rs");

            }
        }


        double dx = odo.getPosX(DistanceUnit.INCH) - prevX;
        double dy = odo.getPosY(DistanceUnit.INCH) - prevY;
        double dh = normalizeRadians(odo.getHeading() - prevH);

        double offset = normalizeRadians(loc.getH() - prevH);
        dxField = dx * Math.cos(offset) - dy * Math.sin(offset);
        dyField = dx * Math.sin(offset) + dy * Math.cos(offset);

        loc.predict(dxField, dyField, dh);
        prevX = odo.getPosX(DistanceUnit.INCH);
        prevY = odo.getPosY(DistanceUnit.INCH);
        prevH = normalizeRadians(odo.getHeading());
//        loc.update (camX, camY, camH);

        turret.x = loc.getX();
        turret.y = loc.getY();
        turret.heading = loc.getH();
        double vx = odo.getVelX(DistanceUnit.INCH);
        double vy = odo.getVelY(DistanceUnit.INCH);
        turret.vx = vx * Math.cos(offset) - vy * Math.sin(offset);
        turret.vy = vx * Math.sin(offset) + vy * Math.cos(offset);
        turret.wbot = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        turret.tUpdate();          // also computes the virtual goal + turret.d (virtual distance)
        long t2 = System.nanoTime();

        // shooter AFTER the turret, so RPM/hood use this loop's (virtual) distance
        intakeSensor.update(turret);                     // ball at intake? shootable spot? -> LEDs
        shooter.intakeBlocked = intakeSensor.blocked;
        shooter.setShot(turret.d); // only does something when Shooter.AUTO_SHOT = true
        shooter.update();
        long t3 = System.nanoTime();
        boolean panelsDue = System.currentTimeMillis() - lastPanelsMs >= PANELS_PERIOD_MS;

//        if (apriltag.localize(turret.turretAngle(), loc.getH())) {
//            if (pads.g2.left_trigger_pressed) {
//                loc.update(apriltag.camX, apriltag.camY, apriltag.camH);
//                camOn = true;
//            } else {
//                camOn = false;
//            }
//            sawTag = true;
//        } else {
//            sawTag = false;
//            camOn = false;
//        }
        //everything above is tracking

        double rot = Math.toRadians(FIELD_DRAW_ROT_DEG), cr = Math.cos(rot), sr = Math.sin(rot);
        double fx = loc.getX() * cr - loc.getY() * sr;   // rotated for the drawing only
        double fy = loc.getX() * sr + loc.getY() * cr;
        double fh = loc.getH() + rot;
        if (panelsDue) {
        field.moveCursor(fx, fy);
        field.setStyle("black", "red", 1);
        field.circle(9.0);
        field.moveCursor(fx, fy);
        field.setStyle("lightblue", "lightblue", 1);
        field.line(fx + 9 * Math.cos(fh), fy + 9 * Math.sin(fh));
        double aim = fh + turret.turretAngle();
        field.moveCursor(fx, fy);
        field.setStyle("yellow", "yellow", 0.5);
        field.line(fx + 30 * Math.cos(aim), fy + 30 * Math.sin(aim));

//        if (sawTag) {
//            field.moveCursor(apriltag.camX, apriltag.camY);
//            field.setStyle("black", "orange", 2.0);
//            field.circle(5.0);
//            field.moveCursor(apriltag.camX, apriltag.camY);
//            field.setStyle("green", "green", 2.0);
//            field.line(apriltag.camX + 5 * Math.cos(apriltag.camH), apriltag.camY + 5 * Math.sin(apriltag.camH));
//        }

        //telmetry block
        field.update();
        }
        telemetry.addData("start pose", usedAutonPose ? "FROM AUTON" : "taped start pose");
        if (alliance){
            telemetry.addData("alliance:", "BLUE");
        } else{
            telemetry.addData("alliance:", "RED");
        }
        telemetry.addData("sawTag:", sawTag).addData("xDiff:", apriltag.camX - loc.getX()).addData("yDiff:", apriltag.camY - loc.getY());
        telemetry.addData("offset deg", Math.toDegrees(offset));
        telemetry.addData("loop ms", "avg %.0f  worst %.0f  (last 1 s)", loopAvgShow, loopMaxShow);
        telemetry.addData("worst ms: drive+odo / turret / shooter+sensor / telemetry", "%.0f / %.0f / %.0f / %.0f",
                secShow[0], secShow[1], secShow[2], secShow[3]);
        telemetry.addData("turret want (deg)", "%.1f", Math.toDegrees(turret.theta));
        telemetry.addData("turret at (deg)", "%.1f", Math.toDegrees(turret.turretAngle()));
        telemetry.addData("turret error (deg)", "%.1f", Math.toDegrees(turret.pError));
        telemetry.addData("reachable", turret.reachable);
        telemetry.addData("aim", "off-axis %.0f deg  depth %.1f in", turret.offAxisDeg, turret.aimDepth);
        telemetry.addData("ff / cmd vel (rad/s)", "%.2f / %.2f", turret.wturret, turret.wCmd);
        telemetry.addData("actual vel (rad/s)", "%.2f", turret.tVelocity());
        telemetry.addData("turret power", "%.2f", turret.lastPower);
        telemetry.addData("turret FAULT", turret.faulted ? "YES: " + turret.faultReason : "no");
        telemetry.addData("kP / kD", "%.3f / %.5f", turret.kP, turret.kD);
        telemetry.addData("robot x, y, h", "%.1f, %.1f, %.1f", loc.getX(), loc.getY(), Math.toDegrees(loc.getH()));
        if (panelsDue) {
        panels.addData("turret error (deg)", Math.toDegrees(turret.pError));
        panels.addData("turret target (deg)", Math.toDegrees(turret.theta));
        panels.addData("turret at (deg)", Math.toDegrees(turret.turretAngle()));
        panels.addData("turret power", turret.lastPower);
        panels.addData("dist to goal (in)", turret.d);
        panels.addData("real dist (in)", turret.dReal);
        panels.addData("tof (s)", turret.tofNow);
        panels.addData("flywheel rpm", shooter.rpm);
        panels.addData("target rpm", Shooter.TARGET_RPM);
        panels.update();
        lastPanelsMs = System.currentTimeMillis();
        }
        telemetry.addData("SOTM", "%s  virtual d %.1f  real d %.1f  tof %.2f  lead %.1f in",
                turret.SOTM ? "on" : "off", turret.d, turret.dReal, turret.tofNow,
                Math.hypot(turret.virX - turret.targetx, turret.virY - turret.targety));
        telemetry.addData("shot", "%s  rpm %.0f  hood %.3f",
                Shooter.AUTO_SHOT ? "AUTO (table)" : "manual", Shooter.TARGET_RPM, Shooter.HOOD_POS);
        telemetry.update();

        // ---- profiler ----
        long t4 = System.nanoTime();
        double[] parts = {(t1 - t0) / 1e6, (t2 - t1) / 1e6, (t3 - t2) / 1e6, (t4 - t3) / 1e6};
        for (int i = 0; i < 4; i++) secMax[i] = Math.max(secMax[i], parts[i]);
        double loopMs = (t4 - lastLoopNanos) / 1e6;   // full loop, including SDK time between loops
        lastLoopNanos = t4;
        loopMax = Math.max(loopMax, loopMs);
        loopSum += loopMs;
        loopCount++;
        if (t4 - profWindowStart > 1e9) {
            System.arraycopy(secMax, 0, secShow, 0, 5);
            java.util.Arrays.fill(secMax, 0);
            loopMaxShow = loopMax;
            loopAvgShow = loopSum / Math.max(loopCount, 1);
            loopMax = 0; loopSum = 0; loopCount = 0;
            profWindowStart = t4;
        }
    }

    @Override
    public void stop() {
        intakeSensor.stop();
    }
}
