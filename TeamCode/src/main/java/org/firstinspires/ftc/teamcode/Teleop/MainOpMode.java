package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subSystems.AprilTag;
import org.firstinspires.ftc.teamcode.subSystems.MotorOuttake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.fieldCentricMecanum;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.localizer;
import org.firstinspires.ftc.teamcode.subSystems.turret;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.FieldManager;


@TeleOp(name="MainOpMode")
    public class MainOpMode extends OpMode {
    TelemetryManager panels;
    FieldManager field;
    private double axial, lateral, yaw;
    fieldCentricMecanum drive = new fieldCentricMecanum();
    Intake Intake = new Intake();
    MotorOuttake Outtake = new MotorOuttake();
    Shooter shooter = new Shooter();

    //    localizer loc = new localizer(-63, -63, 0);
    localizer loc = new localizer(-63, -63, Math.PI/2);

    GoBildaPinpointDriver odo;
    turret turret = new turret();
    AprilTag apriltag = new AprilTag();
    public double prevX, prevY, prevH, dxField, dyField;
    public boolean sawTag;
    public boolean camOn = true;
    public boolean alliance = true; //true = blue, red = false

    long lastLoopNanos = System.nanoTime();

    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
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
//        Intake.init(this);
//        Outtake.init(this);
        turret.init(this);
//        apriltag.init(this);
        odo = drive.odo;
    }

    @Override
    public void loop() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.clearBulkCache();

        axial = gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;
        yaw = -gamepad1.right_stick_x;

        drive.fieldCentric(axial, lateral, yaw);
//        Intake.runIntake();
//        Outtake.runOuttake();

        if (gamepad2.rightBumperWasPressed()) {
            alliance = !alliance;
        }

        if (alliance) { //BLUE
            if (loc.getY() <= 0){
                turret.targetUpdate("ba");
            } else {
                turret.targetUpdate("bs");

            }
        } else {
            if (loc.getY() <= 0){ //red
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

        // shooter AFTER the turret, so RPM/hood use this loop's (virtual) distance
        shooter.setShot(turret.d); // only does something when Shooter.AUTO_SHOT = true
        shooter.update();

//        if (apriltag.localize(turret.turretAngle(), loc.getH())) {
//            if (gamepad2.left_trigger_pressed) {
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

        double fx = loc.getX(), fy = loc.getY(), fh = loc.getH();
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
        if (alliance){
            telemetry.addData("alliance:", "BLUE");
        } else{
            telemetry.addData("alliance:", "RED");
        }
        telemetry.addData("sawTag:", sawTag).addData("xDiff:", apriltag.camX - loc.getX()).addData("yDiff:", apriltag.camY - loc.getY());
        telemetry.addData("offset deg", Math.toDegrees(offset));
        long now = System.nanoTime();
        telemetry.addData("loop ms", "%.0f", (now - lastLoopNanos) / 1e6);
        lastLoopNanos = now;
        telemetry.addData("turret want (deg)", "%.1f", Math.toDegrees(turret.theta));
        telemetry.addData("turret at (deg)", "%.1f", Math.toDegrees(turret.turretAngle()));
        telemetry.addData("turret error (deg)", "%.1f", Math.toDegrees(turret.pError));
        telemetry.addData("reachable", turret.reachable);
        telemetry.addData("ff / cmd vel (rad/s)", "%.2f / %.2f", turret.wturret, turret.wCmd);
        telemetry.addData("actual vel (rad/s)", "%.2f", turret.tVelocity());
        telemetry.addData("turret power", "%.2f", turret.lastPower);
        telemetry.addData("turret FAULT", turret.faulted ? "YES: " + turret.faultReason : "no");
        telemetry.addData("kP / kD", "%.3f / %.5f", turret.kP, turret.kD);
        telemetry.addData("robot x, y, h", "%.1f, %.1f, %.1f", loc.getX(), loc.getY(), Math.toDegrees(loc.getH()));
        panels.addData("turret error (deg)", Math.toDegrees(turret.pError));
        panels.addData("turret target (deg)", Math.toDegrees(turret.theta));
        panels.addData("turret at (deg)", Math.toDegrees(turret.turretAngle()));
        panels.addData("turret power", turret.lastPower);
        panels.addData("dist to goal (in)", turret.d);
        panels.addData("real dist (in)", turret.dReal);
        panels.addData("tof (s)", turret.tofNow);
        panels.addData("flywheel rpm", shooter.rpm);
        panels.addData("target rpm", Shooter.TARGET_RPM);
        telemetry.addData("SOTM", "%s  virtual d %.1f  real d %.1f  tof %.2f  lead %.1f in",
                turret.SOTM ? "on" : "off", turret.d, turret.dReal, turret.tofNow,
                Math.hypot(turret.virX - turret.targetx, turret.virY - turret.targety));
        telemetry.addData("shot", "%s  rpm %.0f  hood %.3f",
                Shooter.AUTO_SHOT ? "AUTO (table)" : "manual", Shooter.TARGET_RPM, Shooter.HOOD_POS);
        panels.update();
        telemetry.update();
    }
}
