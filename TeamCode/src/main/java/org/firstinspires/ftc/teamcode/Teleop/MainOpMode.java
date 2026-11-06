package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subSystems.AprilTag;
import org.firstinspires.ftc.teamcode.subSystems.MotorOuttake;
import org.firstinspires.ftc.teamcode.subSystems.fieldCentricMecanum;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.localizer;
import org.firstinspires.ftc.teamcode.subSystems.turret;
import com.bylazar.field.PanelsField;
import com.bylazar.field.FieldManager;


@TeleOp(name="MainOpMode")
    public class MainOpMode extends OpMode {
    FieldManager field;
    private double axial, lateral, yaw;
    fieldCentricMecanum drive = new fieldCentricMecanum();
    Intake Intake = new Intake();
    MotorOuttake Outtake = new MotorOuttake();
//    localizer loc = new localizer(-63, -63, 0);
    localizer loc = new localizer(-0, -0, Math.PI/2);

    GoBildaPinpointDriver odo;
    turret turret = new turret();
    AprilTag apriltag = new AprilTag();
    public double prevX, prevY, prevH, dxField, dyField;
    public boolean sawTag;

    public void init() {
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());
        turret.instantiateSubsystem(this);
        drive.instantiateSubsystem(this);
        Intake.instantiateSubsystem(this);
        Outtake.instantiateSubsystem(this);
        apriltag.instantiateSubsystem(this);
        loc.instantiateSubsystem(this);
        drive.init(this);
        Intake.init(this);
        Outtake.init(this);
//        turret.init(this);
        apriltag.init(this);
        odo = drive.odo;
    }

    @Override
    public void loop() {

        axial = -gamepad1.left_stick_y;
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;

        drive.fieldCentric(axial, lateral, yaw);
        Intake.runIntake();
        Outtake.runOuttake();
//        turret.tUpdate();

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
        if (apriltag.localize(/*turret.turretAngle()*/0, loc.getH())) {
            loc.update(apriltag.camX, apriltag.camY, apriltag.camH);
            sawTag = true;
        } else {
            sawTag = false;
        }
        //everything above is tracking

        double fx = loc.getX(), fy = loc.getY(), fh = loc.getH();
        field.moveCursor(fx, fy);
        field.setStyle("black", "blue", 2.0);
        field.circle(9.0);
        field.moveCursor(fx, fy);
        field.setStyle("red", "red", 2.0);
        field.line(fx + 9 * Math.cos(fh), fy + 9 * Math.sin(fh));

        if (sawTag) {
            field.moveCursor(apriltag.camX, apriltag.camY);
            field.setStyle("black", "orange", 2.0);
            field.circle(5.0);
            field.moveCursor(apriltag.camX, apriltag.camY);
            field.setStyle("green", "green", 2.0);
            field.line(apriltag.camX + 5 * Math.cos(apriltag.camH), apriltag.camY + 5 * Math.sin(apriltag.camH));
        }

        field.update(); //panels hopefully?
        telemetry.addData("sawTag:", sawTag).addData("xDiff:", apriltag.camX - loc.getX()).addData("yDiff:", apriltag.camY - loc.getY());
        ;
        telemetry.addData("offset deg", Math.toDegrees(offset));

        telemetry.update();
    }
}
