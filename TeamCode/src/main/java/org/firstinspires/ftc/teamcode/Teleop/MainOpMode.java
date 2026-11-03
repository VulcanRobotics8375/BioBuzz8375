package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
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
    localizer loc = new localizer (0,0,0);
    GoBildaPinpointDriver odo;
    turret turret = new turret();
    public double prevX, prevY, prevH;
    public double camX, camY, camH;


    public void init(){
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        turret.instantiateSubsystem(this);
        drive.instantiateSubsystem(this);
        Intake.instantiateSubsystem(this);
        Outtake.instantiateSubsystem(this);
        drive.init(this);
        Intake.init(this);
        Outtake.init(this);
        turret.init(this);
        odo = drive.odo;
//        odo.setOffsets(0, 0); // TODO: set measured pod offsets (mm)
    }

    @Override
    public void loop() {

        axial = gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;

        drive.fieldCentric(axial, lateral, yaw);
        Intake.runIntake();
        Outtake.runOuttake();
//        turret.tUpdate();

        double dx = odo.getPosX(DistanceUnit.INCH) - prevX;
        double dy = odo.getPosY(DistanceUnit.INCH) - prevY;
        double dh = normalizeRadians(odo.getHeading() - prevH);

        loc.predict(dx , dy, dh);
        prevX = odo. getPosX(DistanceUnit.INCH);
        prevY = odo. getPosY(DistanceUnit.INCH);
        prevH = normalizeRadians(odo.getHeading());
//        loc.update (camX, camY, camH);
        turret.x = loc.getX();
        turret.y = loc.getY();
        turret.heading = loc.getH();
        turret.vx = odo.getVelX(DistanceUnit.INCH);
        turret.vy = odo.getVelY(DistanceUnit.INCH);
        turret.wbot = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        //TODO loc.udpate(camX, camY, camH)
        //everything above is tracking

        double fx = loc.getX(), fy = loc.getY(), fh = loc.getH();
        field.moveCursor(fx, fy);
        field.setStyle("lightblue", "blue", 2.0);
        field.circle(9.0);
        field.moveCursor(fx, fy);
        field.setStyle("red", "red", 2.0);
        field.line(fx + 9 * Math.cos(fh), fy + 9 * Math.sin(fh));
        field.update(); //panels hopefully?

        telemetry.update();
    }
}
